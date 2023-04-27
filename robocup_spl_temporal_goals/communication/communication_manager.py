import time
import math
from typing import Dict, Type, Union
from enum import Enum

import twisted
from twisted.internet import reactor, task

from lib.misc.singleton import Singleton
from communication.socket_utils import setup_read_socket, setup_write_socket
from communication.nao_communication_controller import NAOCommunicationController
from lib.dfa.dfa_handler import DFAHandler
from lib.plan.policy_handler import PolicyHandler
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry

class BehaviorControlMode(Enum):
    TASK_MODE = 0
    PLAN_MODE = 1

class CommunicationManager(metaclass=Singleton):
    def __init__(self,
            local_interface_ip : str,                           #Local LAN interface IP
            robor_number_to_robot_IP_map : Dict[str, str],      #Maps robot number to data (including robot ip)
            robot_udp_base_read_port : int = 65100,             #Listening port for incoming messages from robots
            robot_udp_base_write_port : int = 65200,            #Socket port of outgoing messages for each robot (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
            robot_udp_base_dest_port : int = 65000,             #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)

            robot_read_socket_timeout : int = 0.05,             #Timeout after which the read operation on a socket is aborted
            robot_alive_timeout : int = 0.05,                   #Timeout after which a non-responding robot is considered "not alive" (crashed or disconnected)
            last_task_id_timeout : int = 1,                     #TIME between two subsequent "taskQueue?" requests, that periodically ask the task
            update_tasks_timeout : int = 0.2,                   #TIME between two subsequent calls to the update_assigned_tasks in the NAOCommunicationController

            frontend_controller = None,
            frontend_alive_timeout : int = 1,                    #Timeout after which the frontend is considered "not alive" (crashed or disconnected)

            default_behavior_control_mode = BehaviorControlMode.TASK_MODE
        ):

        #print("LOCAL INTERFACE IP: %s" % local_interface_ip)

        ''' 
        ___________________
        |                  |
        |  SETUP FRONTEND  |
        |__________________|

        '''
        
        self.frontend_controller = frontend_controller
        if frontend_controller is not None:
            self.frontend_controller.set_manager(self)
        
        self.control_shell = None

        

    #-----------------------------------------------------------------

        ''' 
        _____________________
        |                    |
        |  SETUP ROBOT DATA  |
        |____________________|

        '''
        self.ball_position = {"position_timestamp": None, "update_timestamp": time.time(), "last_seen_by_robot" : -1, "position" : (0, 0)}
        self.robot_positions = {}


        self.robot_role_to_number = {}
        self.robot_role_to_behavior_control_mode = {}
        self.default_behavior_control_mode = default_behavior_control_mode

        self.role_to_plan = {}
        #Value used when the current role of the robot is unknown
        self.UNKNOWN_ROLE = "unknown"
        #Value used by the framework when the current role of the robot can not be established yet (depends on the coordination algorithm of the robot)
        self.UNDEFINED_ROLE = "undefined"
        #Prefix used by the framework for robot roles that search the ball
        self.SEARCHER_ROLE_PREFIX = "searcher"

        self.robot_tasks = {}




        self.obstacles = {"position_timestamp": None, "update_timestamp": time.time(), "obstacles" : []}
    #-----------------------------------------------------------------

        ''' 
        ___________________________________
        |                                  |
        |  SETUP ROBOT NETWORK INTERFACES  |
        |__________________________________|

        '''
        self.local_interface_ip = local_interface_ip
        self.robot_udp_base_read_port = robot_udp_base_read_port
        self.robot_udp_base_write_port = robot_udp_base_write_port
        self.robot_udp_base_dest_port = robot_udp_base_dest_port

#TODO: Populate this with a broadcast discovery process (e.g. broadcast a message on the LAN until some robot responds) maybe asyncronous and continuous
        self.robor_number_to_robot_IP_map = robor_number_to_robot_IP_map

        self.robot_read_socket_timeout = robot_read_socket_timeout
        self.robot_alive_timeout = robot_alive_timeout
        self.last_task_id_timeout = last_task_id_timeout                   
        self.update_tasks_timeout = update_tasks_timeout                    


#TODO: Populate this by broadcast (as above)
#To do this the alive/not alive check in the NAOCommunicationController should be revised as well
        self.robot_communication_controllers = {}
        for robot_number, robot_data in self.robor_number_to_robot_IP_map.items():
            write_port = self.robot_udp_base_write_port + robot_number
            read_port = self.robot_udp_base_read_port + robot_number
            dest_port = self.robot_udp_base_dest_port + robot_number
            self.robot_communication_controllers[robot_number] = \
                NAOCommunicationController(
                    reactor, 
                    robot_number, 
                    robot_data["robotName"],
                    self.local_interface_ip, 
                    
                    write_port, self.robor_number_to_robot_IP_map[robot_number]["robotIP"], dest_port, 
                    read_port,  self.robot_read_socket_timeout,
                    
                    self.robot_alive_timeout,
                    
                    self
                )
        #-----------------------------------------------------------------

        ''' 
        ________________________
        |                        |
        |  SETUP PERIODIC TASKS  |
        |________________________|

        '''

        #Start LoopingCalls for each robot controller

        #Does two things:
        # 1) If the robot is currently alive but it has not sent any message since controller.alive_client_timeout seconds, will set it to "not alive"
        # 2) If the robot has recently (less than controller.alive_client_timeout seconds ago) sent the any message, will
        #       send the message "uthere?", as a KEEPALIVE request to the robot. The response to the KEEPALIVE is DISABLED on the robot as the
        #       regular traffic is already more than enough.                                    
        for controller in self.robot_communication_controllers.values():
            controller.check_alive_task.start(robot_alive_timeout)      

        for controller in self.robot_communication_controllers.values():
            
            #Sends a "lastTaskID?" message to the robot every LAST_TASK_ID_TIMEOUT seconds, to which the robot will answer with a message like:
            #   "lastTaskID,<last task ID received by robot>,<last task ID completed by robot>"
            #See the NAOCommunicationController documentation to see how the "lastTaskID" message is handled         
            #NOTICE: This is done even when in PLAN_MODE because it populates the robot_tasks dictionary as soon as a new robot is found
            controller.check_last_task_id_task.start(last_task_id_timeout)
            
            if self.default_behavior_control_mode == BehaviorControlMode.TASK_MODE:
                #Sends a "lastTaskQueue?" message to the robot every LAST_TASK_ID_TIMEOUT seconds, to which the robot will answer with a message like:
                #   "lastTaskQueue;lastTaskID,<last task ID received by robot>,<last task ID completed by robot>;<task type>,<task fields separated by commas>;..."
                #where "..." represents other task types and their fields (i.e. destination for robot or for ball)
                #   NOTICE: this task is activated when the Python server first starts or whenever the robot is detected as "not alive". It is deactivated when the robot
                #   is detected to be "alive"
                controller.check_task_queue_task.start(last_task_id_timeout)

        for controller in self.robot_communication_controllers.values():
            controller.update_assigned_tasks_task.start(update_tasks_timeout)

#TODO: move to the frontend (maybe in the constructor)
        #Start LoopingCalls for the frontend controller
        if self.frontend_controller is not None:
            if hasattr(frontend_controller, "check_alive_task"):
                if hasattr(frontend_controller, "start_check_client_alive_task"):
                    if frontend_controller.start_check_client_alive_task:
                        self.frontend_controller.check_alive_task.start(frontend_alive_timeout)
                else:
                    self.frontend_controller.check_alive_task.start(frontend_alive_timeout)
    #-----------------------------------------------------------------

    def exit_gracefully(self, signal=None, frame=None):
        print("Exiting server gracefully...")
        reactor.stop()
        for controller in self.robot_communication_controllers.values():
            controller.close_sockets()
        
        if self.frontend_controller is not None:
            self.frontend_controller.close_sockets()
        
        if self.control_shell is not None:
            self.control_shell.close()

    #--------------------
    # 
    #   ACCESS METHODS
    # 
    #--------------------   

    def get_robot_controller(self, robot_number):
        return self.robot_communication_controllers[robot_number]

    def getRobotTasks(self, robot_number):
        return self.robot_tasks[robot_number]["tasks"]

    def getLastReceivedTaskID(self, robot_number):
        return self.robot_tasks[robot_number]["lastReceivedTaskID"]

    def getLastCompletedTaskID(self, robot_number):
        return self.robot_tasks[robot_number]["lastCompletedTaskID"]
    
    def getRobotRoleFromNumber(self, robot_number):
        for role, role_data in self.robot_role_to_number.items():
            if role_data["robot_number"] == robot_number:
                return role
        return self.UNKNOWN_ROLE

    #----------------------
    # 
    #   BEHAVIOR CONTROL
    # 
    #----------------------

    #--- CONTROL MODE ---

    def set_default_behavior_control_mode(self, robot_role : str = None, robot_number : int = None):
        if self.default_behavior_control_mode == BehaviorControlMode.TASK_MODE:
            self.set_task_mode(robot_role=robot_role, robot_number=robot_number)
        else:
            self.set_plan_mode(robot_role=robot_role, robot_number=robot_number)

    def set_task_mode(self, robot_role : str = None, robot_number : int = None):
        assert robot_role is not None or robot_number is not None

        if robot_role is not None:
            robot_number = self.robot_role_to_number[robot_role]
        elif robot_number is not None:
            robot_role = self.getRobotRoleFromNumber(robot_number)

        if robot_role != self.UNKNOWN_ROLE:
            self.robot_role_to_behavior_control_mode[robot_role] = BehaviorControlMode.TASK_MODE

        if robot_number is not None:
            self.scheduleRobotTasksReset(robot_number=robot_number)
                    

    def set_plan_mode(self, robot_role : str = None, robot_number : int = None):
        assert robot_role is not None or robot_number is not None

        if robot_role is not None:
            if robot_role in self.robot_role_to_number:
                robot_number = self.robot_role_to_number[robot_role]
        elif robot_number is not None:
            robot_role = self.getRobotRoleFromNumber(robot_number)

        if robot_number is not None and robot_role != self.UNDEFINED_ROLE and not robot_role.startswith(self.SEARCHER_ROLE_PREFIX) and self.robot_role_to_behavior_control_mode[robot_role] != BehaviorControlMode.PLAN_MODE:
            self.scheduleRobotTasksReset(robot_number=robot_number)

        if robot_role != self.UNKNOWN_ROLE:
            self.robot_role_to_behavior_control_mode[robot_role] = BehaviorControlMode.PLAN_MODE
                


    def is_task_mode(self, robot_role : str = None, robot_number : int = None):
        assert robot_role is not None or robot_number is not None

        if robot_role is not None:
            robot_number = self.robot_role_to_number[robot_role]
        elif robot_number is not None:
            robot_role = self.getRobotRoleFromNumber(robot_number)

        if robot_role == self.UNKNOWN_ROLE or robot_role not in self.robot_role_to_behavior_control_mode:
            return self.default_behavior_control_mode == BehaviorControlMode.TASK_MODE
        else:
            return self.robot_role_to_behavior_control_mode[robot_role] == BehaviorControlMode.TASK_MODE




    #--- TASK CONTROL ---

    
    def generateLastQueueMessage(self, robotNumber):
        message = "taskQueue;"
        #TaskIDs
        message += "lastTaskID,"+str(self.getLastReceivedTaskID(robotNumber))+","+str(self.getLastCompletedTaskID(robotNumber))
        
        if(len(self.getRobotTasks(robotNumber)) == 0):
            return message

        #TaskQueues
        message += ";"
        for i, task in enumerate(self.getRobotTasks(robotNumber)):
            
            message += str(task["taskType"])+","+str(task["taskID"])
            
            if task["parameters"] is not None:
                message += ","+(",".join([str(param) for param in task["parameters"]]))
            
            if len(self.getRobotTasks(robotNumber)) > 0 \
            and \
            i < len(self.getRobotTasks(robotNumber)) - 1: 
                message += ";"
        
        return message

    def updateRobotLastTaskID(self, robot_number, timestamp, lastReceivedTaskID, lastCompletedTaskID):
        if robot_number not in self.robot_tasks.keys():
            self.robot_tasks[robot_number] = {"last_timestamp": timestamp, "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : [], "reset" : False, "tasksToDelete" : []}

        else:
        #elif timestamp > self.robot_tasks[robot_number]["last_timestamp"]:

            #In this case scan the list of tasks currently assigned to the robot #robot_number and delete the ones with
            #a taskID lower than the last ID of the last completed task received by the robot controller
            old_tasks = self.robot_tasks[robot_number]["tasks"]
            new_tasks = []
            for task in old_tasks:
                if task["taskID"] > self.robot_tasks[robot_number]["lastCompletedTaskID"]:
                    new_tasks.append(task)

            self.robot_tasks[robot_number] = {"last_timestamp": timestamp, "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : new_tasks, "reset" : False, "tasksToDelete" : self.robot_tasks[robot_number]["tasksToDelete"]}
        

        if self.frontend_controller is not None  and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, self.generateLastQueueMessage(robot_number))

        
        self.get_robot_controller(robot_number).update_assigned_tasks()

    def addTask(self, robot_number, taskType, taskID, parameters = None):
        self.set_task_mode(robot_number = robot_number)
        #Don't add the task if it is already contained in the task list for the robot #robot_number
        for task in self.robot_tasks[robot_number]["tasks"]:
            if task["taskID"] == taskID:
                return
        
        self.robot_tasks[robot_number]["tasks"].append({"taskType" : taskType, "taskID" : taskID, "parameters" : parameters})    
        
    def scheduleTaskDeletion(self, robot_number : int, taskID : str):
        self.robot_tasks[robot_number]["tasksToDelete"].append(str(taskID))

    def resetTasksToDelete(self, robot_number : int):
        self.robot_tasks[robot_number]["tasksToDelete"] = []
    
    #--- Plan-based CONTROL ---

    def update_plan(self, plan_wrapper : Union[PolicyHandler, DFAHandler], robot_role = None, robot_number = None):
        assert robot_number is not None or robot_role is not None
        self.set_plan_mode(robot_role = robot_role, robot_number=robot_number)
        self.role_to_plan[robot_role] = plan_wrapper
    
    def get_current_plan_action(self, robot_role):
        assert robot_role is not None
        assert robot_role in self.role_to_plan.keys()
        return self.role_to_plan[robot_role].get_current_action()
    
    def get_next_plan_action(self, robot_role, verbose = False):
        assert robot_role is not None
        assert robot_role in self.role_to_plan.keys()
        if verbose: 
            print("Chosing action for role: "+robot_role+"\n------------\n")
        return self.role_to_plan[robot_role].get_next_action(verbose = verbose)

    def reset_plan(self, robot_role):
        assert robot_role is not None
        assert robot_role in self.role_to_plan.keys()
        self.role_to_plan[robot_role].reset()

    #--------------------
    # 
    #   RESET METHODS
    # 
    #--------------------   

    def scheduleRobotTasksReset(self, robot_number):
        if robot_number in self.robot_tasks.keys():
            self.robot_tasks[robot_number]["reset"] = True
        else:
            print("Unknown robot number: %d. No need to schedule task reset!" % (robot_number))
            
    def unscheduleRobotTasksReset(self, robot_number):
        if robot_number in self.robot_tasks.keys():
            self.robot_tasks[robot_number]["reset"] = False
        else:
            print("Unknown robot number: %d. No need to unschedule task reset!" % (robot_number))
    
    def isRobotScheduledForTasksReset(self, robot_number):
        return self.robot_tasks[robot_number]["reset"]

    #Called at the beginning of the handleTaskQueueMessage for a specific robot (that is received when the server crashed while the
    # robot was still running, to ask the task list to the robot), resets the task queue and fills it with the current tasks
    def resetRobotTaskQueue(self, robot_number, timestamp, lastReceivedTaskID, lastCompletedTaskID):
        #if robot_number not in self.robot_tasks.keys():
        self.robot_tasks[robot_number] = {"last_timestamp": time.time(), "update_timestamp" : time.time(), "lastCompletedTaskID" : lastCompletedTaskID, "lastReceivedTaskID" : lastReceivedTaskID, "tasks" : [], "reset" : False, "tasksToDelete" : []}
    
    #Called when the robot is detected as "NOT ALIVE" to reset the task list that is currently known to the server
    def removeRobotTaskQueue(self, robot_number):
        if robot_number in self.robot_tasks.keys():
            self.robot_tasks.pop(robot_number) 


    #--------------------
    # 
    #   UPDATE METHODS
    # 
    #--------------------   


    #--- FIELD DATA ---



    def updateRobotPosition(self, robot_number, timestamp, angle, position):
        #if robot_number not in self.robot_positions.keys() or timestamp > self.robot_positions[robot_number]["position_timestamp"]:
        self.robot_positions[robot_number] = {"position_timestamp": timestamp, "update_timestamp": time.time(), "position" : position}

        #Update robot position for specific robot number
        ValueRegistry().set(value_name = "position", value = position, robot_number = robot_number)

        #Update robot position for specific robot role
        robot_role = self.getRobotRoleFromNumber(robot_number)
        if robot_role != self.UNKNOWN_ROLE and robot_role != self.UNDEFINED_ROLE:
            ValueRegistry().set(value_name = "position", value = position, robot_role = robot_role)

        #print("Robot %d [%s] position updated: %s" % (robot_number, robot_role, position))
        if self.frontend_controller is not None  and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, message = "robotPosition:"+str(robot_number)+","+str(angle)+","+str(self.robot_positions[robot_number]["position"][0])+","+str(self.robot_positions[robot_number]["position"][1]))



    #If it is not "unknown" or "undefined", resets the role for a specific robot number by:
    # 1) Removing it from the robot_role_to_number dictionary
    # 2) Removes all entries for that role from the registries
    # 3) In Plan mode, it also resets the plan to the initial state
    def resetRobotRole(self, robot_number):
        robot_role = self.getRobotRoleFromNumber(robot_number)
        
        if robot_role != self.UNKNOWN_ROLE:
            self.robot_role_to_number.pop(robot_role)

            if not self.is_task_mode(robot_number = robot_number):
                #Remove all entries for that role
                ValueRegistry().remove_all_items_for_robot_role(robot_role)

                if robot_role in self.role_to_plan.keys():
                    self.reset_plan(robot_role)
            
                
        if self.frontend_controller is not None and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, "robotRole:"+str(robot_number)+",unknown")


    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateRobotRole(self, robot_number, timestamp, new_role):

        #Information already up-to-date
        if new_role in self.robot_role_to_number.keys() and self.robot_role_to_number[new_role]["robot_number"] == robot_number:
            return

        old_role = self.getRobotRoleFromNumber(robot_number)
        
        #If role changed...
        if new_role != old_role:

            print("\nRobot %d: OLD ROLE: %s, NEW ROLE: %s" % (robot_number, old_role, new_role))
            #...Remove current robot role entry (given that roles are stored in a role -> robot_number map, if robot_number now has a different role, we want to remove the existing entry)
            self.resetRobotRole(robot_number) 
        

        #Set new role with the new robot number assignment
        self.robot_role_to_number[new_role] = {"role_timestamp": timestamp, "update_timestamp": time.time(), "robot_number" : robot_number}
        print("\nRobot %d, current roles: %s" % (robot_number, self.robot_role_to_number))

        #If we have encountered a new robot role, set it to the default control mode for the time being (until the control mode is set explicitly)
        #print(self.robot_role_to_behavior_control_mode)
        if new_role not in self.robot_role_to_behavior_control_mode.keys():
            self.set_default_behavior_control_mode(robot_number = robot_number)
        
        if self.frontend_controller is not None  and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, message = "robotRole:"+str(robot_number)+","+new_role)

        if self.is_task_mode(robot_number=robot_number):
            if not self.robot_communication_controllers[robot_number].check_task_queue_task.running:
                #Sends a "lastTaskQueue?" message to the robot every LAST_TASK_ID_TIMEOUT seconds, to which the robot will answer with a message like:
                #   "lastTaskQueue;lastTaskID,<last task ID received by robot>,<last task ID completed by robot>;<task type>,<task fields separated by commas>;..."
                #where "..." represents other task types and their fields (i.e. destination for robot or for ball)
                #   NOTICE: this task is activated when the Python server first starts or whenever the robot is detected as "not alive". It is deactivated when the robot
                #   is detected to be "alive"
                self.robot_communication_controllers[robot_number].check_task_queue_task.start(self.last_task_id_timeout)
        else:
            if self.robot_communication_controllers[robot_number].check_task_queue_task.running:
                self.robot_communication_controllers[robot_number].check_task_queue_task.stop()



    #MIGHT ADD A CONTROL ON THE DISTANCE OF THE BALL TO CHOOSE THE MOST PRECISE OBSERVATION
    def updateBallPosition(self, robot_number, timestamp, position):
        #if timestamp > self.ball_position["position_timestamp"]:
        self.ball_position = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : robot_number, "position" : position}
        
        #Update ball position for specific robot number
        ValueRegistry().set(value_name = "ball_position", value = position, robot_number = robot_number)

        #Update ball position for specific robot role
        robot_role = self.getRobotRoleFromNumber(robot_number)
        if robot_role != self.UNKNOWN_ROLE and robot_role != self.UNDEFINED_ROLE:
            ValueRegistry().set(value_name = "ball_position", value = position, robot_role = robot_role)

        #Update last perceived ball position
        ValueRegistry()["last_ball_position"] = position
        
        if self.frontend_controller is not None  and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, message = "ballPosition:"+str(self.ball_position["position"][0])+","+str(self.ball_position["position"][1]))
    


    def updateObstaclesPosition(self, robot_number, timestamp, obstacles):
        #if timestamp is None or timestamp > self.obstacles["position_timestamp"]:
        self.obstacles = {"position_timestamp": timestamp, "update_timestamp": time.time(), "last_seen_by_robot" : robot_number, "obstacles" : obstacles}
        
        robot_role = self.getRobotRoleFromNumber(robot_number)
        if robot_role != self.UNKNOWN_ROLE and robot_role != self.UNDEFINED_ROLE:
            ValueRegistry().set(value_name = "obstacles", value = obstacles, robot_role = robot_role)

        if self.frontend_controller is not None  and self.is_task_mode(robot_number=robot_number):
            self.update_frontend(robot_number, message = "obstacles:"+";".join([str(obstacle[0]) + "," + str(obstacle[1]) for obstacle in self.obstacles["obstacles"]]))



    def updateBooleanFlags(self, robot_number, timestamp, booleanFlags : Dict[str, bool]):
        #if timestamp is None or timestamp > self.obstacles["position_timestamp"]:
        for literal_name, literal_value in booleanFlags.items():
            if isinstance(literal_value, str):
                literal_value = bool(literal_value)
            LiteralRegistry().set(literal_name, literal_value, robot_number = robot_number)
        
        #self.update_frontend(robot_number, message = "flags:"+";".join([flag_name + "," + str(flag_value) for flag_name, flag_value in LiteralRegistry().get_literal_values()]))



    #----------------------
    # 
    #   FRONTEND CONTROL
    # 
    #----------------------

    def update_frontend(self, robot_number, message):
        if self.frontend_controller is not None:
            self.frontend_controller.update(robot_number, message = message)

    def enable_frontend(self):
        if self.frontend_controller is not None:
            self.frontend_controller.enable()
    
    def sendRobotNotRespondingMessageToFrontend(self, robotNumber):
        if hasattr(self.frontend_controller, "send_robot_not_responding_message"):
            self.frontend_controller.sendRobotNotRespondingMessage(robotNumber)

    #def register_shell(self, shell : InputShell):
    #    assert isinstance(shell, InputShell)
    #    shell.set_manager(self)



    ''' 
     _____________________________
    |                             |
    |  START TWISTED UPDATE LOOP  |
    |_____________________________|

    '''

    def start(self):

        try:
            reactor.run()
        except:
            self.exit_gracefully()