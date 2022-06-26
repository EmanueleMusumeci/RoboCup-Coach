import time
import math
from typing import Dict, Type

import twisted
from twisted.internet import task

from communication.socket_utils import setup_read_socket, setup_write_socket
from lib.registries.action import ActionRegistry
from lib.registries.registry import RegistryItem


class NAOCommunicationController(twisted.internet.protocol.DatagramProtocol):
    def __init__(self, 
        reactor, 
        robot_number, 
        robot_name,
        local_interface_ip, 
        write_port, remote_dest_ip, dest_port, 
        read_port, read_socket_timeout, 
        active_client_timeout, 
        
        communication_manager : Type["CommunicationManager"] = None
    ):
        
        self.alive = False
        self.robot_number = robot_number
        self.robot_name = robot_name

        self.active_client_timeout = active_client_timeout
        
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(local_interface_ip, write_port, remote_dest_ip, dest_port=dest_port, 
                                                                                debug_message=("[NAO COMMUNICATION CONTROLLER] Writing from %s, with local port %s\n\t-> Robot %d: IP %s, UDP write port: %s" % (local_interface_ip, write_port, robot_number, remote_dest_ip, dest_port)))
        self.last_sent_message_timestamp = time.time()

        self.read_socket, self.read_addr = setup_read_socket(self, reactor, local_interface_ip, read_port, read_socket_timeout, 
                                                                debug_message=("[NAO COMMUNICATION CONTROLLER] Listening on %s, with local port %s\n\t-> Robot %d: IP %s, UDP read port: %s" % (local_interface_ip, read_port, robot_number, remote_dest_ip, write_port)))
        self.last_received_message_timestamp = time.time()
        self.read_socket_timeout = read_socket_timeout

        self.check_alive_task = task.LoopingCall(self.check_alive_states)
        self.check_last_task_id_task = task.LoopingCall(self.check_last_task_id)
        self.update_assigned_tasks_task = task.LoopingCall(self.update_assigned_tasks)
        
        #Should be empty in case the robot just started as well but in case the server crashed and the robot was still running
        #this updates the server with the latest issued tasks
        self.check_task_queue_task = task.LoopingCall(self.check_task_queue)

        self.lastReceivedTaskID = None
        self.lastCompletedTaskID = None

        self.communication_manager = communication_manager


    def close_sockets(self):
        self.write_socket.close()
        self.read_socket.close()

    def send_keepalive_request(self):
        self.send_string("uthere?")

    def send_keepalive_response(self):
        self.send_string("yeah")
        #print("KEEPALIVE response sent %s" % b"yeah\x00")

    #MESSAGE HANDLERS
    def handleLastTaskIDMessage(self, content, message_info):
        #print(content)
        content_fields = content.split(",")
        lastReceivedTaskID = int(content_fields[1])
        lastCompletedTaskID = int(content_fields[2])

        self.communication_manager.updateRobotLastTaskID(self.robot_number, message_info["timestamp"], lastReceivedTaskID, lastCompletedTaskID)

        if self.lastCompletedTaskID is not None and not self.communication_manager.is_task_mode(robot_number = self.robot_number):
            if lastCompletedTaskID > self.lastCompletedTaskID:
                robot_role = self.communication_manager.getRobotRoleFromNumber(self.robot_number)
                plan_action : RegistryItem = self.communication_manager.get_current_plan_action(robot_role)
                ActionRegistry().signal_action_completed(plan_action.name_in_registry)

        self.lastReceivedTaskID = lastReceivedTaskID
        self.lastCompletedTaskID = lastCompletedTaskID
    
    def handleRobotPoseMessage(self, content, message_info):
        content_fields = content.split(",")
        robot_angle = float(content_fields[1])
        robot_position = (float(content_fields[2]), float(content_fields[3]))

        #print("[Robot %d] Angle: %f, Position: (%f, %f)" % (self.robot_number, robot_angle, robot_position[0], robot_position[1]))

        self.communication_manager.updateRobotPosition(self.robot_number, message_info["timestamp"], robot_angle, robot_position)

    def handleBallPositionMessage(self, content, message_info):
        content_fields = content.split(",")
        position = (float(content_fields[1]), float(content_fields[2]))

        #print("[Robot %d] Ball position: (%f, %f)" % (self.robot_number, position[0], position[1]))

        self.communication_manager.updateBallPosition(self.robot_number, message_info["timestamp"], position)
    
    def handleRobotRoleMessage(self, content, message_info):
        content_fields = content.split(",")
        new_role = content_fields[1]

        #print("[Robot %d] Robot role: %s" % (self.robot_number, role))

        self.communication_manager.updateRobotRole(self.robot_number, message_info["timestamp"], new_role)
    
    def handleObstaclesMessage(self, content, message_info):
        content_fields = content.split(";")[1:]
        obstacle_list = [(float(obsX), float(obsY)) for (obsX, obsY) in [obs.split(",") for obs in content_fields]]
        
        #print("[Robot {}] Obstacles: ".format(self.robot_number), ", ".join(["({}, {})".format(obsX, obsY) for (obsX, obsY) in obstacle_list]))

        self.communication_manager.updateObstaclesPosition(self.robot_number, message_info["timestamp"], obstacle_list)
    
    def handleBooleanFlagsMessage(self, content, message_info):
        content_fields = content.split(";")[1:]
        flags_list = {flag_name : flag_value for (flag_name, flag_value) in [flag.split(":") for flag in content_fields]}
        #print("[Robot {}] Boolean flags: ".format(self.robot_number), ", ".join(["({}, {})".format(flag_name, flag_value) for flag_name, flag_value in flags_list.items()]))

        self.communication_manager.updateBooleanFlags(self.robot_number, message_info["timestamp"], flags_list)

    
    def handleTaskQueueMessage(self, content, message_info):
        content_fields = list(filter(None, content.split(";")[1:]))
        #print(content_fields)

        lastReceivedTaskID = int(content_fields[0].split(",")[1])
        lastCompletedTaskID = int(content_fields[0].split(",")[2])
        
        self.communication_manager.resetRobotTaskQueue(self.robot_number, message_info["timestamp"], lastReceivedTaskID, lastCompletedTaskID)
        
        #If there isn't any task return
        if(len(content_fields) == 1): return

        for task in content_fields[1:]:
            task_fields = task.split(",")
            
            taskType = task_fields[0]
            taskID = int(task_fields[1])

            if len(task_fields)==2:
                self.communication_manager.addTask(self.robot_number, taskType, taskID)
            elif len(task_fields)==4:
                xPos = int(math.floor(float(task_fields[2])))
                yPos = int(math.floor(float(task_fields[3])))
                self.communication_manager.addTask(self.robot_number, taskType, taskID, parameters = [xPos, yPos])
            else:
                print("Wrong taskQueue format")
                raise
        

    def handlePlanActionCompletedMessage(self, content, message_info):
        content_fields = content.split(";")[1:]

        robot_role = self.communication_manager.getRobotRoleFromNumber(self.robot_number)
        completed_task_id = int(content_fields.split(",")[1])

        if completed_task_id == self.lastCompletedTaskID +1:
            plan_action : RegistryItem = self.communication_manager.get_current_plan_action(robot_role)
            ActionRegistry().signal_action_completed(plan_action.name_in_registry)
    


    def handleMessage(self, data):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        #print("Received message from robot %d: %s" % (self.robot_number, data))

        message_fields = data.split("|")

        header = message_fields[0]
        content = message_fields[1]
        
        message_info = {"timestamp" : None, "robot_number" : self.robot_number, "content" : content}

        if len(header)>0:
            for field in header.split("."):
                if field.startswith("timestamp"):
                    message_info["timestamp"] = int(field.split(",")[1])

                #Not necessary to extract robot number because we know it already
                #elif field.startswith("robot_number"):
                #    message_info["robot_number"] = field.split(",")[1] 

        #Set last_received_message_timestamp
        self.last_received_message_timestamp = time.time()
        self.set_to_alive()
        
        #print(message_info)

        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from robot %s" % self.robot_number)
            self.send_keepalive_response()
        else:
            if content.startswith("lastTaskID"):
                self.handleLastTaskIDMessage(content, message_info)

            elif content.startswith("robot_pose"):
                self.handleRobotPoseMessage(content, message_info)

            elif content.startswith("ball_position"):
                self.handleBallPositionMessage(content, message_info)

            elif content.startswith("robot_role"):
                self.handleRobotRoleMessage(content, message_info)

            elif content.startswith("obstacles"):
                self.handleObstaclesMessage(content, message_info)

            #TASK_MODE specific messages
            if self.communication_manager.is_task_mode(robot_number = self.robot_number):
                if content.startswith("lastTaskQueue"):
                    #print(content)
                    self.handleTaskQueueMessage(content, message_info)
                    print("Stopping task queue check")
                    if self.check_task_queue_task.running:
                        self.check_task_queue_task.stop()
            #PLAN_MODE specific messages
            else:
                if content.startswith("booleanFlags"):
                    #print(content)
                    self.handleBooleanFlagsMessage(content, message_info)
                #elif content.startswith("planActionCompleted"):
                #    print(content)
                #    self.handlePlanActionCompletedMessage(content, message_info)

    def send_string(self, string, terminator="\x00"):
        #print("Sending data to robot "+str(self.robot_number)+": "+string)
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        #print(self.dest_addr[4])
        self.write_socket.sendto(data, self.dest_addr[4])
        self.last_sent_message_timestamp = time.time()

    #Overridden method
    def datagramReceived(self, data, addr):
        self.handleMessage(data)

    def set_to_alive(self):
        if not self.alive:
            #Set as alive 
            print("Robot {} is alive".format(self.robot_number))

            self.alive = True
            
            self.communication_manager.enable_frontend()

    def set_to_not_alive(self):
        print("Robot %d has not sent for %f seconds: setting to not alive" % (self.robot_number, time.time() - self.last_received_message_timestamp))
        self.alive = False
        self.lastReceivedTaskID = None
        self.lastCompletedTaskID = None
        #If the robot is not alive (e.g. it crashed) we reset the task list. If the connection dropped, the robot will keep executing the tasks anyway
        #and also the last task ID will be correctly received upon reconnection
        self.communication_manager.removeRobotTaskQueue(self.robot_number)
        #We also reset the current robot role to 'unknown' and clean its Plan
        self.communication_manager.resetRobotRole(self.robot_number)

        if self.communication_manager.frontend_controller is not None:
    #TODO: architectural weakness
            self.communication_manager.sendRobotNotRespondingMessageToFrontend(self.robot_number)
            print("Starting task queue check")
            if not self.check_task_queue_task.running:
                self.check_task_queue_task.start(self.communication_manager.last_task_id_timeout)


    def check_alive_states(self):
        #print("Starting alive check")
        if self.alive:
            if time.time() - self.last_received_message_timestamp > self.active_client_timeout:
                self.set_to_not_alive()
            

    def check_last_task_id(self):
        #print("Asking last task ID")
        if self.alive:
            self.send_string("lastTaskID?")

    def check_task_queue(self):
        if self.alive and self.communication_manager.is_task_mode(robot_number = self.robot_number):
            self.send_string("lastTaskQueue?")

    def update_assigned_tasks(self):
        if self.alive:
            #Check that:
            # 1) The robot role is known
            # 2) There is a Plan for that role


            if self.robot_number in self.communication_manager.robot_tasks:
                if self.communication_manager.getRobotRoleFromNumber(self.robot_number) != self.communication_manager.UNKNOWN_ROLE:
                    #Send task reset or delete message
                    if self.communication_manager.isRobotScheduledForTasksReset(self.robot_number):
                        self.communication_manager.unscheduleRobotTasksReset(self.robot_number)
                        self.send_string("resetTasks")

                    if len(self.communication_manager.robot_tasks[self.robot_number]["tasksToDelete"]) > 0:
                        for taskID in self.communication_manager.robot_tasks[self.robot_number]["tasksToDelete"]:
                            self.send_string("deleteTask,"+taskID)
                        self.communication_manager.resetTasksToDelete(self.robot_number)

                if not self.communication_manager.is_task_mode(robot_number = self.robot_number):

                    #print(self.communication_manager.robot_role_to_number)
                    assert self.communication_manager.getRobotRoleFromNumber(self.robot_number) in self.communication_manager.role_to_plan.keys(), \
                        "Role assigned to role "+self.communication_manager.getRobotRoleFromNumber(self.robot_number)+" (number: "+str(self.robot_number)+") is in Plan control mode but no Plan is present"
                    

                    robot_role = self.communication_manager.getRobotRoleFromNumber(self.robot_number)

                    old_action = self.communication_manager.get_current_plan_action(robot_role)
                    new_action = self.communication_manager.get_next_plan_action(robot_role)
                    

                    #Create message string
                    plan_string = "PlanAction|"

                    #If lastCompletedTaskID is None, return (wait for the robot to communicate last taskID)
                    if self.lastCompletedTaskID is None:
                        return
                    
                    #If this is the first task at all, lastReceivedTaskID will be -1
                    lastCompletedTaskID = self.lastCompletedTaskID +1               
                    
                    plan_string += new_action.get()+","+str(lastCompletedTaskID)+";"
                    plan_string += new_action.get_parameter_string()

                    if old_action != new_action:
                        print("\n------------------\nSending new action for robot %d, role %s:\n\t%s" % (self.robot_number, robot_role, plan_string))

                    self.send_string(plan_string)

                else:
                        
                    if self.lastReceivedTaskID is not None and self.robot_number in self.communication_manager.robot_tasks.keys():
                        task_list_string = "taskQueue|"

                        #if len(self.communication_manager.robot_tasks[self.robot_number]["tasks"]) == 0:
                        #    return

                        for i, task in enumerate(self.communication_manager.robot_tasks[self.robot_number]["tasks"]):
                            if(i > 0): 
                                task_list_string+= ";"
                            task_list_string += task["taskType"]

                            task_list_string+=","+str(task["taskID"])

                            if task["parameters"] is not None:
                                for parameter in task["parameters"]:
                                    task_list_string+=","+str(parameter)
                            
                        
                        #print(task_list_string)
                        self.send_string(task_list_string)
                    else:
                        return
            else:
                raise Exception("STILL TO BE DECIDED: CONTROL PARADIGM IN CASE ROBOT ROLE IS UNKNOWN")
