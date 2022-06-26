from typing import Dict
import signal
import socket

from twisted.internet import reactor

from communication.communication_manager import BehaviorControlMode, CommunicationManager
#from frontend.web_frontend_controller import WebGUIController
#from frontend.constraints_frontend_controller import ConstraintGUIController
#from frontend.shell import ConstraintShell

def setup(
    robot_formation : Dict, 

    BASE_FRAMEWORK_UDP_PORT,

    USE_LOCALHOST : bool = False,

    frontend_controller = None,
    DEFAULT_BEHAVIOR_CONTROL_MODE = BehaviorControlMode.PLAN_MODE,

    ALIVE_CLIENT_TIMEOUT = 1,
    READ_SOCKET_TIMEOUT = 0.05
    ):
    #-----------------------------------------------------------------

    ''' 
     ____________________________
    |                            |
    |         TIMEOUTS           |
    |____________________________|

    '''

    #TODO maybe i'll have to use ms instead for these two as well
    #TIME between two subsequent "lastTaskID?" requests, that periodically check the last executed task ID
    LAST_TASK_ID_TIMEOUT = 1 
    #TIME between two subsequent "taskQueue?" requests, that periodically ask the task
    UPDATE_TASKS_TIMEOUT = 0.2
    #-----------------------------------------------------------------

    #TODO: replace this with an automatic robot discovery broadcast
    ''' 
     ____________________________
    |                            |
    |  ROBOTS NETWORK ADDRESSES  |
    |____________________________|

    '''
    
    robot_number_to_robot_ip_map = get_robot_number_to_IP_map(robot_formation)
    if(USE_LOCALHOST):
        LOCAL_INTERFACE_IP = "127.0.0.1"
        for robot_number, robot_data in robot_number_to_robot_ip_map.items():
            robot_data["robotIP"] = LOCAL_INTERFACE_IP
    else:
        LOCAL_INTERFACE_IP = "10.0.255.226"
        for robot_number, robot_data in robot_number_to_robot_ip_map.items():
            assert robot_data["robotIP"].startswith((".").join(LOCAL_INTERFACE_IP.split(".")[:2]))

    #Notice: 
    # - the DEST_PORTs here have to be the READ_PORTs on the robots
    # - the READ_PORTs here have to be the DEST_PORTs on the robots
    # - the WRITE_PORTs here have to be different from those of the robots
    UDP_BASE_READ_PORT = BASE_FRAMEWORK_UDP_PORT + 100 #Listening port for incoming messages from robots
    UDP_BASE_WRITE_PORT = BASE_FRAMEWORK_UDP_PORT + 200  #Socket port of outgoing messages for each robot (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
    UDP_BASE_DEST_PORT = BASE_FRAMEWORK_UDP_PORT + 000 #Destination port of outgoing messages (each message will be sent to 127.0.0.1:UDP_BASE_WRITE_PORT+<robot_number>)
    #-----------------------------------------------------------------




    ''' 
     ______________________________
    |                             |
    |  SETUP BEHAVIOR CONTROLLER  |
    |_____________________________|

    '''

    behavior_controller = CommunicationManager(
        LOCAL_INTERFACE_IP,                                             #Local interface ip address
        
        #Robot communication info
        robor_number_to_robot_IP_map = robot_number_to_robot_ip_map,    #Robot number to robot data (including IP) map (will be replaced by an automatic discovery mechanism)
        robot_udp_base_read_port = UDP_BASE_READ_PORT,                  
        robot_udp_base_write_port = UDP_BASE_WRITE_PORT,
        robot_udp_base_dest_port = UDP_BASE_DEST_PORT,
        
        robot_read_socket_timeout = READ_SOCKET_TIMEOUT,
        robot_alive_timeout = ALIVE_CLIENT_TIMEOUT,
        last_task_id_timeout = LAST_TASK_ID_TIMEOUT,
        update_tasks_timeout = UPDATE_TASKS_TIMEOUT,                   

        frontend_controller = frontend_controller,
        frontend_alive_timeout = ALIVE_CLIENT_TIMEOUT,
        default_behavior_control_mode = DEFAULT_BEHAVIOR_CONTROL_MODE
    )




    ''' 
     ________________________________________
    |                                        |
    |  CORRECTLY HANDLE TERMINATION SIGNALS  |
    |________________________________________|

    '''

    #Handle interrupt signals
    signal.signal(signal.SIGINT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGABRT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGTERM, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGQUIT, behavior_controller.exit_gracefully)
    signal.signal(signal.SIGHUP, behavior_controller.exit_gracefully)
    #-----------------------------------------------------------------

    return behavior_controller

def get_robot_name_to_robot_IP_map():
    return {
        "Cesare" : "10.0.19.16",
        "Caligola" : "10.0.19.17",
        "Nerone" : "10.0.19.18",
        "Claudio" : "10.0.19.19",
        "Augusto" : "10.0.19.20",
        "Tiberio" : "10.0.19.21",
    }

def get_robot_number_to_IP_map(number_to_name : Dict):
    robot_name_to_ip_map = get_robot_name_to_robot_IP_map()
    
    robot_number_to_robot_ip_map = {}
    for number, name in number_to_name.items():
        robot_number_to_robot_ip_map[number] = {"robotName" : name, "robotIP" : robot_name_to_ip_map[name]}

    return robot_number_to_robot_ip_map