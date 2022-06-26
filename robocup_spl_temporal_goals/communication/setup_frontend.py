from typing import Dict
import signal
import socket

from twisted.internet import reactor

from communication.communication_manager import BehaviorControlMode
from frontend.vocal_frontend_controller import ConstraintsFrontendController
from frontend.web_frontend_controller import WebGUIController
#from frontend.constraints_frontend_controller import ConstraintGUIController
#from frontend.shell import ConstraintShell

def setup_vocal_frontend(
    experiment_domain,

    LOCAL_FRONTEND_SOCKET_IP = "127.0.0.1",
    REMOTE_FRONTEND_SOCKET_IP = None,

    ALIVE_CLIENT_TIMEOUT = 1,
    READ_SOCKET_TIMEOUT = 0.05

    ):
    ''' 
     ______________________________
    |                              |
    |  FRONTEND NETWORK ADDRESSES  |
    |______________________________|

    '''

    #-----------------------------------------------------------------

    ''' 
     _____________________________________
    |                                    |
    |  SETUP FRONTEND NETWORK INTERFACE  |
    |____________________________________|

    '''
    FRONTEND_READ_PORT = 64300
    FRONTEND_WRITE_PORT = 64400
    FRONTEND_REMOTE_READ_PORT = 64600

    try:
        socket.inet_aton(REMOTE_FRONTEND_SOCKET_IP)
        socket.inet_aton(LOCAL_FRONTEND_SOCKET_IP)
        # legal
    except socket.error as e:
        # Not legal
        print("Illegal IP specified for frontend interface")
        raise e

    frontend_controller = ConstraintsFrontendController(
        experiment_domain,
        
        reactor, 
        LOCAL_FRONTEND_SOCKET_IP, FRONTEND_READ_PORT, 
        REMOTE_FRONTEND_SOCKET_IP, FRONTEND_WRITE_PORT, FRONTEND_REMOTE_READ_PORT, 
        
        READ_SOCKET_TIMEOUT, 
        ALIVE_CLIENT_TIMEOUT,
        start_check_client_alive_task = False
    )
    DEFAULT_BEHAVIOR_CONTROL_MODE = BehaviorControlMode.PLAN_MODE
    
    return frontend_controller, DEFAULT_BEHAVIOR_CONTROL_MODE


def setup_web_frontend(
    LOCAL_FRONTEND_SOCKET_IP = "127.0.0.1",
    REMOTE_FRONTEND_SOCKET_IP = None,

    ALIVE_CLIENT_TIMEOUT = 1,
    READ_SOCKET_TIMEOUT = 0.05

    ):
    ''' 
     ______________________________
    |                              |
    |  FRONTEND NETWORK ADDRESSES  |
    |______________________________|

    '''

    #-----------------------------------------------------------------

    ''' 
     _____________________________________
    |                                    |
    |  SETUP FRONTEND NETWORK INTERFACE  |
    |____________________________________|

    '''
    FRONTEND_READ_PORT = 64300
    FRONTEND_WRITE_PORT = 64400
    FRONTEND_REMOTE_READ_PORT = 64301
    
    frontend_controller = WebGUIController(
        reactor, 
        LOCAL_FRONTEND_SOCKET_IP, FRONTEND_READ_PORT, 
        REMOTE_FRONTEND_SOCKET_IP, FRONTEND_WRITE_PORT, FRONTEND_REMOTE_READ_PORT, 
        
        READ_SOCKET_TIMEOUT, 
        ALIVE_CLIENT_TIMEOUT
    )
    DEFAULT_BEHAVIOR_CONTROL_MODE = BehaviorControlMode.TASK_MODE
    
    
    return frontend_controller, DEFAULT_BEHAVIOR_CONTROL_MODE