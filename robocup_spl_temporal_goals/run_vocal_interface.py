import argparse
import socket
import signal

from twisted.internet import reactor

from frontend.vocal_interface import VocalInterface

#from frontend.shell import InputShell

def setup_vocal_interface(
    LOCAL_SOCKET_IP = "127.0.0.1",
    REMOTE_SOCKET_IP = None,

    ALIVE_CLIENT_TIMEOUT = 1,
    READ_SOCKET_TIMEOUT = 0.05
    ):

    LOCAL_READ_PORT = 64600
    LOCAL_WRITE_PORT = 64700
    REMOTE_READ_PORT = 64300    


    try:
        socket.inet_aton(REMOTE_SOCKET_IP)
        socket.inet_aton(LOCAL_SOCKET_IP)
        # legal
    except socket.error as e:
        # Not legal
        print("Illegal IP specified for frontend interface")
        raise e

    ''' 
     ________________________________________
    |                                        |
    |  CORRECTLY HANDLE TERMINATION SIGNALS  |
    |________________________________________|

    '''

    vocal_interface = VocalInterface(
        reactor, 
        
        LOCAL_SOCKET_IP, LOCAL_READ_PORT, 
        REMOTE_SOCKET_IP, LOCAL_WRITE_PORT, REMOTE_READ_PORT,
        
        READ_SOCKET_TIMEOUT,
        ALIVE_CLIENT_TIMEOUT
    )

    #Handle interrupt signals
    signal.signal(signal.SIGINT, vocal_interface.exit_gracefully)
    signal.signal(signal.SIGABRT, vocal_interface.exit_gracefully)
    signal.signal(signal.SIGTERM, vocal_interface.exit_gracefully)
    signal.signal(signal.SIGQUIT, vocal_interface.exit_gracefully)
    signal.signal(signal.SIGHUP, vocal_interface.exit_gracefully)
    #-----------------------------------------------------------------

    return vocal_interface


if __name__ == "__main__":
    
    parser = argparse.ArgumentParser(description='Run the vocal interface to specify plan constraints vocally.')
    parser.add_argument('--connect_to_IP', '-c', type=str, help='Use to specify that we want to use the constraints frontend and the remote IP of the frontend interface (localhost if only the "constraints_fronted" argument is specified).')    
    
    args = parser.parse_args()
    
    if args.connect_to_IP is None:
        REMOTE_IP = "127.0.0.1"
    else:
        assert len(args.connect_to_IP) == 1, "Specify at most one IP address"    
        try:
            socket.inet_aton(args.connect_to_IP)
            # legal
        except socket.error as e:
            # Not legal
            print("Illegal IP specified for frontend interface")
            raise e

        REMOTE_IP = args.connect_to_IP

    vocal_interface = setup_vocal_interface(
        REMOTE_SOCKET_IP=REMOTE_IP
    )

    vocal_interface.start()
    