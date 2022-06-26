import socket

def setup_read_socket(udpProtocol, reactor, local_interface_ip, read_port, read_timeout, debug_message):
    #Setup read socket
    if debug_message is not None: 
        print(debug_message)
    
    read_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    read_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    read_addr = socket.getaddrinfo(local_interface_ip, read_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    read_socket.bind((local_interface_ip, read_port))
    read_socket.setblocking(False)
    read_socket.settimeout(read_timeout)
    
    reactor_read_socket = reactor.adoptDatagramPort(read_socket.fileno(), socket.AF_INET, udpProtocol)
    
    return read_socket, read_addr

def setup_write_socket(local_interface_ip, write_port, remote_dest_ip, dest_port = None, debug_message = None):
    #Setup write socket
    if debug_message is not None: 
        print(debug_message)
    
    write_socket = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    write_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    write_addr = socket.getaddrinfo(local_interface_ip, write_port, socket.AF_INET, socket.SOCK_DGRAM)[0]
    
    dest_addr = None
    if dest_port is not None:
        dest_addr = socket.getaddrinfo(remote_dest_ip, dest_port, socket.AF_INET, socket.SOCK_DGRAM)[0]

    try:
        write_socket.bind((local_interface_ip, write_port))
    except Exception as e:
        print("Address: %s, Port: %s" % (local_interface_ip, write_port))
        raise e
    write_socket.setblocking(False)
    
    return write_socket, write_addr, dest_addr