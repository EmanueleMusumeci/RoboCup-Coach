from abc import ABC, abstractmethod
from asyncore import write
from typing import Type, Callable
import time
import socket
import uuid

import twisted
from twisted.internet import task

from communication.socket_utils import setup_read_socket, setup_write_socket
from communication.communication_manager import CommunicationManager

class NetworkFrontendController(ABC, twisted.internet.protocol.DatagramProtocol):
    def __init__(self, 
        reactor, 
        
        local_frontend_interface_ip, read_port, 
        remote_dest_ip, write_port, client_port,
        
        read_socket_timeout,
        active_client_timeout,

        communication_manager : Type["CommunicationManager"] = None,
        start_check_client_alive_task : bool = True,

        registration_message = None
    ):
        self.reactor = reactor
        
        self.write_port = write_port
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(local_frontend_interface_ip, write_port, remote_dest_ip, dest_port=client_port, 
                                                                                debug_message=("[FRONTEND] Writing from %s, with local port %s\n\t-> IP %s, UDP write port: %s" % (local_frontend_interface_ip, write_port, remote_dest_ip, client_port)))
        self.last_sent_message_timestamp = None

        self.read_port = read_port
        self.client_port = client_port
        self.read_socket, self.read_addr = setup_read_socket(self, reactor, local_frontend_interface_ip, read_port, read_socket_timeout, 
                                                                debug_message=("[FRONTEND] Listening on %s, with local port %s\n\t-> IP %s, UDP read port: %s" % (local_frontend_interface_ip, read_port, remote_dest_ip, write_port)))
        self.last_received_message_timestamp = None

        self.last_client_id = None
        self.alive = False
        self.active_client_timeout = active_client_timeout

        self.registration_message = registration_message

        self.start_check_client_alive_task = start_check_client_alive_task
        self.check_alive_task = task.LoopingCall(self.check_client_alive)
        if start_check_client_alive_task:
            self.check_alive_task.start(self.active_client_timeout)

        if communication_manager is not None:
            assert isinstance(communication_manager, CommunicationManager)
        self.communication_manager = communication_manager

        self.uuid = uuid.uuid4()


    ################
    # Architecture #
    ################

    def set_manager(self, communication_manager : Type["CommunicationManager"]):
        assert self.communication_manager is None or isinstance(communication_manager, CommunicationManager)
        self.communication_manager = communication_manager
        

    #################
    # Client status #
    #################

    def register_new_client(self, address, port, client_id):
        self.dest_addr = socket.getaddrinfo(address, port, socket.AF_INET, socket.SOCK_DGRAM)[0]
        self.last_client_id = client_id
        self.alive = True

    def check_client_alive(self):
        #print("Starting alive check")
        if self.alive:
            if time.time() - self.last_received_message_timestamp > self.active_client_timeout:
                #print("Client %s has not sent for %d seconds: setting to not alive" % (self.last_client_id, time.time() - self.last_received_message_timestamp))
                self.set_to_not_alive()
            else:
                pass
                #print("Client {} is alive".format(self.last_client_id))
        else:
            self.send_keepalive_request()

    def set_to_not_alive(self):
        self.alive = False
    
    #Enable frontend
    @abstractmethod
    def enable(self):
        pass

    #Update frontend status (params may be any number and kind of parameters in the concrete implementation)
    @abstractmethod
    def update(self, params):
        pass  

    #####################
    # Connection status #
    #####################

    def close_sockets(self):
        self.write_socket.close()
        self.read_socket.close()



    ############
    # Messages #
    ############


    def datagramReceived(self, data, addr):
        self.handleClientMessage(data)

    @abstractmethod
    def generate_message_header(self, robotNumber):
        pass

    def generate_client_specific_message_header(self):
        return str("client_id,"+self.read_addr[4][0]+","+str(self.read_addr[4][1])+","+str(self.uuid))

    def send_string(self, string, terminator="\x00"):
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        #print(data)
        if self.dest_addr is not None:
            self.write_socket.sendto(data, self.dest_addr[4])
            self.last_sent_message_timestamp = time.time()

    def send_keepalive_request(self):
        self.send_string(self.generate_client_specific_message_header() + "|uthere?")

    def send_keepalive_response(self):
        self.send_string(self.generate_client_specific_message_header() + "|yeah")
    

    ### Message handlers ###

    def handleClientMessage(self, data):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        #print("Received message from client")
        #print(data)

        message_fields = data.split("|")

        header = message_fields[0]
        content = message_fields[1].rstrip('\x00')
        
        message_info = {"timestamp" : None, "client_id" : None, "content" : content}

        if len(header)>0:
            for field in header.split(";"):
                if field.startswith("timestamp"):
                    message_info["timestamp"] = int(field.split(",")[1])

                if field.startswith("client_id"):
                    message_info["client_id"] = field.split(",")[1:]
                    
        if message_info["client_id"] is None:
            print("Client not recognized (NO CLIENT ID RECEIVED)")
            raise


        #Update the client with the latest info if the last_client_id changed (new client)
        if self.alive and self.last_client_id != message_info["client_id"][2]:
            self.register_new_client(message_info["client_id"][0], message_info["client_id"][1], message_info["client_id"][2])

        #Set last_received_message_timestamp
        self.last_received_message_timestamp = time.time()

        #Set as alive 
        self.alive = True

        #if self.registration_message is not None and content == self.registration_message:
        #    self.handleRegistrationMessage(header, content)
        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from client %s" % message_info["client_id"])
            self.send_keepalive_response()
        elif content == "yeah":
            return

        self.handleClientMessageContent(content)
    
    def handleRegistrationMessage(self, header, content):
        pass

    @abstractmethod
    def handleClientMessageContent(self, content):
        pass


