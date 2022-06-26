from abc import abstractmethod
import time
import socket
import uuid

import twisted
from twisted.internet import task, threads

import speech_recognition as sr

from communication.socket_utils import setup_read_socket, setup_write_socket

class VocalInterface(twisted.internet.protocol.DatagramProtocol):
    def __init__(self, 
        reactor, 
        
        local_frontend_interface_ip, read_port, 
        remote_dest_ip, write_port, client_port,
        
        read_socket_timeout,
        active_client_timeout,

        role_listening_timeout = 2,
        constraint_listening_timeout = 4,

        TEST_MODE = False
    ):
        self.reactor = reactor
        
        self.write_socket, self.write_addr, self.dest_addr = setup_write_socket(local_frontend_interface_ip, write_port, remote_dest_ip, dest_port=client_port, 
                                                                                debug_message=("[VOCAL INTERFACE] Writing from %s, with local port %s\n\t-> IP %s, UDP write port: %s" % (local_frontend_interface_ip, write_port, remote_dest_ip, client_port)))
        self.last_sent_message_timestamp = None

        self.read_socket, self.read_addr = setup_read_socket(self, reactor, local_frontend_interface_ip, read_port, read_socket_timeout, 
                                                                debug_message=("[VOCAL INTERFACE] Listening on %s, with local port %s\n\t-> IP %s, UDP read port: %s" % (local_frontend_interface_ip, read_port, remote_dest_ip, write_port)))
        self.last_received_message_timestamp = None

        self.last_client_id = None
        self.alive = False
        self.active_client_timeout = active_client_timeout

        self.role_listening_timeout = role_listening_timeout
        self.constraint_listening_timeout = constraint_listening_timeout



        self.uuid = uuid.uuid4()


        #self.check_alive_task = task.LoopingCall(self.check_client_alive)
        self.send_registration_message_task = task.LoopingCall(self.send_registration_message)
        self.send_registration_message_task.start(self.active_client_timeout)
        
        #Handle needed to stop the deferred thread when exiting gracefully
        self.recognizer_deferred = None
        self.calibrated = False

        self.domain_description = {}

        self.ALL_ROLES_KEYWORDS = ["all", "guys", "team", "robots", "spqr", "everyone"]

        self.TEST_MODE = TEST_MODE

    ####################
    # User interaction #
    ####################

    def print_available_constraint_templates(self):
        print("[VOCAL INTERFACE] Domain description")
        for role, role_data in self.domain_description.items():
            print("Role: "+str(role))
            print("\tGOAL: "+role_data["goal"])
            print("\tPREDICATES:")
            #print(role_data)
            for predicate, synonims in role_data["predicates"].items():
                print("\t\t" + predicate + ": "+ str(synonims))
            
        

    def send_consume_recognized_command(self, command):
        if command is not None:
            roles = []
            role = command[0]
            recognized_command = command[1]
            if role in self.ALL_ROLES_KEYWORDS:
                roles = [role for role in self.domain_description.keys()]
            else:
                assert role in self.domain_description.keys()
                roles = [role]
            
            assert roles, "No roles available in domain"
            for role in roles:
                self.send_string(self.generate_client_specific_message_header() + "|role:"+role+",command:"+recognized_command)
        else:
            time.sleep(2)

        if self.TEST_MODE:
            time.sleep(5)

        self.recognizer_deferred = threads.deferToThread(self.listen_for_commands)
        self.recognizer_deferred.addCallback(self.send_consume_recognized_command)


    def listen_for_commands(self):

        if self.TEST_MODE:
            print("TEST MODE")
            return "striker", "prevent high battery consumption"

        printed_waiting_message = False
        recognizer_instance = sr.Recognizer()
        recognizer_instance.pause_threshold = 0.8
        if self.last_client_id is not None:
            printed_waiting_message = False
            audio = ""
            
            while True:
                with sr.Microphone() as source:
                    #Listen
                    if not self.calibrated:
                        print("[VOCAL INTERFACE] Calibrating for ambient noise...")
                        recognizer_instance.adjust_for_ambient_noise(source, duration = 3)
                        self.calibrated = True
                        print("[VOCAL INTERFACE] Calibrated")

                    print("\n\n\n")
                    self.print_available_constraint_templates()
                    
                    #Recognize role
                    print("[VOCAL INTERFACE] Listening for role to be constrained (use one of the available roles %s or %s to influence all available roles)." % (str(*self.domain_description.keys()), self.ALL_ROLES_KEYWORDS))
                    role_audio = recognizer_instance.listen(source, timeout = self.role_listening_timeout)

                    recognized_role = ""
                    try:
                        recognized_role = recognizer_instance.recognize_google(role_audio, language="it-IT")
                        print("[VOCAL INTERFACE] Recognized role: "+recognized_role)
                        if recognized_role not in self.domain_description.keys() and recognized_role not in self.ALL_ROLES_KEYWORDS:
                            print("Role %s not found in domain description. Try again." % (recognized_role))
                            continue
                    except:
                        print("Role %s could not be recognized. Try again." % (recognized_role))
                        continue

                    #Recognize command
                    print("\n\n[VOCAL INTERFACE] Listening for constraint (chosen role %s)" % (recognized_role))
                    command_audio = recognizer_instance.listen(source, timeout = self.constraint_listening_timeout)
                    
                    #Recognize
                    recognized_command = ""
                    try:
                        recognized_command = recognizer_instance.recognize_google(command_audio, language="it-IT")
                        print("[VOCAL INTERFACE] Recognized command: "+recognized_command)
                        return recognized_role, recognized_command
                    except:
                        print("Message could not be recognized. Try again.")
                        continue
                    
                    #print_available_constraint_templates()
        else:
            if not printed_waiting_message:
                print("WAITING FOR CLIENT CONNECTION")
                printed_waiting_message = True

    #################
    # Client status #
    #################

    def register_new_controller(self, address, port, client_id):
        self.dest_addr = socket.getaddrinfo(address, port, socket.AF_INET, socket.SOCK_DGRAM)[0]
        self.last_client_id = client_id
        self.alive = True
        self.send_registration_message_task.stop()

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
        print(string)
        self.send_data(str.encode(string+terminator))
    
    def send_data(self, data):
        if self.dest_addr is not None:
            self.write_socket.sendto(data, self.dest_addr[4])
            self.last_sent_message_timestamp = time.time()

    def send_keepalive_request(self):
        self.send_string(self.generate_client_specific_message_header() + "|uthere?")

    def send_keepalive_response(self):
        self.send_string(self.generate_client_specific_message_header() + "|yeah")
        #print("KEEPALIVE response sent: %s" % b"yeah\x00")
    
    def send_registration_message(self):
        print("[VOCAL INTERFACE] Sending registration message. Waiting for domain description...")
        self.send_string(self.generate_client_specific_message_header()+"|vocal_interface")

    ### Message handlers ###

    def handleClientMessage(self, data):
        #Decode data into a string (might be improved later)
        data = data.decode('utf-8')

        print("Received message from client")
        print(data)

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
                    
                    #Update the client with the latest info if the last_client_id changed (new client)
                    if self.last_client_id != message_info["client_id"][2]:
                        self.register_new_controller(message_info["client_id"][0], message_info["client_id"][1], message_info["client_id"][2])

        #Set last_received_message_timestamp
        self.last_received_message_timestamp = time.time()

        #Set as alive 
        self.alive = True

        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from client %s" % message_info["client_id"])
            self.send_keepalive_response()

        self.handleClientMessageContent(content)
    
    def handleClientMessageContent(self, content):
        self.domain_description = {}
        print(content)
        if content.startswith("role"):
            role_descriptions = content.split("/")
            for desc in role_descriptions:
                
                fields = desc.split(";")
                
                role_field = fields[0]
                predicates_field = fields[1]
                goal_field = fields[2]

                role = role_field.split(":")[1]
                self.domain_description[role] = {}

                goal = goal_field.split(":")[1]
                self.domain_description[role]["goal"] = goal

                self.domain_description[role]["predicates"] = {}
                predicates = predicates_field.split(",")
                predicate_name = predicates[0].split(":")[1]
                self.domain_description[role]["predicates"][predicate_name] = []
                for synonim in predicates[1:]:
                    self.domain_description[role]["predicates"][predicate_name].append(synonim)

                # get our Deferred which will be called when a command is recognized
                self.recognizer_deferred = threads.deferToThread(self.listen_for_commands)
                # add our callback to print it out
                self.recognizer_deferred.addCallback(self.send_consume_recognized_command)
        

    def exit_gracefully(self, signal=None, frame=None):
        print("Exiting vocal interface gracefully...")
        if self.send_registration_message_task.running:
            self.send_registration_message_task.stop()
        if self.recognizer_deferred is not None:
            self.recognizer_deferred.cancel()
        if self.reactor.running:
            self.reactor.stop()
        self.close_sockets()

    def start(self):
        try:
            self.reactor.run()
        except:
            self.exit_gracefully()


if __name__ == "__main__":

    recognizer_instance = sr.Recognizer()
    while True:
        audio = ""
        
        #Listen
        with sr.Microphone() as source:
            if not calibration:
                recognizer_instance.adjust_for_ambient_noise(source)
                calibration = True
            audio = recognizer_instance.listen(source)
        
        #Recognize
        query_string = ""
        try:
            query_string = recognizer_instance.recognize_google(audio, language="it-IT")
        except:
            print("Message could not be recognized")
        
        #print_available_constraint_templates()