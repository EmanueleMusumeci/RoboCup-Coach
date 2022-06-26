
from twisted.internet import task

from frontend.network_frontend_controller import NetworkFrontendController


class WebGUIController(NetworkFrontendController):
    def __init__(self, 
        reactor, 
        
        local_frontend_interface_ip, read_port, 
        remote_dest_ip, write_port, client_port,
        
        read_socket_timeout,
        active_client_timeout,

        communication_manager
    ):
        super().__init__(
            reactor, 
            
            local_frontend_interface_ip, read_port, 
            remote_dest_ip, write_port, client_port,
            
            read_socket_timeout,
            active_client_timeout,
            communication_manager
        )

    ### CUSTOM OUTBOUND MESSAGES ###

    def generate_message_header(self, robotNumber):
        return str("robotNumber,"+str(robotNumber))

    def update(self, robotNumber, message):
        self.send_string(self.generate_message_header(robotNumber) + "|" + message)

    def sendRobotNotRespondingMessage(self, robotNumber):
        self.send_string(self.generate_header(robotNumber) + "|robotNotResponding")


    ### CUSTOM INBOUND MESSAGE HANDLING ###

    def handleClientMessageContent(self, content):
        #print(content)

        #keep-alive message
        if content == "uthere?":
            #print("KEEPALIVE received from client %s" % message_info["client_id"])
            self.send_keepalive_response()
        else:   
            if content.startswith("resetTasks"):
                print(content)
                print("resetTasks")
                robotNumber = int(content.split(",")[1])
                self.communication_manager.scheduleRobotTasksReset(robotNumber)
            elif content.startswith("deleteTask"):
                print(content)
                robotNumber = int(content.split(",")[1])
                taskID = int(content.split(",")[2])
                self.communication_manager.scheduleTaskDeletion(robotNumber, taskID)

            elif content.startswith("taskType"):
                content_fields = content.split(":")[1].split(",")
                robotNumber = int(content_fields[0])
                selectionMode = content_fields[1]
                taskType = content_fields[2]
                taskID = int(content_fields[3])
                if selectionMode == "noSelection":
                    self.communication_manager.addTask(robotNumber, taskType, taskID)
                elif selectionMode == "singlePosition":
                    xPos = int(content_fields[4])
                    yPos = int(content_fields[5])
                    self.communication_manager.addTask(robotNumber, taskType, taskID, parameters = [xPos, yPos])


    ### CUSTOM ACTION ON CLIENT DISCOVERY ###

    def enable(self):
        pass
