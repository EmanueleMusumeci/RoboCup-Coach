from typing import Type

from twisted.internet import task

from frontend.network_frontend_controller import NetworkFrontendController
from lib.experiment import ExperimentType
from lib.constraints import match_string_with_constraint_template

#TODO: controller should send a uuid so that the vocal fronted may register its address


class ConstraintsFrontendController(NetworkFrontendController):
    def __init__(self,
        experiment_domain,

        reactor, 
        
        local_frontend_interface_ip, read_port, 
        remote_dest_ip, write_port, client_port,
        
        read_socket_timeout,
        alive_client_timeout,

        communication_manager : Type["CommunicationManager"] = None,
        start_check_client_alive_task : bool = True
    ):
        super().__init__(
            reactor, 
            
            local_frontend_interface_ip, read_port, 
            remote_dest_ip, write_port, client_port,
            
            read_socket_timeout,
            alive_client_timeout,
            communication_manager,
            start_check_client_alive_task,
            
            registration_message = "vocal_interface"
        )
        assert hasattr(experiment_domain, "get_experiment_type")
        assert experiment_domain.get_experiment_type() == ExperimentType.CONSTRAINABLE_POLICY

        assert hasattr(experiment_domain, "setup")
        assert hasattr(experiment_domain, "get_problem_name")
        assert hasattr(experiment_domain, "get_robot_formation")
        assert hasattr(experiment_domain, "role_to_generation_data")
        assert hasattr(experiment_domain, "initialize_registries")

        self.experiment_domain = experiment_domain

    def plan_with_new_constraints(self, additional_constraints : str):
        print(additional_constraints)

        plan_handlers = self.experiment_domain.setup(additional_constraints)
    
        for role, handler in plan_handlers.items():
            #We need to first updateRobotRole as, in normal conditions, we would already know the robot role as it is announced as soon as the robot connects
            self.communication_manager.update_plan(handler, robot_role=role)
    

    ### CUSTOM OUTBOUND MESSAGES ###

    def generate_message_header(self, robotRole):
        return str("robot_role,"+str(robotRole))

    def update(self, robotRole, message):
        self.send_string(self.generate_message_header(robotRole) + "|" + message)

    def send_domain_description_message(self):
        role_to_domain_description = {}

        for role, data in self.experiment_domain.role_to_generation_data().items():
            domain_predicates = {}

            if "constrainable_predicates" in data:
                for predicate, synonims in data["constrainable_predicates"].items():
                    domain_predicates[predicate] = synonims

            role_to_domain_description[role] = {}
            role_to_domain_description[role]["goal"] = data["goal"]
            role_to_domain_description[role]["predicates"] = domain_predicates

        string = ""
        for i, (role, domain_data) in enumerate(role_to_domain_description.items()):
            assert "goal" in domain_data.keys(), "Malformed domain description (should be a dict {'<ROLE>' : { 'goal' : <GOAL>, 'predicates' : { <PREDICATE_NAME> : [<SYNONIM1>, <SYNONIM2>] ...} }...}"
            assert "predicates" in domain_data.keys(), "Malformed domain description (should be a dict {'<ROLE>' : { 'goal' : <GOAL>, 'predicates' : { <PREDICATE_NAME> : [<SYNONIM1>, <SYNONIM2>] ...} }...}"
        
            string += "|role:" + role
            string += ";predicates:"
            for predicate_name, predicate_synonims in domain_data["predicates"].items():
                string+=predicate_name
                for j, syn in enumerate(predicate_synonims):    
                    string+=","                    
                    string+=syn
            string+=";goal:"+domain_data["goal"]
            if i < len(role_to_domain_description.keys()) - 1:
                string+="/"

        self.send_string(self.generate_client_specific_message_header() + string)


    ### CUSTOM INBOUND MESSAGE HANDLING ###

    def handleRegistrationMessage(self, header, content):
        if content.startswith("vocal_interface"):
            self.register_new_client()


    def handleClientMessageContent(self, content):
        if content.startswith("vocal_interface"):
            return

        print("Received message from client")
        print(content)

        if content.startswith("role"):    
            constraints = {}

            messages = content.split("/")
            for message in messages:
                message_fields = message.split(",")
                role = message_fields[0].split(":")[1]
                constraint_string = message_fields[1].split(":")[1]
                
                constraints[role] = {}

                constraints[role]["constraint_string"] = constraint_string
                
                domain_generation_data = self.experiment_domain.role_to_generation_data()
                #print(domain_generation_data)
                assert role in domain_generation_data
                #print(domain_generation_data[role])
                assert "constrainable_predicates" in domain_generation_data[role]
                constraints[role]["constrainable_predicates"] = domain_generation_data[role]["constrainable_predicates"]

            additional_contraints_per_role = {}
            
            for role, role_data in constraints.items():
                additional_contraints_per_role[role] = match_string_with_constraint_template(input_string=role_data["constraint_string"], constrainable_predicates=role_data["constrainable_predicates"])
            
            self.plan_with_new_constraints(additional_contraints_per_role)

    ### CUSTOM ACTION ON CLIENT DISCOVERY ###

    def enable(self):
        pass

    def register_new_client(self, address, port, client_id):
        super().register_new_client(address, port, client_id)
        self.send_domain_description_message()