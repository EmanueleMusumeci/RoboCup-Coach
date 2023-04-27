import argparse
import sys, os
import socket

from twisted.internet import reactor

from lib.utils import find_similar_strings
from communication.setup_env import setup
import experiments
from lib.dfa.dfa_handler import DFAHandler
from lib.plan.policy_handler import PolicyHandler

from lib.constraints import ask_additional_constraints, match_string_with_constraint_template
from lib.experiment import ExperimentType
from robocup_spl_temporal_goals.communication.communication_manager import BehaviorControlMode
from robocup_spl_temporal_goals.communication.setup_frontend import setup_vocal_frontend

#from frontend.shell import InputShell


if __name__ == "__main__":
    #https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
    dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "experiments")
    sys.path.append(dir)

    from experiments import experiments
    loaded_experiments = {str(experiment_module.__name__): experiment_module for experiment_module in experiments}
    

    parser = argparse.ArgumentParser(description='Run an experiment (which has to be contained in a subdirectory inside the "experiments" directory).')
    parser.add_argument('experiment', type=str, help='Use to specify the experiment name in the format "<experiment_subfolder>.<experiment_name>". The name has to refer to a "<experiment_name>.py" file of the same name inside the subdirectory "<experiment_subfolder>" of the "experiments" folder.')
    parser.add_argument('--localhost', '-l', action="store_true", help='Use to tell if the robot is simulated.')
    parser.add_argument('--simulator', '-s', action="store_true", help='Use to tell if the robot is simulated.')
    #parser.add_argument('--frontend', '-f', action="store_true", help='Use to tell if the graphical frontend is to be used.')
    #parser.add_argument('--GUI', '-g', action="store_true", help='Use to tell if the graphical frontend is to be used.')
    #parser.add_argument('--shell', '-s', action="store_true", help='Use to launch a separate shell for interactive behavior conditioning through PLTLf constraints.')
    
    parser.add_argument('--opponent_team', '-o', action="store_true", help='This server instance controls the opponent team of the GameFast1vs1.ros scenario.')
    parser.add_argument('--ask_constraints', '-c', action="store_true", help='Use to require constraints in advance for each role for interactive behavior conditioning through PLTLf constraints.')
    parser.add_argument('--specify_constraints_by_role', '-C', nargs="+", help='Use to specify constraints in advance for each role for behavior conditioning through PLTLf constraints.')
    parser.add_argument('--constraints_frontend', '-f',
                        action='store',
                        nargs='*',
                        type=str,
                        help='Use to specify that we want to use the constraints frontend and the remote IP of the frontend interface (localhost if only the "constraints_fronted" argument is specified).'
                       )    
    #parser.add_argument('--tasks_frontend', '-f',
    #                    action='store',
    #                    nargs='*',
    #                    type=str,
    #                    help='Use to specify that we want to use the tasks frontend and the remote IP of the frontend interface (localhost if only the "constraints_fronted" argument is specified).'
    #                   )

    args = parser.parse_args()
    


    ''' 
    ____________________
    |                   |
    |  LOAD EXPERIMENT  |
    |___________________|

    '''

    experiment_name = "experiments."+args.experiment
    
    if experiment_name not in loaded_experiments.keys():
        similar_strings = find_similar_strings(experiment_name, loaded_experiments.keys())
        if similar_strings:
            print("Unknown experiment '%s'. Maybe you meant '%s'?" % (experiment_name, similar_strings[0]))
        else:
            print("Unknown experiment '%s'." % (experiment_name))
        exit()
    
    chosen_domain = loaded_experiments[experiment_name]

    #Check that the experiment module has all necessary functions
    assert hasattr(chosen_domain, "setup")
    assert hasattr(chosen_domain, "get_problem_name")
    assert hasattr(chosen_domain, "get_robot_formation")
    assert hasattr(chosen_domain, "role_to_generation_data")
    assert hasattr(chosen_domain, "initialize_registries")

    #Check that the module has a get_robot_formation method
    robot_formation = chosen_domain.get_robot_formation()

    #Initialize registries with correct variables
    chosen_domain.initialize_registries()


        #for role_constraint_tuple in constraints_by_roles_tuples:
        #    match_string_with_constraint_template()

    
    CONSTRAINTS_FRONTEND_IP = None

    #Ask for additional constraints to the goal of each robot
    if chosen_domain.get_experiment_type() == ExperimentType.CONSTRAINABLE_POLICY:

        policy_generation_data = chosen_domain.role_to_generation_data()

        additional_constraints = {}
        if args.specify_constraints_by_role:
            assert len(args.specify_constraints_by_role) % 2 == 0, "When specifying constraints by role, the constraints should be specified in tuples \"role, '<constraint>'\""
            constraints_by_roles_tuples = zip(args.specify_constraints_by_role[::2], args.specify_constraints_by_role[1::2])
            for role_constraint_tuple in constraints_by_roles_tuples:
                assert role_constraint_tuple[0] in policy_generation_data.keys(), "No role "+role_constraint_tuple[0]+" featured in this experiment"
                if role_constraint_tuple[0] not in additional_constraints:
                    additional_constraints[role_constraint_tuple[0]] = [match_string_with_constraint_template(role_constraint_tuple[1], policy_generation_data[role_constraint_tuple[0]]["constrainable_predicates"])]
                else:
                    additional_constraints[role_constraint_tuple[0]].append(match_string_with_constraint_template(role_constraint_tuple[1], policy_generation_data[role_constraint_tuple[0]]["constrainable_predicates"]))
                        
        if args.ask_constraints:
            asked_constraints = ask_additional_constraints(role_to_generation_data = policy_generation_data).items()
            print(asked_constraints)
            for role, constraints in asked_constraints:
                if role not in additional_constraints:
                    additional_constraints[role] = []    
                additional_constraints[role].extend(constraints)
            
        plan_handlers = chosen_domain.setup(additional_constraints)

    else:
        plan_handlers = chosen_domain.setup()
    
    #Check that the handlers are of the correct type(s)
    assert isinstance(plan_handlers, dict)
    for role, handler in plan_handlers.items():
        assert isinstance(role, str)
        assert isinstance(handler, PolicyHandler) or isinstance(handler, DFAHandler)

    localhost = args.localhost or args.simulator

    
    ''' 
    __________________________
    |                         |
    |   SETUP COMMUNICATION   |
    |_________________________|

    '''


    ##################
    # SETUP FRONTEND #
    ##################

    frontend_controller = None
    DEFAULT_BEHAVIOR_CONTROL_MODE = BehaviorControlMode.PLAN_MODE
    if args.constraints_frontend is not None:
        if not args.constraints_frontend:
            FRONTEND_REMOTE_IP = "127.0.0.1"
        else:
            assert len(args.constraints_frontend) == 1, "Specify at most one IP address"    
            try:
                socket.inet_aton(args.constraints_frontend[0])
                # legal
            except socket.error as e:
                # Not legal
                print("Illegal IP specified for frontend interface")
                raise e

            FRONTEND_REMOTE_IP = args.constraints_frontend[0]

        frontend_controller, DEFAULT_BEHAVIOR_CONTROL_MODE = setup_vocal_frontend(
            chosen_domain,
            
            REMOTE_FRONTEND_SOCKET_IP=FRONTEND_REMOTE_IP
        )
    #elif args.tasks_frontend is not None:
    #    if not args.tasks_frontend:
    #        FRONTEND_REMOTE_IP = "127.0.0.1"
    #    else:
    #        assert len(args.tasks_frontend) == 1, "Specify at most one IP address"    
    #        try:
    #            socket.inet_aton(args.tasks_frontend[0])
    #            # legal
    #        except socket.error as e:
    #            # Not legal
    #            print("Illegal IP specified for frontend interface")
    #            raise e
    #
    #        FRONTEND_REMOTE_IP = args.tasks_frontend[0]
    #
    #    frontend_controller, DEFAULT_BEHAVIOR_CONTROL_MODE = setup_vocal_frontend(
    #        REMOTE_FRONTEND_SOCKET_IP=FRONTEND_REMOTE_IP
    #    )
    


    ###############################
    # SETUP COMMUNICATION MANAGER #
    ###############################

    base_framework_port = 64000
    if args.opponent_team:
        base_framework_port = 65000

    #Setup behavior controller and pass policy handler
    behavior_controller = setup(
        robot_formation, 
        BASE_FRAMEWORK_UDP_PORT = base_framework_port, USE_LOCALHOST = localhost,

        frontend_controller = frontend_controller,
        DEFAULT_BEHAVIOR_CONTROL_MODE = DEFAULT_BEHAVIOR_CONTROL_MODE
    )



    #if args.shell or args.constraints:
    #    shell = InputShell()
    #else:
    for role, handler in plan_handlers.items():
        #We need to first updateRobotRole as, in normal conditions, we would already know the robot role as it is announced as soon as the robot connects
        behavior_controller.update_plan(handler, robot_role=role)

    behavior_controller.start()

            