import os
import inspect

from lib.dfa.dfa import DFA
from lib.dfa.dfa_handler import DFAHandler, remove_initial_dummy_state
from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.utils import linear_distance, angular_distance
from pathlib import Path

from lib.experiment import setup_LTL_DFA_for_experiment, ExperimentType

def get_experiment_type():
    return ExperimentType.DFA

def get_problem_name():
    return "patrolling_with_static_waypoints_with_mutual_exclusion_stateful"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)

    return {
        "striker" : {
            "goal" : "G(\
                        (striker_has_reached_waypoint_1 && is_next_waypoint_2 && action_go_to_waypoint_2) ||\
                            (striker_has_reached_waypoint_2 && is_next_waypoint_1 && action_go_to_waypoint_1) ||\
                                (!striker_has_reached_waypoint_1 && !striker_has_reached_waypoint_2 && is_next_waypoint_1 && action_go_to_waypoint_1) ||\
                                (!striker_has_reached_waypoint_1 && !striker_has_reached_waypoint_2 && is_next_waypoint_2 && action_go_to_waypoint_2) \
                        )",
        }
    }

def setup():
    return setup_LTL_DFA_for_experiment(get_problem_name(), role_to_generation_data())
    
def initialize_registries():
    ''' 
    ___________________
    |                  |
    |  DFA Experiment  |
    |__________________|

    '''

    #EXPERIMENT: Try to realize a patrolling behavior through 4 waypoints. Actions use "static parameters" (values of parameters are not computed functionally through FunctionalValues).
    #This experiment is supposed to show why it is necessary to give actions the possibility to use "dynamic parameters" that are computed through functions, which leads to
    # 1) More manageable DFAs
    # 2) Mutual exclusion of actions is required!!!
    # 3) Also shows the necessity for a DFA post-processing step in which we delete all edges having no actions or only negated actions


    #Setup ValueRegistry
    ValueRegistry()["waypoint_distance_threshold"] = 600
    ValueRegistry()["waypoint1"] = (-1500, -1500)
    ValueRegistry()["waypoint2"] = (-1500, 1500)


    #Setup LiteralRegistry
    def striker_has_reached_waypoint_1(striker_position, waypoint_distance_threshold, is_next_waypoint_1):
        reached_1 = linear_distance(striker_position, ValueRegistry()["waypoint1"]) < waypoint_distance_threshold
        if reached_1:
            LiteralRegistry()["is_next_waypoint_2"] = True
            LiteralRegistry()["is_next_waypoint_1"] = False
        return reached_1

    LiteralRegistry().add_function(striker_has_reached_waypoint_1)

    def striker_has_reached_waypoint_2(striker_position, waypoint_distance_threshold, is_next_waypoint_2):
        reached_2 = linear_distance(striker_position, ValueRegistry()["waypoint2"]) < waypoint_distance_threshold
        if reached_2:
            LiteralRegistry()["is_next_waypoint_1"] = True
            LiteralRegistry()["is_next_waypoint_2"] = False
        return reached_2

    LiteralRegistry().add_function(striker_has_reached_waypoint_2)

    LiteralRegistry()["is_next_waypoint_1"] = True
    LiteralRegistry()["is_next_waypoint_2"] = False

    #Setup ActionRegistry
    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_go_to_waypoint_1"] = ("ReachPosition", [("positionX", ValueRegistry()["waypoint1"][0]), ("positionY", ValueRegistry()["waypoint1"][1])])
    ActionRegistry()["action_go_to_waypoint_2"] = ("ReachPosition", [("positionX", ValueRegistry()["waypoint2"][0]), ("positionY", ValueRegistry()["waypoint2"][1])])
    