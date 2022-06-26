import os
import inspect
from pathlib import Path

from lib.dfa.dfa import DFA
from lib.dfa.dfa_handler import DFAHandler, remove_initial_dummy_state
from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.utils import linear_distance

from lib.experiment import setup_LTL_DFA_for_experiment, ExperimentType

def get_experiment_type():
    return ExperimentType.DFA

def get_problem_name():
    return "reach_waypoint_then_reach_ball_and_kick_until_goal"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)

    return {
        "striker" : {
            "goal" : "(action_go_to_waypoint U striker_has_reached_waypoint) -> ((is_striker_near_ball & action_kick_ball) | (!(is_striker_near_ball) & action_reach_ball) U is_ball_in_goal)",
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

    ValueRegistry()["waypoint"] = (2000, 2000)

    #Setup ActionRegistry
    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_kick_ball"] = ("Kick", [("positionX", 0), ("positionY", 0)])
    ActionRegistry()["action_reach_ball"] = "ReachBall"
    ActionRegistry()["action_go_to_waypoint"] = ("ReachPosition", [("positionX", ValueRegistry()["waypoint"][0]), ("positionY", ValueRegistry()["waypoint"][1])])

    #Setup ValueRegistry
    ValueRegistry()["ball_distance_threshold"] = 500
    ValueRegistry()["waypoint_distance_threshold"] = 600


    def striker_distance_from_ball(last_ball_position, striker_position):
        return linear_distance(last_ball_position, striker_position)
    ValueRegistry().add_function(striker_distance_from_ball)

    def is_striker_near_ball(striker_distance_from_ball, ball_distance_threshold):
        return striker_distance_from_ball < ball_distance_threshold
    LiteralRegistry().add_function(is_striker_near_ball)

    ValueRegistry()["goal_corner_1"] = (-1000, -1000)
    ValueRegistry()["goal_corner_2"] = (1000, 1000)

    def is_inside_bounding_box(point, bbox):
        return point[0] >= bbox[0][0] and point[0] <= bbox[1][0] and point[1] >= bbox[0][1] and point[0] <= bbox[1][1]

    def is_ball_in_goal(last_ball_position, goal_corner_1, goal_corner_2):
        return is_inside_bounding_box(last_ball_position, (goal_corner_1, goal_corner_2))
    LiteralRegistry().add_function(is_ball_in_goal)
    

    def striker_has_reached_waypoint(striker_position, waypoint_distance_threshold, waypoint):
        return linear_distance(striker_position, waypoint) < waypoint_distance_threshold
    LiteralRegistry().add_function(striker_has_reached_waypoint)

    #Simple approacher DFA
    ltl_formula_str = "(action_go_to_waypoint U striker_has_reached_waypoint) -> ((is_striker_near_ball & action_kick_ball) | (!(is_striker_near_ball) & action_reach_ball) U is_ball_in_goal)"
