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
    return "reach_ball_and_kick"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)

    return {
        "striker" : {
            "goal" : "G((is_striker_near_ball && action_kick_ball) || (!is_striker_near_ball && action_reach_ball))",
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

    #Setup ActionRegistry
    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_kick_ball"] = ("Kick", [("positionX", 2000), ("positionY", 0)])
    ActionRegistry()["action_reach_ball"] = "ReachBall"

    #Setup ValueRegistry
    ValueRegistry()["ball_distance_threshold"] = 500

    def striker_distance_from_ball(last_ball_position, striker_position):
        return linear_distance(last_ball_position, striker_position)
    ValueRegistry().add_function(striker_distance_from_ball)

    def is_striker_near_ball(striker_distance_from_ball, ball_distance_threshold):
        return striker_distance_from_ball < ball_distance_threshold
    
    #Setup LiteralRegistry
    LiteralRegistry().add_function(is_striker_near_ball)