import math 

import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler
from lib.registries.fluents import FluentRegistry

from lib.experiment import setup_FOND_policy_for_experiment, ExperimentType

def get_experiment_type():
    return ExperimentType.POLICY

def get_problem_name():
    return "fond_planning_striker_single_goal"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))

    return {
        "striker" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain_fond.pddl"),
            "pddl_problem_path" : os.path.join(currentdir, "PDDL", "striker_single_goal_problem_fond.pddl"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),

            "restart_when_completed" : True,
        }
    }

def setup():
    return setup_FOND_policy_for_experiment(get_problem_name(), role_to_generation_data())

def initialize_registries():

    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance_in_degrees

    ActionRegistry(robot_idle_skill="Idle")


    ''' 
    ___________________
    |                  |
    |  STRIKER VALUES  |
    |__________________|

    '''

    #Ground :objects in ValueRegistry
    ValueRegistry()["minimum_opponent_distance"] = 1000


    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_ball_position_alias(alias_name="ball-current-position")


    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("move-robot", "ReachPosition", [2])
    ActionRegistry().register_action_template("reach-ball", "ReachBall", [])
    ActionRegistry().register_action_template("defend-ball", "DefendBall", [])
    ActionRegistry().register_action_template("carry-ball" ,"CarryBall", [3])
    ActionRegistry().register_action_template("kick-to-goal", "CarryAndKickToGoal", [])
    ActionRegistry().register_action_template("kick-ball", "KickBall", [3])
    ActionRegistry().register_action_template("pass-ball" ,"Kick", ["jolly_position"])

    #Ground :objects in ValueRegistry
    #ValueRegistry()["kicking-position"] = (3000, 0)
    #ValueRegistry()["goal-target"] = (5100, 0)

    #TODO: Temporary, replace with real landmarks streamed from framework
    ValueRegistry()["striker_landmarks"] = []
    ValueRegistry()["kicking_distance"] = 1500

    def compute_goal_target(striker_landmarks):
        return striker_landmarks["opponent_goal"]

    def compute_kicking_position(striker_landmarks, striker_position, kicking_distance):
        assert "opponent_goal" in striker_landmarks
        return (striker_landmarks["opponent_goal"][0] - kicking_distance, striker_landmarks["opponent_goal"][1]) 

    ValueRegistry().add_function(compute_kicking_position, aliases = ["kicking-position"])
    ValueRegistry().add_function(compute_goal_target, aliases = ["goal-target"])

    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-position")

    def is_opponent_near(striker_obstacles, striker_position, minimum_opponent_distance):
        for obstacle_position in striker_obstacles:
            if math.sqrt(pow(obstacle_position[0] - striker_position[0],2) + pow(obstacle_position[1] - striker_position[1])) < minimum_opponent_distance:
                return True   
        return False


    def is_opponent_goal_landmark_available(striker_landmarks):
        #Only lists are supported in ValueRegistry so we need to use
        for landmark in striker_landmarks:
            if landmark[0] == "opponent_goal":
                return True
        return False

    FluentRegistry().add_function(is_opponent_near, aliases=["fluent-opponent-near"])
    FluentRegistry().add_function(is_opponent_goal_landmark_available, aliases=["fluent-opponent-goal-landmark"])

    def jolly_available(striker_position, jolly_position):
            return jolly_position[0] > striker_position[0]

    FluentRegistry().add_function(jolly_available, aliases=["fluent-jolly-available"], default_value_if_not_evaluable=False)
        