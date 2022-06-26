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
    return "fond_planning_striker_with_pass"

def get_robot_formation():
    return {3 : "Caligola", 5 : "Nerone"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)

    return {
        "striker" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "robocup_striker_domain_fond.pddl"),
            "pddl_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem_fond.pddl"),
            #"pddl_mapping_path" : os.path.join(currentdir, "PDDL", "simple_striker_mapping.map"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),
        },
        "jolly" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "robocup_jolly_domain_fond.pddl"),
            "pddl_problem_path" : os.path.join(currentdir, "PDDL", "jolly_problem_fond.pddl"),
            #"pddl_mapping_path" : os.path.join(currentdir, "PDDL", "simple_striker_mapping.map"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),
        }
    }

def setup():
    return setup_FOND_policy_for_experiment(get_problem_name(), role_to_generation_data())

def initialize_registries():

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
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
    ValueRegistry()["kicking_position"] = (2500, 0)
    ValueRegistry()["goal_position"] = (3000, 0)
    ValueRegistry()["field_sideline"] = 3000

    def jolly_pass_position(striker_obstacles, jolly_position):
        if striker_obstacles:
            centroid = striker_obstacles[0]
            for obstacle in striker_obstacles[1:]:
                centroid = ((centroid[0] + obstacle[0])/2, (centroid[1] + obstacle[1])/2)
            if centroid[1] > 0:
                return (math.radians(120), 2000, -2000)
            else:
                return (math.radians(-120), 2000, 2000)

        else:
            if jolly_position[1] < 0:
                return (math.radians(120), 2000, -2000)
            else:
                return (math.radians(-120), 2000, 2000)
        print(centroid)


                
    ValueRegistry().add_function(jolly_pass_position)


    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_alias(item_name="jolly_position", alias_name="jolly-current-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-current-position")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("carry-ball-to-kick", "CarryBall", ["kicking_position"])
    ActionRegistry().register_action_template("move-to-ball", "ReachBall", [])
    ActionRegistry().register_action_template("pass-ball-to-jolly" ,"Kick", ["jolly_position"])
    ActionRegistry().register_action_template("kick-to-goal", "Kick", ["goal_position"])
    ActionRegistry().register_action_template("dribble-opponent", "CarryBall", ["goal_position"])
    ActionRegistry().register_action_template("wait-for-jolly", "Idle", [])


    def is_obstacle_blocking(position, obstacle_position, target_position):
        return obstacle_position[0] > position[0] and obstacle_position[0] < target_position[0]

    def is_obstacle_left(obstacle_position, field_sideline):
        return obstacle_position[1] > 0 and obstacle_position[1] < field_sideline
        
    def is_obstacle_right(obstacle_position,field_sideline):
        return obstacle_position[1] < 0 and obstacle_position[1] > -field_sideline

    def obstacle_blocking_goal(striker_position, striker_obstacles, field_sideline):
        for obstacle_position in striker_obstacles:
            if is_obstacle_blocking(striker_position, obstacle_position, target_position=(4500,0)):
                return True
        return False

    FluentRegistry().add_function(obstacle_blocking_goal, aliases=["fluent-obstacle-blocking-goal"])

    def jolly_available(striker_position, jolly_position):
        return jolly_position[0] > striker_position[0]

    def jolly_in_position(jolly_position, jolly_pass_position):
        return linear_distance(jolly_position, jolly_pass_position) < 500

    def jolly_aligned(jolly_position, jolly_pass_position):
        return angular_distance_in_degrees(jolly_position, jolly_pass_position) < 10


    FluentRegistry().add_function(jolly_available, aliases=["fluent-jolly-available"], default_value_if_not_evaluable=False)
    FluentRegistry().add_function(jolly_in_position, aliases=["fluent-jolly-in-position"], default_value_if_not_evaluable = False)
    FluentRegistry().add_function(jolly_aligned, aliases=["fluent-jolly-aligned-to-striker"], default_value_if_not_evaluable = False)

    
    ''' 
    _________________
    |                |
    |  JOLLY VALUES  |
    |________________|

    '''


    ActionRegistry().register_action_template("move-to-receiving-position", "ReachPositionAndAngle", ["jolly_pass_position"])
    ActionRegistry().register_action_template("turn-to-striker", "ReachPositionAndAngle", ["jolly_pass_position"])
    