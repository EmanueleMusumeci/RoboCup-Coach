
from ast import alias
import sys,os,inspect
from pathlib import Path

import networkx as nx
import matplotlib.pyplot as plt

from lib.plan.policy_handler import PolicyHandler

from lib.experiment import setup_FOND_policy_for_experiment, ExperimentType

def get_experiment_type():
    return ExperimentType.POLICY

def get_problem_name():
    return "basic_striker_policy"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)

    return {
        "striker" : {
            "goal" : None,
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "robocup_domain_deterministic.pddl"),
            "pddl_problem_path" : os.path.join(currentdir, "PDDL", "simple_striker_problem_deterministic.pddl"),
            #"pddl_mapping_path" : os.path.join(currentdir, "PDDL", "simple_striker_mapping.map"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),

            "constrainable_predicates" : None,
        }
    }

def initialize_registries():

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry

    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("move-robot", "ReachPosition", [2])
    ActionRegistry().register_action_template("kick-ball", "Kick", [3])
    ActionRegistry().register_action_template("carry-ball" ,"CarryBall", [3])
    ActionRegistry().register_action_template("kick-to-goal", "CarryAndKickToGoal", [])

    #Ground :objects in ValueRegistry
    ValueRegistry()["kicking-position"] = (2000, 0)
    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="striker-current-position")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ball-current-position")

def setup():
    return setup_FOND_policy_for_experiment(get_problem_name(), role_to_generation_data())