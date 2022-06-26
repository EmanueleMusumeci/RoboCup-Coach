
from ast import alias
import sys,os,inspect
from pathlib import Path
from typing import Dict, List

import networkx as nx
import matplotlib.pyplot as plt

from lib.experiment import setup_conditioned_FOND_policy_for_experiment, ExperimentType

def get_experiment_type():
    return ExperimentType.CONSTRAINABLE_POLICY

def get_problem_name():
    return "adversarial_striker_A"

def get_robot_formation():
    return {3 : "Caligola"}

def role_to_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)
    parentparentdir = os.path.dirname(parentdir)

    return {
        "striker" : {
            "goal" : "isat_ball_goaltarget",
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain.pddl"),
            "pddl_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem.pddl"),
            "pddl_mapping_path" : None,
            "working_dir" : os.path.join(currentdir, "output"),

            "constrainable_predicates" : ["isat", "ballkickedto", "balldribbledto", "movedto"],
        }
    }

def initialize_registries():
    #sys.path.insert(0, parentparentdir) 

    from lib.plan.policy import Policy
    #from lib.plan.policy_handler import PolicyHandler
    from lib.registries.action import ActionRegistry
    from lib.registries.values import ValueRegistry
    from lib.utils import linear_distance, angular_distance


    ActionRegistry(robot_idle_skill="Idle")

    #Ground actions to robot skills
    #Create ActionTemplates by specifying:
    #Argument 1: ActionTemplate name (all actions created from this template will have this as a base name plus a uuid)
    #Argument 2: robot skill name 
    #Argument 3: which parameters should be selected from the list of parameters in the .pddl domain specification 
    #   ([] means no parameter, not specifying the argument instead means ALL parameters)
    ActionRegistry().register_action_template("moverobot", "ReachPosition", [2])
    ActionRegistry().register_action_template("kickball", "Kick", [3])
    ActionRegistry().register_action_template("carryball" ,"CarryBall", [3])
    ActionRegistry().register_action_template("kicktogoal", "CarryAndKickToGoal", [])

    #Ground :objects in ValueRegistry
    ValueRegistry()["kickingposition"] = (3500, 0)
    ValueRegistry()["goaltarget"] = (5100, 0)
        
    ValueRegistry()["rightflank"] = (2000, -1500)
    ValueRegistry()["leftflank"] = (2000, 1500)

    ValueRegistry()["rightareaentrypoint"] = (3000, -1000)
    ValueRegistry()["leftareaentrypoint"] = (3000, 1000)
    
    ValueRegistry()["middleareaentrypoint"] = (2800, 0)

    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="strikercurrentposition")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ballcurrentposition")

def setup(role_to_additional_constraints : Dict[str, List[str]] = {}):
    return setup_conditioned_FOND_policy_for_experiment(get_problem_name(), role_to_generation_data(), role_to_additional_constraints = role_to_additional_constraints)