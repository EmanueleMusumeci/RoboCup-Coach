import os
from pathlib import Path
from typing import Dict, Callable
from enum import Enum

from lib.plan.policy_handler import PolicyHandler
from lib.dfa.dfa_handler import DFAHandler, remove_initial_dummy_state
from lib.dfa.dfa import DFA


class ExperimentType(Enum):
    DFA = 0
    POLICY = 1
    CONSTRAINABLE_POLICY = 2

def check_adjacency_ready_problem(problem_file_path):
    with open(problem_file_path, mode = "r") as f:
        for line in f.readlines():
            if "ADJACENCY_PREDICATES" in line:
                return True
    return False

def setup_LTL_DFA_for_experiment(problem_name : str, role_to_generation_data : dict):
    role_to_handler = {}

    for role, generation_data in role_to_generation_data.items():
        
        assert "goal" in generation_data.keys()             

        ltl_goal = generation_data["goal"]

        print("Creating DFA from formula: %s with post-processing step 'remove_initial_dummy_state'" % (ltl_goal))
        try:
            dfa = DFA.DFA_from_LTL_formula_string(ltl_goal)
            dfa.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", Path(os.path.abspath(__file__)).stem+".png"), show_plot = False)
            role_to_handler[role] = DFAHandler(dfa, dfa_postprocessing_functions = [remove_initial_dummy_state])
        except AssertionError as e:
            raise e
        else:
            print("OK")

    return role_to_handler

def setup_conditioned_FOND_policy_for_experiment(problem_name : str, role_to_generation_data : dict, role_to_additional_constraints : dict):

    role_to_handler = {}

    for role, generation_data in role_to_generation_data.items():
        
        assert "goal" in generation_data.keys()        
        assert "pddl_domain_path" in generation_data.keys()        
        assert "pddl_problem_path" in generation_data.keys()        
        assert "pddl_mapping_path" in generation_data.keys()        
        assert "working_dir" in generation_data.keys()        

        goal = generation_data["goal"]
        domain_path = generation_data["pddl_domain_path"]
        problem_path = generation_data["pddl_problem_path"]
        mapping_path = generation_data["pddl_mapping_path"]
        working_dir = generation_data["working_dir"]


        if role in role_to_additional_constraints:
            if isinstance(role_to_additional_constraints[role], list):
                for constraint in role_to_additional_constraints[role]:
                    goal += " && "+constraint
            else:
                goal += " && "+role_to_additional_constraints[role]

        print("Creating FOND Policy for '%s' role from domain file: %s with problem file %s, conditioned by PLTLf formula '%s'\n" % (role, domain_path, problem_path, goal))
        try:
            role_to_handler[role] = PolicyHandler.create_FOND_policy_with_PLTLf(
                domain_path, problem_path, 
                goal, 
                
                working_dir, 

                PLTLf_mapping_path=mapping_path,
                problem_name = problem_name + "_" + role, 
                plot = True,

                domain_preprocessing_functions = [],
                problem_preprocessing_functions = [],
                plan_postprocessing_functions = [],
            )
        except AssertionError as e:
            raise e
        else:
            print("OK")
    
    return role_to_handler

def setup_FOND_policy_for_experiment(problem_name : str, role_to_generation_data : dict):

    role_to_handler = {}

    for role, generation_data in role_to_generation_data.items():
        
        assert "pddl_domain_path" in generation_data.keys()        
        assert "pddl_problem_path" in generation_data.keys()        
        assert "pddl_mapping_path" in generation_data.keys()        
        assert "working_dir" in generation_data.keys()        

        domain_path = generation_data["pddl_domain_path"]
        problem_path = generation_data["pddl_problem_path"]
        working_dir = generation_data["working_dir"]


        print("Creating Policy for '%s' role from domain file: %s with problem file %s\n" % (role, domain_path, problem_path))
        try:
            role_to_handler[role] = PolicyHandler.create_FOND_policy(
                domain_path, problem_path, 
                
                working_dir, 

                problem_name = problem_name + "_" + role, 
                plot = True,

                domain_preprocessing_functions = [],
                problem_preprocessing_functions = [],
                plan_postprocessing_functions = [],
            )
        except AssertionError as e:
            raise e
        else:
            print("OK")
    
    return role_to_handler
