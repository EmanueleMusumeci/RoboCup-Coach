import time
import os
from pathlib import Path

from lib.plan.policy import Policy, PolicyEdge, PolicyNode 


from lib.registries.action import *

#TODO: get_current_state should 1) Update the Policy 2) return the literals and are actions for the current state (only action is needed but we want this to be more general purpose)
class PolicyHandler:
    def __init__(self, 
            policy : Policy, 
            plan_postprocessing_functions = [],

            pddl_domain_path : str = None,
            pddl_problem_path : str = None,
            goal : str = None,
            PLTLf_mapping_path : str = None,

            working_dir : str = None,

            FOND : bool = False,

            problem_name : str = None,
        ):

        self.pddl_domain_path = pddl_domain_path
        self.pddl_problem_path = pddl_problem_path
        self.goal = goal
        self.PLTLf_mapping_path = PLTLf_mapping_path

        self.working_dir = working_dir

        self.FOND = FOND

        if problem_name is not None:
            assert isinstance(problem_name, str)
        else:
            problem_name = Path(os.path.abspath(pddl_problem_path)).stem
        self.problem_name = problem_name

        assert isinstance(policy, Policy), "This instance should wrap a networkx DiGraph"
        self.policy = policy

        if plan_postprocessing_functions:
            for preprocessing_step in plan_postprocessing_functions:
                preprocessing_step(policy)
        
        self.__current_state : PolicyNode = policy.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_idle_action(), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    


    @classmethod
    def create_policy(cls, 
        pddl_domain_path : str, 
        pddl_problem_path : str, 
        
        working_dir : str = None, 
        problem_name : str = None, 
        plot : bool = False, 
        
        domain_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        problem_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        plan_postprocessing_functions : List[Dict[Dict[str, Any], Callable]] = []
    ):
        working_dir, problem_name = check_and_preprocess_policy_generation_data(pddl_domain_path, pddl_problem_path, None, working_dir, problem_name, None, domain_preprocessing_functions=domain_preprocessing_functions, problem_preprocessing_functions=problem_preprocessing_functions)

        #print("Creating Policy for striker role from domain file: %s with problem file %s\n" % (pddl_domain_path, pddl_problem_path))
        policy = Policy.build_from_PDDL(pddl_domain_path, pddl_problem_path, working_dir)
        
        if plot:
            policy.plot(save_to=os.path.join(working_dir, "plan_preview", problem_name+".png"), show_plot = False)
        
        policy_handler = PolicyHandler(

            policy,
            pddl_domain_path = pddl_domain_path,
            pddl_problem_path = pddl_problem_path,

            working_dir = working_dir,

            problem_name = problem_name
        )
    
        return policy_handler



    @classmethod
    def create_policy_with_PLTLf(cls, 
        pddl_domain_path : str, 
        pddl_problem_path : str, 
        goal : str, 
        
        working_dir : str, 
        
        PLTLf_mapping_path : str = None, 
        
        problem_name : str = None, 
        plot : bool = False, 
            
        domain_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        problem_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        plan_postprocessing_functions : List[Dict[Dict[str, Any], Callable]] = []
    ):
        #It has the same result (but the policy will actually have no branches so it will be a plan)
        return PolicyHandler.create_FOND_policy_with_PLTLf(pddl_domain_path, pddl_problem_path, goal, working_dir, PLTLf_mapping_path, problem_name, plot, PLTLf_mapping_path = PLTLf_mapping_path, domain_preprocessing_functions = domain_preprocessing_functions, problem_preprocessing_functions = problem_preprocessing_functions, plan_postprocessing_functions = plan_postprocessing_functions)



    @classmethod
    def create_FOND_policy(cls, 
        pddl_domain_path : str, 
        pddl_problem_path : str, 
        
        working_dir : str = None, 
        
        problem_name : str = None, 
        plot : bool = False, 
            
        domain_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        problem_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        plan_postprocessing_functions : List[Dict[Dict[str, Any], Callable]] = []        
    ):
        working_dir, problem_name = check_and_preprocess_policy_generation_data(pddl_domain_path, pddl_problem_path, None, working_dir, problem_name, None, domain_preprocessing_functions, problem_preprocessing_functions)

        #print("Creating FOND Policy for striker role from domain file: %s with problem file %s\n" % (pddl_domain_path, pddl_problem_path))
        policy = Policy.build_from_FOND_PDDL(pddl_domain_path, pddl_problem_path, working_dir)
        
        if plot:
            policy.plot(save_to=os.path.join(working_dir, "plan_preview", problem_name+".png"), show_plot = False)
        
        policy_handler = PolicyHandler(

            policy,
            pddl_domain_path = pddl_domain_path,
            pddl_problem_path = pddl_problem_path,
            FOND = True,
            
            working_dir = working_dir,

            problem_name = problem_name
        )

        return policy_handler



    @classmethod
    def create_FOND_policy_with_PLTLf(cls, 
        pddl_domain_path : str, 
        pddl_problem_path : str, 
        goal : str, 
        
        working_dir : str, 
        
        PLTLf_mapping_path : str = None, 
        
        problem_name : str = None, 
        plot : bool = False, 
        
        domain_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        problem_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = [], 
        plan_postprocessing_functions : List[Dict[Dict[str, Any], Callable]] = []
    ):
        working_dir, problem_name = check_and_preprocess_policy_generation_data(pddl_domain_path, pddl_problem_path, goal, working_dir, problem_name, PLTLf_mapping_path = PLTLf_mapping_path, domain_preprocessing_functions=domain_preprocessing_functions, problem_preprocessing_functions=problem_preprocessing_functions)

        #print("Creating FOND Policy for striker role from domain file: %s with problem file %s and PLTLf formula '%s'\n" % (pddl_domain_path, pddl_problem_path, goal))
        policy = Policy.build_from_FOND_PDDL_and_PLTLf_formula(pddl_domain_path, pddl_problem_path, working_dir, goal, mapping_path = PLTLf_mapping_path)
        
        if plot:
            policy.plot(save_to=os.path.join(working_dir, "plan_preview", problem_name+".png"), show_plot = False)
        
        policy_handler = PolicyHandler(

            policy,
            pddl_domain_path = pddl_domain_path,
            pddl_problem_path = pddl_problem_path,
            goal = goal,
            PLTLf_mapping_path = PLTLf_mapping_path,

            working_dir = working_dir,

            FOND = True,

            problem_name = problem_name
        )

        return policy_handler
    





    def replan(self, goal, plot : bool = False):
        assert self.pddl_domain_path is not None
        assert self.pddl_problem_path is not None
        assert self.working_dir is not None
        
        assert goal is not None
        assert isinstance(goal, str)
        
        if goal == self.goal:
            print("Goal is the same, no replan needed. Goal: \n\t%s" % (self.goal))
            return

        #We only cover the PLTLf case (because that's the only case where the goal is specified dynamically)
        self.policy = Policy.build_from_FOND_PDDL_and_PLTLf_formula(self.pddl_domain_path, self.pddl_problem_path, self.working_dir, goal, self.PLTLf_mapping_path)

        if plot:
            self.policy.plot(save_to=os.path.join(self.working_dir, "plan_preview", self.problem_name+".png"), show_plot = False)

        self.reset()


    def reset(self):
        self.__current_state : PolicyNode = self.policy.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_idle_action(), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    
    def get_next_action(self, verbose = False):
        action = self.update(verbose)
        #print(self.__trace)
        if verbose: print("Chosen action: "+str(action)+"\n------------\n")
        return action

    def get_current_action(self):
        if len(self.__trace) == 1:
            return ActionRegistry().get_idle_action()
        else:
            return self.__trace[-1]["performed_action"]

    def get_previous_action(self):
        if len(self.__trace) == 1:
            return ActionRegistry()["action_idle"]
        else:
            return self.__trace[-2]["performed_action"]
    
    def get_current_state(self):
        self.update()
        return self.__current_state.node_id

    def get_previous_state(self):
        return self.__previous_state.node_id

    def get_last_transition_timestamp(self):
        return self.__trace[-1]["timestamp"]
    
    def update(self, verbose):
        outgoing_edges : List[PolicyEdge] = self.__current_state.get_outgoing_edges()
        
        if verbose: print("------------\nUPDATE\n------------\nCurrent node: %s" % (str(self.__current_state.node_id)))
        #Select only edges where all literals are verified
        verified_edge = None
        #print(outgoing_edges)
        if self.policy.is_plan:
            assert len(outgoing_edges) == 1 or len(outgoing_edges) == 0
        
        verified_edges = []
        for edge in outgoing_edges:
            if edge.is_verified():
                verified_edges.append(edge)
                #Even if we found a verified edge, keep looping to check consistency of this plan/policy
                # (a plan/policy always has ONE verified outgoing edge or ONE verified edge and ONE edge without conditions at each time)
        
        #print("\n"+str(verified_edges))
        
        
        if self.__trace[-1]["performed_action"].completed or self.__trace[-1]["performed_action"].is_idle_action():
            if len(verified_edges) > 1:
                assert len(verified_edges) == 2, "More than 2 edges are verified"
                assert (verified_edges[0].get_fluents() and not verified_edges[1].get_fluents()) or (not verified_edges[0].get_fluents() and verified_edges[1].get_fluents())
                if not verified_edges[0].get_fluents():
                    chosen_transition : PolicyEdge = verified_edges[1]
                else:
                    chosen_transition : PolicyEdge = verified_edges[0]

            elif not verified_edges:
                chosen_transition : PolicyEdge = None
            else:
                chosen_transition : PolicyEdge = verified_edges[0]
            if verbose: print("Last action completed")
        else:
            chosen_transition = self.__trace[-1]["edge"]
            if chosen_transition is not None:
                if verbose: print("Last action NOT completed\nChosing last action")
                
        #print(chosen_transition)
        if chosen_transition is not None:
            chosen_action : Action = chosen_transition.guard_action
            destination_state : PolicyNode = chosen_transition.to_node
            #print("\tCurrent action: "+str(chosen_transition.guard_action))
        else:
            chosen_action : Action = ActionRegistry().get_idle_action()
            destination_state : PolicyNode = self.__current_state
            #print("\tCurrent action: Idle Action")

        #If we're not still repeating the same current action
        if chosen_transition != self.__current_edge:
            if verbose: print("\tUpdating trace with chosen action '%s'" % (str(chosen_action)))
            #Update previous state/edge and trace in case the new one is different from the previous one
            self.__previous_edge = self.__current_edge
            self.__previous_state = self.__current_state

            self.__trace.append({"edge" : chosen_transition, "performed_action" : chosen_action, "destination_state" : destination_state, "timestamp" : time.time()})

            if verbose: print("\t\tCurrent trace:\n"+self.trace_to_string())

            #Transition through the chosen edge
            self.__current_state = destination_state
            self.__current_edge = chosen_transition
    
        if verbose: print("------------")
        return chosen_action
        
#TODO: add some logic to choose an edge (maybe planning techniques)    
    def choose_best_edge(self, edges):
        return edges[0]


    def trace_to_string(self):
        trace_string = ""
        for edge in self.__trace:
            trace_string += "\t"+str(edge["performed_action"])+"\n"
        
        return trace_string

    def __str__(self):
        return "PolicyHandler:\nCurrent state: %s\n,Policy: %s" % (str(self.__current_state), str(self.dfa)) 


def apply_preprocessing(pddl_path, preprocessing_steps):
    domain_str = ""
    with open(pddl_path, mode="r") as domain_file:
        domain_str = domain_file.readlines()
    
    assert domain_str

    for preprocessing_step_tuple in preprocessing_steps:
        assert isinstance(preprocessing_step_tuple, tuple)
        assert len(preprocessing_step_tuple) == 2
        assert isinstance(preprocessing_step_tuple[0], dict)
        assert isinstance(preprocessing_step_tuple[1], Callable)
            
        domain_str = preprocessing_step_tuple(domain_str, function = preprocessing_step_tuple[1], parameters = preprocessing_step_tuple[0])

    with open(pddl_path, mode="w") as domain_file:
        domain_file.write(domain_str)


def check_and_preprocess_policy_generation_data(pddl_domain_path : str, pddl_problem_path : str, goal : str, working_dir : str, problem_name : str, PLTLf_mapping_path : str = None, domain_preprocessing_functions : List[Tuple[Dict[str, Any], Callable]] = [], problem_preprocessing_functions : List[Dict[Dict[str, Any], Callable]] = []):
 
    assert isinstance(pddl_domain_path, str)
    assert os.path.exists(pddl_domain_path)

    assert isinstance(pddl_problem_path, str)
    assert os.path.exists(pddl_problem_path)
    
    if goal is not None:
        assert isinstance(goal, str)

    if working_dir is not None:
        assert isinstance(working_dir, str)
        if not os.path.exists(working_dir):
            os.makedirs(working_dir)
    
    if problem_name is not None:
        assert isinstance(problem_name, str)
    else:
        problem_name = Path(os.path.abspath(pddl_problem_path)).stem

    if PLTLf_mapping_path is not None:
        assert isinstance(PLTLf_mapping_path, str)
        assert os.path.exists(PLTLf_mapping_path)

    if domain_preprocessing_functions:
        apply_preprocessing(pddl_domain_path, domain_preprocessing_functions)
    
    if problem_preprocessing_functions:
        apply_preprocessing(pddl_problem_path, problem_preprocessing_functions)

    return working_dir, problem_name