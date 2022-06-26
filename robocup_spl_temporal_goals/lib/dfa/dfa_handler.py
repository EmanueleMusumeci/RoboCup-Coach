import time

from lib.dfa.dfa import DFANode, DFAEdge, DFA

from lib.registries.action import *

#TODO: get_current_state should 1) Update the DFA 2) return the literals and are actions for the current state (only action is needed but we want this to be more general purpose)
class DFAHandler:
    def __init__(self, dfa, dfa_postprocessing_functions = []):
        if dfa_postprocessing_functions:
            for preprocessing_step in dfa_postprocessing_functions:
                preprocessing_step(dfa)
        self.dfa = dfa
        self.__current_state : DFANode = dfa.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_instance("action_idle"), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
    
    def reset(self):
        self.__current_state : DFANode = self.dfa.initial_state
        self.__current_edge = None

        self.__previous_state = None
        self.__previous_edge = None
        self.__trace = [{"edge" : None, "performed_action" : ActionRegistry().get_instance("action_idle"), "destination_state" : self.__current_state.node_id, "timestamp" : time.time()}]
        

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
        outgoing_edges : List[DFAEdge] = self.__current_state.get_outgoing_edges()

        if verbose: print("------------\nUPDATE\n------------\nCurrent node: %s" % (str(self.__current_state.node_id)))
        #Select only edges where all literals are verified
        verified_edges = []
        #print(outgoing_edges)
        for edge in outgoing_edges:
            if edge.is_verified():
                verified_edges.append(edge)

        print(verified_edges)

#TODO: remove this step after the pruning of negated actions has been implemented
        #Filter out edges that have a negated action
        filtered_verified_edges = []
        for edge in verified_edges:
            non_negated_actions = edge.get_non_negated_actions()
            #print(non_negated_actions)
            if not non_negated_actions:
                continue
            #TODO: relax this if multiple actions are supported
            assert len(non_negated_actions) == 1
            #If the action is "negated", ignore this edge
            assert non_negated_actions[0][1] == True
            filtered_verified_edges.append(edge)
        
        assert filtered_verified_edges
        #print("\n"+str(filtered_verified_edges))
        
        
        if not filtered_verified_edges:
            print("USING PREVIOUS ACTION")
            chosen_transition = self.__trace[-1]["edge"]
        else:
            #print("CHOOSING BEST ACTION")
            chosen_transition : DFAEdge = self.choose_best_edge(filtered_verified_edges)
        
        #print(chosen_transition)
        if chosen_transition is not None:
    #TODO: relax this if multiple actions are going to be supported        
            assert len(chosen_transition.get_non_negated_actions()) == 1

            chosen_action : Action = chosen_transition.get_non_negated_actions()[0][0]
            destination_state : DFANode = chosen_transition.to_node
        else:
            chosen_action : Action = ActionRegistry()["action_idle"]
            destination_state : DFANode = self.__current_state.node_id

        #If we're not still repeating the same current action
        if chosen_transition != self.__current_edge:
            print("UPDATING TRACE WITH CHOSEN ACTION '%s'" % (str(chosen_action)))
            #Update previous state/edge and trace in case the new one is different from the previous one
            self.__previous_edge = self.__current_edge
            self.__previous_state = self.__current_state

            self.__trace .append({"edge" : chosen_transition, "performed_action" : chosen_action, "destination_state" : destination_state, "timestamp" : time.time()})
        
            #Transition through the chosen edge
            self.__current_state = destination_state
            self.__current_edge = chosen_transition

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
        return "DFAHandler:\nCurrent state: %s\n,DFA: %s" % (str(self.__current_state), str(self.dfa)) 


#DFA Post-Processing functions

#1) Remove the dummy initial state: 
#   if the DFA has a "dummy" initial state (initial state with only an outgoing edge with a true condition)
#   remove that state and set the next state as the initial one
def remove_initial_dummy_state(dfa : DFA, verbose=False):
    #Don't post-process DFAs consisting of a single state with a single loop edge
    if len(dfa.accepting_states) + len(dfa.rejecting_states) == 1:
        return

    initial_state_outgoing_edges : DFAEdge = dfa.initial_state.get_outgoing_edges()
    if len(initial_state_outgoing_edges) == 1:
        next_state = initial_state_outgoing_edges[0].to_node

        edge_literals = initial_state_outgoing_edges[0].get_literals()
        edge_actions = initial_state_outgoing_edges[0].get_actions()
        
        #If the edge hasn't got any guard literal nor action, this is a dummy state and we can prune it
        if not edge_literals and not edge_actions:
            
            assert ((dfa.initial_state.node_id in dfa.rejecting_states.keys()) or (dfa.initial_state.node_id in dfa.accepting_states.keys())) and (dfa.initial_state.node_id in dfa.states.keys())

            #Remove dummy state
            if dfa.initial_state.node_id in dfa.rejecting_states.keys():
                dfa.rejecting_states.pop(dfa.initial_state.node_id)
            else:
                dfa.accepting_states.pop(dfa.initial_state.node_id)
            dfa.states.pop(dfa.initial_state.node_id)
            
            #Set new initial state
            dfa.initial_state = next_state
            #Remove edge incoming from dummy state from the next state
            next_state.remove_edge(initial_state_outgoing_edges[0])


