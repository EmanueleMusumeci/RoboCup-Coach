from operator import ne
import os
import sys
import inspect
from pathlib import Path
from typing import List, Type, Tuple

import networkx
import matplotlib.pyplot as plt
import pygraphviz as pgv



from lib.registries.fluents import *
from lib.registries.action import *
import lib.utils


import fond4ltlfpltlf
from fond4ltlfpltlf.core import execute

from pylogics.parsers import parse_pltl

import networkx as nx
import matplotlib.pyplot as plt

from robocup_spl_temporal_goals import PACKAGE_ROOT

from robocup_spl_temporal_goals.third_party.planning_with_past.planners.mynd.base import MyNDPlanner
from robocup_spl_temporal_goals.third_party.planning_with_past.planners.downward import DownwardPlanner
from robocup_spl_temporal_goals.third_party.pddl.pddl.parser.domain import DomainParser
from robocup_spl_temporal_goals.third_party.pddl.pddl.parser.problem import ProblemParser
from robocup_spl_temporal_goals.third_party.planning_with_past.compiler import Compiler
from robocup_spl_temporal_goals.third_party.pddl.pddl.formatter import domain_to_string, problem_to_string


#TODO might consider adding predicates
class PolicyEdge:
    def __init__(self, from_node : Type['PolicyNode'], to_node : Type['PolicyNode'], guard_action : Action):

        #print(from_node.get_fluents())
        for source_fluent in from_node.get_fluents():
            assert isinstance(source_fluent, Tuple)
            assert isinstance(source_fluent[0], Fluent)
            assert isinstance(source_fluent[1], bool)
        
        #to_node.get_fluents()
        for destination_fluent in to_node.get_fluents():
            assert isinstance(destination_fluent, Tuple)
            assert isinstance(destination_fluent[0], Fluent)
            assert isinstance(destination_fluent[1], bool)
        
        #Fluents that have to change for us to perform this action
        self.guard_fluents = []
        #Check that fluents are available in both the source and the destination node (does not matter if negated or not)
        #print(from_node.node_id)
        #print(from_node.get_fluents())
        #Check that fluent is in both source and destination node
        for source_fluent in from_node.get_fluents():
            is_fluent_in_both = False
            for destination_fluent in to_node.get_fluents():
                if source_fluent[0] == destination_fluent[0]:
                    is_fluent_in_both = True
                    break
            #Fluent should be in both
            assert is_fluent_in_both

            #Add fluents as transition conditions for this branch only if they appear in both and
            # A) They have a different value in the successor node
            #   OR
            # B) We are in the root node
            if is_fluent_in_both and \
                (\
                    source_fluent[1] != destination_fluent[1] \
                        or \
                    from_node.is_root()
                ):            
                self.guard_fluents.append(destination_fluent)

        #NOTICE: we can not perform this action unless all guard_fluents are verified 

        assert guard_action is not None
        assert isinstance(guard_action, Action)
        self.guard_action = guard_action
        
        '''
        #Literals that change if we perform this action
        self.guard_literals = []
        #Check that fluents are available in both the source and the destination node (does not matter if negated or not)
        for source_literal in from_node.get_literals():
            is_literal_in_both = False
            for destination_literal in to_node.get_literals():
                if source_literal[0] == destination_literal[0]:
                    is_literal_in_both = True
            assert is_literal_in_both
            
            if source_literal[1] != destination_literal[1]:
                #If the destination literal is negated wrt the source literal, 
                # we append the literal in its destination state as a guard literal 
                # (a literal that has to switch boolean state in order for us to perform this action)
                self.guard_literals.append(destination_literal)
        '''

        assert isinstance(from_node, PolicyNode)
        assert isinstance(to_node, PolicyNode)

        self.from_node = from_node
        self.to_node = to_node

    def get_action(self) -> Tuple[Action, bool]:
        return self.guard_action
    
    def get_fluents(self) -> List[Tuple[Fluent, bool]]:
        return self.guard_fluents
    
    '''
    def get_literals(self) -> List[Tuple[Fluent, bool]]:
        return self.guard_fluents
    '''

    def is_verified(self, verbose = True) -> bool:
        all_verified = True
        for fluent_tuple in self.guard_fluents:
            
            try:
                current_fluent_value = bool(fluent_tuple[0])
                if verbose:
                    print("Fluent '%s' should be %s and is %s" % (fluent_tuple[0], fluent_tuple[1], bool(fluent_tuple[0])))
            except KeyError as ke:
                print("KeyError raised: %s.\n\nCan't evaluate fluent %s: no transition is possible." % (str(ke), fluent_tuple[0].name_in_registry))
                return False
            except TypeError as te:
                print("TypeError raised: %s. Something went wrong when evaluating fluent '%s' to bool." % (str(te), fluent_tuple[0].name_in_registry))
                return False
            except Exception as e:
                raise e

            expected_fluent_value = fluent_tuple[1]
            this_fluent_verified = (current_fluent_value == expected_fluent_value)
            all_verified = all_verified and this_fluent_verified
            if not all_verified:
                break
        
        if verbose:
            print("\n")
        return all_verified
    
    def __str__(self):
        if self.guard_fluents:
            fluents_string = "\nFluents:"
            for fluent in self.guard_fluents:
                fluents_string += "\n\t\t"+("NOT " if not fluent[1] else "") + str(fluent[0])
        else:
            fluents_string = "\nFluents: []"

        action_string = "\n\tActions:\n\t\t"+self.guard_action.name_in_registry

        return "Edge: %s -> %s%s%s" % (self.from_node.node_id, self.to_node.node_id, fluents_string, action_string) + "\n"

    def get_label_string(self, shorten_fluents = True, show_aliases = False):
        if not self.guard_action and not self.guard_fluents:
            return "True"
        label_string = ""
        if self.get_fluents():
            label_string += "Fluents:\n"
            for i, fluent in enumerate(self.guard_fluents):
                fluent_str = fluent[0].name_in_registry
                if shorten_fluents:
                    fluent_str = fluent_str.replace(FluentRegistry.ITEM_PREFIX, "")
                label_string += (" & " if i > 0 else "") +("~" if not fluent[1] else "") + str(fluent_str) + "\n"
            #label_string += "]\n"
            label_string += " Action:\n" 
        label_string += str(self.guard_action.base_name)
        if self.guard_action.parameters:
            label_string += "("
            for i, parameter in enumerate(self.guard_action.parameters):
                if isinstance(parameter, Tuple):
                    label_string += str(parameter[1])
                else:
                    #check if the parameter name is actually an alias
                    if ValueRegistry().is_alias_name(parameter) and show_aliases:
                        label_string += ValueRegistry().get_aliased_name(parameter)
                        label_string += "(alias '"+parameter+"')"
                    else:
                        label_string += parameter
                if i < len(self.guard_action.parameters) - 1:
                    label_string += ", "
            label_string += ")"
        return label_string

    def __repr__(self):
        return str(self)
        

class PolicyNode:
    def __init__(self, node_id, fluents : List[Tuple[Fluent, bool]] = None, predicates : List[Tuple[str, Any, bool]] = None, edges : List[PolicyEdge] = None):

        self.__outgoing_edges = []
        self.__incoming_edges = []
        if edges is None:
            self.__edges = []
        else:
            self.__edges = edges
            for edge in edges:
                assert isinstance(edge, PolicyEdge)
                assert edge.from_node == self or edge.to_node == self
                if edge.from_node == self:
                    self.__outgoing_edges.append(edge)
                if edge.to_node == self:
                    self.__incoming_edges.append(edge)

        self.node_id = node_id

        if fluents is not None:
            for fluent in fluents:
                assert isinstance(fluent, Tuple)
                assert len(fluent) == 2
                assert isinstance(fluent[0], Fluent)
                assert isinstance(fluent[1], bool)
            self.fluents = fluents
        else:
            self.fluents = []

        if predicates is not None:
            for predicate in predicates:
                assert isinstance(predicate, Tuple)
                assert len(predicate) == 3
                assert isinstance(predicate[0], str)
                assert isinstance(predicate[1], int) or isinstance(predicate[1], float) or isinstance(predicate[1], bool) or isinstance(predicate[1], str)
                assert isinstance(predicate[2], str)
        self.predicates = predicates

    def is_root(self):
        return not self.__incoming_edges

    def get_edges(self):
        return self.__edges

    def get_incoming_edges(self):
        return self.__incoming_edges

    def get_outgoing_edges(self):
        return self.__outgoing_edges

    def get_true_edges(self) -> List[PolicyEdge]:
        valid_edges = []
        for edge in self.__edges:
            if edge.is_verified():
                valid_edges.append(edge)
        return valid_edges
    
    def add_edge(self, edge) -> None:
        #print("NODE_ID: "+str(self.node_id)+", ADDING EDGE: "+str(edge))
        assert edge.from_node == self or edge.to_node == self
        self.__edges.append(edge)
        if edge.from_node == self:
            self.__outgoing_edges.append(edge)
        if edge.to_node == self:
            self.__incoming_edges.append(edge)
    
    def remove_edge(self, edge_to_remove) -> None:
        assert edge_to_remove.from_node.node_id == self.node_id or edge_to_remove.to_node.node_id == self.node_id
        if edge_to_remove.from_node == self:
            self.__outgoing_edges.remove(edge_to_remove)

        if edge_to_remove.to_node == self:
            self.__incoming_edges.remove(edge_to_remove)
    
    def get_edges(self) -> List[PolicyEdge]:
        return self.__edges

    def get_fluents(self):
        return self.fluents

    #This is a final state if there isn't any outgoing edge
    def is_final_state(self) -> bool:
        return not self.__outgoing_edges

    #This is an initial state if there isn't any incoming edge
    def is_initial_state(self) -> bool:
        return not self.__incoming_edges

    def __str__(self):
        edges_string = ""
        #for edge in self.__edges:
        for edge in self.__outgoing_edges:
            edges_string += str(edge)
        edges_string = lib.utils.tab_all_lines_in_string(edges_string, times = 1)
        edges_string = "\nOutgoing edges:\n" + edges_string
        #edges_string = "\nEdges:\n" + edges_string
        return "\nNode ID: %s%s%s" % (str(self.node_id), (" (final state)" if self.is_final_state() else ""), edges_string)

    def __repr__(self):
        return str(self)
    
    def __eq__(self, other_node : Type['PolicyNode']):
        assert isinstance(other_node, PolicyNode)
        return self.node_id == other_node.node_id

class Policy:
    def __init__(self, nodes : Dict[str, PolicyNode], is_plan : bool = False):
        assert isinstance(nodes, dict)
        self.is_plan = is_plan
        self.nodes = nodes
        self.initial_state = self.nodes[0]
        self.edges = []
        for node_id, node_instance in self.nodes.items():
            for outgoing_edge in node_instance.get_edges():
                self.edges.append(outgoing_edge)
            if self.is_plan:
                assert not node_instance.get_fluents()
        

    def add_edge(node1 : PolicyNode, node2 : PolicyNode, action : Action) -> None:
        new_edge = PolicyEdge(node1, node2, guard_action=action)
        node1.add_edge(new_edge)
        node2.add_edge(new_edge)
    
    def remove_edge(edge : PolicyEdge):
        edge.from_node.remove_edge(edge)
        #Call twice only if the edge is not a loop
        if edge.from_node != edge.to_node:
            edge.to_node.remove_edge(edge)

    @classmethod 
    def Idle(cls):
        return Policy({0 : PolicyNode(node_id = 0)}, is_plan = True)

    #A Plan is a policy but without any fluent
    @classmethod
    def build_plan_from_networkx_digraph(cls, networkx_digraph : networkx.classes.digraph.DiGraph):
        
        actions = {}

        
        # where EQUAL tells if we should have an equality or not
        nodes = {}

        for node_data in networkx_digraph.nodes.data():
            node_id = int(node_data[0])
            nodes[node_id] = PolicyNode(node_id = node_id)

        for edge_data in networkx_digraph.edges.data():
            #print(edge_data[2]["label"])
            from_node_id = int(edge_data[0])
            assert from_node_id in nodes.keys()

            to_node_id = int(edge_data[1])
            assert to_node_id in nodes.keys()

            action_strings = edge_data[2]["action"].strip().replace('(', "").replace(")", "").split(" ")
            action_parameter_names = []
            
            action_name = action_strings[0]
            if len(action_strings)>1:
                for param in action_strings[1:]:
                    action_parameter_names.append(param)

            #print(action_name)
            new_action_instance : RegistryItem = ActionRegistry().create_action_from_template(action_template_name = action_name, parameter_list = action_parameter_names)
            
            new_edge = PolicyEdge(from_node = nodes[from_node_id], to_node = nodes[to_node_id], guard_action = new_action_instance)
            nodes[from_node_id].add_edge(new_edge)
            if to_node_id != from_node_id:
                nodes[to_node_id].add_edge(new_edge)



        #print(nodes)
        #Create Policy/Plan with the collected information
        policy = Policy(nodes, is_plan = True)
        return policy
#TODO
    @classmethod
    def build_policy_from_networkx_digraph(cls, networkx_digraph : networkx.classes.digraph.DiGraph):
        
        actions = {}

        
        # where EQUAL tells if we should have an equality or not
        predicates = {}
        nodes = {}

        for node_data in networkx_digraph.nodes.data():
            #Save the predicates as a dictionary entry PREDICATE_NAME -> (PREDICATE_VALUE, EQUAL) 
            #   where EQUAL means that the predicate value has to be EQUAL to PREDICATE_VALUE (otherwise NOT EQUAL)
            #Save the fluents as tuples (FLUENT_REGISTRY_ENTRY, FLUENT_EXPECTED_VALUE) in the fluent_tuples list
            fluent_tuples = []
            predicates_string = node_data[1]["label"].replace('"{',"").replace('}"', "").strip().split(",")
            for predicate in predicates_string:
                predicate = predicate.strip()
                if predicate.startswith("(not ("):
                    assert predicate.endswith("))")
                    predicate = predicate.replace("(not (","").replace("))","")
                    is_predicate_equality = False
                elif predicate.startswith("("):
                    assert predicate.endswith(")")
                    predicate = predicate.replace("(", "").replace(")", "")
                    is_predicate_equality = True

                #print(predicate)
                predicate_args = predicate.split(" ")
                #print(predicate_args)
                predicate_name = predicate_args[0]
                if len(predicate_args) > 1:
                    predicate_values = predicate_args[1:]
                else:
                    predicate_values = None

                if predicate_name.startswith(FluentRegistry().ITEM_PREFIX) or predicate_name.startswith(FluentRegistry().ITEM_PREFIX.replace("_", "-")) or predicate_name in FluentRegistry():
                    assert predicate_name in FluentRegistry(),"Fluent '"+predicate_name+"' not available in FluentRegistry"
                    fluent_tuples.append((FluentRegistry().get_instance(predicate_name), is_predicate_equality))
#                elif predicate_values is not None:
#                    for value in predicate_values:
#                        assert value in ValueRegistry(), "Value '"+value+"' for predicate '"+predicate_name+"' not available in ValueRegistry"
#                        predicates[predicate_name] = (predicate_values, is_predicate_equality)

            node_id = int(node_data[0])
            nodes[node_id] = PolicyNode(node_id = node_id, fluents = fluent_tuples)

        for edge_data in networkx_digraph.edges.data():
            #print(edge_data[2]["label"])
            from_node_id = int(edge_data[0])
            assert from_node_id in nodes.keys()

            to_node_id = int(edge_data[1])
            assert to_node_id in nodes.keys()

            action_strings = edge_data[2]["label"].strip().replace('"', "").split(" ")
            action_parameter_names = []
            
            action_name = action_strings[0]
#TODO: Actions starting with OBSERVE are just checks on available variables and are automatically mapped to IdleAction
            if len(action_strings)>1:
                for param in action_strings[1:]:
                    action_parameter_names.append(param)

            #print(action_name)
            new_action_instance : RegistryItem = ActionRegistry().create_action_from_template(action_template_name = action_name, parameter_list = action_parameter_names)
            
            new_edge = PolicyEdge(from_node = nodes[from_node_id], to_node = nodes[to_node_id], guard_action = new_action_instance)
            nodes[from_node_id].add_edge(new_edge)
            if to_node_id != from_node_id:
                nodes[to_node_id].add_edge(new_edge)

        #print(nodes)
        #Create Policy/Plan with the collected information
        policy = Policy(nodes)
        return policy

    @classmethod
    def build_from_FOND_PDDL(cls, domain_path, problem_path, output_dir):

        planner = MyNDPlanner()
        plan = planner.plan(
            domain_path = Path(domain_path), 
            problem_path = Path(problem_path),
            working_dir= Path(output_dir)
        )

        #for node_data in plan.nodes.data():
        #    print(node_data)
        #print(plan.nodes.data())

        #for edge_data in plan.edges.data():
        #    print(edge_data)
        #print(plan.edges.data())

        return Policy.build_policy_from_networkx_digraph(plan)

    @classmethod
    def build_from_FOND_PDDL_and_PLTLf_formula_with_fond4ltlfpltlf(cls, domain_path, problem_path, output_dir, formula, mapping_path = None):


        try:
            with open(domain_path, "r") as d:
                pddl_domain = d.read()
            with open(problem_path, "r") as p:
                pddl_problem = p.read()
        except Exception:
            raise IOError(
                "[ERROR]: Something wrong occurred while parsing the domain and problem."
            )

        compiled_domain, compiled_problem = execute(pddl_domain, pddl_problem, formula)

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        try:
            with open(Path(output_dir) / "compiled_pddl" / "new_domain.pddl", "w+") as dom:
                dom.write(str(compiled_domain))
            with open(Path(output_dir) / "compiled_pddl" / "new_problem.pddl", "w+") as prob:
                prob.write(str(compiled_problem))
        except Exception:
            raise IOError(
                "[ERROR]: Something wrong occurred while writing the compiled domain and problem."
            )
        
        if not os.path.exists(os.path.join(output_dir, "compiled_pddl")):
            os.makedirs(os.path.join(output_dir, "compiled_pddl"))

        planner = MyNDPlanner()
        plan = planner.plan(
            Path(output_dir) / "compiled_pddl" / "new_domain.pddl", 
            Path(output_dir) / "compiled_pddl" / "new_problem.pddl"
        )

        return Policy.build_policy_from_networkx_digraph(plan)
    
    @classmethod
    def build_from_FOND_PDDL_and_PLTLf_formula(cls, domain_path, problem_path, output_dir, formula, mapping_path = None):

        formula = parse_pltl(formula)

        domain_parser = DomainParser()
        problem_parser = ProblemParser()
        domain = domain_parser(Path(domain_path).read_text())
        problem = problem_parser(Path(problem_path).read_text())
        
        mapping = None
        if mapping_path is not None:
            mapping = lib.utils.mapping_parser(Path(mapping_path).read_text(), formula)

        compiler = Compiler(domain, problem, formula, from_atoms_to_fluent=mapping)
        compiler.compile()
        compiled_domain, compiled_problem = compiler.result

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        try:
            with open(Path(output_dir) / "compiled_pddl" / "new_domain.pddl", "w+") as dom:
                dom.write(domain_to_string(compiled_domain))
            with open(Path(output_dir) / "compiled_pddl" / "new_problem.pddl", "w+") as prob:
                prob.write(problem_to_string(compiled_problem))
        except Exception:
            raise IOError(
                "[ERROR]: Something wrong occurred while writing the compiled domain and problem."
            )

        if not os.path.exists(os.path.join(output_dir, "compiled_pddl")):
            os.makedirs(os.path.join(output_dir, "compiled_pddl"))

        planner = MyNDPlanner()
        plan = planner.plan(
            Path(output_dir) / "compiled_pddl" / "new_domain.pddl", 
            Path(output_dir) / "compiled_pddl" / "new_problem.pddl"
        )

        return Policy.build_policy_from_networkx_digraph(plan)
    
    @classmethod
    def build_from_PDDL_and_PLTLf_formula_with_fond4ltlfpltlf(cls, domain_path, problem_path, output_dir, formula, mapping_path = None):


        try:
            with open(domain_path, "r") as d:
                pddl_domain = d.read()
            with open(problem_path, "r") as p:
                pddl_problem = p.read()
        except Exception:
            raise IOError(
                "[ERROR]: Something wrong occurred while parsing the domain and problem."
            )

        compiled_domain, compiled_problem = execute(pddl_domain, pddl_problem, formula)

        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        try:
            with open(Path(output_dir) / "compiled_pddl" / "new_domain.pddl", "w+") as dom:
                dom.write(str(compiled_domain))
            with open(Path(output_dir) / "compiled_pddl" / "new_problem.pddl", "w+") as prob:
                prob.write(str(compiled_problem))
        except Exception:
            raise IOError(
                "[ERROR]: Something wrong occurred while writing the compiled domain and problem."
            )
        
        if not os.path.exists(os.path.join(output_dir, "compiled_pddl")):
            os.makedirs(os.path.join(output_dir, "compiled_pddl"))

        planner = DownwardPlanner()
        plan = planner.plan(
            domain_path = Path(domain_path), 
            problem_path = Path(problem_path)
        ).graph

        return Policy.build_policy_from_networkx_digraph(plan)
    

    @classmethod
    def build_from_PDDL(cls, domain_path, problem_path, output_dir):

        planner = DownwardPlanner()
        plan = planner.plan(
            domain_path = Path(domain_path), 
            problem_path = Path(problem_path)
        ).graph

        return Policy.build_plan_from_networkx_digraph(plan)


#TODO: Implement dfa printing (might also implement a method for a graphical render of the Policy/Plan)
    def __str__(self):
        dfa_string = ""
        for node_id, state in self.nodes.items():
            dfa_string += str(state)
        return dfa_string

    def __repr__(self):
        return str(self)

    
    def plot(self, save_to : str, show_plot = False, chosen_layout = "dot"): #neato, dot, twopi, circo, fdp, nop, wc, acyclic, gvpr, gvcolor, ccomps, sccmap, tred, sfdp, unflatten.

        G=pgv.AGraph(directed=True)

        #Attributes can be added when adding nodes or edge
        for node in self.nodes.values():
            G.add_node(str(node.node_id), color='black')
        
        for edge in self.edges:
            #print(edge.get_label_string())
            G.add_edge(edge.from_node.node_id, edge.to_node.node_id, color='black', label=edge.get_label_string())

        #create a png file
        assert isinstance(save_to, str)
        if save_to is not None:
            if not os.path.exists(os.path.dirname(save_to)):
                os.makedirs(os.path.dirname(save_to))
        
            #plt.savefig(fname=save_to, dpi = 600)
            G.write(save_to.replace(".png", "."+chosen_layout))
            G.layout(prog=chosen_layout) # use dot
            G.draw(save_to.replace("."+chosen_layout, ".png"))
        