import os
from pathlib import Path
import math
import json
from typing import List

import matplotlib
from matplotlib import pyplot as plt

from lib.utils import mapping_parser
from lib.plan.policy import Policy

from robocup_spl_temporal_goals.third_party.planning_with_past.planners.mynd.base import MyNDPlanner
from robocup_spl_temporal_goals.third_party.planning_with_past.planners.downward import DownwardPlanner
from robocup_spl_temporal_goals.third_party.pddl.pddl.parser.domain import DomainParser
from robocup_spl_temporal_goals.third_party.pddl.pddl.parser.problem import ProblemParser
from robocup_spl_temporal_goals.third_party.planning_with_past.compiler import Compiler
from robocup_spl_temporal_goals.third_party.pddl.pddl.formatter import domain_to_string, problem_to_string

from pylogics.parsers import parse_pltl

def generate_position_once_constraint(position_predicate_token, constrained_entity, waypoint_symbol, x, y):
    return "O("+position_predicate_token+" "+constrained_entity+" "+get_waypoint_string(waypoint_symbol, x, y)+")"

def get_waypoint_string(waypoint_symbol, x, y):
    return waypoint_symbol+"c"+str(x)+"r"+str(y)

#Generate waypoints matrix ([columns][rows])
def get_waypoints_matrix(waypoint_str, waypoints_x, waypoints_y):
    
    waypoints_matrix = []
    for x in range(waypoints_x):
        #Append new column to matrix
        column = []
        for y in range(waypoints_y):
            waypoint_name = get_waypoint_string(waypoint_str, x, y)
            column.append(waypoint_name)
        waypoints_matrix.append(column)
    
    return waypoints_matrix

def generate_adjacency_reticle_string(adjacency_predicate_str, waypoint_str, waypoints_x, waypoints_y, allow_diagonal_adjacency, include_symmetric_adjacency_predicates = True):
    
    #Generate waypoints matrix ([columns][rows])
    waypoints_matrix = get_waypoints_matrix(waypoint_str, waypoints_x, waypoints_y)
    
    result = ""
    #Generate adjacency predicates matrix (matrix of lists, one list for each waypoint)
    for col_idx, column in enumerate(waypoints_matrix):
        for row_idx, _ in enumerate(column):

            #Left
            if col_idx>0:
                result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx-1][row_idx]+")\n"
            #Top
            if row_idx>0:
                result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx][row_idx-1]+")\n"
            
            if include_symmetric_adjacency_predicates:
                #Right
                if col_idx<waypoints_y-1:
                    result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx+1][row_idx]+")\n"
                
                #Bottom
                if row_idx<waypoints_x-1:
                    result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx][row_idx+1]+")\n"
    
            if allow_diagonal_adjacency:
                #Top-Left
                if row_idx>0 and col_idx>0:
                    result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx-1][row_idx-1]+")\n"
                #Top-Right
                if row_idx>0 and col_idx<waypoints_y-1:
                    result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx+1][row_idx-1]+")\n"
                
                if include_symmetric_adjacency_predicates:
                    #Bottom-Left
                    if row_idx<waypoints_x-1 and col_idx>0:
                        result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx-1][row_idx+1]+")\n"
                    
                    #Bottom-Right
                    if row_idx<waypoints_x-1 and col_idx<waypoints_y-1:
                        result+="("+adjacency_predicate_str+" "+waypoints_matrix[col_idx][row_idx]+" "+waypoints_matrix[col_idx+1][row_idx+1]+")\n"

    return result

def get_initial_robot_ball_position(position_predicate, waypoint_symbol, max_waypoint_y):
    robot_position_predicate = "("+position_predicate + " robot " + waypoint_symbol+"c0r" + str(int(math.floor(max_waypoint_y/2))) + ")\n"
    ball_position_predicate = "("+position_predicate + " ball " + waypoint_symbol+"c1r" + str(int(math.floor(max_waypoint_y/2))) + ")\n"
    return robot_position_predicate + " " + ball_position_predicate

def get_goal_scored_condition(position_predicate, waypoint_symbol, max_waypoint_x, max_waypoint_y):
    return "("+position_predicate + " ball " + waypoint_symbol+"c" + str(max_waypoint_x-1) + "r" + str(int(math.floor(max_waypoint_y/2))) + ")\n"

def benchmark_conditioned_FOND_policy_over_generated_problems_with_fixed_constraints(benchmark_name : str, benchmark_generation_data : dict, benchmark_name_to_additional_constraints : dict):

    time_benchmarks = {}
            
    assert "pddl_domain_path" in benchmark_generation_data.keys()        
    assert "pddl_generated_problems_dir" in benchmark_generation_data.keys()               
    assert "working_dir" in benchmark_generation_data.keys()        

    domain_path = benchmark_generation_data["pddl_domain_path"]
    generated_problems_dir = benchmark_generation_data["pddl_generated_problems_dir"]
    working_dir = benchmark_generation_data["working_dir"]



    

    domain_parser = DomainParser()
    problem_parser = ProblemParser()
    domain = domain_parser(Path(domain_path).read_text())

    assert benchmark_name in os.listdir(generated_problems_dir), "No generated problems directory available for benchmark "+benchmark_name

    for generated_problem_file in os.listdir(os.path.join(generated_problems_dir, benchmark_name)):

        constraints_string = ""
        if benchmark_name in benchmark_name_to_additional_constraints:
            for constraint in benchmark_name_to_additional_constraints[benchmark_name]:
                constraints_string += " && "
                constraints_string += constraint.replace(" ", "_")

        if generated_problem_file.endswith(".json"):
            continue
    
        generated_problem_data_file_path = os.path.join(generated_problems_dir, benchmark_name, generated_problem_file).replace("pddl", "json")
        with open(generated_problem_data_file_path, "r") as generated_problem_data_file:
            generated_problem_json_data = json.loads(generated_problem_data_file.read())

            #Add benchmark-specific additional constraints if present
            if "additional_constraints" in generated_problem_json_data.keys():
                for constraint in generated_problem_json_data["additional_constraints"]:
                    constraints_string += " && "
                    #Preprocess constraint for compiler
                    constraints_string += constraint.replace(" ", "_")

            #Preprocess goal for compiler
            formula = generated_problem_json_data["base_goal_condition"].replace(" ", "_")

            #Add constraints to goal
            formula += constraints_string

            waypoints_x = generated_problem_json_data["waypoints_x"]
        
        #print(generated_problem_json_data["additional_constraints"])
        #print(formula)
        formula = parse_pltl(formula)
        problem = problem_parser(Path(os.path.join(generated_problems_dir, benchmark_name, generated_problem_file)).read_text())
        
        compiler = Compiler(domain, problem, formula)
        compiler.compile()
        compiled_domain, compiled_problem = compiler.result

        if not os.path.exists(working_dir):
            os.makedirs(working_dir)

        if not os.path.exists(os.path.join(working_dir, "compiled_pddl")):
            os.makedirs(os.path.join(working_dir, "compiled_pddl"))

        try:
            with open(Path(working_dir) / "compiled_pddl" / ("compiled_domain_"+os.path.splitext(generated_problem_file)[0]+".pddl"), "w+") as dom:
                dom.write(domain_to_string(compiled_domain))
            with open(Path(working_dir) / "compiled_pddl" / ("compiled_problem_"+os.path.splitext(generated_problem_file)[0]+".pddl"), "w+") as prob:
                prob.write(problem_to_string(compiled_problem))
        except Exception as e:
            raise e
            raise IOError(
                "[ERROR]: Something wrong occurred while writing the compiled domain and problem."
            )

        if not os.path.exists(os.path.join(working_dir, "compiled_pddl")):
            os.makedirs(os.path.join(working_dir, "compiled_pddl"))

        planner = MyNDPlanner()
        
        planning_time = planner.plan(
            Path(working_dir) / "compiled_pddl" / ("compiled_domain_"+os.path.splitext(generated_problem_file)[0]+".pddl"), 
            Path(working_dir) / "compiled_pddl" / ("compiled_problem_"+os.path.splitext(generated_problem_file)[0]+".pddl"),
            working_dir=Path(working_dir),
            output_policy_file_name=os.path.splitext(generated_problem_file)[0],
            time_benchmark_mode=True
        )

        #policy = Policy.build_policy_from_networkx_digraph(plan)
        #policy.plot(save_to=os.path.join(working_dir, "plan_preview", os.path.splitext(generated_problem_file)[0]+".png"), show_plot = False)

        time_benchmarks[waypoints_x] = planning_time

    #Sort by key
    time_benchmarks = dict(sorted(time_benchmarks.items()))
    return time_benchmarks

def plot(plot_title, plots_values : dict, subtitle : str = None, plot_colors : dict = None, x_axis_label = "", y_axis_label = "", save_to_dir = None, show = False, prevent_overlaying_tick_labels = True):
    
    plt.clf()

    ax = plt.axes()
    
    if subtitle is not None:
        plt.title(plot_title + "\n"+subtitle)
    else:
        plt.title(plot_title)

    plots_values_x = [] 
    plots_values_y = [] 
    for x, y in plots_values.items():
        plots_values_x.append(x)
        plots_values_y.append(y)
    
    xticks = list(range(len(plots_values_x))[::5]) + [len(plots_values_x)-1]
    xticklabels = plots_values_x[::5] + [plots_values_x[-1]]
    if prevent_overlaying_tick_labels and len(xticks) > 1 and abs(xticks[-2] - xticks[-1]) < 2:
        xticks.pop(-2)
        xticklabels.pop(-2)

    plt.plot(plots_values_x, plots_values_y)

    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels)

    if x_axis_label:
        plt.xlabel(x_axis_label)

    if y_axis_label:
        plt.ylabel(y_axis_label)

    if show:
        plt.show()
    
    if save_to_dir is not None:
        plt.savefig(os.path.join(save_to_dir, plot_title+" "+subtitle), dpi = 300)


def multi_plot(plot_title, plots_values : dict, subtitle : str = None, plot_colors : dict = None, x_axis_label = "", y_axis_label = "", save_to_dir = None, show = False, prevent_overlaying_tick_labels = True, plot_style : List[str] = None, font_size = 15):
    

    plt.clf()

    ax = plt.axes()
    
    if subtitle is not None:
        plt.title(plot_title + "\n"+subtitle, fontsize=font_size)
    else:
        plt.title(plot_title, fontsize=font_size)

    if plot_style is not None:
        assert len(plot_style) == len(plots_values.keys())

    plt.rcParams.update({'font.size': font_size})

    for i, (plot_name, plot_values) in enumerate(plots_values.items()):
        plots_values_x = [] 
        plots_values_y = [] 
        for x, y in plot_values.items():
            plots_values_x.append(x)
            plots_values_y.append(y)

        xticks = list(range(len(plots_values_x))[::5]) + [len(plots_values_x)-1]
        xticklabels = plots_values_x[::5] + [plots_values_x[-1]]
        if prevent_overlaying_tick_labels and len(xticks) > 1 and abs(xticks[-2] - xticks[-1]) < 2:
            del xticks[-2]
            del xticklabels[-2]
            
        plt.plot(plots_values_x, plots_values_y, "k"+plot_style[i])

    ax.set_xticks(xticks)
    ax.set_xticklabels(xticklabels, fontsize=font_size)

    if x_axis_label:
        plt.xlabel(x_axis_label, fontsize=font_size)

    if y_axis_label:
        plt.ylabel(y_axis_label, fontsize=font_size)

    plt.legend(list(plots_values.keys()), prop={'size': font_size})

    if show:
        plt.show()
    
    if save_to_dir is not None:
        plt.savefig(os.path.join(save_to_dir, plot_title+" "+subtitle), dpi = 300)