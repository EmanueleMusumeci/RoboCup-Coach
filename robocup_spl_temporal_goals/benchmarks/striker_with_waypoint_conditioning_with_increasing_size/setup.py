import os,inspect
import shutil
from typing import Dict, List
import json
import math

from lib.experiment import ExperimentType, check_adjacency_ready_problem
from lib.benchmarks import generate_adjacency_reticle_string, generate_position_once_constraint, benchmark_conditioned_FOND_policy_over_generated_problems_with_fixed_constraints, get_goal_scored_condition, get_initial_robot_ball_position, get_waypoints_matrix, multi_plot, plot

from matplotlib import pyplot as plt

GENERATED_PROBLEMS_FOLDER = "generated_problems"

def get_experiment_type():
    return ExperimentType.POLICY

def get_base_problem_name():
    return "striker_with_waypoint_conditioning_with_increasing_size"

def get_robot_formation():
    return {3 : "Caligola"}

def base_benchmark_name_to_benchmark_generation_data():
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    parentdir = os.path.dirname(currentdir)

    return {
        "without_diagonal_adjacency_with_position_constraints" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain_with_adjacency.pddl"),
            "pddl_base_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem_with_adjacency_template.pddl"),
            "pddl_generated_problems_dir" : os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER),
            "working_dir" : os.path.join(currentdir, "output"),

            "generation_parameters" : {
                "min_x" : 4,
                "max_x" : 35,
                "min_y" : 3,
                "max_y" : 3,
                "allow_diagonal_adjacency" : False,
                "include_symmetric_adjacency_predicates" : False,
                "middle_row_position_constraint" : True,
            },
            "constrainable_predicates" : {
                "isat" : [
                    "at",
                    "is at",
                    "in"
                ]
            }
        },
        "with_diagonal_adjacency_with_position_constraints" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain_with_adjacency.pddl"),
            "pddl_base_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem_with_adjacency_template.pddl"),
            "pddl_generated_problems_dir" : os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER),
            "working_dir" : os.path.join(currentdir, "output"),

            "generation_parameters" : {
                "min_x" : 4,
                "max_x" : 35,
                "min_y" : 3,
                "max_y" : 3,
                "allow_diagonal_adjacency" : True,
                "include_symmetric_adjacency_predicates" : False,
                "middle_row_position_constraint" : True,
            },
            "constrainable_predicates" : {
                "isat" : [
                    "at",
                    "is at",
                    "in"
                ]
            }
        },
        "without_diagonal_adjacency" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain_with_adjacency.pddl"),
            "pddl_base_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem_with_adjacency_template.pddl"),
            "pddl_generated_problems_dir" : os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER),
            "working_dir" : os.path.join(currentdir, "output"),

            "generation_parameters" : {
                "min_x" : 4,
                "max_x" : 35,
                "min_y" : 3,
                "max_y" : 3,
                "allow_diagonal_adjacency" : False,
                "include_symmetric_adjacency_predicates" : False,
                "middle_row_position_constraint" : False,
            },
            "constrainable_predicates" : {
                "isat" : [
                    "at",
                    "is at",
                    "in"
                ]
            }
        },
        "with_diagonal_adjacency" : {
            "pddl_domain_path" : os.path.join(currentdir, "PDDL", "striker_domain_with_adjacency.pddl"),
            "pddl_base_problem_path" : os.path.join(currentdir, "PDDL", "striker_problem_with_adjacency_template.pddl"),
            "pddl_generated_problems_dir" : os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER),
            "working_dir" : os.path.join(currentdir, "output"),

            "generation_parameters" : {
                "min_x" : 4,
                "max_x" : 35,
                "min_y" : 3,
                "max_y" : 3,
                "allow_diagonal_adjacency" : True,
                "include_symmetric_adjacency_predicates" : False,
                "middle_row_position_constraint" : False,
            },
            "constrainable_predicates" : {
                "isat" : [
                    "at",
                    "is at",
                    "in"
                ]
            }
        },
    }

#NOTICE: if middle is True, the waypoint constraints will be placed in the middle row of the grid
#else they will be placed at a random location
def generate_additional_position_constraints_in_middle(pddl_position_predicate, constrained_entity, waypoint_symbol, side_x, side_y):
    additional_constraints = []
    
    assert side_x > 3, "Not enough cells in middle row ("+str(side_x)+", should be at least 4)"
    x_coord = int(math.floor(side_x/2))
    y_coord = int(math.floor(side_y/2))

    additional_constraints.append(
        generate_position_once_constraint(pddl_position_predicate, constrained_entity, waypoint_symbol, x_coord, y_coord)
    )
    
    return additional_constraints
    

def generate_problems(square = True, generated_problems_folder = None):
    if square:
        return generate_square_problems(generated_problems_folder = generated_problems_folder)
    else:
        return generate_rectangular_problems(generated_problems_folder = generated_problems_folder)

def generate_rectangular_problems(generated_problems_folder = None, 
    adjacency_predicates_placeholder_token = "ADJACENCY_PREDICATES", 
    initial_position_placeholder_token = "POSITION_PREDICATES", 
    waypoints_placeholder_token = "WAYPOINTS", 
    goal_placeholder_token = "GOAL_CONDITION",
    pddl_waypoint_prefix_symbol = "wp", 
    pddl_adjacency_predicate = "adjacent", 
    pddl_position_predicate = "isat",
    pddl_robot_symbol = "robot"
    ):
    
    generation_data = base_benchmark_name_to_benchmark_generation_data()
    
    if generated_problems_folder is None:
        currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
        generated_problems_dir = os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER)
    else:
        generated_problems_dir = os.path.join(currentdir, generated_problems_folder)

    for benchmark_name, benchmark_generation_data in generation_data.items():
        base_problem_path = benchmark_generation_data["pddl_base_problem_path"]

        benchmark_generated_problems_dir = os.path.join(generated_problems_dir, benchmark_name)
        if not os.path.exists(benchmark_generated_problems_dir):
            os.makedirs(benchmark_generated_problems_dir)
        else:
            shutil.rmtree(benchmark_generated_problems_dir)
            os.makedirs(benchmark_generated_problems_dir)
        
        base_problem_name = get_base_problem_name()

        with open(base_problem_path, mode="r") as base_problem_file:
            #Check that the file is adjacency ready
            assert check_adjacency_ready_problem(base_problem_path)
            base_problem_file_text = base_problem_file.readlines()

            generation_parameters = benchmark_generation_data["generation_parameters"]
            for waypoints_x in range(generation_parameters["min_x"], generation_parameters["max_x"]+1):
                for waypoints_y in range(generation_parameters["min_y"], generation_parameters["max_y"]+1):
                    
                    generated_problem_name = base_problem_name + "__waypoints_x_" + str(waypoints_x) + "_waypoints_y_" + str(waypoints_y)
                    generated_problem_path = os.path.join(benchmark_generated_problems_dir, generated_problem_name)
                    generated_problem_text = ""
                    for line in base_problem_file_text:
                        if adjacency_predicates_placeholder_token in line:
                            line = line.replace(adjacency_predicates_placeholder_token, generate_adjacency_reticle_string(pddl_adjacency_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_y, allow_diagonal_adjacency=generation_parameters["allow_diagonal_adjacency"], include_symmetric_adjacency_predicates=generation_parameters["include_symmetric_adjacency_predicates"]))
                        if initial_position_placeholder_token in line:
                            line = line.replace(initial_position_placeholder_token, get_initial_robot_ball_position(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_y))
                        if waypoints_placeholder_token in line:
                            line = line.replace(waypoints_placeholder_token, " ".join([" ".join(column) for column in get_waypoints_matrix(pddl_waypoint_prefix_symbol, waypoints_x, waypoints_y)]))
                        if goal_placeholder_token in line:
                            line = line.replace(goal_placeholder_token, get_goal_scored_condition(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_y))
                        generated_problem_text += line
                    
                    #Generate additional constraints if required
                    additional_constraints = []
                    if generation_parameters["middle_row_position_constraint"]:
                        additional_constraints = generate_additional_position_constraints_in_middle(pddl_position_predicate, pddl_robot_symbol, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_y, generation_parameters["middle_row_position_constraint"])


                    with open(generated_problem_path+".pddl", mode="w+") as generated_problem_file:
                        generated_problem_file.writelines(generated_problem_text)
                        print("Generated problem file %s\n" % (generated_problem_path + ".pddl"))
                    
                    #Generate JSON file with problem data
                    generated_problem_data = {
                        "generated_problem_file_name" : generated_problem_path+".pddl",
                        "waypoints_x" : waypoints_x,
                        "waypoints_y" : waypoints_y,
                        "diagonal_adjacency" : generation_parameters["allow_diagonal_adjacency"],
                        "include_symmetric_adjacency_predicates" : generation_parameters["include_symmetric_adjacency_predicates"],
                        "base_goal_condition" : get_goal_scored_condition(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_y),
                        "additional_constraints" : additional_constraints,
                    }

                    with open(generated_problem_path+".json", mode="w+") as generated_problem_json_file:
                        generated_problem_json_file.writelines(json.dumps(generated_problem_data))
                        print("Generated problem data file %s\n\n" % (generated_problem_path + ".json"))
    
    return generated_problems_dir, base_problem_name

def generate_square_problems(generated_problems_folder = None, 
    adjacency_predicates_placeholder_token = "ADJACENCY_PREDICATES", 
    initial_position_placeholder_token = "POSITION_PREDICATES", 
    waypoints_placeholder_token = "WAYPOINTS", 
    goal_placeholder_token = "GOAL_CONDITION",
    pddl_waypoint_prefix_symbol = "wp", 
    pddl_adjacency_predicate = "adjacent", 
    pddl_position_predicate = "isat",
    pddl_robot_symbol = "robot"
    ):
    
    generation_data = base_benchmark_name_to_benchmark_generation_data()

    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))    
    if generated_problems_folder is None:
        generated_problems_dir = os.path.join(currentdir, GENERATED_PROBLEMS_FOLDER)
    else:
        generated_problems_dir = os.path.join(currentdir, generated_problems_folder)

    for benchmark_name, benchmark_generation_data in generation_data.items():
        base_problem_path = benchmark_generation_data["pddl_base_problem_path"]

        benchmark_generated_problems_dir = os.path.join(generated_problems_dir, benchmark_name)
        if not os.path.exists(benchmark_generated_problems_dir):
            os.makedirs(benchmark_generated_problems_dir)
        else:
            shutil.rmtree(benchmark_generated_problems_dir)
            os.makedirs(benchmark_generated_problems_dir)
        
        base_problem_name = get_base_problem_name()

        with open(base_problem_path, mode="r") as base_problem_file:
            #Check that the file is adjacency ready
            assert check_adjacency_ready_problem(base_problem_path)
            base_problem_file_text = base_problem_file.readlines()

            generation_parameters = benchmark_generation_data["generation_parameters"]
            for waypoints_x in range(generation_parameters["min_x"], generation_parameters["max_x"]+1):
                    
                generated_problem_name = base_problem_name + "__waypoints_x_" + str(waypoints_x) + "_waypoints_y_" + str(waypoints_x)
                generated_problem_path = os.path.join(benchmark_generated_problems_dir, generated_problem_name)
                generated_problem_text = ""
                for line in base_problem_file_text:
                    if adjacency_predicates_placeholder_token in line:
                        line = line.replace(adjacency_predicates_placeholder_token, generate_adjacency_reticle_string(pddl_adjacency_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_x, allow_diagonal_adjacency=generation_parameters["allow_diagonal_adjacency"], include_symmetric_adjacency_predicates=generation_parameters["include_symmetric_adjacency_predicates"]))
                    if initial_position_placeholder_token in line:
                        line = line.replace(initial_position_placeholder_token, get_initial_robot_ball_position(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_x))
                    if waypoints_placeholder_token in line:
                        line = line.replace(waypoints_placeholder_token, " ".join([" ".join(column) for column in get_waypoints_matrix(pddl_waypoint_prefix_symbol, waypoints_x, waypoints_x)]))
                    if goal_placeholder_token in line:
                        line = line.replace(goal_placeholder_token, get_goal_scored_condition(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_x))
                    generated_problem_text += line
                
                #Generate additional constraints if required
                additional_constraints = []
                if generation_parameters["middle_row_position_constraint"]:
                    additional_constraints = generate_additional_position_constraints_in_middle(pddl_position_predicate, pddl_robot_symbol, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_x)


                with open(generated_problem_path+".pddl", mode="w+") as generated_problem_file:
                    generated_problem_file.writelines(generated_problem_text)
                    print("Generated problem file %s\n" % (generated_problem_path + ".pddl"))
                
                #Generate JSON file with problem data
                generated_problem_data = {
                    "generated_problem_file_name" : generated_problem_path+".pddl",
                    "waypoints_x" : waypoints_x,
                    "waypoints_y" : waypoints_x,
                    "diagonal_adjacency" : generation_parameters["allow_diagonal_adjacency"],
                    "include_symmetric_adjacency_predicates" : generation_parameters["include_symmetric_adjacency_predicates"],
                    "base_goal_condition" : get_goal_scored_condition(pddl_position_predicate, pddl_waypoint_prefix_symbol, waypoints_x, waypoints_x),
                    "additional_constraints" : additional_constraints,
                }

                with open(generated_problem_path+".json", mode="w+") as generated_problem_json_file:
                    generated_problem_json_file.writelines(json.dumps(generated_problem_data))
                    print("Generated problem data file %s\n\n" % (generated_problem_path + ".json"))
    
    return generated_problems_dir, base_problem_name

def plot_benchmarks(save_to_dir : str):
    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe()))) 
    
    benchmarks = {}
    for benchmark_name, benchmark_generation_data in base_benchmark_name_to_benchmark_generation_data().items():
        for filename in os.listdir(currentdir):
            filepath = os.path.join(currentdir, filename)
            if os.path.isfile(filepath) and filename.endswith("benchmarks.json") and filename.split("_benchmarks.json")[0] == benchmark_name:
                with open(filepath, "r") as benchmarks_file:
                    benchmarks[benchmark_name] = json.loads(benchmarks_file.read())
                    print(filename)
                    #print(benchmarks[benchmark_name])

    plot("'Basic striker' scenario with increasing planning depth", benchmarks["with_diagonal_adjacency"], subtitle = "(with diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir)
    plot("'Basic striker' scenario with increasing planning depth", benchmarks["without_diagonal_adjacency"], subtitle = "(without diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir)
    plot("'Basic striker' scenario with increasing planning depth and position constraint", benchmarks["with_diagonal_adjacency_with_position_constraints"], subtitle = "(with diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir)
    plot("'Basic striker' scenario with increasing planning depth and position constraint", benchmarks["without_diagonal_adjacency_with_position_constraints"], subtitle = "(without diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir)

    
    benchmarks["with_diagonal_adjacency"].pop("3")
    multi_plot("Comparison of benchmarks at increasing planning depth", 
        {
            "w. position constraint" : benchmarks["with_diagonal_adjacency_with_position_constraints"],
            "w/o position constraint" : benchmarks["with_diagonal_adjacency"]
        }, subtitle="(with diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir, plot_style=["--", ":"])

    benchmarks["without_diagonal_adjacency"].pop("3")
    multi_plot("Comparison of benchmarks at increasing planning depth", 
        {
            "w. position constraint" : benchmarks["without_diagonal_adjacency_with_position_constraints"],
            "w/o position constraint" : benchmarks["without_diagonal_adjacency"]
        }, subtitle="(without diagonal adjacency)", x_axis_label="Planning depth", y_axis_label="Planning time [s]", save_to_dir = save_to_dir, plot_style=["--",":"])
    
def perform_benchmarks(benchmark_name, benchmark_generation_data, benchmark_name_to_additional_constraints : Dict[str, List[str]] = {}, save_to_dir : str = None):
    problems_dir, problems_base_name = generate_problems()

    time_benchmarks = benchmark_conditioned_FOND_policy_over_generated_problems_with_fixed_constraints(benchmark_name, benchmark_generation_data, benchmark_name_to_additional_constraints = benchmark_name_to_additional_constraints)
    if save_to_dir is not None:
        if benchmark_name is None:
            benchmark_name = "benchmarks.json"        
        with open(os.path.join(save_to_dir, benchmark_name)+"_benchmarks.json", mode="w+") as f:
            f.write(json.dumps(time_benchmarks))
    return time_benchmarks

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
    ValueRegistry()["kickingposition"] = (2000, 0)
    ValueRegistry()["goaltarget"] = (3000, 0)
    #Register aliases to map objects in the domain to actual values (not necessarily already in the registry)
    ValueRegistry().register_alias(item_name="striker_position", alias_name="strikercurrentposition")
    ValueRegistry().register_alias(item_name="last_ball_position", alias_name="ballcurrentposition")
