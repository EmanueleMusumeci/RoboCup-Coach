import argparse
import sys, os
import inspect

from lib.utils import find_similar_strings
from communication.setup_env import setup
import benchmarks
from lib.plan.policy_handler import PolicyHandler
from lib.dfa.dfa_handler import DFAHandler

from lib.constraints import ask_additional_constraints

#This module performs the planning time benchmarks for the benchmark scenarios in the benchmarks folder


if __name__ == "__main__":
    #https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
    dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "benchmarks")
    sys.path.append(dir)

    from benchmarks import benchmarks
    loaded_benchmarks = {str(benchmark_module.__name__): benchmark_module for benchmark_module in benchmarks}
    
    parser = argparse.ArgumentParser(description='Run a benchmark (which has to be contained in a subdirectory inside the "benchmarks" directory).')
    parser.add_argument('benchmark', type=str, help='Use to specify the benchmark name in the format "<benchmark_subfolder>.<benchmark_name>". The name has to refer to a "<benchmark_name>.py" file of the same name inside the subdirectory "<benchmark_subfolder>" of the "benchmarks" folder.')    
    parser.add_argument('--ask_constraints', '-c', action="store_true", help='Use to require constraints in advance for each benchmark for interactive behavior conditioning through PLTLf constraints.')
    parser.add_argument('--perform_benchmarks', '-b', action="store_true", help='Use to perform benchmarks')
    parser.add_argument('--generate_plots', '-p', action="store_true", help='Use to generate plot according to the data returned by the benchmark')

    args = parser.parse_args()
    


    ''' 
    ____________________
    |                   |
    |  LOAD BENCHMARK   |
    |___________________|

    '''

    
    base_benchmark_name = "benchmarks."+args.benchmark
    benchmark_name = base_benchmark_name+".setup"
    
    if benchmark_name not in loaded_benchmarks.keys():
        similar_strings = find_similar_strings(base_benchmark_name, loaded_benchmarks.keys())
        if similar_strings:
            print("Unknown benchmark '%s'. Maybe you meant '%s'?" % (base_benchmark_name, similar_strings[0]))
        else:
            print("Unknown benchmark '%s'." % (base_benchmark_name))
        exit()
    
    
    chosen_benchmark = loaded_benchmarks[benchmark_name]

    print(benchmark_name)
    print(chosen_benchmark)

    #Check that the benchmark module has all necessary functions
    assert hasattr(chosen_benchmark, "perform_benchmarks")
    assert hasattr(chosen_benchmark, "generate_problems")
    assert hasattr(chosen_benchmark, "get_base_problem_name")
    assert hasattr(chosen_benchmark, "base_benchmark_name_to_benchmark_generation_data")
    #assert hasattr(chosen_benchmark, "initialize_registries")

    #Check that the module has a get_robot_formation method
    robot_formation = chosen_benchmark.get_robot_formation()

    #Initialize registries with correct variables
    #chosen_benchmark.initialize_registries()

    additional_constraints = {}
    #Ask for additional constraints to the goal of each robot
    if args.ask_constraints:        
        additional_constraints = ask_additional_constraints(benchmark_name_to_generation_data = chosen_benchmark.base_benchmark_name_to_benchmark_generation_data())

    assert args.perform_benchmarks or args.generate_plots, "Select at least one option between --perform_benchmarks and --generate_plots"

    currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
    
    if args.perform_benchmarks:
        for benchmark_name, benchmark_generation_data in chosen_benchmark.base_benchmark_name_to_benchmark_generation_data().items():
            plan_handlers = chosen_benchmark.perform_benchmarks(benchmark_name, benchmark_generation_data, additional_constraints, save_to_dir = os.path.join(currentdir, "benchmarks", chosen_benchmark.get_base_problem_name()))

    if args.generate_plots:
        chosen_benchmark.plot_benchmarks(os.path.join(currentdir, "benchmarks", chosen_benchmark.get_base_problem_name()))
    

            