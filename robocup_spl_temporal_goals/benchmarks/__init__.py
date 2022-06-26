#https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
import os

dir = os.path.dirname(os.path.abspath(__file__))
benchmark_modules = []
for _benchmarks_dir in os.listdir(dir):
    if not _benchmarks_dir.startswith('__'):
        #print(_benchmarks_dir)
        if not "setup.py" in os.listdir(os.path.join(dir, _benchmarks_dir)):
            print("Benchmark directory %s does not provide a 'setup.py' file")
            continue
        else:
            benchmark_modules.append(_benchmarks_dir)
#print(modules)

benchmarks = []
import_string = ""
for dir in benchmark_modules:
    import_string += 'from benchmarks import {}; '.format(dir)
    assert " " not in dir, "Please rename '"+dir+"' to '"+dir.replace(" ", "_")+"'"
    import_string += 'from benchmarks.{} import setup; '.format(dir)
    import_string += 'benchmarks.append({}.setup); '.format(dir)
exec(import_string)