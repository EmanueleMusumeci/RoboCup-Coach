#https://stackoverflow.com/questions/46980637/importing-dynamically-all-modules-from-a-folder
import os

dir = os.path.dirname(os.path.abspath(__file__))
modules = {}
for _experiments_dir in os.listdir(dir):
    if not _experiments_dir.startswith('__'):
        #print(_experiments_dir)
        modules[_experiments_dir] = []
        for _file  in os.listdir(os.path.join(dir, _experiments_dir)):
            if not _file.startswith('__') and _file.endswith(".py"):
                modules[_experiments_dir].append(os.path.splitext(_file)[0])
    
#print(modules)

experiments = []
import_string = ""
for dir, modules in modules.items():
    import_string += 'from experiments import {}; '.format(dir)
    assert " " not in dir, "Please rename '"+dir+"' to '"+dir.replace(" ", "_")+"'"
    for module_name in modules:
        assert " " not in module_name, "Please rename '"+module_name+"' to '"+module_name.replace(" ", "_")+"'"
        import_string += 'from experiments.{} import {}; '.format(dir, module_name)
        import_string += 'experiments.append({}); '.format(module_name)
exec(import_string)