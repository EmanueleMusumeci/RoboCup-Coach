from lib2to3.pgen2.token import AT
from typing import Type, List, Dict, Any, Tuple, Callable, Union
import uuid

from numpy import isin

from lib.registries.registry import ParameterRegistry
from lib.registries.values import ValueRegistry, Value, SimpleValue, FunctionalValue
from lib.misc.singleton import Singleton
from lib.registries.registry import ConstantRegistryItem, Registry, RegistryItem, SimpleRegistryItem

#Allows specifying an action template, containing its base name in the registry, the robot_skill_name and (optionally) the parameter indices
# that need to be selected from the parameter list passed to the action upon construction
class ActionTemplate:
    def __init__(self, name_in_registry : str, robot_skill_name : str, parameters : List[Union[int,str]] = None):
        
        if parameters is not None:
            assert isinstance(parameters, list)
            for param in parameters:
                assert isinstance(param, int) or isinstance(param, str)

        self.parameters = parameters
        assert isinstance(robot_skill_name, str)

        self.robot_skill_name = robot_skill_name
        self.name_in_registry = name_in_registry

    def __str__(self):
        return "[ActionTemplate]%s(robot_skill_name='%s', parameter_indices=%s)" %(self.name_in_registry, self.robot_skill_name, (str(self.parameter_indices) if self.parameter_indices is not None else "ALL"))
    
    def __repr__(self):
        return str(self)

class Action:
    def __init__(self, name : str, parameters = []):
        for parameter in parameters:
            if isinstance(parameter, str):
                continue
            else:
                assert isinstance(parameter, tuple)
                assert len(parameter) == 2
                if isinstance(parameter[1], RegistryItem):
                    assert isinstance(parameter[1], Value)
                else:
#TODO: at some point, constant action arguments will have to be automatically turned into ValueRegistry items, 
#       but for now we allow native types only if they are the same ones supported by SimpleValues or FunctionalValues
                    assert type(parameter[1]) in [bool, float, int, str, Callable]

        self.parameters = parameters
        self.base_name = name
        self.completed = False
    
    def set_action_completed(self):
        self.completed = True

    def get_parameter_string(self):
        found_parameters = {}
        #print(self.parameters)
        for parameter in self.parameters:
            if isinstance(parameter, list):
                raise NotImplementedError
            elif isinstance(parameter, tuple):
                param_name = parameter[0]

                if isinstance(parameter[1], RegistryItem):
                    param_value = parameter[1].get()
                else:
                    param_value = parameter[1]
                
                if isinstance(param_value, str):
                    found_parameters[param_name] = "'"+param_value+"'" 
                elif isinstance(param_value, RegistryItem):
                    found_parameters[param_name] = param_value.get()
                else:
                    found_parameters[param_name] = param_value
            else:
                assert isinstance(parameter, str)
                param_name = parameter
                if param_name in ValueRegistry():
                    param_value = ValueRegistry().get_instance(param_name).get()
                    found_parameters[param_name] = param_value
                else:
                    raise KeyError("Can't currently find parameter '%s' (for action %s) in any registry! Delayed action parameter check is %s \n" % (param_name, self.name_in_registry, ("ON, so check if the parameter has yet to be added" if self.registry_instance.allow_delayed_parameter_check else "OFF")))

#TODO: finish here: it should retrieve the parameters from their registry given their name ("parameter")
# or raise an exception if they're not yet available (this action is still scheduled for check)
#then correct all other parameter uses to also include the case in which the parameter is only a string (parameter name in registry) and not a tuple
                

        param_substrings = []
        expected_number_of_parameters = len(self.parameters)
        for i, (param_name, parameter_data) in enumerate(found_parameters.items()):
            if isinstance(parameter_data, list):
                raise NotImplementedError
            elif isinstance(parameter_data, tuple):
                if len(parameter_data) > 1:
                    expected_number_of_parameters += len(parameter_data) - 1
                for j, value in enumerate(parameter_data):
                    param_substrings.append(param_name+"_"+ str(j) + ":" + str(value)+","+ type(value).__name__)
            else:
                param_type = type(parameter_data).__name__
                param_substrings.append(param_name + ":" + str(parameter_data)+","+ param_type)
                

        return "/".join(param_substrings)

    def is_idle_action(self):
        return self.get() == ActionRegistry().get_idle_action().get()


class ConstantParameterAction(ConstantRegistryItem, Action):
    def __init__(self, name_in_registry : str, robot_skill_name : str, registry_instance : Type["ActionRegistry"], parameters : List[Tuple[str, Any]] = [], base_name : str = None):
        if base_name is not None:
            assert base_name in name_in_registry
        else:
            base_name = name_in_registry
        ConstantRegistryItem.__init__(self, name_in_registry=name_in_registry, constant_value = robot_skill_name, allowed_types = [str], registry_instance = registry_instance)
        Action.__init__(self, base_name, parameters=parameters)
       
        #Given that only primitive parameters and SimpleValue parameters are allowed for a ConstantParameterAction, check that RegistryItems are Actually SimpleValues
        for parameter in parameters:
            if isinstance(parameter, RegistryItem):
                assert isinstance(parameter, SimpleValue)

        if ActionRegistry().allow_delayed_parameter_check:
            ActionRegistry().action_parameter_check(name_in_registry, parameters)

    def get_robot_skill_name(self):
        return self.get()

    def __str__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

    def __repr__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())


class DynamicParameterAction(ConstantRegistryItem, Action):
    def __init__(self, name_in_registry : str, robot_skill_name : str, registry_instance : Type["ActionRegistry"], parameters : List[Union[Tuple[str, Any], str]], base_name : str = None):
        assert parameters is not None
        if base_name is not None:
            assert base_name in name_in_registry
        else:
            base_name = name_in_registry
        ConstantRegistryItem.__init__(self, name_in_registry=name_in_registry, constant_value = robot_skill_name, allowed_types = [str], registry_instance = registry_instance)
        Action.__init__(self, base_name, parameters=parameters)
        
        if ActionRegistry().allow_delayed_parameter_check:
            ActionRegistry().action_parameter_check(name_in_registry, parameters)

    def get_robot_skill_name(self):
        return self.get()

    def __str__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

    def __repr__(self):
        if self.parameters:
            param_string = self.get_parameter_string()
            return "%s('%s')[%s]" % (self.name_in_registry, self.get(), param_string)
        else:
            return "%s('%s')" % (self.name_in_registry, self.get())

class ActionRegistry(Registry):

    ITEM_PREFIX = "action_"
    ACTION_BASE_NAME = "action_"
    CHECK_ACTION_PREFIX ="check"

    def __init__(self, robot_idle_skill : str = None, idle_skill_parameters : List = [], allow_delayed_parameter_check : bool = True):
        super().__init__(Action)

        self.allow_delayed_parameter_check = allow_delayed_parameter_check
        self.__actions_scheduled_for_check = []

        if robot_idle_skill is not None:
            if idle_skill_parameters:
                self["action_idle"] = (robot_idle_skill, idle_skill_parameters)
            else:
                self["action_idle"] = robot_idle_skill
        
        self.__action_templates = {}
    
        
    def schedule_action_for_delayed_parameter_check(self, action_name_to_be_checked, parameters : List[Tuple[str, Any]]):
        assert self.allow_delayed_parameter_check
        parameter_check_already_scheduled = False
        
        for scheduled_check in self.__actions_scheduled_for_check:
            parameter_check_already_scheduled = parameter_check_already_scheduled or (scheduled_check[0] == action_name_to_be_checked)
            if parameter_check_already_scheduled: return

        if not parameter_check_already_scheduled:
            self.__actions_scheduled_for_check.append((action_name_to_be_checked, parameters))

    def perform_scheduled_parameter_checks(self):
        assert self.allow_delayed_parameter_check
        
        if not self.__actions_scheduled_for_check:
            return
        
        remaining_checks = []
        
        for action_tuple in self.__actions_scheduled_for_check:
            try:
                self.action_parameter_check(action_tuple[0], action_tuple[1])
            except KeyError as ke:
                print("KeyError raised while performing delayed check for parameter '%s'" % (action_tuple[0]))
                print(str(ke))
                remaining_checks.append((action_tuple[0], action_tuple[1]))
            except AssertionError as ae:
                print("AssertionError raised while performing delayed check for parameter '%s'" % (action_tuple[0]))
                print(str(ae))
                remaining_checks.append((action_tuple[0], action_tuple[1]))
            except Exception as e:
                raise e
        self.__actions_scheduled_for_check = remaining_checks
        
        #print("Performed delayed parameter check: %d checks remaining" % (len(self.__actions_scheduled_for_check)))

    def action_parameter_check(self, name_in_registry, parameters : Union[List[Tuple[str, Any]], List[str]]):
        
        #Get correct name for this action given the registry's naming rules
        name_in_registry = self.get_complete_name(name_in_registry)

        #Check all action parameters and schedule them for a delayed check if 
        # a) they're not yet available in the registry
        # b) they're themselves scheduled for a delayed check
        
        for parameter in parameters:
            if isinstance(parameter, list):
                raise NotImplementedError
            elif isinstance(parameter, tuple):
                assert isinstance(parameter[0], str)
                if isinstance(parameter[1], RegistryItem):
                    assert isinstance(parameter[1], Value)
                else:
                    assert type(parameter[1]) in [bool, float, int, str, Callable]
                    continue

                parameter_name = parameter[0]
            else:
                assert isinstance(parameter, str)
                parameter_name = parameter

            try:
                container_registry = ParameterRegistry.get_container_registry_for_parameter(parameter_name)
            
            #KeyError exception is raised if some parameter for this formula is still unknown:
            #   in this case, schedule this parameter for a later check
            except KeyError as e:
                if self.allow_delayed_parameter_check:
                    self.schedule_action_for_delayed_parameter_check(action_name_to_be_checked=name_in_registry, parameters = parameters)
                    break
                else:
                    raise KeyError("Unknown parameter '%s' for action '%s'" % (parameter_name, name_in_registry))
            except Exception as e:
                raise e
        
            parameter_name = container_registry.get_complete_name(parameter_name)
            if container_registry.is_scheduled_for_delayed_parameter_check(parameter_name):
                self.schedule_action_for_delayed_parameter_check(action_name_to_be_checked=name_in_registry, parameters = parameters)

    #Duplicates an already existing parametric action using new parameters and returns its instance
    def duplicate_action_with_new_parameters(self, action_name : str, parameters : List[Union[Tuple[str, Any], str]] = None):
        assert action_name in self, "Action '"+action_name+"' is unknown"
        new_action_name = self.generate_unique_action_name(action_name)

        existing_action = self.get_instance(action_name)
        existing_action_robot_skill = existing_action.get_robot_skill_name()
        
        if parameters is None:
            new_action_data = existing_action_robot_skill
        else:
            new_action_data = (existing_action_robot_skill, parameters)
        
        self.set(action_name=new_action_name, action_data=new_action_data, action_base_name = action_name)

        return self.get_instance(new_action_name)  
    

    def create_action_from_template(self, action_template_name, parameter_list : List[Union[Tuple[str, Any], str]] = None):
        
        if(action_template_name.startswith(self.CHECK_ACTION_PREFIX)):
            new_action_name = self.generate_unique_action_name(action_name=action_template_name)
            new_action_data = ActionRegistry().get_idle_action().get()
            self.set(action_name=new_action_name, action_data=new_action_data, action_base_name=action_template_name)
            return self.get_instance(new_action_name)
            #return ActionRegistry().get_idle_action()
            
        assert action_template_name in self.__action_templates.keys(), "Unknown ActionTemplate "+action_template_name
        action_template_instance : ActionTemplate = self.__action_templates[action_template_name]
    
    
        new_action_parameters = []
        if parameter_list is not None:
            if action_template_instance.parameters is not None:
                int_indices = [param_index for param_index in action_template_instance.parameters if isinstance(param_index, int)]
                if int_indices:
                    max_param_index = max(int_indices)
                    assert len(parameter_list) > max_param_index
                for param_specification in action_template_instance.parameters:
                    if isinstance(param_specification, int):
                        new_action_parameters.append(parameter_list[param_specification])
                    elif isinstance(param_specification, str):
                        new_action_parameters.append(param_specification)

            else:
                new_action_parameters = parameter_list

        if new_action_parameters:
            new_action_data = (action_template_instance.robot_skill_name, new_action_parameters)
        else:
            new_action_data = action_template_instance.robot_skill_name
            
        new_action_name = self.generate_unique_action_name(action_name=action_template_name)
        self.set(action_name=new_action_name, action_data=new_action_data, action_base_name=action_template_name)
        
        return self.get_instance(new_action_name)


    def register_action_template(self, action_template_name : str, robot_skill_name : str, parameters : List[Union[int, str]] = None):
        assert action_template_name not in self.__action_templates.keys(), "ActionTemplate "+action_template_name+" was already registered"
        self.__action_templates[action_template_name] = ActionTemplate(name_in_registry=action_template_name, robot_skill_name=robot_skill_name, parameters=parameters)

    def get_action_template(self, action_template_name : str):
        assert action_template_name in self.__action_templates.keys()
        return self.__action_templates[action_template_name]


    def get_idle_action(self):
        return self.get_instance("action_idle")

    def set(self, action_name : str, action_data, robot_number : int = None, robot_role : str = None, action_base_name : str = None):
        assert isinstance(action_data, str) or isinstance(action_data, tuple)
        #print(action_name, action_data)
        if action_base_name is not None:
            assert action_base_name in action_name
        dynamic_parameters = False
        if isinstance(action_data, tuple):
            assert len(action_data) == 2 or len(action_data) == 3
            assert isinstance(action_data[0], str)
            assert isinstance(action_data[1], list)
            assert len(action_data[1]) > 0
            for action_parameter in action_data[1]:
                if isinstance(action_parameter, tuple):
                    assert len(action_parameter) == 2 or len(action_data) == 3
                    assert isinstance(action_parameter[0], str)
                elif isinstance(action_parameter, str):
                    #if the parameter is a string it must start with the ValueRegistry prefix, 
                    # in this case, the parameter is supposed to be a RegistryItem
                    dynamic_parameters = True
                
        action_name = self.get_complete_name(action_name, robot_number=robot_number, robot_role=robot_role)

        '''
        if not action_name.startswith(ActionRegistry.ITEM_PREFIX):
            action_name = ActionRegistry.ITEM_PREFIX + action_name
        '''

        if isinstance(action_data, tuple):
            if dynamic_parameters:
                self._items[action_name] = DynamicParameterAction(action_name, robot_skill_name=action_data[0], registry_instance=ActionRegistry(), parameters=action_data[1], base_name = action_base_name)
            else:
                self._items[action_name] = ConstantParameterAction(action_name, robot_skill_name = action_data[0], registry_instance = ActionRegistry(), parameters = action_data[1], base_name = action_base_name)
        elif isinstance(action_data, str):
            self._items[action_name] = ConstantParameterAction(action_name, robot_skill_name = action_data, registry_instance = ActionRegistry(), base_name = action_base_name)
        else:
            raise AssertionError
        
        if self.allow_delayed_parameter_check:
            self.perform_scheduled_parameter_checks()

        if robot_role is not None:
            self.register_item_name_for_robot_role(action_name, robot_role)
        if robot_number is not None:
            self.register_item_name_for_robot_number(action_name, robot_number)
        

    def signal_action_completed(self, action_name):
        assert action_name in self
        action_name = self.get_complete_name(action_name)
        self.get_instance(action_name).set_action_completed()

        
    
    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def set_robot_item(self, robot_number : int, item_name : str, item):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def set_robot_role_item(self, robot_role : str, item_name : str, item):
        raise NotImplementedError




    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_robot_item(self, robot_number : int, item_name : str):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_robot_role_item(self, robot_role : str, item_name : str):
        raise NotImplementedError
            
    

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove(self, item_name : str):
        assert isinstance(item_name, str)

        item_name = self.get_complete_name(item_name)

        if item_name in self._items.keys():
            del self._items[item_name]
        else:
            raise KeyError("Unknown item: %s" % (item_name))

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove_robot_item(self, robot_number : int, item_name : str):
        raise NotImplementedError

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def remove_robot_role_item(self, robot_role : str, item_name : str):
        raise NotImplementedError


    #Generates a unique action name (if action_name is not None, the action name will feature action_name as a base plus a unique number)
    def generate_unique_action_name(self, action_name : str = None):
        if action_name is None:
           base_name = ActionRegistry.ACTION_BASE_NAME
        else:
            base_name = action_name

        return base_name + "_" + str(uuid.uuid1())

    #NOTICE: this function signature overrides the use of specific robot roles/numbers
    def get_complete_name(self, action_name : str, robot_number : int = None, robot_role : str = None):
        action_name = action_name.lower()
        if not action_name.startswith(ActionRegistry.ITEM_PREFIX):
            action_name = ActionRegistry.ITEM_PREFIX + action_name
        return action_name