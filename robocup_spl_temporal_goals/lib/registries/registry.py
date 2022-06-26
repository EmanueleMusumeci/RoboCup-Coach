from ast import alias
from typing import Callable, List, Type
from abc import abstractmethod, abstractproperty
import inspect
import time
import math

from numpy import isin

from lib.misc.singleton import Singleton

class Registry(metaclass=Singleton):

    __REGISTRIES = []

    @property
    @abstractmethod
    def ITEM_PREFIX(self):
        pass

    ROBOT_NUMBER_PREFIX = "ROBOT#"
    ROBOT_ROLE_PREFIX = "ROBOT_ROLE_"

    def __init__(self, __base_simple_item_type : Type):
        assert isinstance(__base_simple_item_type, Type)
        self.__base_simple_item_type = __base_simple_item_type
        
        self._items = {}
        self._update_timestamps = {}
        Registry.__REGISTRIES.append(self)

        self.__item_aliases = {}

        self._robot_role_to_item_names = {}
        self._robot_number_to_item_names = {}

    @classmethod
    def get_registries(cls):
        return Registry.__REGISTRIES

    @classmethod
    def get_container_registry_for_item(cls, item_name):
        found_in_registry = None
        for registry in Registry.get_registries():
            if found_in_registry is not None:
                if item_name in registry:
                    raise Exception("Ambiguous item name. Item name '"+item_name+"' found in both in registry '"+found_in_registry.__class__.__name__+"' and registry '"+registry.__class__.__name__+"'")
            else:
                if item_name in registry:
                    found_in_registry = registry
        if found_in_registry is None:
            raise KeyError
        return found_in_registry

    def register_alias(self, item_name : str, alias_name : str):
        assert isinstance(item_name, str)
        assert isinstance(alias_name, str)
        complete_alias_name = self.get_complete_name(alias_name)
        complete_item_name = self.get_complete_name(item_name)
        assert complete_alias_name not in self.__item_aliases.keys()
        self.__item_aliases[complete_alias_name] = complete_item_name

    def is_alias_name(self, item_name : str):
        assert isinstance(item_name, str)
        complete_item_name = self.get_complete_name(item_name)
        return complete_item_name in self.__item_aliases.keys()

    def get_aliases_for_item_name(self, item_name : str):
        assert isinstance(item_name, str)
        aliases = []
        complete_item_name = self.get_complete_name(item_name)

        for complete_alias_name, aliased_item_name in self.__item_aliases.values():
            if aliased_item_name == complete_item_name:
                aliases.append(complete_alias_name)

        return aliases
    
    def get_aliased_name(self, item_name : str):
        assert isinstance(item_name, str)
        complete_item_name = self.get_complete_name(item_name)
        if complete_item_name in self.__item_aliases.keys():
            return self.__item_aliases[complete_item_name]
        else:
            raise KeyError("Unknown alias "+item_name)
    
    def get_aliases(self):
        return self.__item_aliases

    def get_complete_name(self, item_name : str, robot_number : int = None, robot_role : str = None):
        assert not(robot_number is not None and robot_role is not None), "Specify only one among robot_number and robot_role"
        item_name = item_name.lower()
        if not item_name.startswith(self.ITEM_PREFIX):
            if robot_number is not None:
                assert robot_role is None
                if not item_name.startswith(str(robot_number)):
                    item_name = self.ROBOT_NUMBER_PREFIX + str(robot_number) + "_" + item_name
            elif robot_role is not None:
                assert robot_number is None
                if not item_name.startswith(robot_role):
                    item_name = str(robot_role) + "_" + item_name

            item_name = self.ITEM_PREFIX + item_name
        return item_name
    
    def get_update_timestamp(self, item_name):
        if item_name in self._items.keys():
            return self._update_timestamps[item_name]
        else:
            raise KeyError("Unknown item: %s" % (item_name))






    @abstractmethod    
    def set(self, item_name : str, item, robot_number : int = None, robot_role : str = None):
        pass

    def register_item_name_for_robot_role(self, item_name : str, robot_role : str):
        if robot_role not in self._robot_role_to_item_names.keys():
            self._robot_role_to_item_names[robot_role] = [item_name]
        else:
            self._robot_role_to_item_names[robot_role].append(item_name)

    def register_item_name_for_robot_number(self, item_name : str, robot_number : int):
        if robot_number not in self._robot_number_to_item_names.keys():
            self._robot_number_to_item_names[robot_number] = [item_name]
        else:
            self._robot_number_to_item_names[robot_number].append(item_name)





    def get(self, item_name : str, robot_number : int = None, robot_role : str = None):
        assert isinstance(item_name, str)

        item_name = self.get_complete_name(item_name, robot_number = robot_number, robot_role = robot_role)

        if item_name in self._items.keys():
            return self._items[item_name].get()
        elif item_name in self.__item_aliases.keys():
            aliased_item_name = self.__item_aliases[item_name]
            if aliased_item_name not in self._items.keys():
                raise KeyError("Unknown item (alias '%s'): %s" % (item_name, aliased_item_name))
            return self._items[aliased_item_name].get()    
        else:
            raise KeyError("Unknown item: %s" % (item_name))
    
    def get_instance(self, item_name : str):
        assert isinstance(item_name, str)

        item_name = self.get_complete_name(item_name)

        if item_name in self._items.keys():
            return self._items[item_name]
        elif item_name in self.__item_aliases.keys():
            aliased_item_name = self.__item_aliases[item_name]
            if aliased_item_name not in self._items.keys():
                raise KeyError("Unknown item (alias '%s'): %s" % (item_name, aliased_item_name))
            return self._items[aliased_item_name]
        else:
            raise KeyError("Unknown item: %s" % (item_name))
            





    def get_items(self):
        return {item_name : item.get() for item_name, item in self._items.items()}
    






    def remove(self, item_name : str, robot_number : int = None, robot_role : str = None):
        assert isinstance(item_name, str)

        item_name = self.get_complete_name(item_name, robot_number = robot_number, robot_role = robot_role)

        if item_name in self._items.keys():
            del self._items[item_name]
        else:
            raise KeyError("Unknown item: %s" % (item_name))

    def remove_robot_number_item(self, robot_number : int, item_name : str):
        complete_item_name = self.get_complete_name(item_name, robot_number=robot_number)
        if complete_item_name in self._items.keys():
            del self._items[complete_item_name]

    def remove_all_items_for_robot_number(self, robot_number : str):
        if robot_number in self._robot_role_to_item_names:
            for item_name in self._robot_number_to_item_names[robot_number]:
                try:
                    self._items.pop(item_name)
                except:
                    pass
            self._robot_number_to_item_names.pop(robot_number)


    def remove_robot_role_item(self, robot_role : str, item_name : str):
        complete_item_name = self.get_complete_name(item_name, robot_role=robot_role)
        if complete_item_name in self._items.keys():
            del self._items[complete_item_name]

    def remove_all_items_for_robot_role(self, robot_role : str):
        if robot_role in self._robot_role_to_item_names:
            for item_name in self._robot_role_to_item_names[robot_role]:
                try:
                    self._items.pop(item_name)
                except:
                    pass
            self._robot_role_to_item_names.pop(robot_role)



    def reset(self):
        self._items = {}
        self._update_timestamps = {}
        self.__parameters_scheduled_for_check = []


    @classmethod
    def reset_all_registries(cls):
        for registry_instance in ParameterRegistry.__PARAMETER_REGISTRIES:
            registry_instance.reset()
    



    def __setitem__(self, item_name : str, item):
        #print(item_name)
        self.set(item_name, item)

    def __getitem__(self, item_name : str):
        return self.get(item_name)

    def __contains__(self, item_name : str):
        assert isinstance(item_name, str)
        item_name = self.get_complete_name(item_name)
        if item_name in self.__item_aliases.keys():
            aliased_item_name = self.__item_aliases[item_name]
            if aliased_item_name not in self._items.keys():
                #raise KeyError("Unknown item (alias '%s'): %s" % (item_name, aliased_item_name))
                return False
            return aliased_item_name in self._items.keys()
        else:
            return item_name in self._items.keys()
    
    def __str__(self):
        result = "\n"+('*'*100)+"\n"+self.__class__.__name__+"\n"+('*'*100)+"\n"
        for i, (item_name, item) in enumerate(self._items.items()):
            item_str = str(item)
            result+=item_str+(", " if (i < len(self._items)-1 and not item_str.endswith("\n")) else "")
        return result+("\n"+'*'*100)

    def __repr__(self):
        return str(self)
    
    def __len__(self):
        return len(self._items)


class ParameterRegistry(Registry):

    __PARAMETER_REGISTRIES = []

    #__base_simple_item_type should be the type of the "Simple" (non-callable) item type of this registry (like SimpleValue for the ValueRegistry)
    #__base_function_item_type should be the type of the "Simple" (non-callable) item type of this registry (like FunctionalValue for the ValueRegistry) 
    def __init__(self, __base_simple_item_type : Type, __base_function_item_type : Type, allow_delayed_parameter_check = True):
        super().__init__(__base_simple_item_type)

        assert isinstance(__base_function_item_type, Type)
        self.__base_function_item_type = __base_function_item_type

#TODO: consider turning this list into a LIFO structure (it makes more sense, given that the parameter check is recursive)
        self.__parameters_scheduled_for_check = []
        self.allow_delayed_parameter_check = allow_delayed_parameter_check

        ParameterRegistry.__PARAMETER_REGISTRIES.append(self)



    @classmethod
    def get_parameter_registries(cls):
        return ParameterRegistry.__PARAMETER_REGISTRIES

    @classmethod
    def get_container_registry_for_parameter(cls, parameter_name):
        found_in_registry = None
        for registry in ParameterRegistry.get_parameter_registries():
            if found_in_registry is not None:
                if parameter_name in registry:
                    raise Exception("Ambiguous parameter. Parameter '"+parameter_name+"' found in both in registry '"+found_in_registry.__class__.__name__+"' and registry '"+registry.__class__.__name__+"'")
            else:
                if parameter_name in registry:
                    found_in_registry = registry
        if found_in_registry is None:
            raise KeyError
        return found_in_registry


    @classmethod
    def reset_all_parameter_registries(cls):
        for registry_instance in ParameterRegistry.__PARAMETER_REGISTRIES:
            registry_instance.reset()



    #Check if the item is a Callable
    def is_function(self, item_name : str):
        
        item_name = self.get_complete_name(item_name)

        assert isinstance(item_name, str)
        if item_name in self._items.keys():
            return isinstance(self._items[item_name], self.__base_function_item_type)
        else:
            raise KeyError("Unknown item: %s" % (item_name))
    

    def add_function(self, function : Callable, aliases = [], default_value_if_not_evaluable = None):
        item_name = self.get_complete_name(function.__name__)
        #print(item_name)
        
        self[item_name] = self.__base_function_item_type(item_name, formula=function, registry_instance = self, default_value_if_not_evaluable = default_value_if_not_evaluable)

        for alias in aliases:
            self.register_alias(item_name, alias)

        if self.allow_delayed_parameter_check:
            self.perform_scheduled_parameter_checks()


    def is_scheduled_for_delayed_parameter_check(self, parameter_name : str):
        for parameter_tuple in self.__parameters_scheduled_for_check:
            if parameter_tuple[0] == parameter_name:
                return True
        return False

    def schedule_parameter_for_delayed_parameter_check(self, parameter_name_to_be_checked, formula : Callable):
        assert self.allow_delayed_parameter_check
        parameter_check_already_scheduled = False
        
        for scheduled_check in self.__parameters_scheduled_for_check:
            parameter_check_already_scheduled = parameter_check_already_scheduled or (scheduled_check[0] == parameter_name_to_be_checked)
            if parameter_check_already_scheduled: return

        if not parameter_check_already_scheduled:
            self.__parameters_scheduled_for_check.append((parameter_name_to_be_checked, formula))

    def perform_scheduled_parameter_checks(self):
        assert self.allow_delayed_parameter_check
        
        if not self.__parameters_scheduled_for_check:
            return
        
        remaining_checks = []
        
        for parameter_name, formula in self.__parameters_scheduled_for_check:
            try:
                self.parameter_check(parameter_name, formula)
            except KeyError as ke:
                print("KeyError raised while performing delayed check for parameter '%s'" % (parameter_name))
                print(str(ke))
                remaining_checks.append((parameter_name, formula))
            except AssertionError as ae:
                print("AssertionError raised while performing delayed check for parameter '%s'" % (parameter_name))
                print(str(ae))
                remaining_checks.append((parameter_name, formula))
            except Exception as e:
                raise e
        self.__parameters_scheduled_for_check = remaining_checks
        
        #print("Performed delayed parameter check: %d checks remaining" % (len(self.__parameters_scheduled_for_check)))



    #Checks that a Callable value does not use parameters that recursively refer to it and also
    # checks that all the parameter it uses are already registered,
    # by performing a depth-first tree visit to check that all the callable used to ultimately
    # compute the value do not refer to the value itself at some point in the call stack
    #(NOTICE: we perform this check every time a new FunctionalRegistryItem is added, so all the lower-level FunctionalRegistryItems should be already checked)
    def parameter_check(self, name_in_registry : str, formula : Callable):
            
        def _check(aux_name_in_registry : str, original_name : str):
            try:
                container_registry = ParameterRegistry.get_container_registry_for_parameter(parameter_name)
            #KeyError exception is raised if some parameter for this formula is still unknown:
            #   in this case, schedule this parameter for a later check
            except KeyError as e:
                raise KeyError("Unknown parameter %s" % (aux_name_in_registry))

            aux_name_in_registry = container_registry.get_complete_name(aux_name_in_registry)
            instance = container_registry.get_instance(aux_name_in_registry)
            
            parameters_ok = True
            #Only check FunctionalRegistryItem parameters (because the name_in_registry only refers to a FunctionalLiteral)
            if isinstance(instance, FunctionalRegistryItem):
                aux_formula_parameters = FunctionalRegistryItem.get_formula_parameter_names_from_formula(instance.get_formula())
                for aux_parameter_name in aux_formula_parameters:
                    assert isinstance(aux_parameter_name, str)

                    try:
                        container_registry = ParameterRegistry.get_container_registry_for_parameter(parameter_name)
                    #KeyError exception is raised if some parameter for this formula is still unknown:
                    #   in this case, schedule this parameter for a later check
                    except KeyError as e:
                        if self.allow_delayed_parameter_check:
                            self.schedule_parameter_for_delayed_parameter_check(parameter_name, formula)
                            continue
                        else:
                            raise KeyError("Unknown parameter '%s' in FunctionalLiteral '%s', along the evaluation tree of FunctionalLiteral '%s'" % (aux_parameter_name, aux_name_in_registry, original_name))

                    aux_parameter_name = container_registry.get_complete_name(aux_parameter_name)

                    if aux_parameter_name == original_name:
                        return False
                    else:
                        parameters_ok = parameters_ok and _check(aux_parameter_name, original_name)

            return parameters_ok

        #Get correct name for this FunctionalRegistryItem given the registry's naming rules
        name_in_registry = self.get_complete_name(name_in_registry)

        
        parameters_ok = True
        #Check that each formula parameter is contained in one and one registry only
        formula_parameters = FunctionalRegistryItem.get_formula_parameter_names_from_formula(formula)
        for parameter_name in formula_parameters:
            assert isinstance(parameter_name, str)
            
            try:
                container_registry = ParameterRegistry.get_container_registry_for_parameter(parameter_name)
            
            #KeyError exception is raised if some parameter for this formula is still unknown:
            #   in this case, schedule this parameter for a later check
            except KeyError as e:
                if self.allow_delayed_parameter_check:
                    self.schedule_parameter_for_delayed_parameter_check(parameter_name, formula)
                    continue
                else:
                    raise KeyError("Unknown parameter '%s' in FunctionalLiteral '%s'" % (parameter_name, name_in_registry))
            except Exception as e:
                raise e
            
            #Get correct name for this parameter given the registry's naming rules
            parameter_name = container_registry.get_complete_name(parameter_name)
            
            #print((name_in_registry, parameter_name))

            #If the parameter is the item itself, check failed
            if parameter_name == name_in_registry:
                parameters_ok = False

            #else proceed by along the evaluation tree
            else:
                try:
                    #Recursively check this parameter's parameters (if it is a FunctionalRegistryItem)
                    parameters_ok = parameters_ok and _check(parameter_name, name_in_registry)

                #KeyError exception is raised if some parameter along the evaluation tree (at any recursion depth of the call to _check) is still unknown:
                #   in this case, schedule this parameter (at this level) for a later check
                except KeyError as ke:
                    if self.allow_delayed_parameter_check:
                        print("Unknown parameter '%s' in FunctionalLiteral '%s'. Scheduling for delayed parameter check." % (parameter_name, name_in_registry))
                        self.schedule_parameter_for_delayed_parameter_check(parameter_name, formula)
                    else:
                        raise KeyError("Unknown parameter '%s' in FunctionalLiteral '%s'" % (parameter_name, name_in_registry))
                except Exception as e:
                    raise e

        if not parameters_ok:
            raise AssertionError(self.__class__.__name__+" '"+name_in_registry+"' recursively calls itself along the evaluation tree")



class RegistryItem:
    def __init__(self, name_in_registry : str, allowed_types : List[Type], registry_instance : ParameterRegistry):
        assert isinstance(name_in_registry, str)
        self.name_in_registry = name_in_registry
        self.registry_instance = registry_instance
        self.__allowed_types = allowed_types
    
    @abstractmethod
    def set(self, item):
        pass        

    @abstractmethod
    def get(self):
        pass
    
    @abstractmethod
    def __str__(self):
        pass

    @abstractmethod
    def __repr__(self):
        pass
        

    def check_type(self, value):
        correct_type = False
        for allowed_type in self.__allowed_types:
            correct_type = correct_type or isinstance(value, allowed_type)
            if correct_type: break
        return correct_type

class ConstantRegistryItem(RegistryItem):
    def __init__(self, name_in_registry : str, constant_value, allowed_types : List[Type], registry_instance : ParameterRegistry):
        super().__init__(name_in_registry, allowed_types = allowed_types, registry_instance = registry_instance)
        self.__constant_value = constant_value

    def set(self, item):
        raise NotImplementedError    

    def get(self):
        return self.__constant_value


class SimpleRegistryItem(RegistryItem):
    def __init__(self, name_in_registry : str, value, allowed_types : List[Type], registry_instance : ParameterRegistry):
        super().__init__(name_in_registry=name_in_registry, allowed_types = allowed_types, registry_instance = registry_instance)
        self.__value = value
                        
    def get(self):
        return self.__value
        
    def set(self, value):
        assert self.check_type(value), "Item type (%s) is not allowed. Allowed types are %s" % (type(value), self.__allowed_types)
        self.__value = value


class FunctionalRegistryItem(RegistryItem):
    def __init__(self, name_in_registry : str, formula : Callable, registry_instance : ParameterRegistry, default_value_if_not_evaluable = None):
        super().__init__(name_in_registry = name_in_registry, allowed_types = [Callable], registry_instance = registry_instance)    
        
        self.__formula : Callable = formula
        self.default_value_if_not_evaluable = default_value_if_not_evaluable

    def collect_formula_parameters(self):
        values_needed_by_formula = {}
        for parameter_name in FunctionalRegistryItem.get_formula_parameter_names_from_formula(self.__formula):
            parameter_found_in_registry = None
            for registry in ParameterRegistry.get_parameter_registries():
                if parameter_found_in_registry is None:
                    if parameter_name in registry:
                        values_needed_by_formula[parameter_name] = registry[parameter_name]
                        parameter_found_in_registry = registry
                else:
                    if parameter_name in registry:
                        print("Ambigous parameter: parameter '%s' found in more than one registry!" % (parameter_name))
                        raise AssertionError

            if parameter_found_in_registry is None:
                #raise KeyError("Can't currently find parameter '%s' in any registry! Delayed parameter check is %s \nIf calling a FunctionalValue using parameters for specific robot roles or numbers, you should call them:\n\t<robot_role_here_without_angled_parentheses>_<parameter_name_here_without_angled_parentheses>\n\tOR\n\trobot#<robot_number_here_without_angled_parentheses>_<parameter_name_here_without_angled_parentheses>" % (parameter_name, ("ON, so check if the parameter has yet to be added" if self.registry_instance.allow_delayed_parameter_check else "OFF")))
                raise KeyError("Can't currently find parameter '%s' in any registry! Delayed parameter check is %s \n" % (parameter_name, ("ON, so check if the parameter has yet to be added" if self.registry_instance.allow_delayed_parameter_check else "OFF")))

        return values_needed_by_formula

    @classmethod
    def get_formula_parameter_names_from_formula(cls, formula : Callable):
        parameter_names = []
        for name in inspect.getfullargspec(formula).args:
            if "=" in name:
                raise Exception("The Callable passed as formula should not contain default values")
            parameter_names.append(name)
        return parameter_names
    
    def get(self):
        try:
            formula_parameters = self.collect_formula_parameters() 
        except KeyError as ke:
            if self.default_value_if_not_evaluable is not None:
                return self.default_value_if_not_evaluable
            else:
                raise ke
                
        return self.__formula(**formula_parameters)
    
    def set(self, formula : Callable):
        assert self.check_type(formula), "Item type (%s) is not allowed. Allowed types are %s" % (type(formula), self.__allowed_types)
        self.__formula = formula

    def get_formula(self):
        return self.__formula
