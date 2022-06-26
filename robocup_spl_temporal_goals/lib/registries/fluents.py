from turtle import distance
from typing import Callable, List, Type
from abc import abstractmethod
import time

from numpy import isin

from lib.registries.registry import ParameterRegistry, SimpleRegistryItem, FunctionalRegistryItem


class FluentRegistry(ParameterRegistry):

    ITEM_PREFIX = "fluent_"

    def __init__(self, allow_delayed_parameter_check = True):
        super().__init__(SimpleFluent, FunctionalFluent, allow_delayed_parameter_check)


    def set(self, fluent_name : str, fluent, robot_number : int = None, robot_role : str = None):
        assert isinstance(fluent_name, str)
        assert isinstance(fluent, SimpleFluent) or isinstance(fluent, FunctionalFluent) or isinstance(fluent, bool) or isinstance(fluent, Callable), "Unrecognized type of fluent '"+str(fluent)+"' (type: '"+str(type(fluent))+"'"
        
        fluent_name = self.get_complete_name(fluent_name, robot_number=robot_number, robot_role=robot_role)

        if isinstance(fluent, SimpleFluent) or isinstance(fluent, FunctionalFluent):
            self._items[fluent_name] = fluent
        elif isinstance(fluent, bool):
            if fluent_name in self._items.keys():
                assert type(fluent) == type(bool(self._items[fluent_name]))
                self._items[fluent_name].set(fluent)
            else:
                self._items[fluent_name] = SimpleFluent(fluent_name, value=fluent, registry_instance=self)
        elif isinstance(fluent, Callable):
            if fluent_name in self._items.keys():
                assert type(fluent) == type(bool(self._items[fluent_name]))
                self._items[fluent_name].set(fluent)
            else:
                self._items[fluent_name] = FunctionalFluent(fluent_name, formula=fluent, registry_instance=self)
        else:
            raise Exception("Should not be getting here")
        self._update_timestamps[fluent_name] = time.time()

        if self.allow_delayed_parameter_check:
            self.perform_scheduled_parameter_checks()

        if robot_role is not None:
            self.register_item_name_for_robot_role(fluent_name, robot_role)
        if robot_number is not None:
            self.register_item_name_for_robot_number(fluent_name, robot_number)
            

    def reset(self):
        super().reset()

'''
Current inheritance scheme

RegistryItem                                Fluent             (in case we want to add logical builtins, the Fluent -> RegistryItem inheritance is necessary because we need the "get" method for logical built-in methods)
^       ^                                   ^    ^
|       |                                   |    |
|   SimpleRegistryItem <------- SimpleFluent    |              (this inheritance specializes the "get" and "set" to access the __value field)
|                                                |
|                                                |
FunctionalRegistryItem <---------------- FunctionalFluent      (this inheritance instead specializes the "get" and "set" to access the __formula field and evaluate its paramaters)

'''

#Defines a Fluent item used by the FluentRegistry
#Just requires to implement a __bool__ method so that an instance of this class may be treated as a boolean value
class Fluent:

    @abstractmethod
    def __bool__(self):
        pass

    '''
    #IN CASE WE WANT TO ADD LOGICAL OPERATIONS
    #Might be a good idea to structure this differently using mixins
    @abstractmethod
    def check_type(self, other):
        pass

    @abstractmethod
    def __and__(self, other_fluent):
        pass

    @abstractmethod
    def __or__(self, other_fluent):
        pass

    @abstractmethod
    def __xor__(self, other_fluent):
        pass
    '''

class SimpleFluent(SimpleRegistryItem, Fluent):
    def __init__(self, name_in_registry : str, value : bool, registry_instance : FluentRegistry):
        super().__init__(name_in_registry, value, allowed_types=[bool], registry_instance = registry_instance)

    #NOTICE: "get" is inherited from SimpleRegistryItem
    #NOTICE: "set" is inherited from SimpleRegistryItem

    def __bool__(self):
        return self.get()

    def __str__(self):
        return "[SimpleFluent]%s(%s)" % (self.name_in_registry, bool(self))
    
    def __repr__(self):
        return "[SimpleFluent]%s(%s)" % (self.name_in_registry, bool(self))


class FunctionalFluent(FunctionalRegistryItem, Fluent):
    def __init__(self, name_in_registry : str, formula : Callable, registry_instance : FluentRegistry, default_value_if_not_evaluable):
        FunctionalRegistryItem.__init__(self, name_in_registry, formula, registry_instance = registry_instance, default_value_if_not_evaluable = default_value_if_not_evaluable)

        FluentRegistry().parameter_check(name_in_registry, formula)

    #NOTICE: "set" is inherited from FunctionalRegistryItem
    #NOTICE: "get" is inherited from FunctionalRegistryItem

    def __bool__(self):
        return self.get()

    def __str__(self):
        try:
            formula_fluent = self.get()
        except KeyError as ke:
            return "[FunctionalFluent]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalFluent]%s(%s)" % (self.name_in_registry, str(formula_fluent))
        '''
        try:
            formula_value = self.get()
        except KeyError as ke:
            return "\nFluent name: %s\nCode:\n%s\nCan't compute current fluent due to a missing parameter\n\nFull exception message:\n%s\n\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(ke))
        except Exception as e:
            raise e
        return "\n\nFluent name: %s\nCode:\n%s\nCurrent fluent: %s\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(formula_value))
        '''

    def __repr__(self):
        try:
            formula_fluent = self.get()
        except KeyError as ke:
            return "[FunctionalFluent]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalFluent]%s(%s)" % (self.name_in_registry, str(formula_fluent))
