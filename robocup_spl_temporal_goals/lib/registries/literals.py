from turtle import distance
from typing import Callable, List, Type
from abc import abstractmethod
import time

from numpy import isin

from lib.registries.registry import ParameterRegistry, SimpleRegistryItem, FunctionalRegistryItem


class LiteralRegistry(ParameterRegistry):

    ITEM_PREFIX = "literal_"

    def __init__(self, allow_delayed_parameter_check = True):
        super().__init__(SimpleLiteral, FunctionalLiteral, allow_delayed_parameter_check)


    def set(self, literal_name : str, literal, robot_number : int = None, robot_role : str = None):
        assert isinstance(literal_name, str)
        assert isinstance(literal, SimpleLiteral) or isinstance(literal, FunctionalLiteral) or isinstance(literal, bool) or isinstance(literal, Callable), "Unrecognized type of literal '"+str(literal)+"' (type: '"+str(type(literal))+"'"
        
        literal_name = self.get_complete_name(literal_name, robot_number=robot_number, robot_role=robot_role)

        if isinstance(literal, SimpleLiteral) or isinstance(literal, FunctionalLiteral):
            self._items[literal_name] = literal
        elif isinstance(literal, bool):
            if literal_name in self._items.keys():
                assert type(literal) == type(bool(self._items[literal_name]))
                self._items[literal_name].set(literal)
            else:
                self._items[literal_name] = SimpleLiteral(literal_name, value=literal, registry_instance=self)
        elif isinstance(literal, Callable):
            if literal_name in self._items.keys():
                assert type(literal) == type(bool(self._items[literal_name]))
                self._items[literal_name].set(literal)
            else:
                self._items[literal_name] = FunctionalLiteral(literal_name, formula=literal, registry_instance=self)
        else:
            raise Exception("Should not be getting here")
        self._update_timestamps[literal_name] = time.time()

        if self.allow_delayed_parameter_check:
            self.perform_scheduled_parameter_checks()

        if robot_role is not None:
            self.register_item_name_for_robot_role(literal_name, robot_role)
        if robot_number is not None:
            self.register_item_name_for_robot_number(literal_name, robot_number)

    def reset(self):
        super().reset()

'''
Current inheritance scheme

RegistryItem                                Literal             (in case we want to add logical builtins, the Literal -> RegistryItem inheritance is necessary because we need the "get" method for logical built-in methods)
^       ^                                   ^    ^
|       |                                   |    |
|   SimpleRegistryItem <------- SimpleLiteral    |              (this inheritance specializes the "get" and "set" to access the __value field)
|                                                |
|                                                |
FunctionalRegistryItem <---------------- FunctionalLiteral      (this inheritance instead specializes the "get" and "set" to access the __formula field and evaluate its paramaters)

'''

#Defines a Literal item used by the LiteralRegistry
#Just requires to implement a __bool__ method so that an instance of this class may be treated as a boolean value
class Literal:

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
    def __and__(self, other_literal):
        pass

    @abstractmethod
    def __or__(self, other_literal):
        pass

    @abstractmethod
    def __xor__(self, other_literal):
        pass
    '''

class SimpleLiteral(SimpleRegistryItem, Literal):
    def __init__(self, name_in_registry : str, value : bool, registry_instance : LiteralRegistry):
        super().__init__(name_in_registry, value, allowed_types=[bool], registry_instance = registry_instance)

    #NOTICE: "get" is inherited from SimpleRegistryItem
    #NOTICE: "set" is inherited from SimpleRegistryItem

    def __bool__(self):
        return self.get()

    def __str__(self):
        return "[SimpleLiteral]%s(%s)" % (self.name_in_registry, bool(self))
    
    def __repr__(self):
        return "[SimpleLiteral]%s(%s)" % (self.name_in_registry, bool(self))


class FunctionalLiteral(FunctionalRegistryItem, Literal):
    def __init__(self, name_in_registry : str, formula : Callable, registry_instance : LiteralRegistry, default_value_if_not_evaluable):
        FunctionalRegistryItem.__init__(self, name_in_registry, formula, registry_instance = registry_instance, default_value_if_not_evaluable = default_value_if_not_evaluable)

        LiteralRegistry().parameter_check(name_in_registry, formula)

    #NOTICE: "set" is inherited from FunctionalRegistryItem
    #NOTICE: "get" is inherited from FunctionalRegistryItem

    def __bool__(self):
        return self.get()

    def __str__(self):
        try:
            formula_literal = self.get()
        except KeyError as ke:
            return "[FunctionalLiteral]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalLiteral]%s(%s)" % (self.name_in_registry, str(formula_literal))
        '''
        try:
            formula_value = self.get()
        except KeyError as ke:
            return "\nLiteral name: %s\nCode:\n%s\nCan't compute current literal due to a missing parameter\n\nFull exception message:\n%s\n\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(ke))
        except Exception as e:
            raise e
        return "\n\nLiteral name: %s\nCode:\n%s\nCurrent literal: %s\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(formula_value))
        '''

    def __repr__(self):
        try:
            formula_literal = self.get()
        except KeyError as ke:
            return "[FunctionalLiteral]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalLiteral]%s(%s)" % (self.name_in_registry, str(formula_literal))
