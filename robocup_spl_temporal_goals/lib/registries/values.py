from typing import Callable, List, Type
import time

from lib.registries.registry import ParameterRegistry, SimpleRegistryItem, FunctionalRegistryItem

class ValueRegistry(ParameterRegistry):

    ITEM_PREFIX = "value_"

    def __init__(self, allow_delayed_parameter_check = True):
        super().__init__(SimpleValue, FunctionalValue, allow_delayed_parameter_check)
            
    def set(self, value_name : str, value, robot_number : int = None, robot_role : str = None):
        assert isinstance(value_name, str)
        assert isinstance(value, SimpleValue) or \
               isinstance(value, FunctionalValue) or \
               isinstance(value, Callable) or \
               isinstance(value, float) or \
               isinstance(value, int) or \
               isinstance(value, str) or \
               isinstance(value, bool) or \
               (isinstance(value, tuple) and len(value) == 2) or \
               (isinstance(value, tuple) and len(value) == 3) or \
               (isinstance(value, list))

        if isinstance(value, list) and value:
            item_type = type(value[0])
            assert item_type != Callable
            for item in value:
                assert isinstance(item, item_type)
        
        value_name = self.get_complete_name(value_name, robot_number=robot_number, robot_role=robot_role)

        if isinstance(value, SimpleValue) or isinstance(value, FunctionalValue):
            self._items[value_name] = value
        elif isinstance(value, bool) or isinstance(value, float) or isinstance(value, int) or isinstance(value, str):
            if value_name in self._items.keys():
                assert type(value) == type(self._items[value_name].get()), "Type of new value '"+str(type(value))+"' is different from type of existing value '"+str(type(self._items[value_name]))+"'"
                self._items[value_name].set(value)
            else:
                self._items[value_name] = SimpleValue(value_name, value=value, registry_instance=self)
        elif isinstance(value, Callable):
            if value_name in self._items.keys():
                assert type(value) == type(self._items[value_name].get())
                self._items[value_name].set(value)
            else:
                self._items[value_name] = FunctionalValue(value_name, formula=value, registry_instance=self)
        elif isinstance(value, tuple) or isinstance(value, list):
            self._items[value_name] = SimpleValue(value_name, value=value, registry_instance = self)
        else:
            raise Exception("Should not be getting here")
        self._update_timestamps[value_name] = time.time()

        if robot_role is not None:
            self.register_item_name_for_robot_role(value_name, robot_role)
        if robot_number is not None:
            self.register_item_name_for_robot_number(value_name, robot_number)
            
        if self.allow_delayed_parameter_check:
            self.perform_scheduled_parameter_checks()

    def reset(self):
        super().reset()



'''
Current inheritance scheme

RegistryItem                                Value             (the Value -> RegistryItem inheritance is necessary because we need the "get" method for logical built-in methods)
^       ^                                   ^    ^
|       |                                   |    |
|   SimpleRegistryItem <--------- SimpleValue    |            (this inheritance specializes the "get" and "set" to access the __value field)
|                                                |
|                                                |
FunctionalRegistryItem <---------------- FunctionalValue      (this inheritance instead specializes the "get" and "set" to access the __formula field and evaluate its paramaters)

'''
    

#Defines a value item used by the ValueRegistry
class Value:
    def __init__(self):
        pass

    '''
    #IN CASE WE WANT TO ADD LOGICAL OPERATIONS
    #Might be a good idea to structure this differently using mixins
    @abstractmethod
    def check_type(self, other):
        pass

    #Comparison
    @abstractmethod
    def __eq__(self, other):
        own_value = self.get()
        return self.get() == other.get()

    @abstractmethod
    def __ne__(self, other):
        return self.get() != other.get()

    @abstractmethod
    def __lt__(self, other):
        return self.get() < other.get()

    @abstractmethod
    def __gt__(self, other):
        return self.get() > other.get()

    @abstractmethod
    def __le__(self, other):
        return self.get() <= other.get()

    @abstractmethod
    def __ge__(self, other):
        return self.get() >= other.get()

    #Arithmetic
    @abstractmethod
    def __add__(self, other):
        return self.get() + other.get()

    @abstractmethod
    def __radd__(self, other):        
        return self.__add__(other)

    @abstractmethod
    def __sub__(self, other):        
        return self.get() - other.get()

    def __rsub__(self, other):
        return -self.__sub__(other)

    @abstractmethod
    def __mul__(self, other):
        return self.get() * other.get()

    @abstractmethod
    def __int__(self):
        pass

    @abstractmethod
    def __float__(self):
        pass

    @abstractmethod
    def __pos__(self):
        pass

    @abstractmethod
    def __neg__(self):
        pass
    '''

class SimpleValue(SimpleRegistryItem, Value):
    def __init__(self, name_in_registry : str, value, registry_instance : ValueRegistry):
        super().__init__(name_in_registry, value, allowed_types=[bool, float, int, str], registry_instance = registry_instance)

    #NOTICE: "get" is inherited from SimpleRegistryItem
    #NOTICE: "set" is inherited from SimpleRegistryItem

    '''
    def __iadd__(self):
        pass

    def __isub__(self):
        pass
    '''

    def __str__(self):
        return "[SimpleValue]%s(%s)" % (self.name_in_registry, str(self.get()))
    
    def __repr__(self):
        return "[SimpleValue]%s(%s)" % (self.name_in_registry, str(self.get()))

class FunctionalValue(FunctionalRegistryItem, Value):
    def __init__(self, name_in_registry : str, formula : Callable, registry_instance : ValueRegistry, default_value_if_not_evaluable):
        
        super().__init__(name_in_registry, formula, registry_instance = registry_instance, default_value_if_not_evaluable = default_value_if_not_evaluable)

        ValueRegistry().parameter_check(name_in_registry, formula)

    #NOTICE: "set" is inherited from FunctionalRegistryItem
    #NOTICE: "get" is inherited from FunctionalRegistryItem

    #NOTICE: "get_formula" is inherited from FunctionalRegistryItem

    def __str__(self):
        try:
            formula_value = self.get()
        except KeyError as ke:
            return "[FunctionalValue]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalValue]%s(%s)" % (self.name_in_registry, str(formula_value))
        '''
        try:
            formula_value = self.get()
        except KeyError as ke:
            return "\nValue name: %s\nCode:\n%s\nCan't compute current value due to a missing parameter\n\nFull exception message:\n%s\n\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(ke))
        except Exception as e:
            raise e
        return "\n\nValue name: %s\nCode:\n%s\nCurrent value: %s\n" % (self.name_in_registry, "\n".join(inspect.getsource(self.get_formula()).split("\n")[1:]).strip(), str(formula_value))
        '''

    def __repr__(self):
        try:
            formula_value = self.get()
        except KeyError as ke:
            return "[FunctionalValue]%s(Unknown)" % (self.name_in_registry)
        except Exception as e:
            raise e
        return "[FunctionalValue]%s(%s)" % (self.name_in_registry, str(formula_value))

