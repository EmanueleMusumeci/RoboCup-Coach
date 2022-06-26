import math
import os
import sys

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )

from lib.registries.values import ValueRegistry
from lib.registries.literals import LiteralRegistry
from lib.utils import linear_distance, angular_distance

if __name__ == "__main__":

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: ValueRegistry creation and print")
    value_registry = ValueRegistry()

    

    value_registry["relative_x_from_striker"] = 1000
    value_registry["relative_y_from_striker"] = 500
    value_registry["supporter_stay_behind_striker"] = True
    value_registry["supporter_stay_to_the_left_of_striker"] = True
    print(value_registry)
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: FunctionalValue")

    def offset_y_from_striker(relative_y_from_striker, supporter_stay_to_the_left_of_striker):
        y_offset = relative_y_from_striker
        if not supporter_stay_to_the_left_of_striker:
            y_offset *= -1
        return y_offset

    def offset_x_from_striker(relative_x_from_striker, supporter_stay_behind_striker):
        x_offset = relative_x_from_striker
        if supporter_stay_behind_striker:
            x_offset *= -1
        return x_offset

    #Register functions to compute striker offset
    ValueRegistry().add_function(offset_x_from_striker)
    ValueRegistry().add_function(offset_y_from_striker)

    def follow_striker_offset(offset_x_from_striker, offset_y_from_striker):
        return (offset_x_from_striker, offset_y_from_striker)

    #Register function to compute striker offset tuple
    ValueRegistry().add_function(follow_striker_offset)
    print("Striker offset (should be (-1000, 500): "+str(ValueRegistry()["follow_striker_offset"]))

    #Set striker position to (-2000, 500)
    ValueRegistry().set(value_name="position", value = (-2000, 500), robot_role="striker")
    print("Striker position (should be (-2000, 500)): "+str(ValueRegistry().get("position", robot_role="striker")))

    #Set supporter position to (-2000, -2000)
    ValueRegistry().set(value_name="position", value = (-2000, -2000), robot_role="supporter")
    print("Supporter position (should be (-2000, -2000)): "+str(ValueRegistry().get("position", robot_role="supporter")))
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")

    def follow_striker_position(striker_position, follow_striker_offset):
        return (striker_position[0] + follow_striker_offset[0], striker_position[1] + follow_striker_offset[1])

    #Register function to compute striker offset tuple
    ValueRegistry().add_function(follow_striker_position)
    print("Follow striker position (should be (-3000, 1000): "+str(ValueRegistry()["follow_striker_position"]))


    ValueRegistry()["striker_follow_position_distance_threshold_from_position"] = 100
    print("Supporter to striker following position distance threshold (should be 100): "+str(ValueRegistry()["striker_follow_position_distance_threshold_from_position"]))
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")

    def supporter_distance_from_striker_following_position(supporter_position, follow_striker_position):
        return distance(supporter_position, follow_striker_position)

    ValueRegistry().add_function(supporter_distance_from_striker_following_position)

    print("Distance of supporter to striker following position (should be 2500): "+str(ValueRegistry()["supporter_distance_from_striker_following_position"]))

    print(ValueRegistry())
    print("\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")

    print("Tests: LiteralRegistry creation and print")
    literal_registry = LiteralRegistry()
    literal_registry["this_value_should_be_true"] = True

    LiteralRegistry()["this_value_should_be_false"] = False
    print(LiteralRegistry())
    print("\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: SimpleLiteral evaluation")
    print("this_value_should_be_true AND this_value_should_be_false SHOULD BE False: "+str(literal_registry["this_value_should_be_true"] and literal_registry["this_value_should_be_false"]))
    print("this_value_should_be_true OR this_value_should_be_false SHOULD BE True: "+str(literal_registry["this_value_should_be_true"] or literal_registry["this_value_should_be_false"]))
    print("\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: FunctionalLiteral evaluation")
    def supporter_distance_from_destination_lower_than_threshold(supporter_distance_from_striker_following_position, striker_follow_position_distance_threshold_from_position):
        return supporter_distance_from_striker_following_position < striker_follow_position_distance_threshold_from_position

    LiteralRegistry().add_function(supporter_distance_from_destination_lower_than_threshold)
    print("Evaluating: %s\nValue: %s (should be False)" % (LiteralRegistry().get_instance("supporter_distance_from_destination_lower_than_threshold"), LiteralRegistry()["supporter_distance_from_destination_lower_than_threshold"]))
    print("\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")

    print("Test: CROSS-REGISTRY parameters\nFunctionalValue using a FunctionalLiteral (that in turn uses a FunctionalValue, a SimpleValue, a FunctionalLiteral a SimpleLiteral), a SimpleLiteral, a FunctionalValue and a SimpleValue")
    LiteralRegistry()["simple_literal"] = True
    ValueRegistry()["simple_value"] = math.pi

    def function_value():
        return math.pi * 2
    ValueRegistry().add_function(function_value)

    def function_literal():
        return True
    LiteralRegistry().add_function(function_literal)

    def mixed_function_value_using_also_literals(simple_literal, function_literal, simple_value, function_value):
        if simple_literal:
            if function_literal:
                return function_value - simple_value
        return 0
    ValueRegistry().add_function(mixed_function_value_using_also_literals)
    print("Evaluating: %s\nValue: %s (should be PI)" % (ValueRegistry().get_instance("mixed_function_value_using_also_literals"), ValueRegistry()["mixed_function_value_using_also_literals"]))
    print("\n\n")

    def mixed_function_literal_using_also_values(simple_literal, function_literal, simple_value, function_value):
        if simple_literal:
            if function_literal:
                return function_value - simple_value > 0
        return False
    LiteralRegistry().add_function(mixed_function_literal_using_also_values)
    print("Evaluating: %s\nValue: %s (should be True)" % (LiteralRegistry().get_instance("mixed_function_literal_using_also_values"), LiteralRegistry()["mixed_function_literal_using_also_values"]))
    print("\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")

    #Set supporter position to (-3000, -1450) (nearby and below threshold)
    ValueRegistry().set(value_name="position", value = (-3000, 1050), robot_role="supporter")
    print("NEW supporter position (should be (-3000, -1050)): "+str(ValueRegistry().get("position", robot_role="supporter")))
    print("NEW distance of supporter to striker following position (should be 50): "+str(ValueRegistry()["supporter_distance_from_striker_following_position"]))

    LiteralRegistry().add_function(supporter_distance_from_destination_lower_than_threshold)
    print("Evaluating: %s\nValue: %s (should be True)" % (LiteralRegistry().get_instance("supporter_distance_from_destination_lower_than_threshold"), LiteralRegistry()["supporter_distance_from_destination_lower_than_threshold"]))
    print("\n\n")


#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: Check for missing FunctionalLiterals or FunctionalValue parameters (parameters of a FunctionalLiteral or a FunctionalValue that have never been registered in the ValueRegistry or in the Literal Registry before)")
    print("Disabling delayed parameter check")
    LiteralRegistry().allow_delayed_parameter_check = False
    ValueRegistry().allow_delayed_parameter_check = False
    print("Test#A: Level 1 (FunctionalLiteral with an unregistered parameter)")
    def function_literal_ok(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_literal_ok):
        return function_literal_ok
    try:
        LiteralRegistry().add_function(function_literal_ok)
        LiteralRegistry().add_function(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except KeyError as ke:
        print("KeyError raised: "+str(ke))
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of LiteralRegistry)")



    print("Test#B: Level 1 (FunctionalValue with an unregistered parameter)")
    def function_value_ok(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_value_ok):
        return function_value_ok
    try:
        ValueRegistry().add_function(function_value_ok)
        ValueRegistry().add_function(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except KeyError as ke:
        print("KeyError raised: "+str(ke))
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of ValueRegistry)")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: Registry reset")
    print("# of items in ValueRegistry %d" % (len(ValueRegistry())))
    ValueRegistry().reset()
    print("# of items in ValueRegistry after reset %d" % (len(ValueRegistry())))
    assert len(ValueRegistry()) == 0, "# of items in ValueRegistry should be 0 after reset"

    print("# of items in LiteralRegistry %d" % (len(LiteralRegistry())))
    LiteralRegistry().reset()
    print("# of items in LiteralRegistry after reset %d" % (len(LiteralRegistry())))
    assert len(LiteralRegistry()) == 0, "# of items in ValueRegistry should be 0 after reset"
    print("OK")

   

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: Check for missing FunctionalLiterals or FunctionalValue parameters (parameters of a FunctionalLiteral or a FunctionalValue that have never been registered in the ValueRegistry or in the Literal Registry before)")
    print("Disabling delayed parameter check")
    LiteralRegistry().allow_delayed_parameter_check = True
    ValueRegistry().allow_delayed_parameter_check = True
    print("Test#A: Level 1 (FunctionalLiteral with an unregistered parameter)")
    def function_literal_ok(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_literal_ok):
        return function_literal_ok
    try:
        LiteralRegistry().add_function(function_literal_ok)
        LiteralRegistry().add_function(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except AssertionError as e:
        print("AssertionError raised: "+str(e))
        print("OK\n\n")
    else:
        raise Exception("An AssertionError should have been raised (by the parameter_check method of LiteralRegistry)")



    print("Test#B: Level 1 (FunctionalValue with an unregistered parameter)")
    def function_value_ok(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_value_ok):
        return function_value_ok
    try:
        ValueRegistry().add_function(function_value_ok)
        ValueRegistry().add_function(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except AssertionError as e:
        print("AssertionError raised: "+str(e))
        print("OK\n\n")
    else:
        raise Exception("An AssertionError should have been raised (by the parameter_check method of ValueRegistry)")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: Registry reset")
    print("# of items in ValueRegistry %d" % (len(ValueRegistry())))
    ValueRegistry().reset()
    print("# of items in ValueRegistry after reset %d" % (len(ValueRegistry())))
    assert len(ValueRegistry()) == 0, "# of items in ValueRegistry should be 0 after reset"

    print("# of items in LiteralRegistry %d" % (len(LiteralRegistry())))
    LiteralRegistry().reset()
    print("# of items in LiteralRegistry after reset %d" % (len(LiteralRegistry())))
    assert len(LiteralRegistry()) == 0, "# of items in ValueRegistry should be 0 after reset"
    print("OK")


#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: delayed parameter check. The FunctionalLiteral and FunctionalValue will use parameters not yet declared, which should raise a KeyError exception upon the 'get' call. Then the parameters will be added and the delayed check performed, allowing the subsequent 'get' call to succeed.")

    def mixed_function_value_using_also_literals(simple_literal, function_literal, simple_value, function_value):
        if simple_literal:
            if function_literal:
                return function_value - simple_value
        return 0
    ValueRegistry().add_function(mixed_function_value_using_also_literals)
    
    print("Evaluating: %s\n (should raise a KeyError)" % (ValueRegistry().get_instance("mixed_function_value_using_also_literals")))
    try:
        value = ValueRegistry()["mixed_function_value_using_also_literals"]
    except KeyError as ke:
        print("KeyError raised: "+str(ke))
        print("OK")
    else:
        raise Exception("A KeyError should have been raised (by the get method of ValueRegistry)")
    print("\n\n")

    def mixed_function_literal_using_also_values(simple_literal, function_literal, simple_value, function_value):
        if simple_literal:
            if function_literal:
                return function_value - simple_value > 0
        return False
    LiteralRegistry().add_function(mixed_function_literal_using_also_values)
    print("Evaluating: %s\n (should raise a KeyError)" % (LiteralRegistry().get_instance("mixed_function_literal_using_also_values")))
    try:
        value = LiteralRegistry()["mixed_function_value_using_also_literals"]
    except KeyError as ke:
        print("KeyError raised: "+str(ke))
        print("OK")
    else:
        raise Exception("A KeyError should have been raised (by the get method of LiteralRegistry)")
    print("\n\n")

    LiteralRegistry()["simple_literal"] = True
    ValueRegistry()["simple_value"] = math.pi

    def function_value():
        return math.pi * 2
    ValueRegistry().add_function(function_value)

    def function_literal():
        return True
    LiteralRegistry().add_function(function_literal)

    print("Evaluating: %s\nValue: %s (should be PI)" % (ValueRegistry().get_instance("mixed_function_value_using_also_literals"), ValueRegistry()["mixed_function_value_using_also_literals"]))
    print("\n\n")
    
    print("Evaluating: %s\nValue: %s (should be True)" % (LiteralRegistry().get_instance("mixed_function_literal_using_also_values"), LiteralRegistry()["mixed_function_literal_using_also_values"]))
    print("\n\n")

'''
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    
    print("Tests: Check for recursive FunctionalLiterals (FunctionalLiterals that at some point along the evaluation tree call themselves, they should be NOT ALLOWED)")
    print("Test#1A: Level 0 (FunctionalLiteral itself in its own parameter list)")
    def function_literal_with_itself_in_parameters(function_literal_with_itself_in_parameters):
        return None
    try:
        LiteralRegistry().add_function_literal(function_literal_with_itself_in_parameters)
    except KeyError:
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of LiteralRegistry)")
    


    print("Test#1B: Level 0 (FunctionBalue itself in its own parameter list)")
    def function_value_with_itself_in_parameters(function_value_with_itself_in_parameters):
        return False
    try:
        ValueRegistry().add_function_value(function_value_with_itself_in_parameters)
    except KeyError:
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of ValueRegistry)")






    print("Test#2A: Level 1 (FunctionalLiteral itself in the parameter list of one of its parameters)")
    def function_literal_ok(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_literal_ok):
        return function_literal_ok
    try:
        LiteralRegistry().add_function_literal(function_literal_ok)
        LiteralRegistry().add_function_literal(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except KeyError:
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of LiteralRegistry)")



    print("Test#2B: Level 1 (FunctionalValue itself in the parameter list of one of its parameters)")
    def function_value_ok(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return False
    def function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_value_ok):
        return function_value_ok
    try:
        LiteralRegistry().add_function_literal(function_value_ok)
        ValueRegistry().add_function_value(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except KeyError:
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the parameter_check method of ValueRegistry)")






    print("Test#3A: Level 2 (FunctionalLiteral itself in the parameter list of a parameter in the parameter list of one of its parameters)")
    def function_literal_ok():
        return False
    def function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_literal_ok):
        return function_literal_ok
    try:
        LiteralRegistry().add_function_literal(function_literal_with_itself_in_the_parameter_list_of_one_of_its_parameters)
    except AssertionError:
        print("OK\n\n")
    else:
        raise Exception("An AssertionError should have been raised (by the parameter_check method of LiteralRegistry)")



    print("Test#3A: Level 2 (FunctionalValue itself in the parameter list of a parameter in the parameter list of one of its parameters)")
    def function_value_ok():
        return False
    def function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters(function_value_ok):
        return function_value_ok
    def function_value_with_itself_in_the_parameter_list_of_one_of_a_parameter_in_the_parameter_list_of_one_of_its_parameters(function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters):
        return function_value_with_itself_in_the_parameter_list_of_one_of_its_parameters
    try:
        ValueRegistry().add_function_value(function_value_with_itself_in_the_parameter_list_of_one_of_a_parameter_in_the_parameter_list_of_one_of_its_parameters)
    except AssertionError:
        print("OK\n\n")
    else:
        raise Exception("An AssertionError should have been raised (by the parameter_check method of ValueRegistry)")
'''
#TODO Test functions that set Values or Literals