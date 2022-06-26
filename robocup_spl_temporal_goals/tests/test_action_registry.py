from multiprocessing.sharedctypes import Value
import os
import sys

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )

from lib.registries.action import Action, ActionRegistry
from lib.registries.values import ValueRegistry

if __name__ == "__main__":
#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: ActionRegistry creation")
    action_registry = ActionRegistry(robot_idle_skill="Idle")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: create ConstantParameterAction")


    print("Test A: create action using only skill name")
    ActionRegistry()["action_reach_position"] = "ReachPosition"
    print(ActionRegistry().get_instance("action_reach_position").get_parameter_string())

    print("Test B: create action using tuple with constant parameters")
    ActionRegistry()["action_reach_field_center"] = ("ReachPosition", [("X", 0),("Y", 0)])
    print(ActionRegistry().get_instance("action_reach_field_center").get_parameter_string())

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: create DynamicParameterAction")

    print("Test A: create action using tuple with SimpleValue parameter names")
    ActionRegistry()["action_reach_field_center_parametric"] = ("ReachPosition", ["field_center_X","field_center_Y"])
    
    print("Test B: create action using tuple with FunctionalValue parameter names")
    ActionRegistry()["action_reach_striker_follow_position_parametric"] = ("ReachPosition", ["offset_x_from_striker","offset_y_from_striker"])

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: DynamicParameterAction get_parameter_string methods with SimpleValue parameters")

    print("Test A: get parameter string for action using SimpleValue parameter names, with missing parameters")
    try:
        print(ActionRegistry().get_instance("action_reach_field_center_parametric").get_parameter_string())
    except KeyError as e:
        print("KeyError raised: "+str(e))
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the get_parameter_string method of Action)")


    ValueRegistry()["field_center_X"] = 0
    ValueRegistry()["field_center_Y"] = 0


    print("Test B: get parameter string for action using SimpleValue parameter names with correct parameters")
    try:
        print(ActionRegistry().get_instance("action_reach_field_center_parametric").get_parameter_string())
    except KeyError as e:
        raise
    else:
        print("OK\n\n")
   

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: DynamicParameterAction get_parameter_string methods with FunctionalValue parameters")

    print("Test A: get parameter string for action using FunctionalValue parameter names, with missing parameters")
    try:
        print(ActionRegistry().get_instance("action_reach_striker_follow_position_parametric").get_parameter_string())
    except KeyError as e:
        print("KeyError raised: "+str(e))
        print("OK\n\n")
    else:
        raise Exception("A KeyError should have been raised (by the get_parameter_string method of Action)")
    

    ValueRegistry()["relative_x_from_striker"] = 1000
    ValueRegistry()["relative_y_from_striker"] = 500
    ValueRegistry()["supporter_stay_behind_striker"] = True
    ValueRegistry()["supporter_stay_to_the_left_of_striker"] = True

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


    print("Test B: get parameter string for action using FunctionalValue parameter names, with missing parameters")
    try:
        print(ActionRegistry().get_instance("action_reach_striker_follow_position_parametric").get_parameter_string())
    except KeyError as e:
        raise
    else:
        print("OK\n\n")

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: Assign ValueRegistry alias")

    print("Test: Assigning alias 'relative_x' to value 'relative_x_from_striker'")
    ValueRegistry().register_alias(item_name="relative_x_from_striker", alias_name="relative_x")
    
    print(ValueRegistry())
    print("'relative_x_from_striker' = %d, 'relative_x' = %d (should be %d)" % (ValueRegistry()["relative_x_from_striker"], ValueRegistry()["relative_x"], ValueRegistry()["relative_x_from_striker"]))
    assert ValueRegistry()["relative_x_from_striker"] == ValueRegistry()["relative_x"]

#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: Create DynamicParameterAction from ActionTemplate that retains a subset of the provided parameters")

    print("Test A: Register new ActionTemplate in ActionRegistry")
    ActionRegistry().register_action_template("move-robot", robot_skill_name="ReachPosition", parameter_indices=[1,2])

    print("Test B: get registered ActionTemplate")    
    try:
        print(ActionRegistry().get_action_template("move-robot"))
    except AssertionError as e:
        raise
    else:
        print("OK\n\n")

    print("Test C: create action instance from registered ActionTemplate")
    new_action : Action = None 
    try:
        ValueRegistry()["useful_parameter1"] = ""
        ValueRegistry()["useful_parameter2"] = ""
        ValueRegistry()["useless_parameter2"] = ""
        ValueRegistry()["useless_parameter2"] = ""
        new_action = ActionRegistry().create_action_from_template("move-robot", parameter_list=["useless_parameter1", "useful_parameter1", "useful_parameter2", "useless_parameter2"])
        print(new_action)
    except AssertionError as e:
        raise e
    except KeyError as e:
        raise e
    else:
        assert new_action is not None
        print("Action created from template with parameters: %s" % (str(new_action.parameters)))
        assert new_action.parameters == ["useful_parameter1", "useful_parameter2"]
        print("OK\n\n")


#-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_-_
    print("-_"*50+"\n")
    print("Tests: Create DynamicParameterAction from ActionTemplate that retains all provided parameters")

    print("Test A: Register new ActionTemplate in ActionRegistry")
    ActionRegistry().register_action_template("move-robot-all-parameters", robot_skill_name="ReachPosition", parameter_indices=None)

    print("Test B: get registered ActionTemplate")    
    try:
        print(ActionRegistry().get_action_template("move-robot-all-parameters"))
    except AssertionError as e:
        raise
    else:
        print("OK\n\n")

    print("Test C: create action instance from registered ActionTemplate")
    new_action : Action = None 
    try:
        ValueRegistry()["useful_parameter1"] = ""
        ValueRegistry()["useful_parameter2"] = ""
        ValueRegistry()["useful_parameter3"] = ""
        ValueRegistry()["useful_parameter4"] = ""
        new_action = ActionRegistry().create_action_from_template("move-robot-all-parameters", parameter_list=["useful_parameter1", "useful_parameter2", "useful_parameter3", "useful_parameter4"])
        print(new_action)
    except AssertionError as e:
        raise e
    except KeyError as e:
        raise e
    else:
        assert new_action is not None
        print("Action created from template with parameters: %s" % (str(new_action.parameters)))
        assert new_action.parameters == ["useful_parameter1", "useful_parameter2", "useful_parameter3", "useful_parameter4"]
        print("OK\n\n")
