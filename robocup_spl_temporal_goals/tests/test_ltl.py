import os
import sys

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )

from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.dfa.LTL import LTLRule

if __name__ == "__main__":
    print("Tests: ActionRegistry creation and print")
    action_registry = ActionRegistry()
    action_registry["action_kick_ball"] = "kick"
    print(action_registry)
    ActionRegistry()["action_reach_ball"] = "reachBall"
    print(ActionRegistry())
    print("\n\n")

    print("Tests: ValueRegistry creation and print")
    value_registry = ValueRegistry()

    value_registry["distance"] = 100
    print(value_registry)

    ValueRegistry()["threshold1"] = 200
    ValueRegistry()["threshold2"] = 50
    print(ValueRegistry())
    print("\n\n")

    print("Tests: LiteralRegistry creation and print")
    literal_registry = LiteralRegistry()
    literal_registry["this_value_should_be_true"] = True
    print(literal_registry)

    LiteralRegistry()["this_value_should_be_false"] = False
    print(LiteralRegistry())
    print("\n\n")

    print("Tests: SimpleLiteral evaluation")
    print("this_value_should_be_true AND this_value_should_be_false SHOULD BE False: "+str(literal_registry["this_value_should_be_true"] and literal_registry["this_value_should_be_false"]))
    print("this_value_should_be_true OR this_value_should_be_false SHOULD BE True: "+str(literal_registry["this_value_should_be_true"] or literal_registry["this_value_should_be_false"]))
    print("\n\n")

    print("Tests: FunctionalLiteral evaluation")
    def distance_lower_than_threshold1(distance, threshold1):
        return distance < threshold1

    def distance_lower_than_threshold2(distance, threshold2):
        return distance < threshold2

    LiteralRegistry().add_function(distance_lower_than_threshold1)
    LiteralRegistry().add_function(distance_lower_than_threshold2)
    print(LiteralRegistry())
    print("\n\n")


    print("Tests: LTLRule construction")
    ltl_formula_1_str = "G(a)"
    print("Testing formula: %s" % (ltl_formula_1_str))
    try:
        ltl_formula = LTLRule(ltl_formula_1_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register literal 'a' or literal 'literal_a'")
    print("\n")
        
    ltl_formula_2_str = "G(distance_lower_than_threshold1)"
    print("Testing formula: %s" % (ltl_formula_2_str))
    try:
        ltl_formula = LTLRule(ltl_formula_2_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")
        
    ltl_formula_3_str = "G(distance_lower_than_threshold1 -> a)"
    print("Testing formula: %s" % (ltl_formula_3_str))
    try:
        ltl_formula = LTLRule(ltl_formula_3_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register action 'a' or action 'action_a'")
    print("\n")

    ltl_formula_3_str = "G(distance_lower_than_threshold1 -> action_a)"
    print("Testing formula: %s" % (ltl_formula_3_str))
    try:
        ltl_formula = LTLRule(ltl_formula_3_str)
    except AssertionError:
        print("OK")
    else:
        raise Exception("Should raise an AssertionError because we did not register action 'a' or action 'action_a'")
    print("\n")
        
    ltl_formula_4_str = "G(distance_lower_than_threshold1 -> kick_ball)"
    print("Testing formula: %s" % (ltl_formula_4_str))
    try:
        ltl_formula = LTLRule(ltl_formula_4_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")

    ltl_formula_4_str = "G(distance_lower_than_threshold1 -> action_kick_ball)"
    print("Testing formula: %s" % (ltl_formula_4_str))
    try:
        ltl_formula = LTLRule(ltl_formula_4_str)
    except AssertionError:
        raise Exception("Should not raise an error")
    else:
        print("OK")
    print("\n")
        