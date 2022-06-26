import os
import sys

sys.path.append( os.path.dirname( os.path.dirname( os.path.abspath(__file__) ) ) )

from lib.registries.action import ActionRegistry
from lib.registries.literals import LiteralRegistry
from lib.registries.values import ValueRegistry
from lib.dfa.dfa import DFA
from lib.dfa.dfa_handler import DFAHandler, remove_initial_dummy_state
from lib.dfa.LTL import LTLRule


if __name__ == "__main__":

    ActionRegistry(robot_idle_skill="Idle")
    ActionRegistry()["action_kick_ball"] = "Kick"
    ActionRegistry()["action_reach_ball"] = "ReachBall"

    ValueRegistry()["ball_distance"] = 100
    ValueRegistry()["ball_distance_threshold"] = 50
    def is_robot_near_ball(ball_distance, ball_distance_threshold):
        return ball_distance < ball_distance_threshold
        
    LiteralRegistry().add_function(is_robot_near_ball)

    print("Tests: LTLRule construction")
    ltl_formula_1_str = "G(is_robot_near_ball -> action_reach_ball)"
    print("Testing formula: %s" % (ltl_formula_1_str))
    print("Using the DFA.DFA_from_LTL_formula class method: %s" % (ltl_formula_1_str))
    try:
        ltl_rule = LTLRule(ltl_formula_1_str)
        dfa_1 = DFA.DFA_from_LTL_rule(ltl_rule)
        print(dfa_1)
        dfa_1.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", "DFA1.png"), show_plot = False)
    except AssertionError as e:
        raise e
    else:
        print("OK")
    print("\n")

    print("Using the DFA.DFA_from_LTL_formula_string class method: %s" % (ltl_formula_1_str))
    try:
        dfa_2 = DFA.DFA_from_LTL_formula_string(ltl_formula_1_str)
        print(dfa_2)
        dfa_2.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", "DFA2.png"), show_plot = False)
    except AssertionError as e:
        raise e
    else:
        print("OK")
    print("\n")


    ltl_formula_2_str = "G((is_robot_near_ball -> action_kick_ball) && (!is_robot_near_ball -> action_reach_ball))"
    print("Using more complex formula: %s" % (ltl_formula_2_str))
    try:
        dfa_3 = DFA.DFA_from_LTL_formula_string(ltl_formula_2_str)
        print(dfa_3)
        dfa_3.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", "DFA3.png"), show_plot = False)
    except AssertionError as e:
        raise e
    else:
        print("OK")
    print("\n")


    #Test DFAHandler
    ltl_formula_3_str = "G((is_robot_near_ball && action_kick_ball) || (!is_robot_near_ball && action_reach_ball))"
    print("Creating DFA from formula: %s" % (ltl_formula_3_str))
    try:
        dfa_handler = DFAHandler(DFA.DFA_from_LTL_formula_string(ltl_formula_3_str))
    except AssertionError as e:
        raise e
    else:
        print("OK")

    ltl_formula_3_str = "G((is_robot_near_ball && action_kick_ball) || (!is_robot_near_ball && action_reach_ball))"
    print("Creating DFA from formula: %s with post-processing step 'remove_initial_dummy_state'" % (ltl_formula_3_str))
    try:
        dfa4 = DFA.DFA_from_LTL_formula_string(ltl_formula_3_str)
        dfa4.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", "DFA4.png"), show_plot = False)
        dfa_handler = DFAHandler(dfa4, dfa_postprocessing_functions = [remove_initial_dummy_state])
        dfa_handler.dfa.plot(save_to=os.path.join(os.path.dirname(os.path.abspath(__file__)), "dfa_preview", "DFA4_postprocessed.png"), show_plot = False)
    except AssertionError as e:
        raise e
    else:
        print("OK")

    print("Getting next action from DFA")
    try:
        current_action = dfa_handler.get_next_action()
    except AssertionError as e:
        raise e
    else:
        print("OK")
    print("Got current action: %s (should be 'ReachBall')" % (str(current_action)))

    print("Now setting ball distance to 40")
    ValueRegistry()["ball_distance"] = 40
    print("Getting next action from DFA")
    try:
        current_action = dfa_handler.get_next_action()
    except AssertionError as e:
        raise e
    else:
        print("OK")

    print("Got current action: %s (should be 'Kick')" % (str(current_action)))