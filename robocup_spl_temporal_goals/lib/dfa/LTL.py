from typing import List

from ltlf2dfa.parser.ltlf import LTLfParser
import lark

from lib.registries.literals import LiteralRegistry
from lib.registries.action import ActionRegistry


class LTLRule:
    def __init__(self, formula_string : str, mutual_exclusion_groups = []):
        assert isinstance(formula_string, str)

        parser = LTLfParser()
        try:
            formula = parser(formula_string)
        except lark.exceptions.UnexpectedCharacters as e:
            print("UnexpectedCharacter exception occured in LTLfParser:\n"+"-"*100)
            print(e)
            print("-"*100)
            print("HINTS:\n1) Use only lower-case characters in the formula, upper-case are reserved for LTL operators\n2) The only special character allowed is '_'")
            exit(1)
        except lark.exceptions.UnexpectedToken as e:
            print("UnexpectedToken exception occured in LTLfParser:\n"+"-"*100)
            print(e)
            print("-"*100)
            print("HINTS:\n1) Use only lower-case characters in the formula, upper-case are reserved for LTL operators\n2) You might have used a wrong syntax: use parentheses '(',')' to structure your formula\n3) You migh have used an unsuported LTL operator: check this page for syntax: http://ltlf2dfa.diag.uniroma1.it/ltlf_syntax")
            exit(1)
        except Exception as e:
            raise e

        labels = formula.find_labels()
        assert labels

        self.__literals = []
        self.__actions = []
        #print(labels)
        for label in labels:
            if label in LiteralRegistry():
                self.__literals.append(label)
            elif label in ActionRegistry():
                self.__actions.append(label)
            else:
                raise AssertionError("Label '%s' is not a 'literal' nor an 'action'" % (label))

        assert self.__literals or self.__actions, "The formula contains no literals nor actions! REMEMBER: literals should have a 'literal_' prefix and actions should have an 'action_' prefix"

        self.formula = formula
