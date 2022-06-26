(define (domain robocupdeterministic)
    (:requirements :conditional-effects :derived-predicates :negative-preconditions :strips :typing)
    (:types location movable)
    (:predicates (Oisat_robot_wpc17r17) (adjacent ?loc1 ?loc2) (isat ?mov ?loc) (isball ?mov) (isrobot ?mov) (val_Oisat_robot_wpc17r17) (val_isat_ball_wpc34r17) (val_isat_ball_wpc34r17-and-Oisat_robot_wpc17r17) (val_isat_robot_wpc17r17))
    (:derived (val_Oisat_robot_wpc17r17) (or (val_isat_robot_wpc17r17) (Oisat_robot_wpc17r17)))
     (:derived (val_isat_ball_wpc34r17) (isat ball wpc34r17))
     (:derived (val_isat_ball_wpc34r17-and-Oisat_robot_wpc17r17) (and (val_isat_ball_wpc34r17) (val_Oisat_robot_wpc17r17)))
     (:derived (val_isat_robot_wpc17r17) (isat robot wpc17r17))
    (:action carryball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc17r17) (Oisat_robot_wpc17r17)))
    )
     (:action kickball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc17r17) (Oisat_robot_wpc17r17)))
    )
     (:action moverobot
        :parameters (?rob ?from ?to)
        :precondition (and (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to)) (when (val_Oisat_robot_wpc17r17) (Oisat_robot_wpc17r17)))
    )
)