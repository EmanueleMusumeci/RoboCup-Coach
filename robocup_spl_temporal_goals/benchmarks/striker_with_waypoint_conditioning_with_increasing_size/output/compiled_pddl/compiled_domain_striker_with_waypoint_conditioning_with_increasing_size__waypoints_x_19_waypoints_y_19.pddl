(define (domain robocupdeterministic)
    (:requirements :conditional-effects :derived-predicates :negative-preconditions :strips :typing)
    (:types location movable)
    (:predicates (Oisat_robot_wpc9r9) (adjacent ?loc1 ?loc2) (isat ?mov ?loc) (isball ?mov) (isrobot ?mov) (val_Oisat_robot_wpc9r9) (val_isat_ball_wpc18r9) (val_isat_ball_wpc18r9-and-Oisat_robot_wpc9r9) (val_isat_robot_wpc9r9))
    (:derived (val_Oisat_robot_wpc9r9) (or (val_isat_robot_wpc9r9) (Oisat_robot_wpc9r9)))
     (:derived (val_isat_ball_wpc18r9) (isat ball wpc18r9))
     (:derived (val_isat_ball_wpc18r9-and-Oisat_robot_wpc9r9) (and (val_isat_ball_wpc18r9) (val_Oisat_robot_wpc9r9)))
     (:derived (val_isat_robot_wpc9r9) (isat robot wpc9r9))
    (:action carryball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc9r9) (Oisat_robot_wpc9r9)))
    )
     (:action kickball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc9r9) (Oisat_robot_wpc9r9)))
    )
     (:action moverobot
        :parameters (?rob ?from ?to)
        :precondition (and (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to)) (when (val_Oisat_robot_wpc9r9) (Oisat_robot_wpc9r9)))
    )
)