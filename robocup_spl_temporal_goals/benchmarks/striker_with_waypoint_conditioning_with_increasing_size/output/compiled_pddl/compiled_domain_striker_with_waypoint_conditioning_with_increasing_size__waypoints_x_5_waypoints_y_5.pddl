(define (domain robocupdeterministic)
    (:requirements :conditional-effects :derived-predicates :negative-preconditions :strips :typing)
    (:types location movable)
    (:predicates (Oisat_robot_wpc2r2) (adjacent ?loc1 ?loc2) (isat ?mov ?loc) (isball ?mov) (isrobot ?mov) (val_Oisat_robot_wpc2r2) (val_isat_ball_wpc4r2) (val_isat_ball_wpc4r2-and-Oisat_robot_wpc2r2) (val_isat_robot_wpc2r2))
    (:derived (val_Oisat_robot_wpc2r2) (or (val_isat_robot_wpc2r2) (Oisat_robot_wpc2r2)))
     (:derived (val_isat_ball_wpc4r2) (isat ball wpc4r2))
     (:derived (val_isat_ball_wpc4r2-and-Oisat_robot_wpc2r2) (and (val_isat_ball_wpc4r2) (val_Oisat_robot_wpc2r2)))
     (:derived (val_isat_robot_wpc2r2) (isat robot wpc2r2))
    (:action carryball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc2r2) (Oisat_robot_wpc2r2)))
    )
     (:action kickball
        :parameters (?rob ?b ?from ?to)
        :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to)) (when (val_Oisat_robot_wpc2r2) (Oisat_robot_wpc2r2)))
    )
     (:action moverobot
        :parameters (?rob ?from ?to)
        :precondition (and (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (or (adjacent ?from ?to) (adjacent ?to ?from)))
        :effect (and (and (not (isat ?rob ?from)) (isat ?rob ?to)) (when (val_Oisat_robot_wpc2r2) (Oisat_robot_wpc2r2)))
    )
)