;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocupdeterministic)
  (:requirements :strips :typing)
  (:types movable location)


  (:predicates 
           (isrobot ?mov - movable)
           (isball ?mov - movable)
           (isat ?mov - movable ?loc - location)
           (goalscored)
           (highbatteryconsumption)
	       )

  (:action moverobot
	     :parameters (?rob - movable ?from - location ?to - location)
	     :precondition (and (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)))
	     :effect
	     (and (not (isat ?rob ?from)) (isat ?rob ?to))
  )

  (:action kickball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)))
	     :effect
	     (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to))
  )

  (:action carryball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)))
	     :effect
	     (and (not (isat ?rob ?from)) (isat ?rob ?to) (not (isat ?b ?from)) (isat ?b ?to) (highbatteryconsumption))
  )
  
)
