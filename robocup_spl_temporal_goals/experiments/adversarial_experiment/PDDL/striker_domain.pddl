;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocupdeterministic)
  (:requirements :strips :typing :non-deterministic)
  (:types movable location)


  (:predicates 
           (isrobot ?mov - movable)
           (isball ?mov - movable)
           (isat ?mov - movable ?loc - location)
           (goalscored)
           (ballkickedto)
           (balldribbledto)
           (movedto)
           (precedes)
	       )

  (:action moverobot
	     :parameters (?rob - movable ?from - location ?to - location)
	     :precondition (and (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)))
	     :effect
	     (and (not (isat ?rob ?from)) (isat ?rob ?to) (movedto ?to))
  )

  (:action kickball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (precedes ?from ?to) (initialposition ?from)))
	     :effect
  	    (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to) (ballkickedto ?to))
  )

  (:action carryball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (isball ?b) (isrobot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)) (or (precedes ?from ?to) (initialposition ?from)))
	     :effect
	     (and (not (isat ?rob ?from)) (isat ?rob ?to) (not (isat ?b ?from)) (isat ?b ?to) (balldribbledto ?to))
  )
  
)
