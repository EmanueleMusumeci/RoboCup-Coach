;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocup-deterministic)
  (:requirements :strips :typing)
  (:types movable location)
  (:predicates 
           (is-robot ?mov - movable)
           (is-ball ?mov - movable)
           (is-at ?mov - movable ?loc - location)
           (goal-scored)
	       )

  (:action move-robot
	     :parameters (?rob - movable ?from - location ?to - location)
	     :precondition (and (is-robot ?rob) (is-at ?rob ?from) (not (is-at ?rob ?to)))
	     :effect
	     (and (not (is-at ?rob ?from)) (is-at ?rob ?to))
  )

  (:action kick-ball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (is-ball ?b) (is-robot ?rob) (is-at ?rob ?from) (not (is-at ?rob ?to)) (is-at ?b ?from) (not (is-at ?b ?to)))
	     :effect
	     (and (is-at ?rob ?from) (not (is-at ?b ?from)) (is-at ?b ?to))
  )

  (:action carry-ball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (is-ball ?b) (is-robot ?rob) (is-at ?rob ?from) (not (is-at ?rob ?to)) (is-at ?b ?from) (not (is-at ?b ?to)))
	     :effect
	     (and (not (is-at ?rob ?from)) (is-at ?rob ?to) (not (is-at ?b ?from)) (is-at ?b ?to))
  )

  (:action kick-to-goal
	     :parameters (?rob - movable ?b - movable)
	     :precondition (and (is-ball ?b) (is-robot ?rob) (is-at ?rob kicking-position) (is-at ?b kicking-position) )
	     :effect
	     (and (is-at ?b goal-target) (not (is-at ?b kicking-position)) (goal-scored))
  )
  
)
