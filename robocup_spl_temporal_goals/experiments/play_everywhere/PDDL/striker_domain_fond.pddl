;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain for Play Anywhere setting ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocup-deterministic)
  (:requirements :strips :typing)
  (:types movable location)
  (:predicates 
           (is-robot ?mov - movable)
           (is-ball ?mov - movable)
           (is-at ?mov - movable ?loc - location)
           (goal-scored)
		   (ball-safe)
		   (opponent-goal-available)

		   (fluent-opponent-near)
		   (fluent-opponent-goal-landmark)
		   (fluent-own-goal-landmark)
	       )

  (:action move-robot
	    :parameters (?rob - movable ?from - location ?to - location)
	    :precondition (and 
			(is-robot ?rob) 
			(is-at ?rob ?from) 
			(not (is-at ?rob ?to))
		)
	    :effect
	    (and 
			(not (is-at ?rob ?from)) 
			(is-at ?rob ?to)
		)
  )

  (:action reach-ball
  		:parameters (?rob - movable ?b - movable ?from - location ?to - location)
	    :precondition (and 
			(is-robot ?rob) 
			(is-ball ?b) 
			(is-at ?rob ?from) 
			(not (is-at ?rob ?to))
			(is-at ?b ?to)
		)
		
	    :effect
	    (and 
			(not (is-at ?rob ?from)) 
			(is-at ?rob ?to)
			(oneof
				(and (fluent-opponent-near) (not (ball-safe)))
				(and (not (fluent-opponent-near)) (ball-safe))
			)
			(oneof
				(and (fluent-opponent-goal-landmark) (opponent-goal-available))
				(and (not (fluent-opponent-goal-landmark)) (opponent-goal-available))
			)
			(oneof
				(and (fluent-jolly-available) (jolly-available))
				(and (not (fluent-jolly-available)) (jolly-available))
			)
		)
  )

  (:action kick-ball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and (is-ball ?b) (is-robot ?rob) (is-at ?rob ?from) (not (is-at ?rob ?to)) (is-at ?b ?from) (not (is-at ?b ?to)))
	     :effect
	     (and (is-at ?rob ?from) (not (is-at ?b ?from)) (is-at ?b ?to))
  )



  (:action carry-ball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and 
		 	(is-ball ?b) 
			(is-robot ?rob) 
			(is-at ?rob ?from) 
			(not (is-at ?rob ?to)) 
			(is-at ?b ?from) 
			(not (is-at ?b ?to))
			(opponent-goal-available)
		 )
	     :effect
	     (and 
		 	(not (is-at ?rob ?from)) 
			(is-at ?rob ?to) 
			(not (is-at ?b ?from)) 
			(is-at ?b ?to)
			(oneof
				(and (fluent-opponent-near) (not (ball-safe)))
				(and (not (fluent-opponent-near)) (ball-safe))
			)
		)
  )

  (:action kick-to-goal
	     :parameters (?rob - movable ?b - movable)
	     :precondition 
		 (and 
		 	(is-ball ?b) 
			(is-robot ?rob) 
			(is-at ?rob kicking-position) 
			(is-at ?b kicking-position) 
			(ball-safe)
			(opponent-goal-available)
		)
	     :effect
	     (and (is-at ?b goal-target) (not (is-at ?b kicking-position)) (goal-scored))
  )

  (:action defend-ball
	    :parameters (?rob - movable ?b - movable ?loc - location)
	    :precondition 
       		(and (is-ball ?b) (is-robot ?rob) (is-at ?rob ?loc) (is-at ?b ?loc) (not (ball-safe)) (not (jolly-available)))
       	:effect
			(and
				(is-ball ?b) (is-robot ?rob) (is-at ?rob ?loc) (is-at ?b ?loc) (ball-safe)
			)
  )

  (:action pass-ball
	     :parameters (?rob - movable ?b - movable ?loc - location) 
	     :precondition 
		 	(and 
				(is-ball ?b)
				(is-robot ?rob)
				(is-at ?rob ?loc)
				(is-at ?b ?loc)
				(not (ball-safe))
				(jolly-available)
			)
	     :effect
			(and (ball-passed))
  )
  
;  (:action wait-for-landmarks
;	    :parameters ()
;	    :precondition 
;       		(and (is-ball ?b) (is-robot ?rob) (is-at ?rob ?loc) (is-at ?b ?loc))
;       	:effect
;		(and
;			(oneof
;				(fluent-opponent-goal-landmark)
;				(not (fluent-opponent-goal-landmark))
;			)
;		)
;  )

  
)
