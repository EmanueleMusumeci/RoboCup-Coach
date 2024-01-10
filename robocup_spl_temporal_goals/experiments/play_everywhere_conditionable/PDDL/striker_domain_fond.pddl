;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain for Play Anywhere setting ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain robocup)
  (:requirements :strips :typing :non-deterministic)
  (:types movable location)
  (:constants robot1 ball - movable strikerposition ballposition kickingposition goaltarget - location)

  (:predicates 
           (is-robot ?mov - movable)
           (is-ball ?mov - movable)
		   (isat ?mov - movable ?loc - location)
           (goalscored)
		   (ballsafe)
		   (opponent-goal-available)
		   
		   (fluentopponentnear)
		   (fluent-opponent-goal-landmark)
		   (fluent-own-goal-landmark)
	       (fluent-jolly-available)
		   )

  (:action move-robot
	    :parameters (?rob - movable ?from - location ?to - location)
	    :precondition (and 
			(is-robot ?rob) 
			(isat ?rob ?from) 
			(not (isat ?rob ?to))
		)
	    :effect
	    (and 
			(not (isat ?rob ?from)) 
			(isat ?rob ?to)
		)
  )

  (:action reach-ball
  		:parameters (?rob - movable ?b - movable ?from - location ?to - location)
	    :precondition (and 
			(is-robot ?rob) 
			(is-ball ?b) 
			(isat ?rob ?from) 
			(not (isat ?rob ?to))
			(isat ?b ?to)
		)
		
	    :effect
	    (and 
			(not (isat ?rob ?from)) 
			(isat ?rob ?to)
			(oneof
				(and (fluentopponentnear) (not (ballsafe)))
				(and (not (fluentopponentnear)) (ballsafe))
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
	     :precondition (and (is-ball ?b) (is-robot ?rob) (isat ?rob ?from) (not (isat ?rob ?to)) (isat ?b ?from) (not (isat ?b ?to)))
	     :effect
	     (and (isat ?rob ?from) (not (isat ?b ?from)) (isat ?b ?to))
  )



  (:action carry-ball
	     :parameters (?rob - movable ?b - movable ?from - location ?to - location)
	     :precondition (and 
		 	(is-ball ?b) 
		 	(is-location ?from)
		 	(is-location ?to) 
			(is-robot ?rob) 
			(isat ?rob ?from) 
			(not (isat ?rob ?to)) 
			(isat ?b ?from) 
			(not (isat ?b ?to))
			(opponent-goal-available)
		 )
	     :effect
	     (and 
		 	(not (isat ?rob ?from)) 
			(isat ?rob ?to) 
			(not (isat ?b ?from)) 
			(isat ?b ?to)
			(oneof
				(and (fluentopponentnear) (not (ballsafe)))
				(and (not (fluentopponentnear)) (ballsafe))
			)
		)
  )

  (:action kick-to-goal
	     :parameters (?rob - movable ?b - movable)
	     :precondition 
		 (and 
		 	(is-ball ?b) 
			(is-robot ?rob) 
			(isat ?rob kickingposition) 
			(isat ?b kickingposition) 
			(ballsafe)
			(opponent-goal-available)
		)
	     :effect
	     (and (isat ?b goaltarget) (not (isat ?b kickingposition)) (goalscored) (not (ballsafe)))
  )


  (:action carry-to-goal
	     :parameters (?rob - movable ?b - movable)
	     :precondition 
		 (and 
		 	(is-ball ?b) 
			(is-robot ?rob) 
			(isat ?rob kickingposition) 
			(isat ?b kickingposition) 
			(opponent-goal-available)
			(not (ballsafe))
		)
	     :effect
	     (and (isat ?b goaltarget) (not (isat ?b kickingposition)) (goalscored) (ballsafe))
  )

  (:action pass-ball
	     :parameters (?rob - movable ?b - movable ?loc - location) 
	     :precondition 
		 	(and 
				(is-ball ?b)
				(is-robot ?rob)
				(isat ?rob ?loc)
				(isat ?b ?loc)
				(not (ballsafe))
				(jolly-available)
			)
	     :effect
			(and (ballpassed))
  )
  
;  (:action wait-for-landmarks
;	    :parameters ()
;	    :precondition 
;       		(and (is-ball ?b) (is-robot ?rob) (isat ?rob ?loc) (isat ?b ?loc))
;       	:effect
;		(and
;			(oneof
;				(fluent-opponent-goal-landmark)
;				(not (fluent-opponent-goal-landmark))
;			)
;		)
;  )

  
)
