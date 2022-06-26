;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain striker-robocup-fond)
  (:requirements :strips :non-deterministic)
  (:predicates 
		   (fluent-is-striker-obstacle-blocking-goal)
		   (fluent-is-jolly-available)
		   (fluent-is-jolly-in-position)
           (striker-should-dribble-opponent)
		   (goal-scored)
		   (ball-passed)
		   (striker-has-ball)
		   (striker-can-kick)
	       )

  (:action move-to-ball
	     :parameters ()
	     :precondition 
		 (and 
			(not (striker-has-ball))
		 )
	     :effect
	     (oneof 
		 	(and (not (fluent-is-striker-obstacle-blocking-goal)) (not (fluent-is-jolly-available)) (not(fluent-is-jolly-in-position)) (striker-has-ball))
		 	(and (not (fluent-is-striker-obstacle-blocking-goal)) (fluent-is-jolly-available) (striker-has-ball))
		 	(and (fluent-is-striker-obstacle-blocking-goal) (fluent-is-jolly-available) (not (fluent-is-jolly-in-position)) (striker-has-ball))
		 	(and (fluent-is-striker-obstacle-blocking-goal) (fluent-is-jolly-available) (fluent-is-jolly-in-position) (striker-has-ball))
		 	(and (fluent-is-striker-obstacle-blocking-goal) (not(fluent-is-jolly-available)) (not(fluent-is-jolly-in-position)) (striker-has-ball))
		 )
  )

  (:action move-with-ball-to-kicking-position
		 :parameters ()
	     :precondition 
		 (and 
		 	(not (fluent-is-striker-obstacle-blocking-goal))
			(striker-has-ball)
		 )
	     :effect
	     	(striker-can-kick)
		 
  )

  (:action kick-to-goal
	     :parameters ()
	     :precondition 
		 	(and 
			 	(not (fluent-is-striker-obstacle-blocking-goal)) 
				(striker-can-kick)
			)
	     
		 :effect
	     	(and 
			 (goal-scored)
			)
  )

  (:action wait-for-jolly
	     :parameters ()
	     :precondition 
		 	(and 
		 		(fluent-is-striker-obstacle-blocking-goal) 
				(fluent-is-jolly-available)
				(not(fluent-is-jolly-in-position))
				(not(striker-can-kick))
			)
	     :effect
		 	(oneof
				(and (fluent-is-jolly-in-position) (fluent-is-jolly-available) (fluent-is-striker-obstacle-blocking-goal))
				(and (not (fluent-is-jolly-in-position)) (fluent-is-jolly-available) (fluent-is-striker-obstacle-blocking-goal))
			)
  )

  (:action pass-ball-to-jolly
	     :parameters ()
	     :precondition 
		 	(and 
			 	(fluent-is-striker-obstacle-blocking-goal)
				(fluent-is-jolly-in-position)
				(fluent-is-jolly-available)
			)
	     :effect
			(and (ball-passed))
  )

  (:action dribble-opponent
	     :parameters ()
	     :precondition 
		 	(and 
		 		(fluent-is-striker-obstacle-blocking-goal) 
				(not(fluent-is-jolly-available))
			)
		 :effect
	     	(striker-attempting-dribble)
  )
)
