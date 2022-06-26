;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain striker-robocup-fond)
  (:requirements :strips :non-deterministic)
  (:predicates 
		   (fluent-obstacle-blocking-goal)
		   (fluent-jolly-available)
		   (fluent-jolly-in-position)
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
		 	(and (not (fluent-obstacle-blocking-goal)) (not (fluent-jolly-available)) (not(fluent-jolly-in-position)) (striker-has-ball))
		 	(and (not (fluent-obstacle-blocking-goal)) (fluent-jolly-available) (striker-has-ball))
		 	(and (fluent-obstacle-blocking-goal) (fluent-jolly-available) (not (fluent-jolly-in-position)) (striker-has-ball))
		 	(and (fluent-obstacle-blocking-goal) (fluent-jolly-available) (fluent-jolly-in-position) (striker-has-ball))
		 	(and (fluent-obstacle-blocking-goal) (not(fluent-jolly-available)) (not(fluent-jolly-in-position)) (striker-has-ball))
		 )
  )

  (:action carry-ball-to-kick
		 :parameters ()
	     :precondition 
		 (and 
		 	(not (fluent-obstacle-blocking-goal))
			(striker-has-ball)
		 )
	     :effect
	     	(striker-can-kick)
		 
  )

  (:action kick-to-goal
	     :parameters ()
	     :precondition 
		 	(and 
			 	(not (fluent-obstacle-blocking-goal)) 
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
		 		(fluent-obstacle-blocking-goal) 
				(fluent-jolly-available)
				(not(fluent-jolly-in-position))
				(not(striker-can-kick))
			)
	     :effect
		 	(oneof
				(and (fluent-jolly-in-position) (fluent-jolly-available) (fluent-obstacle-blocking-goal))
				(and (not (fluent-jolly-in-position)) (fluent-jolly-available) (fluent-obstacle-blocking-goal))
			)
  )

  (:action pass-ball-to-jolly
	     :parameters ()
	     :precondition 
		 	(and 
			 	(fluent-obstacle-blocking-goal)
				(fluent-jolly-in-position)
				(fluent-jolly-available)
			)
	     :effect
			(and (ball-passed))
  )

  (:action dribble-opponent
	     :parameters ()
	     :precondition 
		 	(and 
		 		(fluent-obstacle-blocking-goal) 
				(not(fluent-jolly-available))
			)
		 :effect
	     	(striker-attempting-dribble)
  )
)
