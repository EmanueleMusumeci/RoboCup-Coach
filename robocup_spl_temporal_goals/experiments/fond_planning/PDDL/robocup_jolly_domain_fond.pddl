;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with Non-Deterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain jolly-robocup-fond)
  (:requirements :strips :non-deterministic)
  (:predicates 
    (fluent-obstacle-blocking-goal)
    (fluent-jolly-in-position)
    (fluent-jolly-aligned-to-striker)
    (jolly-in-position)
    (jolly-aligned-to-striker)
    (jolly-available)
  )

  (:action check-obstacle-position
	     :parameters () 
	     :precondition 
        (jolly-available)
       :effect
	     	(oneof
          (fluent-obstacle-blocking-goal)
          (when (not (fluent-obstacle-blocking-goal)) (jolly-in-position))
        )
  )
  (:action move-to-receiving-position
	     :parameters ()
	     :precondition 
       (and
          (fluent-obstacle-blocking-goal)
          (not (jolly-in-position))
        )
       :effect
        (jolly-in-position)
  )

  (:action turn-to-striker
	     :parameters ()
	     :precondition 
       (and
          (jolly-in-position)
          (or
            (not(jolly-aligned-to-striker))
            (not(fluent-obstacle-blocking-goal))
          )
       )
       :effect
       (and 
          (jolly-aligned-to-striker)
       )
  ) 

  (:action check-jolly-position
	     :parameters ()
	     :precondition 
       (not (jolly-position-ok))
       :effect
        (and
          (when (fluent-jolly-in-position) (jolly-position-ok))
          (when (jolly-in-position) (jolly-position-ok))
        )
  )

  (:action check-jolly-rotation
	     :parameters ()
	     :precondition 
       (not (jolly-rotation-ok))
       :effect
        (and
          (when (fluent-jolly-aligned-to-striker) (jolly-rotation-ok))
          (when (jolly-aligned-to-striker) (jolly-rotation-ok))
        )
  )

  (:action check-jolly-ready
	     :parameters ()
	     :precondition
       (and 
          (jolly-position-ok)
          (jolly-rotation-ok)
        )
       :effect
        (jolly-ready)
  )
  
)
