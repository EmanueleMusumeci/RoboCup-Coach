;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with NonDeterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain jollyrobocupfond)
  (:requirements :strips :non-deterministic)
  (:predicates 
    (fluentisstrikerobstacleblockinggoal)
    (fluentisjollyinposition)
    (fluentisjollyalignedtostriker)
    (jollyinposition)
    (jollyalignedtostriker)
    (jollyavailable)
  )

  (:action checkobstacleposition
	     :parameters () 
	     :precondition 
        (jollyavailable)
       :effect
	     	(oneof
          (fluentisstrikerobstacleblockinggoal)
          (when (not (fluentisstrikerobstacleblockinggoal)) (jollyinposition))
        )
  )
  (:action movetoreceivingposition
	     :parameters ()
	     :precondition 
       (and
          (fluentisstrikerobstacleblockinggoal)
          (not (jollyinposition))
        )
       :effect
        (jollyinposition)
  )

  (:action turntostriker
	     :parameters ()
	     :precondition 
       (and
          (jollyinposition)
          (or
            (not(jollyalignedtostriker))
            (not(fluentisstrikerobstacleblockinggoal))
          )
       )
       :effect
       (and 
          (jollyalignedtostriker)
       )
  ) 

  (:action checkjollyposition
	     :parameters ()
	     :precondition 
       (not (jollypositionok))
       :effect
        (and
          (when (fluentisjollyinposition) (jollypositionok))
          (when (jollyinposition) (jollypositionok))
        )
  )

  (:action checkjollyrotation
	     :parameters ()
	     :precondition 
       (not (jollyrotationok))
       :effect
        (and
          (when (fluentisjollyalignedtostriker) (jollyrotationok))
          (when (jollyalignedtostriker) (jollyrotationok))
        )
  )

  (:action checkjollyready
	     :parameters ()
	     :precondition
       (and 
          (jollypositionok)
          (jollyrotationok)
        )
       :effect
        (jollyready)
  )
  
)
