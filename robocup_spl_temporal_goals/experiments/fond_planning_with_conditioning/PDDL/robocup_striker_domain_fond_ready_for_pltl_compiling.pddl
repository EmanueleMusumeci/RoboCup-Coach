;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; RoboCup Domain with NonDeterministic Actions ;;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain strikerrobocupfond)
  (:requirements :strips :non-deterministic)
  (:predicates 
		   (fluentisstrikerobstacleblockinggoal)
		   (fluentisjollyavailable)
		   (fluentisjollyinposition)
           (strikershoulddribbleopponent)
		   (goalscored)
		   (ballpassed)
		   (strikerhasball)
		   (strikercankick)
	       )

  (:action movetoball
	     :parameters () 
	     :precondition 
		 (and 
			(not (strikerhasball))
		 )
	     :effect
	     (oneof 
		 	(and (not (fluentisstrikerobstacleblockinggoal)) (not (fluentisjollyavailable)) (not(fluentisjollyinposition)) (strikerhasball))
		 	(and (not (fluentisstrikerobstacleblockinggoal)) (fluentisjollyavailable) (strikerhasball))
		 	(and (fluentisstrikerobstacleblockinggoal) (fluentisjollyavailable) (not (fluentisjollyinposition)) (strikerhasball))
		 	(and (fluentisstrikerobstacleblockinggoal) (fluentisjollyavailable) (fluentisjollyinposition) (strikerhasball))
		 	(and (fluentisstrikerobstacleblockinggoal) (not(fluentisjollyavailable)) (not(fluentisjollyinposition)) (strikerhasball))
		 )
  )

  (:action movewithballtokickingposition
	     :parameters () 
	     :precondition 
		 (and 
		 	(not (fluentisstrikerobstacleblockinggoal))
			(strikerhasball)
		 )
	     :effect
		 (and
	     	(strikercankick)
		 )

  )

  (:action kicktogoal
	     :parameters () 
	     :precondition 
		 	(and 
			 	(not (fluentisstrikerobstacleblockinggoal)) 
				(strikercankick)
			)
	     
		 :effect
	     	(and 
			 (goalscored)
			)
  )

  (:action waitforjolly
	     :parameters () 
	     :precondition 
		 	(and 
		 		(fluentisstrikerobstacleblockinggoal) 
				(fluentisjollyavailable)
				(not(fluentisjollyinposition))
				(not(strikercankick))
			)
	     :effect
		 	(oneof
				(and (fluentisjollyinposition) (fluentisjollyavailable) (fluentisstrikerobstacleblockinggoal))
				(and (not (fluentisjollyinposition)) (fluentisjollyavailable) (fluentisstrikerobstacleblockinggoal))
			)
  )

  (:action passballtojolly
	     :parameters () 
	     :precondition 
		 	(and 
			 	(fluentisstrikerobstacleblockinggoal)
				(fluentisjollyinposition)
				(fluentisjollyavailable)
			)
	     :effect
			(and (ballpassed))
  )

  (:action dribbleopponent
	     :parameters () 
	     :precondition 
		 	(and 
		 		(fluentisstrikerobstacleblockinggoal) 
				(not(fluentisjollyavailable))
			)
		 :effect
		 (and
	     	(strikerattemptingdribble)
		 )
  )
)
