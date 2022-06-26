(define (problem strikerfond)
  (:domain strikerrobocupfond)

  (:objects )

  (:init 
    (not (ballpassed))
    (not (goalscored))
    (not (strikerattemptingdribble))
  )

  (:goal (or ballpassed goalscored strikerattemptingdribble))
)