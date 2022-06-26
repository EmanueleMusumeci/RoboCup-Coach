(define (problem striker-fond)
  (:domain striker-robocup-fond)

  (:init )

  (:goal (or (ball-passed) (goal-scored) (striker-attempting-dribble)))
)