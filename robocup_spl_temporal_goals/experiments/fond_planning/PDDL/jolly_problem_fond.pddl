(define (problem jolly-fond)
  (:domain jolly-robocup-fond)

  (:init (jolly-available))

  (:goal 
    (and
      (jolly-ready)
    )
  )
)