(define (problem striker-fond)
(:domain robocup-fond)
(:objects 
  striker-robot 
  ball
    - movable 
  
  striker-current-position 
  ball-current-position 
  kicking-position 
  goal-target 
    - location
)

(:init 
  (is-at striker-robot striker-current-position) 
  (is-at ball ball-current-position) 
  (is-robot striker-robot) 
  (is-ball ball)
)

(:goal 
  (or 
    (goal-scored) 
    (ball-passed)
  )
)
)