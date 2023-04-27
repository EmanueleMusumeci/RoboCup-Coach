(define (problem simple-striker)
(:domain robocup-deterministic)

(:objects robot1 ball - movable striker-position kicking-position ball-position goal-target - location)

(:init (is-at robot1 striker-position) 
  (is-at ball ball-position) 
  (is-robot robot1) 
  (is-ball ball)
)

(:goal (or
          (isat robot1 ballposition)
          (goal-scored)
        )            
)

)