(define (problem simple-striker)
(:domain robocup-deterministic)
(:objects robot1 ball - movable striker-current-position ball-current-position kicking-position goal-target - location)
(:init (is-at robot1 striker-current-position) (is-at ball ball-current-position) (is-robot robot1) (is-ball ball))
(:goal (goal-scored))
)