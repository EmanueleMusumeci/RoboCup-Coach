(define (problem simple-striker)
(:domain robocup-deterministic)

(:objects robot1 ball - movable strikerposition kickingposition ballposition goaltarget - location)

(:init (isat robot1 strikerposition) (isat ball ballposition) (is-robot robot1) (is-ball ball) (is-location strikerposition) (is-location ballposition) (is-location kickingposition) (is-location goaltarget))


(:goal (goalscored)            
)

)