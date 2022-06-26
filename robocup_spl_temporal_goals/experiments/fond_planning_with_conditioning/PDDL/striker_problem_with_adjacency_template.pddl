(define (problem simplestriker)
(:domain robocupdeterministic)
(:objects robot1 ball - movable strikercurrentposition ballcurrentposition kickingposition goaltarget - location)
(:init (isat robot1 strikercurrentposition) (isat ball ballcurrentposition) (isrobot robot1) (isball ball) 
    ADJACENCY_PREDICATES
)
(:goal (goalscored))
)