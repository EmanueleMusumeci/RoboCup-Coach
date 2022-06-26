(define (problem simplestriker)
(:domain robocupdeterministic)
(:objects robot ball - movable 
    WAYPOINTS - location
    )
(:init 
    (isrobot robot) (isball ball) 
ADJACENCY_PREDICATES
POSITION_PREDICATES
)
(:goal 
    GOAL_CONDITION
)
)