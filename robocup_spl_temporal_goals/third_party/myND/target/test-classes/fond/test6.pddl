(define (problem test-1)
  (:domain test6)
  (:objects A B - block)
  (:init (true A) (true B))
  (:goal (and (not (true A)) (not (true B))))
)
