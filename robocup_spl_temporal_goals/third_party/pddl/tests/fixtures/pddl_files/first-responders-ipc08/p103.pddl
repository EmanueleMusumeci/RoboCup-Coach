(define (problem FR_10_3)
 (:domain first-response)
 (:objects  l1 l2 l3 l4 l5 l6 l7 l8 l9 l10  - location
	    f1 f2 f3 f4 f5 f6 f7 - fire_unit
	    v1 v2 v3 - victim
	    m1 m2 m3 m4 m5 m6 - medical_unit
)
 (:init 
	;;strategic locations
     (hospital l4)
     (hospital l3)
     (hospital l1)
     (hospital l10)
     (hospital l9)
     (hospital l9)
     (water-at l6)
     (water-at l5)
     (water-at l3)
     (water-at l2)
     (water-at l1)
     (water-at l9)
     (water-at l8)
     (water-at l6)
	;;disaster info
     (fire l5)
     (victim-at v1 l4)
     (victim-status v1 hurt)
     (fire l3)
     (victim-at v2 l1)
     (victim-status v2 hurt)
     (fire l8)
     (victim-at v3 l7)
     (victim-status v3 dying)
	;;map info
	(adjacent l1 l1)
	(adjacent l2 l2)
	(adjacent l3 l3)
	(adjacent l4 l4)
	(adjacent l5 l5)
	(adjacent l6 l6)
	(adjacent l7 l7)
	(adjacent l8 l8)
	(adjacent l9 l9)
	(adjacent l10 l10)
   (adjacent l1 l1)
   (adjacent l1 l1)
   (adjacent l1 l2)
   (adjacent l2 l1)
   (adjacent l1 l3)
   (adjacent l3 l1)
   (adjacent l1 l4)
   (adjacent l4 l1)
   (adjacent l1 l5)
   (adjacent l5 l1)
   (adjacent l2 l1)
   (adjacent l1 l2)
   (adjacent l2 l2)
   (adjacent l2 l2)
   (adjacent l2 l3)
   (adjacent l3 l2)
   (adjacent l3 l1)
   (adjacent l1 l3)
   (adjacent l3 l2)
   (adjacent l2 l3)
   (adjacent l3 l3)
   (adjacent l3 l3)
   (adjacent l4 l1)
   (adjacent l1 l4)
   (adjacent l4 l2)
   (adjacent l2 l4)
   (adjacent l4 l3)
   (adjacent l3 l4)
   (adjacent l4 l4)
   (adjacent l4 l4)
   (adjacent l5 l1)
   (adjacent l1 l5)
   (adjacent l5 l2)
   (adjacent l2 l5)
   (adjacent l6 l1)
   (adjacent l1 l6)
   (adjacent l6 l2)
   (adjacent l2 l6)
   (adjacent l8 l1)
   (adjacent l1 l8)
   (adjacent l9 l1)
   (adjacent l1 l9)
   (adjacent l9 l2)
   (adjacent l2 l9)
   (adjacent l9 l3)
   (adjacent l3 l9)
   (adjacent l9 l4)
   (adjacent l4 l9)
   (adjacent l9 l5)
   (adjacent l5 l9)
   (adjacent l9 l6)
   (adjacent l6 l9)
   (adjacent l9 l7)
   (adjacent l7 l9)
   (adjacent l9 l8)
   (adjacent l8 l9)
   (adjacent l9 l9)
   (adjacent l9 l9)
   (adjacent l10 l1)
   (adjacent l1 l10)
   (adjacent l10 l2)
   (adjacent l2 l10)
   (adjacent l10 l3)
   (adjacent l3 l10)
   (adjacent l10 l4)
   (adjacent l4 l10)
   (adjacent l10 l5)
   (adjacent l5 l10)
   (adjacent l10 l6)
   (adjacent l6 l10)
   (adjacent l10 l7)
   (adjacent l7 l10)
   (adjacent l10 l8)
   (adjacent l8 l10)
   (adjacent l10 l9)
   (adjacent l9 l10)
	(fire-unit-at f1 l9)
	(fire-unit-at f2 l7)
	(fire-unit-at f3 l6)
	(fire-unit-at f4 l5)
	(fire-unit-at f5 l3)
	(fire-unit-at f6 l2)
	(fire-unit-at f7 l10)
	(medical-unit-at m1 l9)
	(medical-unit-at m2 l8)
	(medical-unit-at m3 l6)
	(medical-unit-at m4 l5)
	(medical-unit-at m5 l3)
	(medical-unit-at m6 l2)
	)
 (:goal (and  (nfire l5) (nfire l3) (nfire l8)  (victim-status v1 healthy) (victim-status v2 healthy) (victim-status v3 healthy)))
 )
