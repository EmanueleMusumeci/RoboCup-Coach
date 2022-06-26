begin_version
3.FOND
end_version
begin_metric
0
end_metric
6
begin_variable
var0
-1
2
Atom fluent-obstacle-blocking-goal()
NegatedAtom fluent-obstacle-blocking-goal()
end_variable
begin_variable
var1
-1
2
Atom jolly-aligned-to-striker()
NegatedAtom jolly-aligned-to-striker()
end_variable
begin_variable
var2
-1
2
Atom jolly-in-position()
NegatedAtom jolly-in-position()
end_variable
begin_variable
var3
-1
2
Atom jolly-position-ok()
NegatedAtom jolly-position-ok()
end_variable
begin_variable
var4
-1
2
Atom jolly-ready()
NegatedAtom jolly-ready()
end_variable
begin_variable
var5
-1
2
Atom jolly-rotation-ok()
NegatedAtom jolly-rotation-ok()
end_variable
0
begin_state
1
1
1
1
1
1
end_state
begin_goal
1
4 0
end_goal
7
begin_operator
check-jolly-position 
1
3 1
1
1
1 2 0 3 1 0
0
end_operator
begin_operator
check-jolly-ready 
2
3 0
5 0
1
1
0 4 -1 0
0
end_operator
begin_operator
check-jolly-rotation 
1
5 1
1
1
1 1 0 5 1 0
0
end_operator
begin_operator
check-obstacle-position 
0
2
1
0 0 -1 0
1
1 0 1 2 -1 0
0
end_operator
begin_operator
move-to-receiving-position 
2
0 0
2 1
1
1
0 2 1 0
0
end_operator
begin_operator
turn-to-striker 
2
0 1
2 0
1
1
0 1 -1 0
0
end_operator
begin_operator
turn-to-striker 
2
1 1
2 0
1
1
0 1 1 0
0
end_operator
0
