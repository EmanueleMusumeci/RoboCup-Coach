begin_version
3.FOND
end_version
begin_metric
0
end_metric
9
begin_variable
var0
-1
2
Atom goal-scored()
NegatedAtom goal-scored()
end_variable
begin_variable
var1
-1
2
Atom is-at(ball, ball-current-position)
NegatedAtom is-at(ball, ball-current-position)
end_variable
begin_variable
var2
-1
2
Atom is-at(ball, goal-target)
NegatedAtom is-at(ball, goal-target)
end_variable
begin_variable
var3
-1
2
Atom is-at(ball, kicking-position)
NegatedAtom is-at(ball, kicking-position)
end_variable
begin_variable
var4
-1
2
Atom is-at(ball, striker-current-position)
NegatedAtom is-at(ball, striker-current-position)
end_variable
begin_variable
var5
-1
2
Atom is-at(robot1, ball-current-position)
NegatedAtom is-at(robot1, ball-current-position)
end_variable
begin_variable
var6
-1
2
Atom is-at(robot1, goal-target)
NegatedAtom is-at(robot1, goal-target)
end_variable
begin_variable
var7
-1
2
Atom is-at(robot1, kicking-position)
NegatedAtom is-at(robot1, kicking-position)
end_variable
begin_variable
var8
-1
2
Atom is-at(robot1, striker-current-position)
NegatedAtom is-at(robot1, striker-current-position)
end_variable
0
begin_state
1
0
1
1
1
1
1
1
0
end_state
begin_goal
1
0 0
end_goal
37
begin_operator
carry-ball robot1 ball ball-current-position goal-target
4
1 0
2 1
5 0
6 1
1
4
0 1 0 1
0 2 1 0
0 5 0 1
0 6 1 0
0
end_operator
begin_operator
carry-ball robot1 ball ball-current-position kicking-position
4
1 0
3 1
5 0
7 1
1
4
0 1 0 1
0 3 1 0
0 5 0 1
0 7 1 0
0
end_operator
begin_operator
carry-ball robot1 ball ball-current-position striker-current-position
4
1 0
4 1
5 0
8 1
1
4
0 1 0 1
0 4 1 0
0 5 0 1
0 8 1 0
0
end_operator
begin_operator
carry-ball robot1 ball goal-target ball-current-position
4
1 1
2 0
5 1
6 0
1
4
0 1 1 0
0 2 0 1
0 5 1 0
0 6 0 1
0
end_operator
begin_operator
carry-ball robot1 ball goal-target kicking-position
4
2 0
3 1
6 0
7 1
1
4
0 2 0 1
0 3 1 0
0 6 0 1
0 7 1 0
0
end_operator
begin_operator
carry-ball robot1 ball goal-target striker-current-position
4
2 0
4 1
6 0
8 1
1
4
0 2 0 1
0 4 1 0
0 6 0 1
0 8 1 0
0
end_operator
begin_operator
carry-ball robot1 ball kicking-position ball-current-position
4
1 1
3 0
5 1
7 0
1
4
0 1 1 0
0 3 0 1
0 5 1 0
0 7 0 1
0
end_operator
begin_operator
carry-ball robot1 ball kicking-position goal-target
4
2 1
3 0
6 1
7 0
1
4
0 2 1 0
0 3 0 1
0 6 1 0
0 7 0 1
0
end_operator
begin_operator
carry-ball robot1 ball kicking-position striker-current-position
4
3 0
4 1
7 0
8 1
1
4
0 3 0 1
0 4 1 0
0 7 0 1
0 8 1 0
0
end_operator
begin_operator
carry-ball robot1 ball striker-current-position ball-current-position
4
1 1
4 0
5 1
8 0
1
4
0 1 1 0
0 4 0 1
0 5 1 0
0 8 0 1
0
end_operator
begin_operator
carry-ball robot1 ball striker-current-position goal-target
4
2 1
4 0
6 1
8 0
1
4
0 2 1 0
0 4 0 1
0 6 1 0
0 8 0 1
0
end_operator
begin_operator
carry-ball robot1 ball striker-current-position kicking-position
4
3 1
4 0
7 1
8 0
1
4
0 3 1 0
0 4 0 1
0 7 1 0
0 8 0 1
0
end_operator
begin_operator
kick-ball robot1 ball ball-current-position goal-target
4
1 0
2 1
5 0
6 1
1
2
0 1 0 1
0 2 1 0
0
end_operator
begin_operator
kick-ball robot1 ball ball-current-position kicking-position
4
1 0
3 1
5 0
7 1
1
2
0 1 0 1
0 3 1 0
0
end_operator
begin_operator
kick-ball robot1 ball ball-current-position striker-current-position
4
1 0
4 1
5 0
8 1
1
2
0 1 0 1
0 4 1 0
0
end_operator
begin_operator
kick-ball robot1 ball goal-target ball-current-position
4
1 1
2 0
5 1
6 0
1
2
0 1 1 0
0 2 0 1
0
end_operator
begin_operator
kick-ball robot1 ball goal-target kicking-position
4
2 0
3 1
6 0
7 1
1
2
0 2 0 1
0 3 1 0
0
end_operator
begin_operator
kick-ball robot1 ball goal-target striker-current-position
4
2 0
4 1
6 0
8 1
1
2
0 2 0 1
0 4 1 0
0
end_operator
begin_operator
kick-ball robot1 ball kicking-position ball-current-position
4
1 1
3 0
5 1
7 0
1
2
0 1 1 0
0 3 0 1
0
end_operator
begin_operator
kick-ball robot1 ball kicking-position goal-target
4
2 1
3 0
6 1
7 0
1
2
0 2 1 0
0 3 0 1
0
end_operator
begin_operator
kick-ball robot1 ball kicking-position striker-current-position
4
3 0
4 1
7 0
8 1
1
2
0 3 0 1
0 4 1 0
0
end_operator
begin_operator
kick-ball robot1 ball striker-current-position ball-current-position
4
1 1
4 0
5 1
8 0
1
2
0 1 1 0
0 4 0 1
0
end_operator
begin_operator
kick-ball robot1 ball striker-current-position goal-target
4
2 1
4 0
6 1
8 0
1
2
0 2 1 0
0 4 0 1
0
end_operator
begin_operator
kick-ball robot1 ball striker-current-position kicking-position
4
3 1
4 0
7 1
8 0
1
2
0 3 1 0
0 4 0 1
0
end_operator
begin_operator
kick-to-goal robot1 ball
2
3 0
7 0
1
3
0 0 -1 0
0 2 -1 0
0 3 0 1
0
end_operator
begin_operator
move-robot robot1 ball-current-position goal-target
2
5 0
6 1
1
2
0 5 0 1
0 6 1 0
0
end_operator
begin_operator
move-robot robot1 ball-current-position kicking-position
2
5 0
7 1
1
2
0 5 0 1
0 7 1 0
0
end_operator
begin_operator
move-robot robot1 ball-current-position striker-current-position
2
5 0
8 1
1
2
0 5 0 1
0 8 1 0
0
end_operator
begin_operator
move-robot robot1 goal-target ball-current-position
2
5 1
6 0
1
2
0 5 1 0
0 6 0 1
0
end_operator
begin_operator
move-robot robot1 goal-target kicking-position
2
6 0
7 1
1
2
0 6 0 1
0 7 1 0
0
end_operator
begin_operator
move-robot robot1 goal-target striker-current-position
2
6 0
8 1
1
2
0 6 0 1
0 8 1 0
0
end_operator
begin_operator
move-robot robot1 kicking-position ball-current-position
2
5 1
7 0
1
2
0 5 1 0
0 7 0 1
0
end_operator
begin_operator
move-robot robot1 kicking-position goal-target
2
6 1
7 0
1
2
0 6 1 0
0 7 0 1
0
end_operator
begin_operator
move-robot robot1 kicking-position striker-current-position
2
7 0
8 1
1
2
0 7 0 1
0 8 1 0
0
end_operator
begin_operator
move-robot robot1 striker-current-position ball-current-position
2
5 1
8 0
1
2
0 5 1 0
0 8 0 1
0
end_operator
begin_operator
move-robot robot1 striker-current-position goal-target
2
6 1
8 0
1
2
0 6 1 0
0 8 0 1
0
end_operator
begin_operator
move-robot robot1 striker-current-position kicking-position
2
7 1
8 0
1
2
0 7 1 0
0 8 0 1
0
end_operator
0
