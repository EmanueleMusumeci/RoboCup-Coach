begin_version
3.FOND
end_version
begin_metric
0
end_metric
14
begin_variable
var0
-1
2
Atom ball-passed()
NegatedAtom ball-passed()
end_variable
begin_variable
var1
-1
2
Atom ball-safe()
NegatedAtom ball-safe()
end_variable
begin_variable
var2
-1
2
Atom fluent-jolly-available()
NegatedAtom fluent-jolly-available()
end_variable
begin_variable
var3
-1
2
Atom fluent-opponent-goal-landmark()
NegatedAtom fluent-opponent-goal-landmark()
end_variable
begin_variable
var4
-1
2
Atom fluent-opponent-near()
NegatedAtom fluent-opponent-near()
end_variable
begin_variable
var5
-1
2
Atom is-at(ball, ball-position)
NegatedAtom is-at(ball, ball-position)
end_variable
begin_variable
var6
-1
2
Atom is-at(ball, goal-target)
NegatedAtom is-at(ball, goal-target)
end_variable
begin_variable
var7
-1
2
Atom is-at(ball, striker-position)
NegatedAtom is-at(ball, striker-position)
end_variable
begin_variable
var8
-1
2
Atom is-at(robot1, ball-position)
NegatedAtom is-at(robot1, ball-position)
end_variable
begin_variable
var9
-1
2
Atom is-at(robot1, goal-target)
NegatedAtom is-at(robot1, goal-target)
end_variable
begin_variable
var10
-1
2
Atom is-at(robot1, striker-position)
NegatedAtom is-at(robot1, striker-position)
end_variable
begin_variable
var11
-1
2
Atom jolly-available()
NegatedAtom jolly-available()
end_variable
begin_variable
var12
0
2
Atom new-axiom@0()
NegatedAtom new-axiom@0()
end_variable
begin_variable
var13
-1
2
Atom opponent-goal-available()
NegatedAtom opponent-goal-available()
end_variable
0
begin_state
1
1
1
1
1
0
1
1
1
1
0
1
1
1
end_state
begin_goal
1
12 0
end_goal
30
begin_operator
carry-ball robot1 ball ball-position goal-target
5
5 0
6 1
8 0
9 1
13 0
2
6
0 1 -1 0
0 4 -1 1
0 5 0 1
0 6 1 0
0 8 0 1
0 9 1 0
6
0 1 -1 1
0 4 -1 0
0 5 0 1
0 6 1 0
0 8 0 1
0 9 1 0
0
end_operator
begin_operator
carry-ball robot1 ball ball-position striker-position
5
5 0
7 1
8 0
10 1
13 0
2
6
0 1 -1 0
0 4 -1 1
0 5 0 1
0 7 1 0
0 8 0 1
0 10 1 0
6
0 1 -1 1
0 4 -1 0
0 5 0 1
0 7 1 0
0 8 0 1
0 10 1 0
0
end_operator
begin_operator
carry-ball robot1 ball goal-target ball-position
5
5 1
6 0
8 1
9 0
13 0
2
6
0 1 -1 0
0 4 -1 1
0 5 1 0
0 6 0 1
0 8 1 0
0 9 0 1
6
0 1 -1 1
0 4 -1 0
0 5 1 0
0 6 0 1
0 8 1 0
0 9 0 1
0
end_operator
begin_operator
carry-ball robot1 ball goal-target striker-position
5
6 0
7 1
9 0
10 1
13 0
2
6
0 1 -1 0
0 4 -1 1
0 6 0 1
0 7 1 0
0 9 0 1
0 10 1 0
6
0 1 -1 1
0 4 -1 0
0 6 0 1
0 7 1 0
0 9 0 1
0 10 1 0
0
end_operator
begin_operator
carry-ball robot1 ball striker-position ball-position
5
5 1
7 0
8 1
10 0
13 0
2
6
0 1 -1 0
0 4 -1 1
0 5 1 0
0 7 0 1
0 8 1 0
0 10 0 1
6
0 1 -1 1
0 4 -1 0
0 5 1 0
0 7 0 1
0 8 1 0
0 10 0 1
0
end_operator
begin_operator
carry-ball robot1 ball striker-position goal-target
5
6 1
7 0
9 1
10 0
13 0
2
6
0 1 -1 0
0 4 -1 1
0 6 1 0
0 7 0 1
0 9 1 0
0 10 0 1
6
0 1 -1 1
0 4 -1 0
0 6 1 0
0 7 0 1
0 9 1 0
0 10 0 1
0
end_operator
begin_operator
defend-ball robot1 ball ball-position
4
1 1
5 0
8 0
11 1
1
1
0 1 1 0
0
end_operator
begin_operator
defend-ball robot1 ball goal-target
4
1 1
6 0
9 0
11 1
1
1
0 1 1 0
0
end_operator
begin_operator
defend-ball robot1 ball striker-position
4
1 1
7 0
10 0
11 1
1
1
0 1 1 0
0
end_operator
begin_operator
kick-ball robot1 ball ball-position goal-target
4
5 0
6 1
8 0
9 1
1
2
0 5 0 1
0 6 1 0
0
end_operator
begin_operator
kick-ball robot1 ball ball-position striker-position
4
5 0
7 1
8 0
10 1
1
2
0 5 0 1
0 7 1 0
0
end_operator
begin_operator
kick-ball robot1 ball goal-target ball-position
4
5 1
6 0
8 1
9 0
1
2
0 5 1 0
0 6 0 1
0
end_operator
begin_operator
kick-ball robot1 ball goal-target striker-position
4
6 0
7 1
9 0
10 1
1
2
0 6 0 1
0 7 1 0
0
end_operator
begin_operator
kick-ball robot1 ball striker-position ball-position
4
5 1
7 0
8 1
10 0
1
2
0 5 1 0
0 7 0 1
0
end_operator
begin_operator
kick-ball robot1 ball striker-position goal-target
4
6 1
7 0
9 1
10 0
1
2
0 6 1 0
0 7 0 1
0
end_operator
begin_operator
move-robot robot1 ball-position goal-target
2
8 0
9 1
1
2
0 8 0 1
0 9 1 0
0
end_operator
begin_operator
move-robot robot1 ball-position striker-position
2
8 0
10 1
1
2
0 8 0 1
0 10 1 0
0
end_operator
begin_operator
move-robot robot1 goal-target ball-position
2
8 1
9 0
1
2
0 8 1 0
0 9 0 1
0
end_operator
begin_operator
move-robot robot1 goal-target striker-position
2
9 0
10 1
1
2
0 9 0 1
0 10 1 0
0
end_operator
begin_operator
move-robot robot1 striker-position ball-position
2
8 1
10 0
1
2
0 8 1 0
0 10 0 1
0
end_operator
begin_operator
move-robot robot1 striker-position goal-target
2
9 1
10 0
1
2
0 9 1 0
0 10 0 1
0
end_operator
begin_operator
pass-ball robot1 ball ball-position
4
1 1
5 0
8 0
11 0
1
1
0 0 -1 0
0
end_operator
begin_operator
pass-ball robot1 ball goal-target
4
1 1
6 0
9 0
11 0
1
1
0 0 -1 0
0
end_operator
begin_operator
pass-ball robot1 ball striker-position
4
1 1
7 0
10 0
11 0
1
1
0 0 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball ball-position goal-target
3
6 0
8 0
9 1
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 8 0 1
0 9 1 0
0 11 -1 0
0 13 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball ball-position striker-position
3
7 0
8 0
10 1
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 8 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball goal-target ball-position
3
5 0
8 1
9 0
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 8 1 0
0 9 0 1
0 11 -1 0
0 13 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball goal-target striker-position
3
7 0
9 0
10 1
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 9 0 1
0 10 1 0
0 11 -1 0
0 13 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball striker-position ball-position
3
5 0
8 1
10 0
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 8 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
0
end_operator
begin_operator
reach-ball robot1 ball striker-position goal-target
3
6 0
9 1
10 0
8
8
0 1 -1 0
0 2 -1 0
0 3 -1 0
0 4 -1 1
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 0
0 3 -1 1
0 4 -1 1
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 0
0 4 -1 1
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 0
0 2 -1 1
0 3 -1 1
0 4 -1 1
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 0
0 4 -1 0
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 0
0 3 -1 1
0 4 -1 0
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 0
0 4 -1 0
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
8
0 1 -1 1
0 2 -1 1
0 3 -1 1
0 4 -1 0
0 9 1 0
0 10 0 1
0 11 -1 0
0 13 -1 0
0
end_operator
1
begin_rule
1
8 0
12 1 0
end_rule
