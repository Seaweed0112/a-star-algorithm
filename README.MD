# A\* Search Algorithm

_Project of EE562 at University of Washington, 2022 Fall_

<!-- [link to course website](https://homes.cs.washington.edu/~shapiro/EE562/hw2/index.html) -->

Given some obstacles in rectangle shape, and a start point and a goal point. Find the shortest path from start point to goal point.
A robot start from the start point, and can only move from a vertex of a rectangle to another.
The robot can not cut cross the rectangle while moving. It can only move along the edge.

A move must be from some vertex I to another vertex J (the robot will never be at a point other than a vertex, unless it is moving from one vertext to another). I and J may be on the same rectangle or on two different ones. He is allowed to move along an edge of an obstacle, just not through it.

### Data 1

figure:
![dataset1 image](/data1.gif "Text to show on mouseover")

input:

```
0 0 # Start State
9 6 # Goal State
2 # Number of obstacles
0 0 4 0 4 4 0 4 # Obstacle 1
7 4 9 6 4 10 2 8 # Obstacle 2

```

expected output:

```
Point                Cumulative Cost
(0.0, 0.0)           0.000
(4.0, 4.0)           5.657
(7.0, 4.0)           8.657
(9.0, 6.0)           11.485
```

### Data 2

figure:
![dataset1 image](/data2.gif "Text to show on mouseover")

input:

```
0 0
25 35
10
0 0 5 0 5 5 0 5
6 2 8 2 8 25 6 25
10 10 12 10 12 12 10 12
15 0 20 5 15 10 10 5
10 15 15 13 17 18 12 20
24 4 26 6 21 11 19 9
28 7 30 9 22 17 20 15
26 19 31 24 30 25 25 20
20 20 25 25 15 35 10 30
22 33 25 33 25 35 22 35
```

expected output:

```
Point                Cumulative Cost
(0.0, 0.0)           0.000
(5.0, 0.0)           5.000
(8.0, 2.0)           8.606
(10.0, 5.0)          12.211
(25.0, 25.0)         37.211
(25.0, 35.0)         47.211
```

### Data 3

figure:
![dataset1 image](/data3.jpeg "Text to show on mouseover")

input:

```
1 1
24 30
8
3 1 3 3 24 3 24 1
0 4 19 4 19 21 0 21
21 4 24 4 24 7 21 7
23 8 25 8 25 12 23 12
23 13 25 13 25 16 23 16
26 20 27 22 15 28 14 26
24 24 26 24 26 26 24 26
15 29 23 25 24 27 16 31
```

expected output:

```
Point                Cumulative Cost
(1.0, 1.0)           0.000
(0.0, 4.0)           3.162
(0.0, 21.0)          20.162
(16.0, 31.0)         39.030
(24.0, 30.0)         47.092
```
