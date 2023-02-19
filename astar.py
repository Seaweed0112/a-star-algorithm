from heapq import heapify, heappush, heappop
import sys


class Point:
    def __init__(self, x, y):
        if isinstance(x, str):
            x = float(x)
        if isinstance(y, str):
            y = float(y)
        self.x = x
        self.y = y
        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None
        self.diagonal = None

    def __str__(self):
        return f"({self.x}, {self.y})"

    def __repr__(self):
        return f"({self.x}, {self.y})"

    def __eq__(self, __o: object) -> bool:
        return self.x == __o.x and self.y == __o.y

    def __lt__(self, other):
        return self.f < other.f

    def euclidean(self, p2):
        '''
        Returns the value of euclidean distance from current point to p2

                Parameters:
                        p2 (Point): another point

                Returns:
                        (float): the value of euclidean distance from current point to p2
        '''
        return ((self.x - p2.x) ** 2 + (self.y - p2.y) ** 2) ** 0.5


def isCounterClockwise(a, b, c):
    '''
    Returns true if the points listed in counterclockwise otherwise false

            Parameters:
                    a, b, c (Point): three different points

            Returns:
                    (bool): true if the points listed in counterclockwise otherwise false
    '''
    # If the slope of the line AB is less than the slope of the line AC,
    # then the three points are listed in a counterclockwise order
    return (c.y - a.y) * (b.x - a.x) > (b.y - a.y) * (c.x - a.x)


def isIntersected(a, b, c, d):
    '''
    Returns true if the two segments intersect otherwise false

            Parameters:
                    a, b, c, d (Point): ab form a segment, cd form the other segment

            Returns:
                    (bool): true if the two segments intersect otherwise false 
    '''
    # If points a and b are separated by segment cd then acd and bcd
    # should have opposite orientation meaning either acd or bcd
    # is counterclockwise but not both
    return isCounterClockwise(a, c, d) != isCounterClockwise(
        b, c, d
    ) and isCounterClockwise(a, b, c) != isCounterClockwise(a, b, d)


def isAligned(a, b, c):
    '''
    Returns true if the points are aligned otherwise false

            Parameters:
                    a, b, c (Point): three points on a plane

            Returns:
                    (bool): true if the points are aligned otherwise false 
    '''

    # the cross product of vector ab and vector ac equal to 0,
    # when they are aligned
    crossproduct = (c.y - a.y) * (b.x - a.x) - (c.x - a.x) * (b.y - a.y)
    # to check if a float number is equal to 0, we compare it to epsilon
    return abs(crossproduct) < sys.float_info.epsilon


def isValid(curr, next, segments):
    '''
    Returns true if the next point is reachable from current point otherwise false

            Parameters:
                    curr (Point): current point
                    next (Point): next point
                    segments (list<tuple<Point>>): the sides of obstacles

            Returns:
                    (bool): true if the next point is reachable otherwise false 
    '''
    if curr.diagonal and next == curr.diagonal:
        # diagonal invalid!
        # the next node should not be across the diagonal of a obstacle
        return False
    # check if there is any intersection with obstacles
    for segment in segments:
        if isIntersected(curr, next, segment[0], segment[1]):
            if not (
                isAligned(curr, next, segment[0]) or isAligned(
                    curr, next, segment[1])
            ):
                # intersected invalid!
                # when there is an intersection, we should further check if the three points
                # are aligned. If they are aligned, then the next node is still reachable
                return False

    return True


# main solution class
class Solution:
    def __init__(self, start, goal, all_nodes, segments):
        self.start = start
        self.goal = goal
        self.all_nodes = all_nodes
        self.segments = segments
        self.open = []  # open (list<Point>): open list of nodes
        self.closed = []  # closed (list<Point>): closed list of nodes

    def generate_children(self, curr):
        '''
        Returns successors of current node

                Parameters:
                        curr (Point): current point
                        segments (list<tuple<Point>>): the sides of obstacles

                Returns:
                        (bool): true if the next point is reachable otherwise false 
        '''
        for i in range(len(self.all_nodes)):
            if curr == self.all_nodes[i]:
                # do not go back to where we come from
                continue

            # check if the next node self.all_nodes[i] is reachable from curr node
            if isValid(curr, self.all_nodes[i], segments):
                current_g = curr.g + curr.euclidean(self.all_nodes[i])
                if self.all_nodes[i] == goal:
                    # reach the goal
                    self.all_nodes[i].parent = curr
                    self.all_nodes[i].g = current_g
                    return True
                if self.all_nodes[i] in self.open and current_g < self.open[self.open.index(self.all_nodes[i])].g:
                    self.all_nodes[i].parent = curr
                    self.open[self.open.index(self.all_nodes[i])].g = current_g
                elif (
                    self.all_nodes[i] not in self.open
                    and self.all_nodes[i] in self.closed
                    and current_g < self.closed[self.closed.index(self.all_nodes[i])].g
                ):
                    # this condition never happens
                    print("IMPOSSIBLE!!!")
                    self.closed.remove(self.all_nodes[i])
                    self.all_nodes[i].parent = curr
                    self.all_nodes[i].g = current_g
                    self.all_nodes[i].f = self.all_nodes[i].g + \
                        self.all_nodes[i].h
                    heappush(self.open, self.all_nodes[i])

                elif self.all_nodes[i] not in self.open and self.all_nodes[i] not in self.closed:
                    self.all_nodes[i].parent = curr
                    self.all_nodes[i].g = current_g
                    self.all_nodes[i].h = self.all_nodes[i].euclidean(goal)
                    self.all_nodes[i].f = self.all_nodes[i].g + \
                        self.all_nodes[i].h
                    heappush(self.open, self.all_nodes[i])
        return False

    def astar(self):
        '''
        Returns true if the goal is reachable from start otherwise false.

                Parameters:

                Returns:
                        (bool): true if the goal is reachable from start otherwise false
        '''
        # initialize heap
        heapify(self.open)
        # initialize start point
        self.start.g = 0
        self.start.h = self.start.euclidean(goal)
        self.start.f = self.start.g + self.start.h
        # push into heap
        heappush(self.open, self.start)
        while len(self.open):
            curr = heappop(self.open)  # get the bestnode from open
            self.closed.append(curr)  # move the bestnode to closed
            if self.generate_children(curr):  # find successors
                return True  # we found the path to goal
        return False  # there is no path to goal

    def reconstruct_path(self):
        '''
        Returns path from start to goal

                Parameters:

                Returns:
                        (list<tuple<Point, float>>): list of tuple, (node, the g of the node)
        '''
        # go back to start from goal
        curr = self.all_nodes[self.all_nodes.index(self.goal)]
        result = []
        # when the parent is None, it means we have reached the start
        while curr.parent:
            result.append((str(curr), curr.g))
            curr = curr.parent
        result.append((str(curr), curr.g))
        return result


if __name__ == "__main__":
    # load data from file
    with open("data2.txt", "r") as infile:
        # parse data into python objects
        lines = infile.readlines()
        lines = [line.rstrip().split(" ") for line in lines]
        start = Point(lines[0][0], lines[0][1])
        goal = Point(lines[1][0], lines[1][1])
        num_of_obstacles = int(lines[2][0])
        all_nodes = []
        segments = []
        for i in range(num_of_obstacles):
            for j in range(4):
                p = Point(lines[i + 3][2 * j], lines[i + 3][2 * j + 1])
                # the point across the diagonal is not reachable
                p.diagonal = Point(
                    lines[i + 3][2 * (j + 2) % 8], lines[i +
                                                         3][2 * (j + 2) % 8 + 1]
                )
                all_nodes.append(p)
                # collect each side of obstacles
                segments += [
                    (
                        Point(lines[i + 3][2 * j], lines[i + 3][2 * j + 1]),
                        Point(
                            lines[i + 3][2 * (j + 1) % 8],
                            lines[i + 3][2 * (j + 1) % 8 + 1],
                        ),
                    )
                ]

        if goal not in all_nodes:
            all_nodes.append(goal)
        # initialization
        sol = Solution(start, goal, all_nodes, segments)
        # run the main algorithm
        sol.astar()
        # reconstruct path from start to goal
        path = sol.reconstruct_path()
        print("Point".ljust(20), "Cumulative Cost")
        for i in range(len(path)-1, -1, -1):
            print(path[i][0].ljust(20), "{:.3f}".format(path[i][1]))
