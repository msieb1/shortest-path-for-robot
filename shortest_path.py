import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.lines as lines
from graham_scan import line_side, diff_, len2

plt.ion()

def main():
    ### create data
    polygon1 = list(np.array([[-2, 1], [3, 1], [2, 2]]))
    polygon2 = list(np.array([[-5, -5], [-1, 2], [0, 6], [-3, 2]]))
    polygon3 = list(np.array([[1, -4], [3, -4], [2.5, 2], [-4, -1]]))
    polygons = [polygon1, polygon2, polygon3]
    start = np.array([-1, -4])
    target = np.array([-1.5, 3.1])

    ### flatten polygons to one list and assign labels in chronological order for shortest-path later
    idx = 1
    vertex_labels = {}
    vertex_list = []
    for poly in polygons:
        for vertex in poly:
            vertex_labels[idx] = vertex
            vertex_list.append(vertex)
            idx += 1
    # append target vertex
    vertex_labels[idx] = target
    vertex_list.append(target)

    lines = create_lines(polygons)
    line_list = []
    for poly in lines:
        for line in poly:
            line_list.append(line)


    ### plot polygons
    plt.figure(1)
    for polygon in polygons:
        buff = polygon[:]
        buff.append(polygon[0])
        plt.plot(np.array(buff)[:,0], np.array(buff)[:,1], marker='o')
    plt.plot(target[0], target[1], 'rx', markersize=8)
    plt.plot(start[0], start[1], 'ro', markersize=8)


    # vis_points, vis_labels = visibility_graph(start, vertex_list, line_list)
    # for point in vis_points:
    #     buff = np.vstack([start, point])
    #     plt.plot(buff[:, 0], buff[:, 1], 'y--')

    dist, prev = dijkstra(start, vertex_list, line_list, vertex_labels, polygons)
    path = create_path(prev)

    for idx, vertex in enumerate(path[:-1]):
        if idx == 0:
            p1 = start
        else:
            p1 = vertex_labels[path[idx]]
        p2 = vertex_labels[path[idx+1]]
        buff = np.vstack([p1, p2])
        plt.plot(buff[:, 0], buff[:, 1], 'r--')

    a = 0

class Line():

    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def intersect(self, l2):
        if line_side(self.p1, self.p2, l2.p2) * line_side(self.p1, self.p2, l2.p1) < 0 and \
                line_side(l2.p1, l2.p2, self.p1) * line_side(l2.p1, l2.p2, self.p2) < 0:
            return True
        else:
            return False

def create_lines(polygons):
    """
    creates all existing lines in each polygon. Lines within polygon are also created to distinguish
    possible paths that lie on the edge of the polygon vs illegal paths through the polygon
    :param polygons:
    :return:
    """
    lines = []
    for polygon in polygons:
        curr_lines = []
        for idx in range(0, len(polygon)):
            for idx_ in range(idx, len(polygon)):
                curr_line = Line(polygon[idx], polygon[idx_])
                curr_lines.append(curr_line)
        lines.append(curr_lines)
    return lines

def inside_polygon(point, polygon):
    """
    returns True if a point is inside a polygon defined by
    a list of points.

    Reference: http://www.ariel.com.au/a/python-point-int-poly.html
    :param point: array
    :param polygons: list of arrays
    :return: boolean
    """
    x = point[0]
    y = point[1]
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(1, n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def visibility_graph(start, vertex_list, line_list):
    """

    :param start: 2D-array point with x-y-coordinate
    :param vertex_list: list of arrays (list of points for each polygon)
    :param line_list: list of lines for each polygon
    :return: list of arrays with all points per polygon visible from the given point
    """
    idx = 1
    vis_labels = {}
    vis_points = []
    for vertex in vertex_list:
        curr_line = Line(start, vertex)
        cross = False
        for line in line_list:
            if curr_line.intersect(line):
                cross = True
        if not cross:
            vis_labels[idx] = vertex
            vis_points.append(vertex)
        idx += 1
        cross = False

    return vis_points, vis_labels


def dijkstra(start, vertex_list, line_list, vertex_labels, polygons):
    """
    Dijsktra. Vertices are numbers, points are specifically declared as points.
    Target is last element of vertex list
    :param start:
    :param vertex_list:
    :param line_list:
    :param vertex_labels:
    :return:
    """
    # create stack Q with all vertices including the arbitrary starting point
    Q = {**vertex_labels}
    Q[0] = start
    vertex_labels_with_start = {**Q}
    dist = {}
    prev = {}
    for key, val in Q.items():
        dist[key] = 1e10
        prev[key] = None
    # start has zero distance to itself
    dist[0] = 0
    while Q:
        min_ = 1e10
        curr_vertex = None
        # simulates priority queue (min heap) with for loop
        for v in Q.keys():
            if dist[v] < min_:
                curr_vertex = v
                min_ = dist[v]
        # curr_vertex = min(dist, key=dist.get)
        if curr_vertex is None:
            print("Target cannot be reached!")
            break
        Q.pop(curr_vertex)
        invalid_point = False
        for poly in polygons:
            if inside_polygon(vertex_labels_with_start[curr_vertex], poly):
                invalid_point = True
                break
        if invalid_point:
            continue
        if curr_vertex == len(vertex_list):
            break
        _, vis_labels = visibility_graph(vertex_labels_with_start[curr_vertex], vertex_list, line_list)
        # Just implement dijkstra - need a way to mark vertices with labels
        for elem in vis_labels:
            if elem in Q:
                alt = dist[curr_vertex] + np.sqrt(len2((diff_(vertex_labels_with_start[curr_vertex],
                                                              vertex_labels_with_start[elem]))))
                if alt < dist[elem]:
                    dist[elem] = alt
                    prev[elem] = curr_vertex
    return dist, prev

def create_path(prev):
    S = []
    u = len(prev)-1
    while prev[u] is not None:
        S.insert(0, u)
        u = prev[u]
    S.insert(0, u)
    return S


if __name__ == '__main__':
    main()
