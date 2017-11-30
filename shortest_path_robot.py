import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.lines as lines
import matplotlib.cm as cm
from graham_scan import *
from shortest_path import *
import matplotlib.patches as mpatches
from matplotlib.patches import Circle, Wedge, Polygon
from matplotlib.collections import PatchCollection



def main():

    ### create data
    polygon1 = list(np.array([[-1,0], [3, 1], [2, 2],
                       [1, 2]]))
    polygon2 = list(np.array([[-5, -5], [-1, 2], [0, 6], [-3, 2]]))
    polygon3 = list(np.array([[1, -4], [3, -4], [3, -2], [1, -1]]))
    polygons = [polygon1, polygon2, polygon3]

    robot = list(np.array([[-1, -4], [-2, -4], [-2, -2]]))

    start = robot[0]
    target = np.array([2, 4])

    ext_polygons = extend_polygons(polygons, robot)

    ### flatten polygons to one list and assign labels in chronological order for shortest-path later
    idx = 1
    vertex_labels = {}
    vertex_list = []
    for poly in ext_polygons:
        for vertex in poly:
            vertex_labels[idx] = vertex
            vertex_list.append(vertex)
            idx += 1
    # append target vertex
    vertex_labels[idx] = target
    vertex_list.append(target)

    lines = create_lines(ext_polygons)
    line_list = []
    for poly in lines:
        for line in poly:
            line_list.append(line)


    #######################################################


    ####### PLOTTING ############
    def label(xy, text):
        y = xy[1] - 0.15  # shift y-value for label so that it's below the artist
        plt.text(xy[0], y, text, ha="center", family='sans-serif', size=14)

    x = np.arange(len(polygons)+1)
    ys = [i + x + (i * x) ** 2 for i in range(len(polygons)+1)]
    colors = cm.rainbow(np.linspace(0, 1, len(ys)))
    ### plot polygons
    fig, ax = plt.subplots()
    for idx, polygon in enumerate(polygons):
        c = colors[idx]
        buff = polygon[:]
        buff.append(polygon[0])
        plt.plot(np.array(buff)[:,0], np.array(buff)[:,1], color=c, markersize=0)
    plt.plot(target[0], target[1], 'rx', markersize=14)
    plt.annotate('Target', xy=target, xytext=target+np.array([0.2,0.2]))
    plt.plot(start[0], start[1], 'rx', markersize=14)
    plt.annotate('Start', xy=start, xytext=start+np.array([0.2,0.2]))

    ### plot extended Cspace polygons
    #plt.figure(2)
    for idx, polygon in enumerate(ext_polygons):
        c = colors[idx]
        buff = polygon[:]
        buff.append(polygon[0])
        plt.plot(np.array(buff)[:, 0], np.array(buff)[:, 1], color=c, linestyle='--', markersize=0)
    robot.append(robot[0])
    c = colors[len(polygons)]


    patches = []
    pat = mpatches.Polygon(np.array(robot))
    patches.append(pat)
    colors = 100 * np.random.rand(len(patches))
    p = PatchCollection(patches, alpha=0.4)
    p.set_array(np.array(colors))
    ax.add_collection(p)
    plt.annotate('Robot', xy=start, xytext=start+np.array([-1,-1]), fontsize=12, fontweight='bold')



    # calculate Cspace polygons
    dist, prev = dijkstra(start, vertex_list, line_list, vertex_labels, ext_polygons)
    path = create_path(prev)

    ### Plot path
    for idx, vertex in enumerate(path[:-1]):
        if idx == 0:
            p1 = start
        else:
            p1 = vertex_labels[path[idx]]
        p2 = vertex_labels[path[idx+1]]
        buff = np.vstack([p1, p2])
        plt.plot(buff[:, 0], buff[:, 1], 'r-', linewidth=2.5)

    #################################################################

    a = 0
def extend_polygons(polygons, robot):
    """
    Creates the (convex) Cspace polygons for a given (convex) robot polygon
    :param polygons: list of list of arrays
    :param robot: list of arrays
    :return: list of list of arrays
    """
    orig = robot[0]
    robot_shifted = [diff_(x, orig) for x in robot]
    ext_polygons = []

    for poly in polygons:
        ext_points = []
        for pp in poly:
            for pr in robot_shifted:
                ext_points.append(diff_(pp, pr))
        hull_ext_poly_shifted = graham_scan(ext_points)
        hull_ext_poly = [sum_(x, orig) for x in hull_ext_poly_shifted]
        # ext_polygons.append(hull_ext_poly)
        ext_polygons.append(hull_ext_poly_shifted)
    return ext_polygons


if __name__ == '__main__':
    main()
