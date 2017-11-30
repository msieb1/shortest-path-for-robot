import numpy as np
import matplotlib.pyplot as plt
import math
import matplotlib.lines as lines

plt.ion()
def main():

    points = list(np.array([[0, 0], [3, 1], [-1, 2], [-3, 2], [2, 2],
                       [1, 1], [0, 6]]))
    points = list(np.random.normal(0, 10, [10,2]))

    fig = plt.figure(0)
    plt.scatter(np.array(points)[:,0], np.array(points)[:,1],s=10)


    hull = graham_scan(points)
    hull.append(hull[0])

    ## plotting ##
    plt.plot(np.array(hull)[:,0], np.array(hull)[:,1], marker='o')



def len2(p):
    """
    return the squared length of a point vector with respect to the origin
    :param p: array
    :return: scalar
    """
    return p[0]**2 + p[1]**2

def diff_(p1, p2):
    """
    computes the difference vector of two points
    :param p1: array
    :param p2: array
    :return: array
    """
    point = np.zeros(2)
    point[0] = p1[0] - p2[0]
    point[1] = p1[1] - p2[1]
    return point

def sum_(p1, p2):
    """
    computes the sum vector of two points
    :param p1: array
    :param p2: array
    :return: array
    """
    point = np.zeros(2)
    point[0] = p1[0] + p2[0]
    point[1] = p1[1] + p2[1]
    return point


def line_side(p1, p2, p3):
    """

    :param p1:
    :param p2:
    :param p3:
    :return:
    """
    cp = cross(diff_(p2,p1), diff_(p3, p1))
    if cp > 0:
        return -1
    elif cp < 0:
        return 1
    else:
        return 0

def cross(p1, p2):
    """
    calculates the cross product of two points
    :param p1:
    :param p2:
    :return:
    """
    return p1[0]*p2[1] - p1[1]*p2[0]

def sort_points(points, min_elem):
    """
    takes a list of points and returns an ordered list of points starting
    with the point with the lowest y-coordinate and then circling counter-clockwise through all points
    :param points: list of arrays
    :return: list of arrays
    """
    angle_lst = []
    min_ = 1e9
    min_idx = 0
    for idx, p in enumerate(points):
        angle_lst.append((p, math.atan2(p[1]-min_elem[1], p[0]-min_elem[0])))

    buff = sorted(angle_lst, key=lambda p: p[1])
    return [x[0] for x in buff]


def convexify(chain, point):
    """
    takes a chain of convex point candidates and another point candidate.
    Repeatedly pops points of the candidate list until new point is a convex extension
    :param chain: list of arrays
    :param point: array
    :return: list of arrays
    """
    while chain:
        test = line_side(chain[-2], chain[-1], point)
        if test == -1:
            chain.append(point)
            return chain
        elif test == 0:
            if len2(point - chain[-2]) > len2(chain[-1] - chain[-2]):
                chain.pop()
                return convexify(chain, point)
            else:
                chain.append(point)
                return chain
        else:
            chain.pop()
            return convexify(chain, point)



def graham_scan(points):
    """
    In essence a wrapper for the convexify function. Adds the first two points in the
    sorted list to the convex hull (these points are always part of it)
    :param sorted_points: sorted list of array with respect to angles
    :return: list of arrays (convex hull)
    """
    min_idx = np.argmin(np.array(points)[:, 1])
    min_elem = points.pop(min_idx)

    sorted_points = sort_points(points, min_elem)
    sorted_points.insert(0, min_elem)

    chain = sorted_points[0:2]
    rest = sorted_points[2:]

    for idx, p in enumerate(rest):
        convexify(chain, p)
    return chain

if __name__ == '__main__':
    main()
