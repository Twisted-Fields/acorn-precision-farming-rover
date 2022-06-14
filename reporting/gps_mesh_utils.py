"""Utilities for procedurally generating meshes and manipulating them."""

import math
import numpy as np
import queue
import sys
import pickle
from scipy.interpolate import griddata
import utm


_NEIGHBOR_DIST_TOLERANCE = 10
_MIN_ALLOWED_PATH_ANGLE = 160


class Mesh(object):
    """A class for generating meshes."""

    def __init__(self, source_points, max_x, max_y):
        """Create the mesh object with these default parameters."""
        self.source_points = source_points
        self.width = int(max_x)
        self.depth = int(max_y)
        self.step = 1.0
        self.width_steps = int(self.width/self.step)
        self.depth_steps = int(self.depth/self.step)
        self.total_vertices = self.width_steps * self.depth_steps
        self.gridmesh = None
        # self.path_width = 20

    def interpolate_to_grid(self):
        grid_x, grid_y = np.mgrid[0:self.width:self.step, 0:self.depth:self.step]
        self.gridmesh = griddata(self.source_points[:,:2], self.source_points[:,2], (grid_x, grid_y), method='linear')

    def generate_point_cloud(self, poly_path, min_x, min_y, zone):
        self.pcd = np.empty((0,3))
        for x_index in range(self.width_steps):
            for y_index in range(self.depth_steps):
                if not math.isnan(self.gridmesh[x_index][y_index]):

                    test_x = x_index * self.step + min_x
                    test_y = y_index * self.step + min_y

                    #print("{} {}".format(test_x, test_y))

                    latlon_point = utm.to_latlon(test_x, test_y, zone[0], zone[1])
                    #print(latlon_point)
                    if poly_path.contains_point((latlon_point[1], latlon_point[0]),radius=0.0):
                        #print("POINT IS IN")
                        self.pcd = np.append(self.pcd,
                        [[x_index * self.step,
                        y_index * self.step,
                        self.gridmesh[x_index][y_index]]],
                        axis=0)
        self.working_point_cloud = self.pcd.copy()

    def get_max_height_point(self):
        """Get maximum height found in the modified, "working" point cloud."""
        return self.working_point_cloud[np.argmax(self.working_point_cloud[:,2])]

    def get_min_height_point(self):
        return self.working_point_cloud[np.argmin(self.working_point_cloud[:,2])]

    def slice_height_points(self, height_center, height_range):
        min_height = height_center - height_range/2
        max_height = height_center + height_range/2
        tmp_mesh = self.working_point_cloud.copy()
        tmp_mesh = tmp_mesh[tmp_mesh[:,2]<max_height]
        self.slice_points = tmp_mesh[tmp_mesh[:,2]>min_height]
        self.current_height = height_center
        return self.slice_points

    def remove_points_near_path(self, path_points, distance_threshold, max_height):
        point_cloud_buffer = np.empty((0,3))
    #    print("$$$$$$$$$$$$")
    #    print(max_height)
        for point1 in self.working_point_cloud:
            reject = False
            if point1[2] > max_height:
                reject = True
            else:
                for point2 in path_points:
                    if self.get_distance(point1, point2) < distance_threshold:
                        reject = True
                        break
            if reject == False:
                point_cloud_buffer = np.append(point_cloud_buffer, [point1], axis=0)
        #print(self.working_point_cloud)
    #    print(point_cloud_buffer)
        self.working_point_cloud = point_cloud_buffer


    def calculate_min_max(self):
        """Calculate the minimum and maximum mesh heights."""
        self.mesh_max = np.amax(self.mesh[:, :, 2])
        self.mesh_min = np.amin(self.mesh[:, :, 2])

    def get_neighbors(self, last_coord, current_coord, dist):
        """Get the neighbor nodes from the graph with some criteria."""
        offsets = range(-1-dist, dist+2)
        # print(current_coord)
        neighbors = []
        rejected_points = []
        for x_coord in offsets:
            for y_coord in offsets:
                if x_coord == 0 and y_coord == 0:
                    continue
                this_neighbor = (
                    current_coord[0] + x_coord * self.step), (current_coord[1] + y_coord * self.step)
                if this_neighbor in self.cost_so_far.keys():
                    continue
                matches = np.where((self.slice_points[:,:2] == this_neighbor).all(axis=1))[0]
                if len(matches) == 0:
                    continue
                this_neighbor = (self.slice_points[matches[0]][0],self.slice_points[matches[0]][1])
                dist_neighbor = self.get_distance(current_coord, this_neighbor)
                if dist_neighbor > dist + 1:
                    continue
                if last_coord:
                    if abs(dist - dist_neighbor) > _NEIGHBOR_DIST_TOLERANCE:
                        rejected_points.append(this_neighbor)
                        continue
                    # angle = self.get_angle(
                    #     last_coord, current_coord, this_neighbor)
                    # if angle < _MIN_ALLOWED_PATH_ANGLE:
                    #     rejected_points.append(this_neighbor)
                    #     continue

                if 0 < this_neighbor[0] < self.width_steps and \
                   0 < this_neighbor[1] < self.depth_steps:
                    neighbors.append(this_neighbor)
        return rejected_points, neighbors

    def check_path_to_path_distance(self, first_path, second_path, limit):
        previous_path_distance = 99999
        for first_point in first_path:
            for second_point in second_path:
                p1 = (first_point[0], first_point[1])
                p2 = (second_point[0], second_point[1])
                dist = self.get_distance(p1, p2)
                if dist < previous_path_distance:
                    previous_path_distance = dist
                    if previous_path_distance < limit:
                        #print("FAILED WITH DISTANCE {}".format(previous_path_distance))
                        return False
        return True


    def get_cost(self, last, current, nextp, previous_path, row_spacing):
        """Calculate the cost to the next point."""
        height_difference = self.current_height - self.get_height(nextp)
        if last:
            angle = self.get_angle(last, current, nextp)
        else:
            angle = 0
        previous_path_distance = 99999
        for point in previous_path:
            dist = self.get_distance(point, nextp)
            if dist < previous_path_distance:
                previous_path_distance = dist

        if row_spacing - previous_path_distance > 0:
            path_dist_cost = 100 * 100000
        else:
            path_dist_cost = (abs(row_spacing - previous_path_distance) * 10) ** 3 * 1000

        angle_cost = abs(angle-180) ** 2 * 10
        #angle_cost = 0
        height_cost = abs(height_difference * 1)
        #print("{}, {} | {}, {}".format(difference, angle, dist_cost, angle_cost))
        #cost = math.pow(abs(difference), 1.3)
        return angle_cost + height_cost

    def get_angle(self, last, current, nextp):
        """Get the 2D planar angle between three grid points."""
        #  (l) *
        #      a  c
        #  (c) * b  * (n)

        dist_a = self.get_distance(last, current)
        dist_b = self.get_distance(current, nextp)
        dist_c = self.get_distance(last, nextp)

        angle = math.degrees(
            np.arccos(
             (dist_a ** 2 + dist_b ** 2 - dist_c ** 2) / (2 * dist_a * dist_b))
            )
        if math.isnan(angle):
            # Check for colinearity by confirming the distances sum as
            # expected.
            distances = [dist_a, dist_b, dist_c]
            distances.sort()
            calculated_diff = distances[0] + distances[1] - distances[2]
            if math.isclose(0, calculated_diff, abs_tol=0.001):
                # Points are colinear.
                if last[0] < current[0] < nextp[0] or \
                  last[0] > current[0] > nextp[0] or \
                  last[1] < current[1] < nextp[1] or \
                  last[1] > current[1] > nextp[1]:
                    return 180
                else:
                    return 0
        return angle

    def get_distance(self, p1, p2):
        """Get the linear distance between two 2d coordinates."""
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def interpolate(self, first, second, ratio):
        """Calculate a value as a ratio of the first and second values.

        Zero means return the second value, one means return the first.
        Values outside of that range are allowed.
        """
        return second + (first-second) * ratio

    def get_height(self, position):
        """Get gridmesh height at a position."""
        x = int(position[0]/self.step)
        y = int(position[1]/self.step)
        # print(x)
        # print(y)
        return self.gridmesh[x][y]

    def is_this_section(self, points, subject):
        """Determine if subject point lies in section p1-2 of p0-4."""
        angle_2_1_S = self.get_angle(points[2], points[1], subject)
        angle_0_1_S = self.get_angle(points[0], points[1], subject)
        angle_1_2_S = self.get_angle(points[1], points[2], subject)
        angle_3_2_S = self.get_angle(points[3], points[2], subject)

        if angle_2_1_S < angle_0_1_S and angle_1_2_S < angle_3_2_S:
            return True
        elif math.isclose(angle_1_2_S, angle_3_2_S, abs_tol=0.0001):
            return True
        else:
            return False

    def test_is_this_section(self):
        """A lonely test, to be moved to a test file."""
        points = [(0, 0), (2, 0), (4, 0), (6, 0)]
        success = True
        success &= self.is_this_section(points, (3, 0.1))
        success |= self.is_this_section(points, (5, 0.1))
        if not success:
            raise RuntimeError("Is_this_section test fail")


    def find_path_costs(self, start, goal, previous_path, row_spacing):
        """Find the cost for possible routes to the goal."""
        # Path finding tutorial:
        # https://www.redblobgames.com/pathfinding/a-star/introduction.html
        self.goal = (goal[0], goal[1])
        self.start = (start[0], start[1])
        frontier = queue.PriorityQueue()
        frontier.put((0, self.start))
        self.came_from = {}
        self.came_from[self.start] = None
        self.cost_so_far = {}
        self.cost_so_far[self.start] = 0
        current = None

        count = 0
        last = None

        while not frontier.empty():
            current = frontier.get()
            current = current[1]
            last = self.came_from[current]
            rejected, neighbors = self.get_neighbors(last, current, 6)
            neighbors = np.array(neighbors)
            rejected = np.array(rejected)

            # if last == current:
            #     print("last == current!")
            #     print("{}".format(current))
            #     print("{}".format(last))
            #     print(neighbors)
            #     raise Exception
            # print(current)
            # print(self.goal)

            if self.goal in self.cost_so_far.keys():
                print()
                print("Found Goal!")
                return True
                # self.goal = current
                break


            for next in neighbors:
                next = tuple(next)
                new_cost = self.cost_so_far[current] + \
                    self.get_cost(last, current, next, previous_path, row_spacing)
                if next not in self.cost_so_far.keys() or new_cost < (
                  0.8 * self.cost_so_far[next]):
                    self.cost_so_far[next] = new_cost
                    self.came_from[next] = current
                    #print("{} {}".format(next, self.goal))

                    priority = new_cost/30 + self.get_distance(next, self.goal)

                    # This still slightly favors smaller coordinates.
                    frontier.put((priority, next))

                    count += 1
                    # if count % 1000:
                    #     sys.stdout.write(
                    #         "\rVisited: {:0.2f}%, Total: {:0.2f}%".format(
                    #           100 * count/len(self.slice_points),
                    #           100 * len(self.cost_so_far)/len(self.slice_points)))
                    #     sys.stdout.flush()
                    if count > 5000000:
                        raise RuntimeError("Exceeded 5000000 count")
            # last = current
        return False

    def reconstruct_path(self):
        """Construct the path from the start to the goal."""
        # Reconstruct path.
        current = self.goal
        self.path = []
        count = 0
        while current != self.start:
            # print("loop")
            self.path.append(current)
            try:
                current = self.came_from[current]
            except:
                print(self.came_from)
                print(current)
                raise Exception
            count += 1
            if count > 250000:
                print(self.path[:-10])
                print(current)
                raise RuntimeError("Exceeded 250000 count on path calculation")
        self.path.append(self.start)  # optional
        self.path.reverse()  # optional
