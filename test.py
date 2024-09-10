import numpy as np
import math


class ORCA:
    def __init__(self) -> None:
        self.neighbor_distance = 5000
        self.time_horizon = 250
        # time window
        self.inv_time_horizon = 1 / self.time_horizon 
        self.k_neighbors

    def get_constraint(self, agent, dt):
        # get neighbors
        neighbors = agent.get_nearest_neighbors(self.k_neighbors, self.neighbor_distance)

        # get prefer velocity
        direction_to_goal = agent.goal - agent.position
        distance_to_goal = np.linalg.norm(direction_to_goal)
        # direction_to_goal / distance_to_goal is a unit vector
        pref_vel = min(agent.maxSpeed, distance_to_goal / dt) * direction_to_goal / distance_to_goal

        # get constraint
        for neighbor in neighbors:
            # Construct ORCA lines
            rel_position = neighbor.position-agent.position
            d = np.linalg.norm(rel_position)
            rel_velocity = agent.velocity - neighbor.velocity

            # if there is no collision
            R = agent.radius * 1.1
            if d > R:
                # vector from the cutoff circle center to the relative velocity
                w = rel_velocity - self.inv_time_horizon * rel_position
                w_norm = np.linalg.norm(w)
                unit_w = w / w_norm

                # project on the circle
                # dot_product1 < 0 is a necessary but not sufficient condition for the projection having to be done on the cutoff circle (maybe in line)
                # the second condition compares the angle lambda and alpha, project on the circle if lambda < alpha => cos^2 (lambda) > cos^2 (alpha) alpha and lambda are between 0 and pi/2
                # lambda: the angle between w and - rel_postion cos^2 lambda = dot_product1**2/(|w|^2*|rel_position|^2)
                # alpha: the angle between rel_position and the radius tangent to the line (alpha) cos^2 alpha = R^2 / |rel_position|^2
                dot_product1 = np.dot(w, rel_position)

                if dot_product1 < 0 and dot_product1 ** 2 > R ** 2 * np.dot(w, w):
                    # u is in the direction of w and its value is the remaining distance to exit the cutoff circle
                    # R * self.inv_time_horizon is the norm of the cutoff circle
                    u = (R * self.inv_time_horizon - w_norm) * unit_w
                    direction = np.array([unit_w[0], unit_w[1]])  # np.array([unit_w[1], -unit_w[0]]) along the line
                else:
                    # Need to project on the sides
                    leg = math.sqrt(d ** 2 - R ** 2)
                    # determine the left side of the right side
                    if np.linalg.det([rel_position, w]) > 0:
                        # find the left side vector: by multiplying by rotation matrix (cone half angle theta): 
                        # [[cos(theta), -sin(theta)], [sin(theta), cos(theta)]] * rel_position
                        # sin theta = R / d and cos theta = leg / d, and get the unit vector
                        direction = np.array([rel_position[0] * leg - rel_position[1] * R, 
                                              rel_position[0] * R + rel_position[1] * leg]) / d ** 2
                    else:
                        # project on the right side, and here don't understand why add '-'
                        direction = -np.array([rel_position[0] * leg + rel_position[1] * R, 
                                               -rel_position[0] * R + rel_position[1] * leg]) / d ** 2

                    # project the relative velocity on the side leg
                    dot_product2 = rel_velocity * direction

                    # u is the vector from the rel_velocity to the boundary
                    u = dot_product2 * direction - rel_velocity

        return u, pref_vel