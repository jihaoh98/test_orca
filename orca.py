import numpy as np
from math import cos, sin, sqrt, asin, atan2
from geometry import Point, Vector, Line, HalfPlane


class ORCA:
    """ compute orca hyper-plane """
    def __init__(self, tau, margin) -> None:
        self.tau = tau
        self.margin = margin
        
    def comute_vo_circle(self, robot_state, obs_state):
        """ 
        compute the velocity obstacle of a circle 
        robot_state: [x, y, vx, vy, radius]
        obs_state: [x, y, vx, vy, radius]
        """
        x, y, vx, vy, r = robot_state[0:5]
        mx, my, mvx, mvy, mr = obs_state[0:5]

        dis_mr = sqrt((my - y) ** 2 + (mx - x) ** 2)
        angle_mr = atan2(my - y, mx - x)
        
        sin_phi = (r + mr + self.margin) / dis_mr
        half_angle = asin(sin_phi)
        line_left_ori = self.wrap_to_pi(angle_mr + half_angle)
        line_right_ori = self.wrap_to_pi(angle_mr - half_angle)
        
        print("half_angle: ", half_angle)
        print("angle_mr: ", angle_mr)
        print("line_left_ori: ", line_left_ori)
        print("line_right_ori: ", line_right_ori)
        
        print("result of left: ", cos(line_left_ori), sin(line_left_ori))
        print("result of right: ", cos(line_right_ori), sin(line_right_ori))
    
    def wrap_to_pi(self, theta):
        """ convert the theta to (-pi, pi) """
        if theta > np.pi:
            theta = theta - 2 * np.pi
        if theta < -np.pi:
            theta = theta + 2 * np.pi

        return theta

    def compute_orca(self, robot, other, type='R') -> HalfPlane:
        """ compute the orca hyper-plane for robot w.r.t other robot / obs """
        # rel_vel: vA - vB 
        rel_vel = Vector(robot[2] - other[2], robot[3] - other[3])

        vo_circle_center = Point(other[0] - robot[0], other[1] - robot[1])
        vo_truncated_circle_center = vo_circle_center / self.tau
        vo_circle_radius = robot[4] + other[4] + self.margin

        axis = Vector(vo_circle_center.x, vo_circle_center.y)
        vo_half_angle = asin(vo_circle_radius / axis.norm())
        left_vec = axis.getRotatedVector(vo_half_angle)
        right_vec = axis.getRotatedVector(-vo_half_angle)
        
        # project rel_vel to left_vec and right_vec
        left_projection = rel_vel.projectToVector(left_vec)
        right_projection = rel_vel.projectToVector(right_vec)
        
        vo_closest_circle_center = Point()
        vo_closest_circle_center.init_from_point(vo_truncated_circle_center)
        # choose a projection between left and right
        if rel_vel * axis > 0.0:
            projection = None
            if left_projection * axis <= 0.0:
                projection = right_projection
            elif right_projection * axis <= 0.0:
                projection = left_projection
            elif left_projection.norm() > right_projection.norm():
                projection = left_projection
            else:
                projection = right_projection
            
            # get the new circle center
            tmp_line = Line()
            tmp_line.init_from_one_point(Point(projection.x, projection.y))

            # get the intersection point of the line and axis
            temp = Line()
            temp.init_from_line_and_point(tmp_line, Point(rel_vel.x, rel_vel.y))

            axis_line = Line()
            axis_line.init_from_one_point(Point(axis.x, axis.y))
            projection_circle_center = temp.getIntersectionPoint(axis_line)
            if (Vector(projection_circle_center.x, projection_circle_center.y).norm() > 
                Vector(vo_truncated_circle_center.x, vo_truncated_circle_center.y).norm()):
                vo_closest_circle_center.init_from_point(projection_circle_center)
        
        # compute the half-plane
        tmp = Vector(vo_closest_circle_center.x, vo_closest_circle_center.y)
        closest_circle_radius = vo_circle_radius *  tmp.norm() / axis.norm()

        centerToV = rel_vel - tmp
        if centerToV.norm() == 0.0:
            centerToBorder = right_projection - rel_vel
        else:
            centerToBorder = Vector(centerToV.x, centerToV.y)
            centerToBorder.normalize(closest_circle_radius)
        
        u = centerToBorder - centerToV
        robot_vel = Vector(robot[2], robot[3])
        
        print("u: ", u)
        print(centerToBorder.norm())
        print(centerToV.norm())
        
        # 这里实际上可以根据centerToBorder和centerToV的大小来判断速度是不是在VO内部
        # 但是这里返回的超平面已经是合理的，因为始终是由圆心指向边界，就算速度在VO外部，这个方向也是合理的
        if type == 'R':
            return HalfPlane(robot_vel + u / 2, centerToBorder)
        else:
            return HalfPlane(robot_vel + u, centerToBorder)



if __name__ == "__main__":
    test_target = ORCA(2.0, 0.1)
    
    robot_state = np.array([0.0, 0.0, 2.0, 0.0, 0.5])
    obs_state = np.array([0.0, 2.0, 0.0, 0.0, 0.4])
    
    # test_target.comute_vo_circle(robot_state, obs_state)
    res = test_target.compute_orca(robot_state, obs_state)
