import rospy
from math import pi, sin, cos, tan, sqrt, fabs
from geometry_msgs.msg import Pose2D, Twist


class task:
    def __init__(self):
        rospy.init_node('go_to_goal', anonymous=False)
        rospy.on_shutdown(self.shutdown)

        self.goal_pose = Pose2D()
        self.dis_to_mid = 0
        self.last_err = 0
        self.kp = 0.3
        self.kd = 8
        self.pos_angle_sub = rospy.Subscriber('/pos_angle', Pose2D, self.pos_angle_callback)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def pos_angle_callback(self, msg):
        # self.goal_pose = msg
        self.goal_pose.x = msg.y
        self.goal_pose.y = msg.x
        self.goal_pose.theta = -msg.theta - pi / 2
        self.k = tan(self.goal_pose.theta + pi)
        self.b = self.goal_pose.y - self.k * self.goal_pose.x
        self.dis_to_mid = self.pointToLine(self.k, -1, self.b, 0, -0.40834389165276)


    def pointToLine(self, A, B, C, x0, y0):
        return fabs(A * x0 + B * y0 + C) / sqrt(A * A + B * B)

    def pid_control(self, dis_err):
        dis_err_diff = dis_err - self.last_err
        ang = self.kp * dis_err + self.kd * dis_err_diff
        self.last_err = dis_err
        return ang

    def move(self):
        rate = 100
        rat = rospy.Rate(rate)

        while self.dis_to_mid > 0.015:
            angle = self.pid_control(self.dis_to_mid)
            move_cmd = Twist()
            move_cmd.linear.x = 0.1
            x_label = (self.k * self.k + self.k * (-0.40834389165276 - self.k - self.b)) / (self.k * self.k + 1)
            #print "x_label: ", x_label
            if x_label > 0:
                move_cmd.angular.z = -angle
            else:
                move_cmd.angular.z = angle
            #print "angle speed: ", angle
            self.cmd_vel.publish(move_cmd)
            rat.sleep()
            print "dis_to_mid: ", self.dis_to_mid

        angle = self.goal_pose.theta
        angular_speed = 0.15

        if -1 * pi < angle < -1 * pi / 2:
            goal_angle = -1 * angle - pi / 2
            angular_duration = abs(goal_angle) / angular_speed
            move_cmd = Twist()
            move_cmd.angular.z = -1 * angular_speed
            ticks = int(angular_duration * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                rat.sleep()
        else:
            goal_angle = angle + pi / 2
            angular_duration = abs(goal_angle) / angular_speed
            move_cmd = Twist()
            move_cmd.angular.z = angular_speed
            ticks = int(angular_duration * rate)
            for t in range(ticks):
                self.cmd_vel.publish(move_cmd)
                rat.sleep()
        # rospy.sleep(1)

        move_cmd = Twist()
        linear_speed = 0.1
        move_cmd.linear.x = linear_speed
        y_label = self.goal_pose.y
        goal_distance = y_label - 0.3

        linear_duration = goal_distance / linear_speed
        ticks = int(linear_duration * rate)
        for t in range(ticks):
            self.cmd_vel.publish(move_cmd)
            rat.sleep()

        move_cmd = Twist()
        self.cmd_vel.publish(move_cmd)
        rospy.sleep(1)

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        self.cmd_vel.publish(Twist())
        rospy.sleep(1)


if __name__ == '__main__':
    Task = task()
    rospy.sleep(1)
    Task.move()
