import sys
import rospy
from new_driver import Driver
from math import atan2, sqrt, pi




class StudentDriver(Driver):
   def __init__(self, threshold=0.1):
       super().__init__('odom')
       self._threshold = threshold


   def close_enough_to_waypoint(self, distance, target, lidar):
       '''
       Determines if the robot is close enough to the waypoint. Stops if:
       - Distance to waypoint is less than the threshold.
       - There are no obstacles blocking the path.


       Parameters:
           distance: Distance to the target waypoint.
           target: (x, y) tuple of the waypoint in robot's frame.
           lidar: LaserScan object with range data.


       Returns:
           bool: True if the robot should stop, False otherwise.
       '''
       # Check if within the target threshold
       if distance < self._threshold:
           # Check for obstacles directly in front (e.g., within a narrow angle range)
           min_front_distance = min(lidar.ranges[len(lidar.ranges) // 3: 2 * len(lidar.ranges) // 3])
           if min_front_distance > self._threshold:
               rospy.loginfo("Close enough to waypoint, stopping.")
               return True
           else:
               rospy.logwarn("Obstacle detected, cannot stop at waypoint yet.")
       return False


   def get_twist(self, target, lidar):
       '''
       Generates a Twist message to navigate to the target while avoiding obstacles.


       Parameters:
           target: (x, y) tuple of the waypoint in robot's frame.
           lidar: LaserScan object with range data.


       Returns:
           Twist: Commanded velocities for the robot.
       '''
       angle = atan2(target[1], target[0])
       distance = sqrt(target[0] ** 2 + target[1] ** 2)
       rospy.loginfo(f'Distance: {distance:.2f}, Angle: {angle:.2f}')


       command = Driver.zero_twist()


       # Obstacle detection: Check front area in lidar ranges
       min_safe_distance = 0.5
       min_front_distance = min(lidar.ranges[len(lidar.ranges) // 3: 2 * len(lidar.ranges) // 3])


       if min_front_distance < min_safe_distance:
           rospy.logwarn("Obstacle detected in path, stopping.")
           command.linear.x = 0.0
           command.angular.z = 0.0
           return command


       # Proportional control for linear velocity
       max_linear_speed = 0.5
       command.linear.x = min(max_linear_speed, distance * 0.5)


       # Proportional control for angular velocity
       max_angular_speed = pi / 4
       command.angular.z = max(-max_angular_speed, min(max_angular_speed, angle * 2))


       # Slow down as the robot gets closer to the target
       if distance < 0.5:
           command.linear.x *= 0.5


       return command




if __name__ == '__main__':
   rospy.init_node('student_driver', argv=sys.argv)
   driver = StudentDriver()
   rospy.spin()



