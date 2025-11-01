#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_custom_interfaces.msg import Lidar, MotorSpeeds, BnoReading, Pwm
import math
import numpy as np
import time

class WallFollower(Node):
    def __init__(self):
        super().__init__('wall_follower')

        self.distance_sub = self.create_subscription(Lidar, '/distance', self.follow_wall, 10)
        self.curr_angle = self.create_subscription(BnoReading, '/current_angle', self.curr_angle_callback, 10)
        self.pwm_check = self.create_subscription(Pwm, '/debug', self.pwm_check, 10)
        self.vel_pub = self.create_publisher(Pwm, '/velocity', 10)

        self.current_dist = 0.0
        self.current_angle = 0.0
        self.ref_angle = 0.0

        self.TARGET_DISTANCE_CM = 80.0

        self.KP_DISTANCE = 6.0
        self.KD_DISTANCE = 0.8
        self.KI_DISTANCE = 0.07

        self.kp_angle = 1.5
        self.kd_angle = 0.3
        self.ki_angle = 0.1

        self.prevDisterr = 0.0
        self.dist_inte = 0.0
        self.dist_inteMax = 100

        self.prevAngerr = 0.0
        self.ang_inte = 0.0
        self.ang_inteMax = 100

        self.prevtime = 0.0
        self.motors = Pwm()

        self.sign_matrix = np.array([
           [ 1, -1,  1],  # Motor 1 (Top-Left)
           [ 1,  1,  1],  # Motor 2 (Top-Right)
           [ 1,  1, -1],  # Motor 3 (Bottom-Left)
           [-1,  1,  1]   # Motor 4 (Bottom-Right)
        ])

    def pwm_check(self, msg = Pwm()):
       pass

    def curr_angle_callback(self, msg = BnoReading()):
        self.current_angle = msg.current_angle
        self.get_logger().info(f"Current Angle = {self.current_angle}")


    def calculate_motor_velocities(self, vx, vy, vw):
      velocity_vector = [vx, vy, vw]
      motor_velocities = [0.0, 0.0, 0.0, 0.0]
      
      for i in range(4):
         for j in range(3):
            motor_velocities[i] += self.sign_matrix[i][j] * velocity_vector[j]
    
      self.motors.m1 = motor_velocities[0]
      self.motors.m2 =  motor_velocities[1]
      self.motors.m3 = motor_velocities[2]
      self.motors.m4 = motor_velocities[3]
      self.vel_pub.publish(self.motors)
      return
    

    def normalize_angle(self, angle):
      while angle > 180.0:
        angle -= 360.0
      while angle < -180.0:
        angle += 360.0
      return angle
    

    def bno_pid(self, target_angle):
       theta = self.current_angle   
       error = self.normalize_angle(target_angle - theta)
       return error


    def follow_wall(self,msg):
      self.current_dist = msg.distance
      self.get_logger().info(f"Current Distance = {self.current_dist}")

      current_dist = self.current_dist
      curr_time = time.time()
      dt = curr_time - self.prevtime

      dist_error = self.TARGET_DISTANCE_CM - current_dist
      ang_error = self.bno_pid(self.ref_angle)
    
      if dt > 0:
        err_derivative = (dist_error - self.prevDisterr) / dt
        self.dist_inte += dist_error * dt
        self.dist_inte = max(-self.dist_inteMax, min(self.dist_inteMax, self.dist_inte))

        ang_derivative = (ang_error - self.prevAngerr) /  dt
        self.ang_inte += ang_error * dt
        self.ang_inte = max(-self.ang_inteMax, min(self.ang_inteMax, self.ang_inte))

      else:
        err_derivative = 0.0
        ang_derivative = 0.0
    
      vy_correction = (self.KP_DISTANCE * dist_error) + (self.KD_DISTANCE * err_derivative) + (self.KI_DISTANCE * self.dist_inte)
      vy_correction = max(-250, min(int(vy_correction), 250))
    
      w1 = (self.kp_angle * ang_error) + (self.kd_angle * ang_derivative) + (self.ki_angle * self.ang_inte)
      w1 = max(-250, min(int(w1), 250))

      if abs(dist_error) < 2.0:
        vy_correction = 0.0
        self.dist_inte = 0.0

      if abs(ang_error) < 2.0:
        w1 = 0.0
        self.ang_inte = 0.0
    
      self.prevDisterr = dist_error
      self.prevAngerr = ang_error
      self.prevtime = curr_time

      self.calculate_motor_velocities(80, -vy_correction, -w1)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
