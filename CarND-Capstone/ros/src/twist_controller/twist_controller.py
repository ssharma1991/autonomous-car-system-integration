from yaw_controller import YawController
from pid import PID
from lowpass import LowPassFilter
import rospy
import math

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # Brake: vehicle_mass, wheel_radius
        # Steering: wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle
        self.yaw_controller=YawController(wheel_base, steer_ratio, .1, max_lat_accel, max_steer_angle)
        
        kp=.1
        ki=.1
        kd=.1
        max_throttle=.25
        min_throttle=0
        self.throttle_controller=PID(kp, ki, kd, min_throttle, max_throttle)
        
        tau=.5
        ts=.02
        self.vel_lowpass=LowPassFilter(tau,ts)
        
        self.prev_time=rospy.get_time()
        self.decel_limit=decel_limit
        self.vehicle_mass=vehicle_mass
        self.wheel_radius=wheel_radius

    def control(self, lin, ang, current_vel, dbw_enabled):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0,0,0
        
        # Calculate Steering
        cur_vel=self.vel_lowpass.filt(current_vel)
        steering=self.yaw_controller.get_steering(lin, ang, cur_vel)
        
        # Calculate Throttle
        vel_error=lin-cur_vel
        self.last_vel=cur_vel
        cur_time=rospy.get_time()
        sample_time=cur_time-self.prev_time
        throttle=self.throttle_controller.step(vel_error,sample_time)
        
        # Calculate Brake
        brake=0
        if (lin==0.0 and cur_vel<.1):
            throttle=0
            brake=400 #N.m  Torque to hold car at rest
        elif (vel_error<0 and throttle<.1):
            throttle=0
            decel=max(vel_error,self.decel_limit)
            brake=abs(decel)*self.vehicle_mass*self.wheel_radius #N.m  Torque
        
        return throttle, brake, steering
