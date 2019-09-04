from yaw_controller import YawController
from pid import  PID
from lowpass import LowPassFilter
import rospy


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, \
                       decel_limit, accel_limit, wheel_radius, wheel_base, \
                       steer_ratio, max_lat_accel, max_steer_angle):

        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius
        self.prev_time = rospy.get_time()

        self.yaw_controller = YawController(wheel_base, steer_ratio, \
                                            0.1, max_lat_accel, max_steer_angle)

        kp = 0.3; ki = 0.1; kd = 0.0
        min = 0.0; max = 0.2
        self.throttle_controller = PID(kp, ki, kd, min, max)

        tau = 0.5; ts = 0.02
        self.vel_low_pass = LowPassFilter(tau, ts)

    def control(self, current_vel, dbw_enabled, linear_vel, angular_vel):
        # if drive by wire is not enabled, initialise the Controller with 0.0
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0.0, 0.0, 0.0

        # rospy.logwarn("angular_vel: {0}\n".format(angular_vel))
        # rospy.logwarn("Target vel: {0}\n".format(linear_vel))
        # rospy.logwarn("Current vel: {0}\n".format(current_vel))
        # rospy.logwarn("Filtered vel: {0}".format(self.vel_low_pass))

        #use a low pass filter to smooth out the current_vel
        current_vel = self.vel_low_pass.filt(current_vel)
        steering = self.yaw_controller.get_steering(linear_vel, angular_vel, current_vel)

        vel_error = linear_vel - current_vel
        self.prev_vel = current_vel                 #store previous velocity

        current_time = rospy.get_time()
        sample_time = current_time - self.prev_time
        self.prev_time = current_time

        throttle = self.throttle_controller.step(vel_error, sample_time)
        brake = 0

        if linear_vel == 0.0 and current_vel < 0.1:
            throttle = 0
            brake = 400 #N m

        elif throttle < 0.1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        return throttle, brake, steering
