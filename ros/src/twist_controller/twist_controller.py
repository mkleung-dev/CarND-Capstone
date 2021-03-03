import rospy
from pid import PID
from yaw_controller import YawController

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, vehicle_mass, fuel_capacity, brake_deadband, decel_limit, accel_limit, wheel_radius, wheel_base, steer_ratio, max_lat_accel, max_steer_angle):
        # TODO: Implement
        self.vehicle_mass = vehicle_mass
        self.fuel_capacity = fuel_capacity
        self.brake_deadband = brake_deadband
        self.decel_limit = decel_limit
        self.accel_limit = accel_limit
        self.wheel_radius = wheel_radius

        throttle_Kp = 0.01
        throttle_Ki = 0.0001
        throttle_Kd = 0.02
        throttle_min = 0.0
        throttle_max = 1.0

        self.throttle_pid = PID(throttle_Kp, throttle_Ki, throttle_Kd, throttle_min, throttle_max)

        min_speed = 0.1
        self.yaw_controller = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)

        self.last_time = rospy.get_time()

    def control(self, dbw_enabled, current_velocity, target_velocity, target_angular_velocity):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        if not dbw_enabled:
            self.throttle_pid.reset()
            return 0.0, 0.0, 0.0

        steering = self.yaw_controller.get_steering(target_velocity, target_angular_velocity, current_velocity)
        current_time = rospy.get_time()
        time_diff = current_time - self.last_time
        

        throttle = self.throttle_pid.step(target_velocity - current_velocity, time_diff)
        brake = 0

        if target_velocity == 0 and current_velocity < 0.1:
            throttle = 0
            brake = 400
        elif throttle < 0.01 and target_velocity < current_velocity:
            throttle = 0
            decel = max(target_velocity - current_velocity, self.decel_limit)
            brake = abs(decel) * self.vehicle_mass * self.wheel_radius

        self.last_time = current_time

        return throttle, brake, steering
