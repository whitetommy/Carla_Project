#pid_utils.py

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import cutils
import numpy as np
import math

class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                = cutils.CUtils()
        self._current_x          = 0
        self._current_y          = 0
        self._current_yaw        = 0
        self._current_speed      = 0
        self._desired_speed      = 0
        self._current_frame      = 0
        self._current_timestamp  = 0
        self._start_control_loop = False
        self._set_throttle       = 0
        self._set_brake          = 0
        self._set_steer          = 0
        self._waypoints          = waypoints
        self._conv_rad_to_steer  = 180.0 / 70.0 / np.pi
        self._pi                 = np.pi
        self._2pi                = 2.0 * np.pi

    def update_values(self, relative_x, relative_y, yaw, speed, timestamp, frame):
        self._current_x         = relative_x
        self._current_y         = relative_y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake

    def update_controls(self):
        # RETRIEVE SIMULATOR FEEDBACK
        x = self._current_x #meter
        y = self._current_y #meter
        yaw = self._current_yaw #radian
        v = self._current_speed #m/s
        self.update_desired_speed()
        v_desired = self._desired_speed #m/s
        t = self._current_timestamp
        waypoints = self._waypoints #x, y, v
        throttle_output = 0 # 0~1
        steer_output = 0 #-1.22~1.22(rad)
        brake_output = 0 #0~1

        # MODULE 7: DECLARE USAGE VARIABLES HERE
        self.vars.create_var('v_prev', 0.0) #velocity
        self.vars.create_var('t_prev', 0.0) #dt = t_prev-t
        self.vars.create_var('e_prev', 0.0) #Error : v_desired-v
        self.vars.create_var('e_iprev', 0.0) #integral Error
        self.vars.create_var('o_tprev', 0.0) #throttle output
        self.vars.create_var('o_sprev', 0.0) #steer output

        # Skip the first frame to store previous values properly
        if self._start_control_loop:
    
            # MODULE 7: IMPLEMENTATION OF LONGITUDINAL CONTROLLER HERE
						# PID Controller

            K_p = 1.0
            K_i = 0.1
            K_d = 0.01

            dt = t-self.vars.t_prev

            throttle_output = 0
            brake_output    = 0

            e_v = v_desired - v
            i_v = self.vars.e_iprev + e_v*dt
            d_v = (e_v - self.vars.e_prev) / dt

            accel = K_p * e_v + K_i*i_v + K_d * d_v

            if(accel >0):
                throttle_output = (np.tanh(accel) + 1)/2
                
                if(throttle_output - self.vars.o_tprev > 0.1):
                    throttle_output = self.vars.o_tprev + 0.1
            
            else:
                throttle_output = 0

            # MODULE 7: IMPLEMENTATION OF LATERAL CONTROLLER HERE
						# PurePursuit

            steer_output = 0
            L = 2.7 #meter - Ford Mustang Wheelbase
            min_ld = 10 #meter
            K_pld = 0.8

            x_desired = x-L*np.cos(yaw)/2
            y_desired = y-L*np.sin(yaw)/2
            l_d = max(min_ld, v*K_pld)
            
            for i in waypoints:
                dist = np.sqrt((i[0] - x_desired)**2 + (i[1] - y_desired)**2)

                if(dist>l_d):
                    target = i
                    break
                else:
                    target = waypoints[0]
                
            alpha = math.atan2(target[1] - y_desired, target[0]-x_desired)-yaw
            delta = math.atan2(2*L*np.sin(alpha)/l_d)
            steer_output = delta

            # SET CONTROLS OUTPUT
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.v_prev = v  # Store forward speed to be used in next step
        self.vars.t_prev = t
        self.vars.e_prev = v_desired-v
        self.vars.e_iprev = i_v
        self.vars.o_tprev = throttle_output
        self.vars.o_sprev = steer_output
