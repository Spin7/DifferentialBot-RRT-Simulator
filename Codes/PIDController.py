import math

class PIDController:
    def __init__(self):
        self.wheel_speeds = 50


    def adjust_wheel_speeds(self, x, y, xd, yd, theta):
        # Calculate the angle towards the destination
        try:
            destiny_vec_angle = math.atan2((yd - y),(xd - x))
        except:
            if((yd - y)*(xd - x)>0):
                destiny_vec_angle = math.pi/2
            else:
                destiny_vec_angle = -math.pi/2

        alpha = destiny_vec_angle
 
        
        # Calculate angle difference and normalize to [-pi, pi]
        angle_diff = alpha - (theta)

        if angle_diff > math.pi / 15:
            left_wheel_speed = -50
            right_wheel_speed = 50
        elif angle_diff < -math.pi / 15:
            left_wheel_speed = 50
            right_wheel_speed = -50
        else:
            left_wheel_speed = 100
            right_wheel_speed = 100

        return left_wheel_speed, right_wheel_speed