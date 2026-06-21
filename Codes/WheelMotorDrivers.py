# from machine import Pin

# pin10 = Pin(10, Pin.OUT) 
# pin11 = Pin(11, Pin.OUT)
# pin12 = Pin(12, Pin.OUT)

# pin10.value(1) 
# pin11.value(1)
# pin12.value(1)

class WheelMotorDrivers:
    def __init__(self, bot, vr, vl):
        self.bot = bot
        self.right_speed = vr  # vr
        self.left_speed = vl   # vl

    def set_motor_speeds(self, new_right_speed, new_left_speed):
        self.right_speed = new_right_speed
        self.left_speed = new_left_speed

        # ONLY FOR SIMULATION
        self.bot.left_wheel_speed = self.left_speed
        self.bot.right_wheel_speed = self.right_speed
        
        # FOR PICO 
        # if(vl>0 and vr>0):
        #     #FORWARD
        #     pin10.value(0) 
        #     pin11.value(0)
        #     pin12.value(1)
        # elif(vl<0 and vr>0):
        #     #LEFT
        #     pin10.value(0) 
        #     pin11.value(1)
        #     pin12.value(1)
        # elif(vl>0 and vr<0):
        #     #RIGHT
        #     pin10.value(1) 
        #     pin11.value(0)
        #     pin12.value(1)
        # elif(vl<0 and vr<0):
        #     #BACKWARD
        #     pin10.value(0) 
        #     pin11.value(0)
        #     pin12.value(0)
        # else:
        #     #STOP
        #     pin10.value(1) 
        #     pin11.value(1)
        #     pin12.value(1)
    
