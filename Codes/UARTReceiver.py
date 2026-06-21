class UARTReceiver():
    def __init__(self, Tx_Pin, Rx_Pin, GPS_module=0, IMU_module=0, distance_sensor=0):
        self.x = 0
        self.y = 0
        self.angle = 0
        self.distance_to_obstacle = 0
        self.Tx_Pin = Tx_Pin
        self.Rx_Pin = Rx_Pin
        self.GPS = GPS_module
        self.IMU = IMU_module
        self.distance_sensor = distance_sensor
    
    def refresh_data(self):

        ## ONLY FOR SIMULATION
        self.x, self.y = self.GPS.get_coordinates()
        self.angle, speed = self.IMU.get_data()
        self.distance_to_obstacle = self.distance_sensor.display_distance()

        ## FOR PICO PI
        
    def get_data(self):
        return self.x, self.y, self.angle, self.distance_to_obstacle