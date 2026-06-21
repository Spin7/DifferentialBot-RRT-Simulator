import pygame
import pymunk
import pymunk.pygame_util
import math
import random
import numpy as np
from collections import deque

pymunk.pygame_util.positive_y_is_up = False
# Kalman Filter Class for Position Estimation
class KalmanFilter:
    def __init__(self, dt, 
                 process_variance=1e-4, 
                 measurement_variance_gps=1e-2, 
                 measurement_variance_imu=1e-2):
        """
        Initializes the Kalman Filter.
        
        :param dt: Time interval between updates.
        :param process_variance: Process variance.
        :param measurement_variance_gps: GPS measurement variance.
        :param measurement_variance_imu: IMU measurement variance.
        """
        # Initial state [x, y, vx, vy]
        self.x = np.zeros((4, 1))
        
        # State covariance matrix
        self.P = np.eye(4) * 1.0
        
        # State transition matrix (F)
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0 ],
                           [0, 0, 0, 1 ]])
        
        # Control matrix (B) - Not used in this case
        self.B = np.zeros((4, 2))
        
        # Observation matrix (H)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]])
        
        # Measurement covariance matrix (R)
        self.R = np.diag([measurement_variance_gps, 
                          measurement_variance_gps, 
                          measurement_variance_imu, 
                          measurement_variance_imu])
        
        # Process covariance matrix (Q)
        self.Q = np.eye(4) * process_variance

    def predict(self):
        """
        Predicts the next state and covariance.
        """
        # State prediction
        self.x = np.dot(self.F, self.x)
        
        # Covariance prediction
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        """
        Updates the state with a new measurement.
        
        :param z: Measurement vector [x_gps, y_gps, vx_imu, vy_imu]
        """
        z = np.array(z).reshape((4, 1))
        
        # Measurement residual
        y = z - np.dot(self.H, self.x)
        
        # S = HPH^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # Kalman Gain
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # State update
        self.x = self.x + np.dot(K, y)
        
        # Covariance update
        I = np.eye(4)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

    def get_estimated_position(self):
        """
        Returns the estimated position.
        """
        return (self.x[0, 0], self.x[1, 0])
