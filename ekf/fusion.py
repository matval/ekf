# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 10:07:27 2018

@author: Mateus Valverde

This is the fusion.cpp used in GPS_INS algorithm, implemented in python by Mateus

//  Created by Karan on 4/9/18.
//  Copyright © 2018 Karan. All rights reserved.
"""

from ekf.DataPoint import *
from ekf.ekf import *

class Fusion:
    def __init__(self, max_acceleration, max_turn_rate, max_yaw_accel, varGPS, varHeading, varSpeed, varYaw, varAcc, xOffset, yOffset):
        self._initialized = False
        self._max_turn_rate = max_turn_rate
        self._max_acceleration = max_acceleration
        self._max_yaw_accel = max_yaw_accel
        self._n = 7
        self._KF = EKF()
        self._xOffset = xOffset
        self._yOffset = yOffset
        
        # Initialize initial uncertainity P0
        self._P = np.array([[1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 1000.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1000.0]])
    
        self._F = np.zeros(self._P.shape)
        self._Q = np.zeros(self._P.shape)

        #_R = Eigen::MatrixXd(5, 5); //Assuming 5 sources of measurement
        self._R = np.array([[varGPS**2, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, varGPS**2, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, varHeading**2, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, varSpeed**2, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, varYaw**2, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, varAcc**2]])

    def updateQ(self, dt):
        # Process Noise Covariance Matrix Q
        self._sGPS = 0.5 * self._max_acceleration * dt**2
        self._sVelocity = self._max_acceleration * dt
        self._sCourse = self._max_turn_rate * dt
        self._sYaw = self._max_yaw_accel * dt
        self._sAccel = self._max_acceleration
        #self._sBias = 0.001
        self._sBias = 0.1*self._max_yaw_accel * dt
        
        self._Q = np.array([[self._sGPS**2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, self._sGPS**2, 0.0, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, self._sCourse**2, 0.0, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, self._sVelocity**2, 0.0, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, self._sYaw**2, 0.0, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, self._sAccel**2, 0.0],
                            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self._sBias**2]])
    
        self._KF.setQ(self._Q)
    
    def start(self, data):
        self._timestamp = data.get_timestamp()
        state = data.get_state()
        self._KF.start(self._n, state, self._P, self._F, self._Q)
        self._initialized = True
    
    def compute(self, data):
        """/*******************************************
         * Prediction Step
         - Assumes current velocity is the same for this dt
         *******************************************/"""
    
        # Assuming 1.e3 for timestamp;
        dt = (data.get_timestamp())/1.e6
        self._timestamp = data.get_timestamp()
    
        # Update Q
        self.updateQ(dt)
        # std::cout << "Updated Q" << "\n";
        # Update state and calculate jacobian
        self._KF.updateJA(dt)
        # std::cout << "Updated JA" << "\n";
        # Prediction
        self._KF.predict()
        # std::cout << "KF Predicted" << "\n";
    
        """/*******************************************
         * Update Step
         - Updates appropriate matrices given a measurement
         - Assumes measurement is received either from GPS or IMU
        *******************************************/"""
        zz = data.get_state()
        
        z = np.array([[zz[0,0]],
                      [zz[1,0]],
                      [zz[2,0]],
                      [zz[3,0]],
                      [zz[4,0]],
                      [zz[5,0]]])
        
        state = self._KF.get_resulting_state()
    
        j13 = - self._xOffset * sin(state[2,0]) - self._yOffset * cos(state[2,0])
        j23 = self._xOffset * cos(state[2,0]) - self._yOffset * sin(state[2,0])
        
        # std::cout << "Checking Data Point Type" << "\n";
        if(data.get_data_point_type() == DataPointType.GPS):
            
            Hx = np.array([[state[0,0] + self._xOffset * cos(state[2,0]) - self._yOffset * sin(state[2,0])],
                           [state[1,0] + self._xOffset * sin(state[2,0]) + self._yOffset * cos(state[2,0])],
                           [state[2,0] - state[6,0]],
                           [state[3,0]],
                           [state[4,0]],
                           [state[5,0]]])
                 
            JH = np.array([[1.0, 0.0, j13, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 1.0, j23, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])
            
        elif(data.get_data_point_type() == DataPointType.IMU):
            
            Hx = np.array([[0.0],
                           [0.0],
                           [state[2,0] - state[6,0]],
                           [state[3,0]],
                           [state[4,0]],
                           [state[5,0]]])
            
            JH = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0],
                           [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0]])
            
        self._KF.update(z, Hx, JH, self._R)
    
    def process(self, data):
        if(data.get_timestamp() > 0.0):
            if(self._initialized):
                self.compute(data)
            else:
                self.start(data)
    
    def get_resulting_state(self):
        return self._KF.get_resulting_state()