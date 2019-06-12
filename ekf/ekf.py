# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 09:23:37 2018

@author: Mateus Valverde

This is the ekf.cpp used in GPS_INS algorithm, implemented in python by Mateus 

//  Created by Karan on 4/9/18.
//  Copyright © 2018 Karan. All rights reserved.
"""

#include "ekf.hpp"
#include <iostream>

import numpy as np
from math import *
from ekf.utils import *

THRESHOLD = inf

class EKF:
    #def __init__(self):
    
    def start(self, nin, xin, Pin, Fin, Qin):
        self._num_states = nin
        self._I = np.eye(self._num_states, self._num_states)
        print("Starting EKF: Number of states ->", nin, "\n")
        self._state = xin
        print("Starting EKF: Size of Input states ->", len(xin), "\n")
        self._P = Pin
        self._JA = Fin
        self._Q = Qin
    
    def setQ(self, Q_in):
        self._Q = Q_in
    
    def updateJA(self, dt):
        """*******************************************
         * State Equation Update Rule
            x + v dt (cos(ψ))
            y + v dt (sin(ψ))
            dtψ' + ψ
            dta + v
            a
        *******************************************"""
        # std::cout << "Updating JA: About to update state equations" << "\n";
        # std::cout << "Updating JA: size of states" << this->_state.rows() << "x" <<this->_state.cols() << "\n";       
    
        # Updating state equations
        self._state[0,0] = self._state[0,0] + (self._state[3,0] * dt) * cos(self._state[2,0])
        self._state[1,0] = self._state[1,0] + (self._state[3,0] * dt) * sin(self._state[2,0])
        #self._state[2,0] = fmod((self._state[2,0] + self._state[4,0] * dt + np.pi), (2.0 * np.pi)) - np.pi
        self._state[2,0] = self._state[2,0] + self._state[4,0] * dt
        self._state[3,0] = self._state[3,0] + self._state[5,0] * dt
        self._state[4,0] = self._state[4,0]
        self._state[5,0] = self._state[5,0]
        self._state[6,0] = self._state[6,0]
            
        # print(dt)
        # std::cout << "Updating JA: About to calculate jacobian" << "\n";
        # Calculate jacobian
        # print("State: ", self._state)
        self._JA =  calculate_jacobian(self._state, dt)
        # print("JA: ", self._JA)
    
    def predict(self):
        # Prediction step
        self._P = np.dot(self._JA, np.dot(self._P, self._JA.T)) + self._Q
    
    def update(self, Z, Hx, JH, R):
        JHT = np.dot(self._P, JH.T)
        # Temporary variable for storing this intermediate value
        self._S = np.dot(JH, JHT) + R
        # Compute the Kalman gain
        self._K = np.dot( JHT, np.linalg.inv(self._S) )
        # Update the estimate
        y = Z - Hx
        
        self._state = self._state + np.dot(self._K, y)
        
        # Update the error covariance
        self._P = np.dot( self._I - np.dot(self._K, JH), self._P )
    
    def get_resulting_state(self):
        return self._state