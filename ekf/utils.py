# -*- coding: utf-8 -*-
"""
Created on Fri Oct  5 10:20:34 2018

@author: Mateus Valverde

This is the utils.cpp used in GPS_INS algorithm, implemented in python by Mateus

Created by Karan on 4/9/18.
Copyright © 2018 Karan. All rights reserved.
"""

import numpy as np
from math import *

def calculate_jacobian(v, dt):
    # Assumes Jacobian is 6 x 6
    JA = np.zeros([6,6])
    
    # Assumes the size of input vector is 6
    psi = v[2,0]
    velocity = v[3,0]
    psi_dot = v[4,0]

    r13 = -velocity*dt*sin(psi);
    r14 = dt*cos(psi);

    r23 = velocity*dt*cos(psi);
    r24 = dt*sin(psi);

    JA = np.array([[1.0, 0.0, r13, r14, 0.0, 0.0, 0.0],
                   [0.0, 1.0, r23, r24, 0.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0, 0.0, dt,  0.0, 0.0],
                   [0.0, 0.0, 0.0, 1.0, 0.0,  dt, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                   [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]])
    
    return JA