# -*- coding: utf-8 -*-
"""
Created on Sun Oct  7 14:44:52 2018

@author: mat_v

//  paramReader.hpp
//  Fusion
//
//  Created by Karan on 6/6/18.
//  Copyright � 2018 Karan. All rights reserved.
"""

from collections import OrderedDict 

class ParameterReader:
    def __init__(self, filename="ekf/parameters.txt"):
        self.data = {}
        
        fin = open(filename, "r") 
        
        for str in fin:
            print("------ Reading in Parameter File...\r\n")
            print(" Line Read: ")
            
            if (str[0] == '#'):
                continue

            pos = str.find("=");
            if (pos == -1):
                print("pos found = -1 ---- Continuing loop...\r\n")
                continue

            key = str[0:pos]
        
            value = float(str[pos+1:len(str)])

            self.data[key] = value
            
            print("	Key Found with Value: ", key, " -> ", value)
            print("	Stored data mapping:   key (", key, ") ------- value(", self.data[key], ")\r\n")

    def getData(self, key):
        value = self.data[key]
        print("Searching for key (", key, ") => ", self.data[key], '\n')
        """if (iter == self.data.end()):
            print(" Parameter name ", key, " not found!")
            return "NOT_FOUND"
        """
        return value
    
