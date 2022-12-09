#!/usr/bin/python
# -*- coding: utf-8 -*-

#from math import *
import math
import numpy as np
#from engr4200_basics_src_lib.functions import angle_wrap, comp, state_inv, state_inv_jacobian
import engr4200_basics_src_lib.functions as funcs
#import scipy.linalg
import rospy

#============================================================================
class EKF_SLAM(object):
    '''
    Class to hold the whole EKF-SLAM.
    '''
    
    #========================================================================
    def __init__(self, x0,y0,theta0, odom_lin_sigma, 
                 odom_ang_sigma, meas_rng_noise, meas_ang_noise):
        '''
        Initializes the ekf filter
        room_map : an array of lines in the form [x1 y1 x2 y2]
        num      : number of particles to use
        odom_lin_sigma: odometry linear noise
        odom_ang_sigma: odometry angular noise
        meas_rng_noise: measurement linear noise
        meas_ang_noise: measurement angular noise
        '''
        
        # Copy parameters
        self.odom_lin_sigma = odom_lin_sigma
        self.odom_ang_sigma = odom_ang_sigma
        self.meas_rng_noise = meas_rng_noise
        self.meas_ang_noise = meas_ang_noise
        self.chi_thres = 0.135 # TODO chose your own value #changed from 0
       
        # Odometry uncertainty 
        self.Qk = np.array([[ self.odom_lin_sigma**2, 0, 0],\
                            [ 0, self.odom_lin_sigma**2, 0 ],\
                            [ 0, 0, self.odom_ang_sigma**2]])
        
        # Measurements uncertainty
        self.Rk=np.eye(3) #was 2, then was 3
        self.Rk[0,0] = self.meas_rng_noise
        self.Rk[1,1] = self.meas_ang_noise
        
        # State vector initialization
        self.xk = np.array([x0,y0,theta0]) # Position
        self.Pk = np.zeros((6,6)) # Uncertainty #changed from 3 by 3
        
        # Initialize buffer for forcing observing n times a feature before 
        # adding it to the map
        self.featureObservedN = np.array([])
        self.min_observations = 5
    
    #========================================================================
    def get_number_of_features_in_map(self):
        '''
        returns the number of features in the map
        '''
        return (self.xk.size-3)/2
    
    #========================================================================
    def get_polar_line(self, line, odom):
        '''
        Transforms a line from [x1 y1 x2 y2] from the world frame to the
        vehicle frame using odometry [x y ang].
        Returns [range theta]
        '''
        # Line points
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        
        # Compute line (a, b, c) and range
        line = np.array([y1-y2, x2-x1, x1*y2-x2*y1])
        pt = np.array([odom[0], odom[1], 1])
        dist = np.dot(pt, line) / np.linalg.norm(line[:2])
        
        # Compute angle
        if dist < 0:
            ang = np.arctan2(line[1], line[0])
        else:
            ang = np.arctan2(-line[1], -line[0])
        
        # Return in the vehicle frame
        return np.array([np.abs(dist), angle_wrap(ang - odom[2])])
        
    #========================================================================
    def predict(self, uk, lines):
        
        '''
        Predicts the position of the robot according to the previous position and the odometry measurements. It also updates the uncertainty of the position
        '''
        #TODO: Program this function
        # - Update self.xk and self.Pk using uk and self.Qk
             
        # Compound robot with odometry
        
        # Compute jacobians of the composition with respect to robot (A_k) 
        # and odometry (W_k)
        
        # Prepare the F_k and G_k matrix for the new uncertainty computation

        # Compute uncertainty
        
        # Update the class variables


	self.xk = funcs.comp(self.xk, uk) #returns array [c1, c2, c3] that is [x, y, theta]

	zk = self.xk

	new_lines = np.zeros((lines.shape[0],3))
	#print('new lines shape', new_lines.shape)
	#print('lines.shape', lines.shape)
	for i in range (0, lines.shape[0]):
		#new_lines[i,:] = [lines[i,0], lines[i,1], math.atan(lines[i, 3]-lines[i,1])/(lines[i,2]-lines[i,0])]
		a = lines[i, 3]-lines[i, 1]
		b = lines[i,2]-lines[i,0]
		c = lines[i,0]*lines[i,3] - lines[i,2]*lines[i, 1]
		new_lines[i,0] = a #lines[i,0]		
		new_lines[i,1] = b #lines[i,1]
		new_lines[i,2] = c #math.atan((lines[i, 3]-lines[i,1])/(lines[i,2]-lines[i,0]))
		zk = np.append(zk, new_lines[i], axis = 0)
		#print('zk ', zk)
	#print('self.xk',self.xk)
	#self.xk = zk
	#Fk = np.array([[math.cos(zk[2]) + 1, -1*math.sin(zk[2]), -uk[0]*math.sin(zk[2])- uk[1]*math.cos(zk[2])],
	#		[math.sin(zk[2]), math.cos(zk[2])+1, uk[0]*math.cos(zk[2]) - uk[1]*math.sin(zk[2])],
	#		[0,0,1]])
	
	Fk = np.array([[1,0,0,0,0,0],
		[0,1,0,0,0,0],
		[0,0,1,0,0,0],
		[math.cos(zk[2]) + 1, -1*math.sin(zk[2]), -uk[0]*math.sin(zk[2])- uk[1]*math.cos(zk[2]), 0,0,0],
		[math.sin(zk[2]), math.cos(zk[2])+1, uk[0]*math.cos(zk[2]) - uk[1]*math.sin(zk[2]), 0,0,0],
		[0,0,1,0,0,0]])

	#Gk = np.array([
	#		[0,-1,0],
	#		[1,0,0],
	#		[-zk[3], zk[4], 0]#[-lines[0,1], lines[0,0],0] #changed from lines[] to zk[]
	#		])

	Gk = np.array([[0,0,0],
			[0,0,0],
			[0,0,0],
			[0,-1,0],
			[1,0,0],
			[-zk[3], zk[4], 0]
			])
        for j in range(0, lines.shape[0]):
		#we have lines from robot pov, we need them from world pov

		robot_lines = funcs.get_polar_line(lines[j]) 
		pr = robot_lines[0]
		phir = robot_lines[1]
		#D = 90 - (alpha - phiw)

                # Compute matrices
		
                # h = funcs.get_polar_line(...)
		x = self.xk[0]
		y = self.xk[1]
		ang_w = phir + self.xk[2]
		pw = pr  + math.sqrt(x**2 + y**2)  * math.cos(math.atan2(y,x) - pr)#pr here should be phir?
		
		h = [pw, ang_w]

		#change this expression using tangent

		dpdx = (-x*math.cos(-phir + math.atan(y/x)) + y*math.sin(-phir + math.atan(y/x))) / math.sqrt(x**2 + y**2)
		dpdy = (-y*math.cos(-phir + math.atan(y/x)) + x*math.sin(-phir + math.atan(y/x))) / math.sqrt(x**2 + y**2)
		dpdt = 0
		dphidx = 0
		dphidy = 0
		dphidt = 1
                H = np.array([[dpdx, dpdy, dpdt],[dphidx, dphidy, dphidt]])
	Ck = np.array([[0,0,0],
			[0,0,0],
			[0,0,0],
			[0,0,0],
			[dpdx, dpdy, dpdt],
			[dphidx, dphidy, dphidt]
			])

	#Pk is uncertainty
	#print(np.dot(self.Rk, np.transpose(Gk)))
	#self.Pk = np.dot(Fk, np.dot(self.Pk, np.transpose(Fk))) + np.dot(Gk, np.dot(self.Rk, np.transpose(Gk)))
	self.Pk = np.dot(Fk, np.dot(self.Pk, np.transpose(Fk))) + np.dot(Ck, np.dot(self.Rk, np.transpose(Ck)))
	#print('end of predict function')

    #========================================================================
        
    def data_association(self, lines):
        '''
        Implements ICCN for each feature of the scan.
        '''
    
        #TODO: Program this function
        # for each sensed line do:
        #   1- Transform the sensed line to polar
        #   2- for each feature of the map (in the state vector) compute the 
        #      mahalanobis distance                
        #   3- Data association          

	for i in range (0, lines.shape[0]):
		print('lines ', lines)
		z = funcs. get_polar_line(lines[i])
		print('z ', z)	
	
	#use machinelearningplus.com/statistics/mahalanobis-distance to learn to compute the mahalanobis distance


        
        # Init variable
        Innovk_List   = np.zeros((0,0))
        H_k_List      = np.zeros((0,0))
        Rk_List       = np.zeros((0,0))
        idx_not_associated = np.array(range(lines.shape[0]))
                
        return Innovk_List, H_k_List, Rk_List, idx_not_associated
        
    #========================================================================
    def update_position(self, Innovk_List, H_k_List, Rk_List) :
        '''
        Updates the position of the robot according to the given the position
        and the data association parameters.
        Returns state vector and uncertainty.
        
        '''
        #TODO: Program this function
        if Innovk_List.shape[0]<1:
            return
            
        # Kalman Gain
        
        # Update Position
        
        # Update Uncertainty
            
    #========================================================================
    def state_augmentation(self, lines, idx):
        '''
        given the whole set of lines read by the kineckt sensor and the
        indexes that have not been associated augment the state vector to 
        introduce the new features
        '''
        # If no features to add to the map exit function
        if idx.size<1:
            return
        
        # TODO Program this function
	


    #========================================================================
    def tfPolarLine(self,tf,line):
        '''
        Transforms a polar line in the robot frame to a polar line in the
        world frame
        '''
        # Decompose transformation
        # Decompose transformation
        x_x = tf[0]
        x_y = tf[1]
        x_ang = tf[2]  
        
        # Compute the new phi
        phi = angle_wrap(line[1] + x_ang)
        
        rho_ = line[0] + x_x * np.cos(phi) + x_y * np.sin(phi)
        sign = 1
        if rho_ <0:
            rho_ = -rho_
            phi = angle_wrap(phi+pi)   
            sign = -1
        
        # Allocate jacobians
        H_tf = np.zeros((2,3))
        H_line = np.eye(2)

        # TODO: Evaluate jacobian respect to transformation
        
        # TODO: Evaluate jacobian respect to line
                
        return np.array([rho,phi]), H_tf, H_line
                
    #========================================================================
    def lineDist(self,z,idx):
        '''
        Given a line and an index of the state vector it computes the
        distance between both lines
        '''        
        # TODO program this function
                
        # Transform the map line into robot frame and compute jacobians
        h = []
        H_position = []
        H_line =[]
        
        # Allocate overall jacobian
        H = []
        
        # Concatenate position jacobians and place them into the position
        
        # Place the position of the jacobina with respec to the line in its
        # position
        
        # Calculate innovation
        v = []
        
        # Calculate innovation uncertainty
        S = []
  
        # Calculate mahalanobis distance
        D = []
        
        return D,v,h,H
