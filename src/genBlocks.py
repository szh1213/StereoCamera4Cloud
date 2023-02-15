#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 09:29:18 2022

@author: xiaoqingtech01
"""

import random
from mpl_toolkits import mplot3d
import numpy as np
import matplotlib.pyplot as plt
from math import sin,cos
fig = plt.figure()
#创建绘图区域
ax = plt.axes(projection='3d')

ax.set_title('3d Scatter plot')

sdf='''<?xml version='1.0'?>
<sdf version="1.4">
<model name="checkerboard">
  <pose>0 0 0 0 0 0</pose>
  <static>false</static>
    <link name="checkerboard">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>
      <inertial>
        <mass>0.01</mass>
        <inertia> <!-- interias are tricky to compute -->
          <!-- http://answers.gazebosim.org/question/4372/the-inertia-matrix-explained/ -->
          <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->
          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->
          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->
          <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->
          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->
          <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->
        </inertia>
      </inertial>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.02 0.02 0.00001</size>
          </box>
        </geometry>
      </collision>
     
'''
def gen(name,x,y,z,size,r,g,b):
    return '''
    <visual name="sqr{}">
        <pose>{} {} {} 0.0 0.0 0.0</pose>
        <geometry>
            <box>
            <size>{} {} {}</size>
            </box>
        </geometry>
        <material>
            <ambient>{} {} {} 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 0</emissive>
            <script>
              <uri>model://camera4cloud/materials/scripts</uri>
              <uri>model://camera4cloud/materials/textures</uri>
              <name>cloud/Image</name>
            </script>
        </material>
    </visual>
              '''.format(name,x,y,z,size,size,1,r,g,b,r,g,b)
        

def helix():
    global sdf
    for i in range(200):
        name = str(i)
        #x,y,z = random.randint(-10000,10000),random.randint(-10000,10000),random.randint(1000,10000)
        angle = 6*3.14159/100*i
        x,y,z = cos(angle)*(i+5)*100,sin(angle)*(i+5)*100,(1.05**i+4)*100
        size = i*4+200#random.randint(100,101)
        r,g,b = random.random(),random.random(),random.random()
        sdf+=gen(name,x,y,z,size,r,g,b)
        c = r
        # ax.scatter3D(x, y, z, c=c)
        
    sdf += ''' 
          
          
        </link>
      </model>
    </sdf>
    '''
    
def box():
    global sdf
    for i in range(1):
        name = str(i)
        #x,y,z = random.randint(-10000,10000),random.randint(-10000,10000),random.randint(1000,10000)
        
        x,y,z = i*100*(1-i%3),i*100*(1-(i+1)%3),0+i*50
        size = i*8+50#random.randint(100,101)
        # r,g,b = random.random(),random.random(),random.random()
        r,g,b=1,0,0
        sdf+=gen(name,x,y,z,size,r,g,b)
        c = r
        # ax.scatter3D(x, y, z, c=c)
        
    sdf += ''' 
          
          
        </link>
      </model>
    </sdf>
    '''

box()
with open('../urdf/lowCloud.sdf','w')as f:
    f.write(sdf)


#plt.show()
