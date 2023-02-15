# -*- coding: utf-8 -*-
"""
Created on Wed Mar 16 09:29:18 2022

@author: xiaoqingtech01
"""

row,col=9,9
size = 500
sdf='''<?xml version='1.0'?>
<sdf version="1.4">
<model name="checkerboard">
  <pose>0 0 0 0 0 0</pose>
  <static>false</static>
    <link name="checkerboard">
      <gravity>false</gravity>
      <kinematic>true</kinematic>
      <pose>-0.07 -0.05 0.0 0.0 0.0 0.0</pose>
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
for i in range(row):
    for j in range(col):
        if (i+j)%2==1:
            sdf += '''
    <visual name="sqr{}">
        <pose>{} {} 0.0 0.0 0.0 0.0</pose>
        <geometry>
            <box>
            <size>{} {} 1</size>
            </box>
        </geometry>
        <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 0</emissive>
        </material>
    </visual>
              '''.format(str(i+1)+str(j+1),(j-col/2)*size,(i-row/2)*size,size,size)
        else:
            
            
            sdf += '''
    <visual name="sqr{}">
        <pose>{} {} 0.0 0.0 0.0 0.0</pose>
        <geometry>
            <box>
            <size>{} {} 1</size>
            </box>
        </geometry>
        <material>
            <ambient>0 0 0 1</ambient>
            <diffuse>0 0 0 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <emissive>0 0 0 0</emissive>
        </material>
    </visual>
              '''.format(str(i+1)+str(j+1),(j-col/2)*size,(i-row/2)*size,size,size)

sdf += ''' 
      
      
    </link>
  </model>
</sdf>
'''
with open('../urdf/bigCheckerBoard.sdf','w')as f:
    f.write(sdf)