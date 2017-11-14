#!/usr/bin/env python
import sys
sys.path.append('/home/ewa/python_cv')

import roslib
roslib.load_manifest('praca_magisterska')

print ('import complete')
import rospy
import cv2
from find_obj import filter_matches,explore_match
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt
import pandas as pd
from collections import deque
import matplotlib.path as mplPath
from numpy import (array, dot, arccos, clip)
from numpy.linalg import norm
import numpy as np
section = np.zeros((640,640,1), np.uint8)



class cross_section:
  
  def __init__(self):
    self.counter = 0
    self.line_counter = 0
    self.histogram = []
    self.l = deque([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]],maxlen=5)
    self.tt = []
    self.good_line = [0,0,0,0]
    #self.tt = deque([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]],maxlen=5)
    self.image_sub = rospy.Subscriber("/ewa/przekroj",Float32MultiArray,self.callback)
    self.color = np.array([])

  def data_filtering(self, data):
        section[:,:,0] = 0
	data_2 = np.array(data.data)
	x = data_2[::5]
	y = data_2[1::5]
	red = data_2[2::5]
	green = data_2[3::5]
	blue = data_2[4::5]
	filter_out = ((y<1.25) & (y>-1.25) & (x>0.5) & (x<=3))
	y = y[np.where(filter_out)]
	x = x[np.where(filter_out)]
	red = red[np.where(filter_out)]
	green = green[np.where(filter_out)]	
	blue = blue[np.where(filter_out)]
	
	y = (y - (-1.25))/(2.5)* 639
	x = ((x-0.5)/(2.5)) * 639
	x = x.astype(int)
	y = y.astype(int)
	return x, y, red, blue, green

  def image_processing(self, x, y):
	section[x,y,0] = 255
	kernel = np.ones((5,5), np.uint8)
	dst = cv2.dilate(section, kernel)
	erode = cv2.erode(dst, kernel)
	rho = 2.0 # distance resolution in pixels of the Hough grid
	theta = (np.pi/180)*0.75 # angular resolution in radians of the Hough grid
	threshold = 150 # minimum number of votes (intersections in Hough grid cell)
	min_line_length = 40 #minimum number of pixels making up a line
	max_line_gap = 30 #20
	lines = cv2.HoughLinesP(erode, rho, theta, threshold, np.array([]), min_line_length, max_line_gap)
	backtorgb = cv2.cvtColor(erode,cv2.COLOR_GRAY2RGB)
	return lines, backtorgb

  def find_door_line(self, lines, backtorgb):
	average_d = np.array(self.l).mean(axis=0)
	#average_w = np.array(self.tt).mean(axis=0)
	average_w = self.tt
	lengths = []
	c = 0
	d = -1
	good_line = [0,0,0,0]
	for val in lines:
		x1,y1,x2,y2 = val[0]
		length = np.sqrt((x2-x1)**2 + (y2 - y1)**2)
		lengths.append(length)
		#cv2.line(backtorgb,(x1+20*c,y1),(x2+20*c,y2),(127, 0, 255),4)
		if (length > 150.0) & (length < 200.0):
	    		diff = np.abs(val[0]-average_d)
			if diff.max() < 20:		
				cv2.line(backtorgb,(x1,y1),(x2,y2),(127, 0, 255),4)
				self.line_counter += 1
				d = c
				good_line = val[0]
			self.l.append(val[0])
		c = c+1
	return lengths, d, backtorgb, average_d, average_w, good_line

  def bounding_box(self, average_w, average_d, backtorgb, d, good_line): 
	#average_d = self.l[0]
	#print d
	u = np.array(average_d)
	cv2.line(backtorgb,((u[0]).astype(int), (u[1]).astype(int)),((u[2]).astype(int), (u[3]).astype(int)),(127, 0, 127),4)
	v = np.array(average_w)
	mini_vector = np.array([(u[2]-u[0])/np.sqrt((u[2]-u[0])**2+(u[3]-u[1])**2), (u[3]-u[1])/np.sqrt((u[2]-u[0])**2+(u[3]-u[1])**2)])
	perp_vector_1 = 20*mini_vector
	perp_vector_2 = -20*mini_vector
	loc = -1	
	#b_box1 = [(self.l[loc][2]+perp_vector_1[1]).astype(int), (self.l[loc][3]-perp_vector_1[0]).astype(int)]
	#b_box2 = [(self.l[loc][2]-perp_vector_1[1]).astype(int), (self.l[loc][3]+perp_vector_1[0]).astype(int)]
	#b_box3 = [(self.l[loc][0]+perp_vector_2[1]).astype(int), (self.l[loc][1]-perp_vector_2[0]).astype(int)]
	#b_box4 = [(self.l[loc][0]-perp_vector_2[1]).astype(int), (self.l[loc][1]+perp_vector_2[0]).astype(int)]
        #print good_line
	b_box1 = [(good_line[2]+perp_vector_1[1]).astype(int), (good_line[3]-perp_vector_1[0]).astype(int)]
	b_box2 = [(good_line[2]-perp_vector_1[1]).astype(int), (good_line[3]+perp_vector_1[0]).astype(int)]
	b_box3 = [(good_line[0]+perp_vector_2[1]).astype(int), (good_line[1]-perp_vector_2[0]).astype(int)]
	b_box4 = [(good_line[0]-perp_vector_2[1]).astype(int), (good_line[1]+perp_vector_2[0]).astype(int)]
	bounding_box = np.array([b_box1, b_box2, b_box3, b_box4])
	if d != -1:
		#print "circles drawn", "d = ", d	
		cv2.circle(backtorgb,((b_box1[0]).astype(int), (b_box1[1]).astype(int)), 5, (127,255,127), -1)
		cv2.circle(backtorgb,((b_box2[0]).astype(int), (b_box2[1]).astype(int)), 5, (127,255,127), -1)
		cv2.circle(backtorgb,((b_box3[0]).astype(int), (b_box3[1]).astype(int)), 5, (127,255,127), -1)
		cv2.circle(backtorgb,((b_box4[0]).astype(int), (b_box4[1]).astype(int)), 5, (127,255,127), -1)
	return u, v, mini_vector, perp_vector_1, perp_vector_2, bounding_box

  def find_wall(self, bounding_box, mini_vector, lines, lengths, backtorgb):
	pad = -40
	bbPath = mplPath.Path(np.array([bounding_box[2]+pad*mini_vector, bounding_box[3]+pad*mini_vector, bounding_box[0]-pad*mini_vector, bounding_box[1]-pad*mini_vector]))
		
	for idx,val in enumerate(lines):
		x1,y1,x2,y2 = val[0]
		if  ((bbPath.contains_point((x1, y1))) & (bbPath.contains_point((x2, y2)))):
			lengths[idx] = 0				 	  		
	e = lengths.index(max(lengths))
	x1_w, y1_w, x2_w, y2_w = lines[e][0][0], lines[e][0][1], lines[e][0][2], lines[e][0][3]
	if (max(lengths)) != 0:	
		cv2.line(backtorgb,(x1_w, y1_w),(x2_w, y2_w),(127, 127, 0),4)	
		#self.tt.append([x1_w, y1_w, x2_w, y2_w])
		self.tt = [x1_w, y1_w, x2_w, y2_w]
	return backtorgb

  def find_color_averages(self, x, y, red, green, blue, bounding_box):
	color_path = mplPath.Path(np.array(bounding_box))
	filter_out_color = ((y<max(bounding_box[:,0])) & (y>min(bounding_box[:,0])) & (x>min(bounding_box[:,1])) & (x<max(bounding_box[:,1])))
	y_color = y[np.where(filter_out_color)]
	x_color = x[np.where(filter_out_color)]
	red_color = red[np.where(filter_out_color)]
	green_color = green[np.where(filter_out_color)]	
	blue_color = blue[np.where(filter_out_color)]
	color_inside_box = color_path.contains_points(zip(y_color,x_color))
	y_color = y_color[np.where(color_inside_box)]
	x_color = x_color[np.where(color_inside_box)]
	red_color = red_color[np.where(color_inside_box)]
	green_color = green_color[np.where(color_inside_box)]	
	blue_color = blue_color[np.where(color_inside_box)]
	self.color = list(self.color)
	self.color.append([np.mean(red_color), np.mean(green_color), np.mean(blue_color)])
	self.color = np.array(self.color)

  def final_door_verification(self, u, v):
	if self.line_counter > 2:
		r = np.dot(u,v)/norm(u)/norm(v) 
		angle = np.rad2deg(np.arccos(clip(r, -1, 1)))
		red_avg = np.mean(self.color[:, 0])
		green_avg = np.mean(self.color[:, 1])
		blue_avg = np.mean(self.color[:, 2])
		if (red_avg > 75 & red_avg < 100 & green_avg > 75 & green_avg < 100 & blue_avg > 35 & blue_avg < 60):
			print "we have a door!"
			print "and it's angle is: ", angle.astype(int) 
	self.counter = 0
	self.line_counter = 0
	self.color = np.array([])
    
  def callback(self, data):
	self.counter += 1

	#u, v = [[0, 0], [0, 0]], [[0, 0], [0, 0]]

	x, y, red, blue, green = self.data_filtering(data)

	lines, backtorgb = self.image_processing(x, y)

	lengths, d, backtorgb, average_d, average_w,good_line = self.find_door_line(lines, backtorgb)

	u, v, mini_vector, perp_vector_1, perp_vector_2, bounding_box = self.bounding_box(average_w, average_d, backtorgb, d, good_line)
	
	if d != -1:
		
		self.find_wall(bounding_box, mini_vector, lines, lengths, backtorgb)

		self.find_color_averages(x, y, red, green, blue, bounding_box)
		
	if self.counter == 10:

		self.final_door_verification(u, v)

	cv2.imshow("Image window", backtorgb)
	cv2.waitKey(1)

	


def main(args):

  cs = cross_section()
  rospy.init_node('door_detection')

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
