"""
Author: Nikhil Pandey
Contact: contact@nikhilpandey.in
"""

import cv2
import math
import numpy as np



class DepthImageToLaserScan(obejct):
    
    def __init__(self,scan_time, scan_height, 
                    range_min, range_max):

        self.angle_increment_step = None
        self.laser_scan =   None #how do I get the length of laser scan, alt is a ls
        self.scan_time  =   None
        self.range_max  =   None
        self.range_min  =   None
        self.scan_height=   None
        self.min_angle  =   None
        self.pinhole_cam=   None

        self.getMinMaxAngles()
        # self.checkScanImgHt(depth_img)



    def checkScanImgHt(self,depth_img):
        pass

    def magnititude_of_ray(self,ray):
        return ((ray.x**2)+(ray.y**2)+(ray.z**2))**0.5
        

    def angle_between_rays(self,pt_1,pt_2 ):

        dot = pt_1.x*pt_2.x + pt_1.y*pt_2.y + pt_1.z*pt_2.z

        mag_1 = self.magnititude_of_ray(pt_1)
        mag_2 = self.magnititude_of_ray(pt_2)

        return math.acos(dot/(mag_1*mag_2)) 

    def use_point(self, new_val, old_val):
        '''
        Logic for update 
        '''
        
        #there are 3 cases 
        #check in range, if not, don't bother
        if new_val not in range(self.range_min,self.range_max):
            return False
        
        # Basically initially the old vals would be NaN. 
        # Infinity is prefered over NaN
        if old_val is None:
            if new_val is not None:
                return True
            else:
                return False

        # if 
        
        pass

    def z_post_proc(self,z):
        pass

    def getMinMaxAngles(self):


        left_ray = self.pinhole_cam.getRay(flag="L") 
        cent_ray = self.pinhole_cam.getRay(flag="C")
        right_ray = self.pinhole_cam.getRay(flag="R")

        self.angle_max =  self.angle_between_rays(left_ray, cent_ray)
        self.angle_min = -self.angle_between_rays(cent_ray,right_ray)
        
        self.angle_increment = (self.angle_max - self.angle_min) / (width - 1)
        



    def hypot(self,a,b):
        return math.hypot(a,b)

    def convert(self,depth_msg):
        
        #principal points [from calibration]
        center_x = self.pinhole_cam.cx()
        center_y = self.pinhole_cam.cy()

        #unit conversion with scaling by focal length computing (X,Y)
        unit_scaling = depth_2_laserscan.depthtraits.tometers(1)#wtf
        
        const_x = unit_scaling / center_x

        depth_img = depth_msg.img#what is this?
        
        self.checkScanImgHt(depth_img)
        
        row_step  = depth_msg.step#what is this as well? 

        offset = center_y-(self.scan_height/2)#put it 0 to begin with
        
        for each_row in depth_img:
            for pixel_location in each_row:
                
                depth = depth_img[each_row][pixel_location]
                th = -math.atan2((pixel_location-center_x)*const_x, unit_scaling)# idk why depth divides out
                idx = (th -self.min_angle)/self.angle_increment_step

                x = (pixel_location-center_x)*depth*const_x
                z = self.z_post_proc(depth)

                r = self.hypot(x,z)

                if self.use_point(r):
                    self.laser_scan[idx]=r


    def getLaserScan(self,depth_msg):



        self.convert(depth_msg)
        return self.laser_scan 
