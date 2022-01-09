"""
Author: Nikhil Pandey
Contact: contact@nikhilpandey.in
"""

import cv2
import numpy as np

"""
OpenCV corrdinate system is based on rows and then columns. 
If you want to get pixels at (x, y) access it using (y, x)
"""


class PinholeCam(object):    
    def __init__(self,P,K):
        
        self.x  = None #w 
        self.y  = None #h

        self.K  = K

        self.P  = P

        self.fx = P(0,0)
        self.fy = P(1,1)
        self.cx = P(0,2) 
        self.cy = P(1,2)
        self.Tx = P(0,3)
        self.Ty = P(1,3)

        self.distortion_state = None


        pass

    def rectifyPoint(self,pt):
        
        if self.distortion_state == None:
            return pt

        raw32 = pt.copy()
        rect32= np.zeros_like(pt)

        # cv::Point2f raw32 = uv_raw, rect32;
        # const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
        # cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);
        # cv::undistortPoints(src_pt, dst_pt, K_, D_, R_, P_);
        return rect32

        
 

    def projectPixelTo3dRay(self, raw_pixel):

        x = (raw_pixel[0]- self.cx() - self.Tx()) / self.fx()
        y = (raw_pixel[1]- self.cy() - self.Ty()) / self.fy()
        z = 1.0 
        
        return np.array([x,y,z],type=np.float16)

    def getRay(self, flag="L"):
        
        if flag == "L":
            raw_pixel = (0,self.cy) 

        if flag == "C":
            raw_pixel = (self.cx,self.cy)

        if flag == "R":
            raw_pixel = (self.x,self.cy)

        raw_pixel = self.rectifyPoint(raw_pixel)
        ray = self.projectPixelTo3dRay(raw_pixel)

        return ray


