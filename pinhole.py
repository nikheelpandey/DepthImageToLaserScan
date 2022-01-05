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
            pass
        
        if self.distortion_state == UNKNOWN:
            pass
        
        if self.distortion_state == CALIBRATED:
            

            """
                cv::Point2f raw32 = uv_raw, rect32;
                const cv::Mat src_pt(1, 1, CV_32FC2, &raw32.x);
                cv::Mat dst_pt(1, 1, CV_32FC2, &rect32.x);

            """
            if self.distortion_model == EQUIDISTANT:
                cv2.fisheye.undistortPoints(src_pt, dst_pt, K, D_, R_, P)
        
            elif self.distortion_model == PLUMB_BOB or distortion_model == RATIONAL_POLYNOMIAL :
                cv2.undistortPoints(src_pt, dst_pt, K, D_, R_, P)


    def projectPixelTo3dRay(self):
        pass

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




#          cv::Point2d PinholeCameraModel::project3dToPixel(const cv::Point3d& xyz) const
#  {
#    assert( initialized() );
#    assert(P_(2, 3) == 0.0); // Calibrated stereo cameras should be in the same plane
  
#    // [U V W]^T = P * [X Y Z 1]^T
#    // u = U/W
#    // v = V/W
#    cv::Point2d uv_rect;
#    uv_rect.x = (fx()*xyz.x + Tx()) / xyz.z + cx();
#    uv_rect.y = (fy()*xyz.y + Ty()) / xyz.z + cy();
#    return uv_rect;
#  }
  
#  cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect) const
#  {
#    return projectPixelTo3dRay(uv_rect, P_);
#  }
  
#  cv::Point3d PinholeCameraModel::projectPixelTo3dRay(const cv::Point2d& uv_rect, const cv::Matx34d& P) const
#  {
#    assert( initialized() );
  
#    const double& fx = P(0,0);
#    const double& fy = P(1,1);
#    const double& cx = P(0,2);
#    const double& cy = P(1,2);
#    const double& Tx = P(0,3);
#    const double& Ty = P(1,3);
  
#    cv::Point3d ray;
#    ray.x = (uv_rect.x - cx - Tx) / fx;
#    ray.y = (uv_rect.y - cy - Ty) / fy;
#    ray.z = 1.0;
#    return ray;
#  }
  
#  void PinholeCameraModel::rectifyImage(const cv::Mat& raw, cv::Mat& rectified, int interpolation) const
#  {
#    assert( initialized() );
  
#    switch (cache_->distortion_state) {
#      case NONE:
#        raw.copyTo(rectified);
#        break;
#      case CALIBRATED:
#        initRectificationMaps();
#        if (raw.depth() == CV_32F || raw.depth() == CV_64F)
#        {
#          cv::remap(raw, rectified, cache_->reduced_map1, cache_->reduced_map2, interpolation, cv::BORDER_CONSTANT, std::numeric_limits<float>::quiet_NaN());
#        }
#        else {
#          cv::remap(raw, rectified, cache_->reduced_map1, cache_->reduced_map2, interpolation);
#        }
#        break;
#      default:
#        assert(cache_->distortion_state == UNKNOWN);
#        throw Exception("Cannot call rectifyImage when distortion is unknown.");
#    }
#  }
  