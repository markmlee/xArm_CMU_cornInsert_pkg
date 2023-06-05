###################################################################
 #
 # This helper class is for detecting ChAruco pose
 # ChAruco generated using the following link:
 # https://calib.io/pages/camera-calibration-pattern-generator
 #
 # Output : 
 # 
 # Input  : /
 #          /
 
 # E-mail : MoonRobotics@cmu.edu    (Lee Moonyoung)
 
 #
 # Versions :
 # v1.0
 ###################################################################

import numpy as np
import random
import cv2
import time
import rospy
import yaml
from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError


class ChAruco:
    def __init__(self):
 
        print(f"initializing ChAruco class")

        with open('camera_matrix.yaml', 'r') as file:
            yaml_load = yaml.safe_load(file)

        #cam matrix, dist coeff from RealsenseD435 factory settings
        #read in from /camera/color/camera_info ROS topic
        self.camera_matrix = np.array([
            [yaml_load['cam_k_matrix'][0], yaml_load['cam_k_matrix'][1], yaml_load['cam_k_matrix'][2]], 
            [yaml_load['cam_k_matrix'][3], yaml_load['cam_k_matrix'][4], yaml_load['cam_k_matrix'][5]],
            [yaml_load['cam_k_matrix'][6], yaml_load['cam_k_matrix'][7], yaml_load['cam_k_matrix'][8]]
        ])
        

        # self.camera_matrix = np.array([431.89080810546875, 0.0, 313.53997802734375, 0.0, 431.46728515625, 242.6866455078125, 0.0, 0.0, 1.0])
        self.dist_coeff = np.array([0,0,0,0,0])
        self.bridge = CvBridge()
        self.image_cv = None
        self.recevied_image = False

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


        #(num COL, num ROW, size of checker, size of aruco tag, aruco definition) ***please verify COL x ROW ordering
        self.board = cv2.aruco.CharucoBoard_create(5, 3, 0.048, 0.037, self.aruco_dict)

        sub_image = rospy.Subscriber("/camera/color/image_raw", Image, self.get_pose_rb0)


    def get_pose_rb0(self,image_in):
        # rospy.loginfo('Got image')
        # rospy.logdebug('Got image')
        self.image_ros = image_in

        # Try to convert the ROS Image message to a CV2 Image
        try:
            self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        except CvBridgeError:
            rospy.logerr("CvBridge Error")
            rospy.loginfo("CvBridge Error")
            rospy.logdebug("CvBridge Error")

        # self.image_cv = self.bridge.imgmsg_to_cv2(image_in, "bgr8")
        self.recevied_image = True

    def get_image(self):
        return self.image_cv

    def get_image_shape(self):
        return self.image_cv.shape



    def get_offset(self, debug: bool = False):
        frame = self.get_image()
        corners, ids, rejected_points = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if corners is None or ids is None:
            return None
        if len(corners) != len(ids) or len(corners) == 0:
            return None

        ret, c_corners, c_ids = cv2.aruco.interpolateCornersCharuco(corners,ids, frame, self.board)
        if debug:
            image_copy = cv2.aruco.drawDetectedCornersCharuco(frame, c_corners, c_ids)

        gotpose, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(c_corners, c_ids, self.board, 
                                                              self.camera_matrix , self.dist_coeff, np.empty(1), np.empty(1))
        if gotpose:
            dist_coeff_copy = np.array(self.dist_coeff, dtype=np.float32).reshape(-1, 1)
            image_disp = cv2.drawFrameAxes(image_copy, self.camera_matrix, dist_coeff_copy, rvec, tvec, 0.1)
            
            #display image 
            cv2.imshow('image', image_disp)
            cv2.waitKey(0)


        return gotpose, rvec, tvec
    

if __name__ == "__main__":
    print(" ================ testing marker detector ============ ")
    rospy.init_node('ChAruco_detect', anonymous=True)
    ChAruco_instance = ChAruco()

    #wait for image callback thread before proceeding
    while ChAruco_instance.recevied_image == False:
        time.sleep(1)

    #display received image for debugging
    img = ChAruco_instance.get_image()
    img_size = ChAruco_instance.get_image_shape
    # print(f"press ENTER to close image viewer")
    # cv2.imshow("image", img) 
    # cv2.waitKey(0)

    print(f"image size: {img_size}, img.shape {img.shape}")
    print(f"ChAruco_instance.camera_matrix: {ChAruco_instance.camera_matrix}, ChAruco_instance.dist_coeff: {ChAruco_instance.dist_coeff}")

    #get offset
    gotpose, rvec, tvec = ChAruco_instance.get_offset(debug=True)
