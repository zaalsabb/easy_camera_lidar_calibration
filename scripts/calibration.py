#!/usr/bin/env python3

import numpy as np
import cv2
import open3d as o3d
import os
import Equirec2Perspec as E2P
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation    

class Node():
    def __init__(self):

        rospy.init_node('calibration')
        
        self.f_pcd = rospy.get_param('common/pcd_file')
        self.f_img = rospy.get_param('common/image_file')
        self.f_out = rospy.get_param('common/result_file')

        self.K = rospy.get_param('camera/camera_matrix')
        self.d = rospy.get_param('camera/dist_coeffs')
        self.panorama = rospy.get_param('camera/panorama',default=False)
        self.fov  = rospy.get_param('camera/fov',default=60)

        self.img = cv2.imread(self.f_img)
        self.pcd = o3d.io.read_point_cloud(self.f_pcd)

        self.pub = rospy.Publisher('/T_cam_lidar',PoseStamped,queue_size=1)

        if self.panorama:
            self.fun_rectify_views()

        self.init_calibration()

    def pick_3d_points(self):
        print("")
        print("1) Please pick at least 4 correspondences using [shift + left click]")
        print("   Press [shift + right click] to undo point picking")
        print("   Press [shift + '+' or '-'] to change point size")
        print("2) Afther picking points, press q for close the window")        

        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window()
        vis.add_geometry(self.pcd)
        vis.run()  # user picks points
        vis.destroy_window()
        print("")
        return np.asarray(self.pcd.points)[vis.get_picked_points()]   

    def pick_2d_points(self):
         
        pt_size = 2
        img_coords=[]

        def interactive_win(event, u, v, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                img_coords.append([u,v])
                print('Picked 2D Points: {}, {}'.format(u,v))
                cv2.circle(self.img, (u, v), int(5*pt_size), (255, 0, 0), -1)       


        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('image', interactive_win)

        while (1):
            cv2.imshow('image', self.img)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:  # 'Esc' Key
                break
            elif k == 13:  # 'Enter' Key
                break
            elif k == ord('q'):  # 'q' Key
                break        

        return np.array(img_coords)

    def fun_rectify_views(self):

        equ = E2P.Equirectangular(self.img)    # Load equirectangular image
        fov = self.fov
        phi = 0
        width = fov*equ._width/360
        height = fov*equ._height/180

        theta_1 = 0
        theta_2  = 180
        theta_3 = 90
        theta_4  = 270

        I1,K1 = equ.GetPerspective(fov, theta_1, phi, height, width)        
        I2,_  = equ.GetPerspective(fov, theta_2, phi, height, width)        
        I3,_  = equ.GetPerspective(fov, theta_3, phi, height, width)        
        I4,_  = equ.GetPerspective(fov, theta_4, phi, height, width)        

        fdir = os.path.dirname(self.f_img)
        cv2.imwrite(os.path.join(fdir,'I1_rect.jpg'), I1)
        cv2.imwrite(os.path.join(fdir,'I2_rect.jpg'), I2)
        cv2.imwrite(os.path.join(fdir,'I3_rect.jpg'), I3)
        cv2.imwrite(os.path.join(fdir,'I4_rect.jpg'), I4)

        print(K1)
        print('height: %i' % height)
        print('width: %i' % width)
        self.K = K1
        self.img = I1
        self.d = [0,0,0,0,0]

    def init_calibration(self):
        
        pts_3D = self.pick_3d_points()
        pts_2D = self.pick_2d_points()

        K=np.array(self.K,dtype=np.float32).reshape(3,3)
        d=np.array(self.d,dtype=np.float32).reshape(-1,1)
        
        pts_3D=np.array(pts_3D,dtype=np.float32)
        pts_2D=np.array(pts_2D,dtype=np.float32)

        _,rvecs,tvecs,inliers=cv2.solvePnPRansac(pts_3D, pts_2D, K, d,flags=cv2.SOLVEPNP_P3P)

        R_ = cv2.Rodrigues(rvecs)[0]
        R = R_.T
        C = -R_.T.dot(tvecs)  

        T_lidar_cam = np.eye(4)
        T_lidar_cam[:3,:3] = R.T
        T_lidar_cam[:3,3] = -R.T.dot(C).reshape(-1)    
        
        np.savetxt(self.f_out,T_lidar_cam,delimiter=',')

        print('\nComputing Transformation From Camera To Lidar..')
        print('Number of inlier points:{}'.format(len(inliers)))
        print('Saved to {}\n'.format(self.f_out))
        print(T_lidar_cam)

        # t = T_lidar_cam[:3,3]
        # R = T_lidar_cam[:3,:3]        
        # q = Rotation.from_matrix(R).as_quat()

        # rvecs = cv2.Rodrigues(R)[0]

        # msg = PoseStamped()
        # msg.pose.position.x = t[0]
        # msg.pose.position.y = t[1]
        # msg.pose.position.z = t[2]

        # msg.pose.orientation.x = q[0]
        # msg.pose.orientation.y = q[1]
        # msg.pose.orientation.z = q[2]
        # msg.pose.orientation.w = q[3]

        # self.pub.publish(msg)
    
        # # check calibration
        # pts3d = np.asarray(self.pcd.points)
        # pts3d = np.array(pts3d,dtype=np.float32).reshape(-1,1)

        # pts2d = cv2.projectPoints(pts3d,rvecs,t,K,d)
        # pass
            

if __name__ == '__main__':
    Node()
    