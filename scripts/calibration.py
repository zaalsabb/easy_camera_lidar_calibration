#!/usr/bin/env python3

import numpy as np
import cv2
import open3d as o3d
import os
import Equirec2Perspec as E2P
import rospy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation    
from render_depthmap import *

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
        
        self.reprojectionError = rospy.get_param('parameters/reprojectionError',default=5)

        self.img = cv2.imread(self.f_img)
        self.pcd = o3d.io.read_point_cloud(self.f_pcd)

        self.pub = rospy.Publisher('/T_cam_lidar',PoseStamped,queue_size=1)

        if self.panorama:
            self.fun_rectify_views()

        self.init_calibration()
        # self.check_pose_estimation()

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

    def pick_3d_points2(self):

        K = np.array(self.K).reshape(3,3)
        I2 = cv2.imread(self.f_img)
        w = I2.shape[1]
        h = I2.shape[0]
        T_cam_lidar = pose2matrix([0,0,0,-0.5,0.5,-0.5,0.5])              
        I2_pcd = project_to_image(self.pcd, T_cam_lidar, K, w, h)

        pt_size = 5
        img_coords=[]

        def interactive_win(event, u, v, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                img_coords.append([u,v])
                print('Picked 2D Points: {}, {}'.format(u,v))
                cv2.circle(I2_pcd, (u, v), int(pt_size), (255, 0, 0), -1)       


        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('image', interactive_win)

        while (1):
            cv2.imshow('image', I2_pcd)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:  # 'Esc' Key
                break
            elif k == 13:  # 'Enter' Key
                break
            elif k == ord('q'):  # 'q' Key
                break        

        pts_2d = np.array(img_coords).T

        D = cloud_to_depth(self.pcd, K, T_cam_lidar, w, h)        

        pts_3d = project_2d_to_3d(pts_2d, D, K, T_cam_lidar, w, h)   

        return pts_3d.T

    def pick_2d_points(self):
         
        pt_size = 5
        img_coords=[]

        def interactive_win(event, u, v, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                img_coords.append([u,v])
                print('Picked 2D Points: {}, {}'.format(u,v))
                cv2.circle(self.img, (u, v), int(pt_size), (255, 0, 0), -1)       


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
        
        pts_3D = self.pick_3d_points2()
        pts_2D = self.pick_2d_points()

        K=np.array(self.K,dtype=np.float32).reshape(3,3)
        d=np.array(self.d,dtype=np.float32).reshape(-1,1)
        
        pts_3D=np.array(pts_3D,dtype=np.float32)
        pts_2D=np.array(pts_2D,dtype=np.float32)

        _,rvecs,tvecs,inliers=cv2.solvePnPRansac(pts_3D, pts_2D, K, d,flags=cv2.SOLVEPNP_P3P, reprojectionError=self.reprojectionError)

        T_lidar_cam = vecs2mat(tvecs,rvecs)
        T_cam_lidar = np.linalg.inv(T_lidar_cam)
        
        np.savetxt(self.f_out,T_cam_lidar,delimiter=',')

        print('\nComputing Transformation From Lidar To Camera..')
        print('Number of inlier points:{}'.format(len(inliers)))
        print('Saved to {}\n'.format(self.f_out))
        print(T_cam_lidar)

        self.check_pose_estimation()
      
    # def refine_calibration(self):
    #     K = np.array(self.K).reshape(3,3)
    #     I2 = cv2.imread(self.f_img)
    #     w = I2.shape[1]
    #     h = I2.shape[0]
    #     T_cam_lidar = np.loadtxt(self.f_out, delimiter=',')        

    #     D = cloud_to_depth(self.pcd, K, T_cam_lidar, w, h)        

    #     pts_3d = project_2d_to_3d(pts_2d, D, K, T_cam_lidar, w, h)

    def check_pose_estimation(self):
        
        K = np.array(self.K).reshape(3,3)
        I2 = cv2.imread(self.f_img)
        T_cam_lidar = np.loadtxt(self.f_out, delimiter=',')        

        I2_pcd = project_to_image(self.pcd, T_cam_lidar, K, I2.shape[1], I2.shape[0])

        alpha_slider_max = 100
        title_window = 'Linear Blend'
        def on_trackbar(val):
            alpha = val / alpha_slider_max
            beta = ( 1.0 - alpha )
            dst = cv2.addWeighted(I2_pcd, alpha, I2, beta, 0.0)
            r = 0.5
            dst = cv2.resize(dst, (int(dst.shape[1]*r), int(dst.shape[0] * r)))
            cv2.imshow(title_window, dst)

        cv2.namedWindow(title_window)
        trackbar_name = 'Alpha x %d' % alpha_slider_max
        cv2.createTrackbar(trackbar_name, title_window , 0, alpha_slider_max, on_trackbar)
        # Show some stuff
        on_trackbar(0)
        on_trackbar(50)
        on_trackbar(100)
        # Wait until user press some key
        cv2.waitKey() 


def project_to_image(pcd,T_c1_m1,K,w,h):

    vis = VisOpen3D(width=w, height=h, visible=False)
    pcd = pcd.voxel_down_sample(0.01)
    vis.add_geometry(pcd)

    vis.update_view_point(K, T_c1_m1)

    img = vis.capture_screen_float_buffer(show=False)
    img = np.uint8(255*np.array(img))
    img = np.dstack([img[:,:,2],img[:,:,1],img[:,:,0]])
    
    cx = K[0,2]
    cy = K[1,2]
    tx = cx-w/2
    ty = cy-h/2
    M = np.array([[1,0,tx],
                  [0,1,ty]])
    img = cv2.warpAffine(img,M,(img.shape[1],img.shape[0]))

    return img  

def vecs2mat(tvecs,rvecs):
    R_ = cv2.Rodrigues(rvecs)[0]
    R = R_.T
    C = -R_.T.dot(tvecs)
    T_m1_c2 = np.eye(4)
    T_m1_c2[:3, :3] = R
    T_m1_c2[:3, 3] = C.reshape(-1) 

    return T_m1_c2   

if __name__ == '__main__':
    Node()
    