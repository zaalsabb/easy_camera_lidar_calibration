# Credit: https://github.com/fuenwang/Equirec2Perspec

import os
import sys
import cv2
import numpy as np

def xyz2lonlat(xyz):
    atan2 = np.arctan2
    asin = np.arcsin

    norm = np.linalg.norm(xyz, axis=-1, keepdims=True)
    xyz_norm = xyz / norm
    x = xyz_norm[..., 0:1]
    y = xyz_norm[..., 1:2]
    z = xyz_norm[..., 2:]

    lon = atan2(x, z)
    lat = asin(y)
    lst = [lon, lat]

    out = np.concatenate(lst, axis=-1)
    return out

def lonlat2XY(lonlat, shape):
    X = (lonlat[..., 0:1] / (2 * np.pi) + 0.5) * (shape[1] - 1)
    Y = (lonlat[..., 1:] / (np.pi) + 0.5) * (shape[0] - 1)
    lst = [X, Y]
    out = np.concatenate(lst, axis=-1)

    return out 

class Equirectangular:
    def __init__(self, img):

        if isinstance(img, str):
            self._img = cv2.imread(img, cv2.IMREAD_COLOR)
        else:
            self._img = img

        [self._height, self._width, _] = self._img.shape
        self._K = None
        #cp = self._img.copy()  
        #w = self._width
        #self._img[:, :w/8, :] = cp[:, 7*w/8:, :]
        #self._img[:, w/8:, :] = cp[:, :7*w/8, :]
    

    def GetPerspective(self, FOV, THETA, PHI, height, width):
        #
        # THETA is left/right angle, PHI is up/down angle, both in degree
        #

        fx = 0.5 * width * 1 / np.tan(0.5 * FOV / 180.0 * np.pi)
        fy = 0.5 * height * 1 / np.tan(0.5 * FOV / 180.0 * np.pi)
        cx = (width - 1) / 2.0
        cy = (height - 1) / 2.0
        K = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0,  1],
            ], np.float32)
        
        K_inv = np.linalg.inv(K)
        
        x = np.arange(width)
        y = np.arange(height)
        x, y = np.meshgrid(x, y)
        z = np.ones_like(x)
        xyz = np.concatenate([x[..., None], y[..., None], z[..., None]], axis=-1)
        xyz = np.dot(xyz , K_inv.T)

        y_axis = np.array([0.0, 1.0, 0.0], np.float32)
        x_axis = np.array([1.0, 0.0, 0.0], np.float32)
        R1, _ = cv2.Rodrigues(y_axis * np.radians(THETA))
        R2, _ = cv2.Rodrigues(np.dot(R1, x_axis) * np.radians(PHI))
        R = np.dot(R2 , R1)
        xyz = np.dot(xyz , R.T)
        lonlat = xyz2lonlat(xyz) 
        XY = lonlat2XY(lonlat, shape=self._img.shape).astype(np.float32)
        persp = cv2.remap(self._img, XY[..., 0], XY[..., 1], cv2.INTER_CUBIC, borderMode=cv2.BORDER_WRAP)

        return persp,K


if __name__ == '__main__':
    fname = '/home/zaid/datasets/processed/rgb/1'
    equ = Equirectangular(fname+'.jpg')    # Load equirectangular image
    
    FOV = 60
    THETA = 0
    PHI = 0
    width = FOV*equ._width/360
    height = FOV*equ._height/180
    img,K = equ.GetPerspective(FOV, THETA, PHI, height, width)
    
    cv2.imwrite(fname+'_rect.jpg', img)
    print(K)
    print('height: %i' % height)
    print('width: %i' % width)