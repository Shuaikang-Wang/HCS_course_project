# analyze the color distribution of the image from the camara_record/

import numpy as np
import sys
current_path=sys.path[0]
figure_path=current_path+'/camera_record/'
# get the image file name in the folder
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import cv2



def get_mask(bgr_array):
    rgb_array=np.zeros(bgr_array.shape)
    rgb_array[:,:,0]=bgr_array[:,:,2]
    rgb_array[:,:,1]=bgr_array[:,:,1]
    rgb_array[:,:,2]=bgr_array[:,:,0]
    K1=-40
    K2=50
    C=800
    mask= np.zeros(rgb_array.shape,np.int8)
    for i in range(0,rgb_array.shape[0]):
        for j in range(0,rgb_array.shape[1]):
            r=rgb_array[i][j][0]
            g=rgb_array[i][j][1]
            b=rgb_array[i][j][2]
            if C+K1*r+K2*g>b:
                mask[i][j][0]=0
                mask[i][j][1]=0
                mask[i][j][2]=0
            else:
                mask[i][j][0]=254
                mask[i][j][1]=254
                mask[i][j][2]=254
    mask = np.uint8(mask)
    print("---------------GENERATE MASK---------------------")
    mask_cv2 = cv2.cvtColor(mask, cv2.COLOR_RGB2BGR)
    print("---------------GENERATE MASK---------------------")
    return mask_cv2

if __name__=="__main__":
    file_list=os.listdir(figure_path)
    file_list.sort()
    for file in file_list:
        if file[-3:]!='jpg':
            file_list.remove(file)
        # read the image
        bgr_array=np.load(figure_path+file)
        rgb_array=np.zeros(bgr_array.shape)
        rgb_array[:,:,0]=bgr_array[:,:,2]
        rgb_array[:,:,1]=bgr_array[:,:,1]
        rgb_array[:,:,2]=bgr_array[:,:,0]
        # print the bgr array as point cloud
        # print(bgr_array.shape)
        rgb_1dim_array=rgb_array.reshape(rgb_array.shape[0]*rgb_array.shape[1],3)
        
        
        #plt.show()
        # plot bgr point cloud
        fig=plt.figure()
        
        ax=fig.add_subplot(111,projection='3d')
        l=rgb_1dim_array.shape[0]
        ax.scatter(rgb_1dim_array[0:l:4,0],rgb_1dim_array[0:l:4,1],rgb_1dim_array[0:l:4,2],s=1,c=rgb_1dim_array[0:l:4]/255)
        ax.set_xlabel('R')
        ax.set_ylabel('G')
        ax.set_zlabel('B')
        # plot a plane to devide the red color and the others
        x=np.linspace(0,255,100)
        y=np.linspace(0,255,100)
        X,Y=np.meshgrid(x,y)
        # K1=-2
        # K2=3
        # C=80
        K1=-40
        K2=50
        C=800
        Z=C+K1*X+K2*Y
        ax.plot_surface(X,Y,Z,color='b',alpha=0.5)
        mask_cv2=get_mask(bgr_array)
        cv2.imshow('cv2_image', mask_cv2)
        plt.show()
        plt.pause(100)
