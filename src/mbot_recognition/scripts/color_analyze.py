# analyze the color distribution of the image from the camara_record/

import numpy as np
import sys
current_path=sys.path[0]
figure_path=current_path+'/camera_record/'
# get the image file name in the folder
import os
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
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
    
    # plot the figure
    plt.figure()
    plt.imshow(rgb_array/255)
    #plt.show()
    # plot bgr point cloud
    fig=plt.figure()
    ax=fig.add_subplot(111,projection='3d')
    ax.scatter(rgb_1dim_array[:,0],rgb_1dim_array[:,1],rgb_1dim_array[:,2],s=1,c=rgb_1dim_array/255)
    ax.set_xlabel('R')
    ax.set_ylabel('G')
    ax.set_zlabel('B')
    plt.show()
    plt.pause(100)
    # for i in range(0,bgr_array.shape[0]):
    #     for j in range(0,bgr_array.shape[1]):
    #         print(bgr_array[i,j,0],bgr_array[i,j,1],bgr_array[i,j,2])
