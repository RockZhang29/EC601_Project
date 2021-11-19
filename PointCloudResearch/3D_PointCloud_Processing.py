import numpy as np
# Handles plotting
import matplotlib.pyplot as plt
# Handles 3d projection
from mpl_toolkits import mplot3d

# Below is grabbing xyz and rgb variables based on spatial query and filtering the initial point cloud
file_data_path="/Users/GA/Documents/_ECE_2022/EC601/Point Cloud Sample/sample.xyz"
point_cloud=np.loadtxt(file_data_path, skiprows=1)
mean_Z=np.mean(point_cloud,axis=0)[2]
spatial_query=point_cloud[abs(point_cloud[:,2]-mean_Z)<1]
xyz=spatial_query[:,:3]
rgb=spatial_query[:,3:]

ax = plt.axes(projection='3d')
ax.scatter(xyz[:,0],xyz[:,1],xyz[:,2],c=rgb/255,s=0.01)
plt.show()
