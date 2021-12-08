import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

# Grab intensity of items = how much it reflects off object
intensity = []
with open('object3dF1.pcd') as f:
     coordinates = f.readlines()

for line in coordinates:  
    try:
        values = list(map(float,line.split()))
        intensity.append(values[3])
    except:
        pass

# Read in point cloud file and visualize

# Full scan
pcd = o3d.io.read_point_cloud('object3dF1.pcd')
o3d.visualization.draw_geometries([pcd])
#pcd = o3d.io.read_point_cloud('object3dF2.pcd')
#o3d.visualization.draw_geometries([pcd])
#pcd = o3d.io.read_point_cloud('object3d2.pcd')

# Hallway
#pcd = o3d.io.read_point_cloud('object3Y.pcd')
# Lab room more points
#pcd = o3d.io.read_point_cloud('labscene.pcd')
# New room with box
#pcd = o3d.io.read_point_cloud('1inliers2.pcd')


#o3d.visualization.draw_geometries([pcd])


# Max angle is calculated using a simple free body diagram, assuming 
# a coefficient of friction on a carpet of 0.016 and assuming
# the robot moves at a constant velocity, making the acceleration = 0
maxangle = 0.92;
min_z = min(pcd.points[2])
#print(min_z, max(pcd.points[2]))
max_z_floor = 0.001*maxangle*min_z

# Translate into 2D by removing anything considered "floor"
# Recognizing anything remaining with a z component to be an obstacle

# with open('inlierfilterpcd.xyz','w') as f:
#     for ptcld in pcd.points:
#         if ptcld[2] > max_z_floor:
#             #print("in loop")
#             f.write(str(ptcld[0]))
#             f.write(" ")
#             f.write(str(ptcld[1]))
#             f.write(" ")
#             f.write(str(ptcld[2]))
#             f.write("\n")

with open('object3dfiltered.xyz','w') as f:
    i = 0
    for ptcld in pcd.points:
        if int(intensity[i]) > 50:
            f.write(str(ptcld[0]))
            f.write(" ")
            f.write(str(ptcld[1]))
            f.write(" ")
            f.write(str(ptcld[2]))
            f.write("\n")
        i = i + 1

newpcd = o3d.io.read_point_cloud('object3dfiltered.xyz')
o3d.visualization.draw_geometries([newpcd])

#Cluster
with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    # labels is an array of the cluster number for each point cloud
    labels = np.array(newpcd.cluster_dbscan(eps=0.2,min_points=12,print_progress=False))
# Each cluster has its own label number
max_label = labels.max()
colors = plt.get_cmap("summer")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
newpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
indices=[];
j = 0;
for color in (np.asarray(newpcd.colors)):
    if color[0] == 0 and color[1] == 0 and color[2] == 0:
        continue
    else:
        indices.append(j)
    j = j + 1
points = np.asarray(newpcd.points)
newpcd.points = o3d.utility.Vector3dVector(points[indices,:])

with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
    labels = np.array(newpcd.cluster_dbscan(eps=0.2,min_points=12,print_progress=True))
max_label = labels.max()
print(f"point cloud has {max_label+1} clusters")
colors = plt.get_cmap("summer")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
newpcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
o3d.visualization.draw_geometries([newpcd])    
#print(labels)
# Octree
octree = o3d.geometry.Octree(max_depth=6)
octree.convert_from_point_cloud(newpcd,size_expand=0.05)
#octree.traverse(f_traverse)
o3d.visualization.draw_geometries([octree])
x = octree.locate_leaf_node(newpcd.points[10])
string1 = str(x[0])
string2 = str(x[1])
new1 = string1.strip('OctreePointColorLeafNode with color ')
new2 = string2.strip('OctreeNodeInfo with origin ')

#print(new1)
#print(new2)
#print(octree.locate_leaf_node(newpcd.points[10]))
#print(octree.locate_leaf_node(newpcd.points[11]))
#print(octree.locate_leaf_node(newpcd.points[15]))


if __name__ == "__main__":
    pass
