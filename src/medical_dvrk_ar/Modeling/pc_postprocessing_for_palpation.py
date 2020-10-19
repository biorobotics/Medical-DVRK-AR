import numpy as np
import open3d as o3d

def pointcloud_downsample(pointcloud):
	point_num = pointcloud.shape[0]
	print(pointcloud.shape)
	dtype = [('x',float),('y',float),('z',float),('qw',float),('qx',float),('qy',float), ('qz',float)]
	pointcloud = pointcloud.view(dtype)
	pointcloud = pointcloud.reshape((point_num,))
	pointcloud = np.sort(pointcloud, order=["x"])

	# first sort in x, then figure out which points should go in rows together
	rows = None
	row = []
	row_size = 0.015
	first_row_point = pointcloud[0]
	DOWNSAMPLING_PARAM = 3
	for point in pointcloud[1:]:
		# choose new first row point
		if abs(point["x"] - first_row_point["x"]) < row_size and first_row_point != point:
			row.append(point)
		# come to the end of a row so sort by y so you visit in a good order
		else:
			row = np.array(row).view(dtype)
			row = np.sort(row, order=["y"])
			# explicltly take every DOWNSAMPLING_PARAM point, can change this
			new_row = row[::DOWNSAMPLING_PARAM].copy()
			if rows is None:
				rows = new_row
			else:
				rows = np.concatenate((rows, new_row))
			row = []
			first_row_point = point
			row.append(point)

	pointcloud = np.array(rows)
	pointcloud = np.array(pointcloud.tolist())
	return pointcloud 

def pointcloud_sort(pointcloud):
 	point_num = pointcloud.shape[0]
 	dtype = [('x',float),('y',float),('z',float),('qw', float), ('qx', float), ('qy', float), ('qz', float)]
 	pointcloud = pointcloud.view(dtype)
 	pointcloud = pointcloud.reshape((point_num,))
 	pointcloud = np.sort(pointcloud, order=["x","y"])
 	pointcloud = np.array(pointcloud.tolist())
 	#to switch from zigzag to corner turn
 	line_startpoint_idx = []
 	for i in range(1,pointcloud.shape[0]-1):
 		last_dis = np.linalg.norm(pointcloud[i]-pointcloud[i-1])
 		next_dis = np.linalg.norm(pointcloud[i+1]-pointcloud[i])
 		if next_dis > 2 * last_dis: # if the moving distance to next point is too large, it's probably the end of the line
 			line_startpoint_idx.append(i+1)

 	print("Line number:",len(line_startpoint_idx))
 	# reverse the order in a line
 	for i in range(len(line_startpoint_idx)-1):
 		pointcloud[line_startpoint_idx[i]:line_startpoint_idx[i+1]] = pointcloud[line_startpoint_idx[i]:line_startpoint_idx[i+1]][::-1]

 	pointcloud[line_startpoint_idx[-1]:] = pointcloud[line_startpoint_idx[-1]:][::-1]
 	return pointcloud

if __name__ == "__main__":
	pointcloud = np.load("/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/blaser_results.npy")
	pointcloud = pointcloud_downsample(pointcloud)
	pointcloud = pointcloud_sort(pointcloud)
	np.save("data/new_planner_points.npy",pointcloud)
	pcd = o3d.geometry.PointCloud()
	pcd.points = o3d.utility.Vector3dVector(pointcloud[:,:3])
	o3d.visualization.draw_geometries([pcd])

