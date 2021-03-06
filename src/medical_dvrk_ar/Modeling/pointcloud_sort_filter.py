"""
Data preprocessing to Get the n by 6 key points pose for path planning,
Every row will contain [x,y,z,x',y',z'] where x' y' z' is the normal vector.
All rows will be in the order for the robot to execute.
"""

import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import AgglomerativeClustering
from scipy.ndimage import gaussian_filter1d

class filter_pointcloud_for_path_planner():
	def __init__(self, max_angle_to_filter_norm, keepRows, keepCols, connectivity_test):
		# the initial point cloud read from the ply file, this one does not has norm in its property
		self.raw_pcl_without_norm = []

		# the point cloud with the normal vector
		self.raw_pcl_with_raw_norm = []

		# the filtered point cloud whose normal vector is all reachable by the robot
		self.raw_pcl_with_filtered_norm = []

		# the points is ordered for the robot to execute, copy() just work as a placeholder
		self.sorted_pcl_with_filtered_norm = []

		# any point with norm larger than this value will be filtered out
		self.max_angle_filtering = max_angle_to_filter_norm

		# how many rows and column need to keep for path planner
		self.keepRows = keepRows
		self.keepCols = keepCols

		# do we need to fix the blaser "two side" bug
		self.connectivity_test = connectivity_test

	def loadData(self, raw_data, isPlyPath):
		self.isPly = isPlyPath
		if isPlyPath:
			if type(raw_data) != str:
				print('You should pass the ply file path as string type')
				exit()
			else:
				self.raw_pcl_without_norm = o3d.io.read_point_cloud(raw_data)
		else:
			pcd = raw_data
			self.raw_pcl_without_norm = o3d.geometry.PointCloud()
			self.raw_pcl_without_norm.points = o3d.utility.Vector3dVector(pcd[:, 0:3])

	def display_inlier_outlier(self, cloud, ind):
		inlier_cloud = cloud.select_by_index(ind)
		outlier_cloud = cloud.select_by_index(ind, invert=True)

		print("Showing outliers (red) and inliers (gray): ")
		outlier_cloud.paint_uniform_color([1, 0, 0])
		inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
		o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud],)

	def downsample_raw_pcl_get_normal_vector(self, dsample_voxelSize=0.005, norm_est_radius=0.01, vis=False):
		"""
		Param:
			1. voxelSize: float,the larger this number is, the sparse the output point cloud will be
			2. norm_est_radius：float,the surface area which the norm is get from, the bigger it is, the more "uniform" normal vector will be
			3. visualize: Bool, if ture, there will be a pop-up window for visualize the points, press "n" to visualize the normal vector
		Output:
			self.sparse_pcl_with_norm : N X 6 nparray, float, N indicates the number of the downsampled point,
									    6 is [x,y,z,u,v,w] where[x,y,z] is the position of the points, [u,v,w] is the normal vector of that point
		"""

		if self.isPly:
			self.raw_pcl_without_norm.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=norm_est_radius, max_nn=50))
			downpcd = self.raw_pcl_without_norm.voxel_down_sample(voxel_size=dsample_voxelSize)
			# downpcd = self.raw_pcl_without_norm


		else:
			cl, ind = self.raw_pcl_without_norm.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
			if vis:
				self.display_inlier_outlier(self.raw_pcl_without_norm, ind)

			# get the np array version of the points
			position = np.asarray(cl.points)

			if self.connectivity_test:
				# filter the position by connectivity clustering
				clustering = AgglomerativeClustering(n_clusters=2, linkage='single')
				clustering.fit(position)
				# delete un-qualified points
				if (np.sum(clustering.labels_ == 1) < np.sum(clustering.labels_ == 0)):
					position = position[clustering.labels_ == 0]
				else:
					position = position[clustering.labels_ == 1]

			# downsample
			downpcd = o3d.geometry.PointCloud()
			downpcd.points = o3d.utility.Vector3dVector(position)

		downpcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=norm_est_radius, max_nn=50))


		position = np.asarray(downpcd.points)
		# get the np array version of the normal
		# get the normal vector of the downsample point cloud

		if vis:
			o3d.visualization.draw_geometries([downpcd])

		normals_outward = np.asarray(downpcd.normals)

		# this is for the robot frame, the z will shift by 0.2
		if self.isPly:
			position[:,2] -= 0.2

		# combine the position and norm
		self.raw_pcl_with_raw_norm = np.concatenate((position, normals_outward),axis=1)
		return self.raw_pcl_with_raw_norm

	def filter_vector_with_angle_threshold(self, vis=False):
		"""only filter out the points with normals within the max_angle cone of z positive direction
		to ensure the robot arm can reach the point
		Param: max_angle_filtering: int, degree, will delete points that is bigger than this angle
		"""

		for point in self.raw_pcl_with_raw_norm:
			# x,-z, y
			angle = np.arccos(np.dot(point[3:],[0,0,1]))
			if (angle <= self.max_angle_filtering * np.pi / 180):
				self.raw_pcl_with_filtered_norm.append(point)
		self.raw_pcl_with_filtered_norm = np.array(self.raw_pcl_with_filtered_norm)
		print("Remaining points:",self.raw_pcl_with_filtered_norm.shape[0])

		if vis:
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(self.raw_pcl_with_filtered_norm[:,0:3])
			o3d.visualization.draw_geometries([pcd])

		self.sorted_pcl_with_filtered_norm = self.raw_pcl_with_filtered_norm

	def sorted_poinst_with_xy_position(self,vis=False):

		self.sorted_pcl_with_filtered_norm = self.raw_pcl_with_filtered_norm

		if vis:
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(self.sorted_pcl_with_filtered_norm[0:180,0:3])
			o3d.visualization.draw_geometries([pcd])

	def Average2DGrid(self, vis=False):
		pcArray = np.asarray(self.raw_pcl_with_filtered_norm)

		xArray = pcArray[:,0]
		yArray = pcArray[:,1]

		width = np.max(xArray) - np.min(xArray)
		height = np.max(yArray) - np.min(yArray)
		wGrid = width/self.keepRows
		hGrid = height/self.keepCols

		newArray = np.empty((0,6))
		ori_gridLeft = np.min(xArray)
		ori_gridRight = np.min(xArray)+wGrid
		halfXlist = []
		for i in range(0, keepRows):
			xArray = pcArray[:, 0]
			gridLeft = ori_gridLeft + i*wGrid
			gridRight = ori_gridRight + (i+1)*wGrid

			x_half = (gridLeft+gridRight)/2

			halfXlist.append(x_half)

			appendArray = pcArray[(xArray<gridRight) * (xArray>gridLeft)]
			appendArray[:,0] = x_half

			newArray = np.concatenate((newArray, appendArray), axis=0)

		pcArray = newArray
		newArray = np.empty((0, 6))
		ori_gridDown = np.min(yArray)
		ori_gridUp = np.min(yArray) + hGrid
		halfYlist = []
		for i in range(0, keepCols):
			yArray = pcArray[:, 1]
			gridDown = ori_gridDown + i * hGrid
			gridUp = ori_gridUp + (i + 1) * hGrid

			y_half = (gridDown + gridUp) / 2

			halfYlist.append(y_half)


			appendArray = pcArray[(yArray < gridUp) * (yArray > gridDown)]
			appendArray[:, 1] = y_half
			newArray = np.concatenate((newArray, appendArray), axis=0)
		pcArray = newArray

		newArray = np.empty((0, 6))

		for idx_x, x in enumerate(halfXlist):
			points_in_a_row = np.empty((0, 6))
			for y in halfYlist:
				sum_z = 0
				points_num = 0
				sum_norm = np.array([0,0,0])
				for points in pcArray:
					if (points[0] == x) and (points[1] == y):
						z = points[2]
						norm = points[3:]

						sum_z += z
						sum_norm = np.add(sum_norm,norm)
						points_num+=1


						# print ("The {}th point z is {}".format(points_num,z))
				if (points_num == 0):
					continue
				else:
					z = sum_z/points_num
					# print("In total the sum of {} point z is {}, so the new z is {}".format(points_num, sum_z, z))
					norm = sum_norm/points_num
					appendArray = np.array([[x,y,z,norm[0],norm[1],norm[2]]])

					points_in_a_row = np.concatenate((points_in_a_row, appendArray), axis=0)

			if (idx_x%2==0):
				newArray = np.concatenate((newArray, points_in_a_row), axis=0)
			else:
				newArray = np.concatenate((newArray, points_in_a_row[::-1]), axis=0)

		pcArray = newArray
		if vis:
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(pcArray[:,0:3])
			o3d.visualization.draw_geometries([pcd])

		self.raw_pcl_with_filtered_norm = pcArray

	def Downsample2DGrid(self, vis=False):
		pcArray = np.asarray(self.raw_pcl_with_filtered_norm)

		xArray = pcArray[:, 0]
		yArray = pcArray[:, 1]

		width = np.max(xArray) - np.min(xArray)
		height = np.max(yArray) - np.min(yArray)
		wGrid = width / self.keepRows
		hGrid = height / self.keepCols

		ori_gridLeft = np.min(xArray)
		halfXlist = []
		for i in range(0, keepRows):
			gridLeft = ori_gridLeft + i * wGrid
			halfXlist.append(gridLeft)

		ori_gridDown = np.min(yArray)
		halfYlist = []
		for i in range(0, keepCols):
			gridDown = ori_gridDown + (i) * hGrid
			halfYlist.append(gridDown)

		newArray = np.empty((0, 6))

		for idx_x, x in enumerate(halfXlist[:-1]):
			points_in_a_row = np.empty((0, 6))
			for idx_y, y in enumerate(halfYlist[:-1]):
				min_distance = 100000000000
				min_idx = 0
				gridPoints = pcArray[(xArray >= x) * (xArray < halfXlist[idx_x+1]) * (yArray >= y) * (yArray < halfYlist[idx_y+1])]

				if gridPoints.shape[0]>0:
					for (idx, points) in enumerate(gridPoints):
						distance = np.linalg.norm(points[0:2] - np.array([x, y]))

						if distance < min_distance:
							min_distance = distance
							min_idx = idx

					points_in_a_row = np.concatenate((points_in_a_row, gridPoints[min_idx, :].reshape((1, 6))), axis=0)

			if (idx_x%2==0):
				newArray = np.concatenate((newArray, points_in_a_row), axis=0)
			else:
				newArray = np.concatenate((newArray, points_in_a_row[::-1]), axis=0)

		pcArray = newArray

		if vis:
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(pcArray[:, 0:3])
			o3d.visualization.draw_geometries([pcd])

		self.raw_pcl_with_filtered_norm = pcArray

	def savefile(self):
		"""
		Param:
			fileType: string, 'quat','euler'
			'quat': save a 7 by N npy file, the first three colomns are position, last four is quat angle
			'euler': save a 6 by N npy file, the first three colomns are position, last four is quat angle
		"""
		save_quat = np.empty((0,7))
		for i in range(self.sorted_pcl_with_filtered_norm.shape[0]):
		# negative because it's pointing inward to the liver
			norm_x = -self.sorted_pcl_with_filtered_norm[i,3]
			norm_y = -self.sorted_pcl_with_filtered_norm[i,4]
			norm_z = -self.sorted_pcl_with_filtered_norm[i,5]

			x_euler, y_euler, z_euler = self.norm2euler_xyz(norm_x, norm_y, norm_z)
			quat_r = R.from_euler('xyz',[x_euler, y_euler, z_euler],degrees=False)
			row = np.zeros((1,7))
			row[0,0] = self.sorted_pcl_with_filtered_norm[i,0]
			row[0,1] = self.sorted_pcl_with_filtered_norm[i,1]
			row[0,2] = self.sorted_pcl_with_filtered_norm[i,2]
			row[0,3] = quat_r.as_quat()[0]
			row[0,4] = quat_r.as_quat()[1]
			row[0,5] = quat_r.as_quat()[2]
			row[0,6] = quat_r.as_quat()[3]
			save_quat = np.concatenate((save_quat, row), axis=0)

		return save_quat
		
	def removeoutlier(self,vis=False):

		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(self.raw_pcl_with_filtered_norm[:, 0:3])
		pcd.normals = o3d.utility.Vector3dVector(self.raw_pcl_with_filtered_norm[:, 3:6])
		cl, ind = pcd.remove_statistical_outlier(nb_neighbors=100, std_ratio=1)
		if vis:
			self.display_inlier_outlier(pcd, ind)
		position = np.asarray(cl.points)
		normals = np.asarray(cl.normals)

		# combine the position and norm
		self.raw_pcl_with_filtered_norm = np.concatenate((position, normals), axis=1)

	def norm2euler_zyx(self, x, y, z):
		x_euler = 0
		z_euler = 0
		length = np.sqrt(x ** 2 + y ** 2)
		if ((x > 0) and (y > 0) and (z > 0)):
			x_euler = np.arctan(y / x)
			z_euler = -np.arctan(z / length)
		if ((x < 0) and (y < 0) and (z < 0)):
			x_euler = np.pi + np.arctan(-y / -x)
			z_euler = np.arctan(z / length)

		if ((x < 0) and (y > 0) and (z < 0)):
			x_euler = np.pi - np.arctan(y / -x)
			z_euler = np.arctan(z / length)
		if ((x > 0) and (y < 0) and (z > 0)):
			x_euler = -np.arctan(-y / x)
			z_euler = -np.arctan(z / length)

		if ((x > 0) and (y > 0) and (z < 0)):
			x_euler = np.arctan(y / x)
			z_euler = -np.arctan(z / length)
		if ((x < 0) and (y < 0) and (z > 0)):
			x_euler = np.pi + np.arctan(-y / -x)
			z_euler = np.arctan(z / length)

		if ((x < 0) and (y > 0) and (z > 0)):
			x_euler = np.pi / 2 + np.arctan(-x / y)
			z_euler = np.arctan(z / length)
		if ((x > 0) and (y < 0) and (z < 0)):
			x_euler = -np.arctan(-y / x)
			z_euler = np.arctan(-z / length)

		return x_euler,0, z_euler

	def norm2euler_xyz(self, x, y, z):
		"""deprecated"""
		x_euler = 0
		y_euler = 0
		length = np.sqrt(x ** 2 + y ** 2 + z ** 2)
		length_cos = np.sqrt(y ** 2 + z ** 2)
		if ((x > 0) and (y > 0) and (z > 0)):
			x_euler = -np.arctan(y / z)
			y_euler = np.arccos(length_cos / length)
		if ((x < 0) and (y < 0) and (z < 0)):
			x_euler = np.pi + np.arctan(y / -z)
			y_euler = np.arccos(length_cos / length)

		if ((x > 0) and (y < 0) and (z > 0)):
			x_euler = np.arctan(-y / z)
			y_euler = np.arccos(length_cos / length)
		if ((x < 0) and (y > 0) and (z < 0)):
			x_euler = np.pi + np.arctan(y / -z)
			y_euler = np.arccos(length_cos / length)

		if ((x < 0) and (y < 0) and (z > 0)):
			x_euler = np.arctan(-y / z)
			y_euler = -np.arccos(length_cos / length)
		if ((x > 0) and (y > 0) and (z < 0)):
			x_euler = np.pi + np.arctan(y / -z)
			y_euler = -np.arccos(length_cos / length)

		if ((x < 0) and (y > 0) and (z > 0)):
			x_euler = -np.arctan(y / z)
			y_euler = -np.arccos(length_cos / length)
		if ((x > 0) and (y < 0) and (z < 0)):
			x_euler = np.pi - np.arctan(-y / -z)
			y_euler = -np.arccos(length_cos / length)

		return x_euler, y_euler, 0

	def filter(self, raw_nby3_data, isPlyPath):
		"""
		if isPlyPath == False, raw_nby3_data should be in nparray data type
		if isPlyPath == True, raw_nby3_data should be in string data type
		"""
		self.loadData(raw_nby3_data, isPlyPath)
		self.downsample_raw_pcl_get_normal_vector(vis=False)
		self.filter_vector_with_angle_threshold(vis=False)
		if isPlyPath==True:
			self.removeoutlier(vis=False)
			self.Downsample2DGrid(vis=False)
		if isPlyPath==False:
			self.Average2DGrid(vis=False)
		self.sorted_poinst_with_xy_position(vis=True)

		return my_filter.savefile()

if __name__ == "__main__":

	file_path = "D:/CMU2020FALL/HapticSurgery/Medical-DVRK-AR/data/"

	# example of processing the blaser result
	blaser_data_name = "blaser_results_moving.npy"
	raw_data = np.load(file_path+blaser_data_name)
	max_angle = 60  # change  the param within [0,90)]
	keepRows = 36
	keepCols = 36

	my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
	a = my_filter.filter(raw_data, isPlyPath=False)
	np.save(file_path+"palpation_path_36cols_36rows.npy", a)


	#  example of processing the ply file
	CAD_data_name = "stl2.ply"
	max_angle = 60  # change  the param within [0,90)]
	keepRows = 10
	keepCols = 20
	my_filter = filter_pointcloud_for_path_planner(max_angle, keepRows, keepCols, connectivity_test=True)
	a = my_filter.filter(file_path+CAD_data_name, isPlyPath=True)
	print(a.shape)
	np.save(file_path+"scanning_path_200_points.npy", a)

