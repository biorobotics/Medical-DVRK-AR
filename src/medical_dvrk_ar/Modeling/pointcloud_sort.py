import numpy as np

def pointcloud_sort(input_path, result_path):
	values = np.load(input_path)
	point_num = values.shape[0]
	print(values.shape)
	dtype = [('x',float),('y',float),('z',float),('qw',float),('qx',float),('qy',float), ('qz',float)]
	pointcloud = values.view(dtype)
	pointcloud = pointcloud.reshape((point_num,))
	pointcloud = np.sort(pointcloud, order=["x"])
	# print(pointcloud["x"])

	# first sort in x, then figure out which points should go in rows together
	rows = None
	row = []
	row_size = 0.015
	reverse = False
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
			if reverse:
				row = row[::-1]

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
	np.save(result_path, pointcloud)

if __name__ == "__main__":
    file_path = "/home/alex/MRSD_sim/src/Medical-DVRK-AR/data/"
    file_name = "60degree_norm.npy"
    sorted_file_name = "sorted_liverGrid_norm.npy"
    pointcloud_sort(file_path+file_name, file_path+sorted_file_name)