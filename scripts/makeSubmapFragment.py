import argparse
import open3d as o3d
import open3d.core as o3c
import numpy as np
import os
from tqdm import tqdm

'''
python makeSubmapFragment.py --root_dir /media/abby/USB/Data/CMU_lidar_indoor --index 6
'''


## flip normal to direction facing (sensor pose 0,0,0)
def compute_normal(pcd_np, _max_nn=60, _radius=0.1):
	pcd_obj = o3d.t.geometry.PointCloud(
            o3c.Tensor(np.zeros((pcd_np.shape[0], 3), dtype=np.float32)))
	pcd_obj.point["positions"] = pcd_np
	pcd_obj.estimate_normals( max_nn=_max_nn, radius=_radius)
	flip_mask = (pcd_obj.point["normals"].mul(pcd_obj.point["positions"] ).sum(1) <0 )
	pcd_obj.point["normals"][flip_mask] *= -1
	o3d.visualization.draw_geometries([pcd_obj.to_legacy()],point_show_normal=True)
	return pcd_obj.point["normals"].numpy()


## all in map frame
if __name__=="__main__":
	parser = argparse.ArgumentParser(description='Make submap into fragments based on sensor position')
	parser.add_argument('--root_dir', type=str, default='/home/abby/Data/CMU_lidar_indoor',
                    help='Base dir for dataset')
	parser.add_argument('--index', type=int, default=1,
				help='Submap index')
	args = parser.parse_args()

	rootDir = args.root_dir
	pcd_index = args.index
	inDataDir = os.path.join(rootDir, 'submaps')

	inPCDName = os.path.join(inDataDir, '{}.txt'.format(pcd_index))

	outDataDir = os.path.join(rootDir, 'submaps_out', '{}'.format(pcd_index))
	outPCDDir = os.path.join(outDataDir, 'VLP16')
	outPoseDir = os.path.join(outDataDir, 'pose')

	os.makedirs(outDataDir, exist_ok=True)
	os.makedirs(outPCDDir, exist_ok=True)
	os.makedirs(outPoseDir, exist_ok=True)
	pcd_in = np.loadtxt(inPCDName)
	num_fields = pcd_in.shape[1]

	n = len(pcd_in)
	print("num of points", n)
	assert num_fields==7


	xyz_in_map = pcd_in[:, :3]
	normal_in_map = compute_normal(xyz_in_map)

	sensor_xyz_in_map = pcd_in[:, 3:6]
	accuracy_in = pcd_in[:, -1]
	last_sensor_xyz_in_map = sensor_xyz_in_map[0]
	start_index_list = [0]

	for i in range(1, n):
		current_sensor_xyz_in_map = sensor_xyz_in_map[i]
		if not np.array_equal(last_sensor_xyz_in_map, current_sensor_xyz_in_map):
			start_index_list.append(i)
			last_sensor_xyz_in_map = current_sensor_xyz_in_map

	start_index_list.append(None)
	print("start_index", len(start_index_list))
	num_unique = len(start_index_list) - 1


	for i in tqdm(range(num_unique)):

		start_index = start_index_list[i]
		end_index = start_index_list[i+1]

		outPoseName = os.path.join(outPoseDir, str(i)+'.txt')
		outPCDName = os.path.join(outPCDDir, str(i)+'.ply')

		current_sensor_xyz_in_map = sensor_xyz_in_map[start_index:end_index] 
		current_xyz_in_map = xyz_in_map[start_index:end_index] # (n,3)
		current_normal_in_map = normal_in_map[start_index:end_index]
		# #
		# 
		# current_normal_in_map = compute_normal(current_xyz_in_map)
		# print("\n Normal computed \n")
		current_accuracy_in = accuracy_in[start_index:end_index] 

		current_sensor_pose = current_sensor_xyz_in_map[0].reshape([3,1])
		np.savetxt(outPoseName, current_sensor_pose)

		f = open(outPCDName, "w")
		f.write("ply\n")
		f.write("format ascii 1.0\n")
		f.write("element vertex {}\n".format(len(current_xyz_in_map)))
		f.write("property float x\n")
		f.write("property float y\n")
		f.write("property float z\n")

		f.write("property float nx\n")
		f.write("property float ny\n")
		f.write("property float nz\n")	

		f.write("property float intensity\n")
		f.write("end_header\n")
		f.close()
		with open(outPCDName, "ab") as f:

			np.savetxt(f, np.hstack([current_xyz_in_map, current_normal_in_map, current_accuracy_in.reshape([-1,1])]))

