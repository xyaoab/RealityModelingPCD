import os, sys
import time

import numpy as np
import open3d as o3d
import open3d.core as o3c
from tqdm import tqdm
import glob

from config import ConfigParser


def load_ply_as_pcd_withnormals(fname):
    ply_pcd = o3d.io.read_point_cloud(fname)
    points = np.asarray(ply_pcd.points, dtype=np.float32)
    normals = np.asarray(ply_pcd.normals, dtype=np.float32)
    pcd_obj = o3d.t.geometry.PointCloud(
            o3c.Tensor(np.zeros((points.shape[0], 3), dtype=np.float32)))
    
    pcd_obj.point["normals"] = normals
    pcd_obj.point["positions"] = points
    return pcd_obj


def tensor_count_nonzero(tensor, state):

    tmp = (tensor.to(o3c.Dtype.Float32).sum())
    # tmp /= tensor.shape[0]
    print("[{}] Out of {} tensor {:4f}".format(state, tensor.shape[0], tmp.cpu().numpy()))


def integrate(range_names, poses, config):

    trunc_multiplier = config.sdf_trunc_multiplier
    trunc = config.voxel_size * trunc_multiplier

    vbg = o3d.t.geometry.VoxelBlockGrid(
        ('tsdf', 'weight', 'normals'), (o3c.Dtype.Float32, o3c.Dtype.Float32, o3c.Dtype.Float32), ((1), (1), (3)),
        config.voxel_size, config.block_resolution, config.block_count,
        o3c.Device(config.device))

    # Get active frustum block coordinates from input
    start = time.time()
    threshold = 0.8
    for i, fname in tqdm(enumerate(range_names)):

        pcd_in_map = load_ply_as_pcd_withnormals(fname).to(o3c.Device(config.device))

        sensor_xyz = poses[i]
        sensor_xyz = o3d.core.Tensor(sensor_xyz) \
                    .to(o3c.Device(config.device), o3d.core.Dtype.Float32)

        # Some legacy-tensor transformations
        print(sensor_xyz[0], sensor_xyz[1], sensor_xyz[2])
        frustum_block_coords, block_pcd_coords, block_pcd_normals = vbg.compute_unique_block_coordinates(
            pcd_in_map, sensor_xyz,
            config.step_size,
            config.tangential_step_size, trunc_multiplier)


    
        # Activate them in the underlying hash map (may have been inserted)
        vbg.hashmap().activate(frustum_block_coords)

        # Find buf indices in the underlying engine
        buf_indices, masks = vbg.hashmap().find(frustum_block_coords)

        voxel_coords, voxel_indices = vbg.voxel_coordinates_and_flattened_indices(
            buf_indices)
        

        resolution3 = config.block_resolution * config.block_resolution * config.block_resolution
        pcd_coords_voxel = np.repeat(block_pcd_coords.cpu().numpy(), resolution3, axis=0)
        pcd_normals_voxel =  np.repeat(block_pcd_normals.cpu().numpy(), resolution3, axis=0)

        pcd_coords_voxel = o3d.core.Tensor(pcd_coords_voxel) \
                            .to(o3c.Device(config.device), o3d.core.Dtype.Float32)
        pcd_normals_voxel = o3d.core.Tensor(pcd_normals_voxel ) \
                            .to(o3c.Device(config.device), o3d.core.Dtype.Float32)
        assert len(pcd_coords_voxel) == len(voxel_coords)    

        # Now project them to the depth and find association
        # (3, N) -> (2, N)

        xyz_voxels = voxel_coords.to(o3d.core.Dtype.Float32)

        xyz_readings = pcd_coords_voxel
        xyz_normals = pcd_normals_voxel

        xyz_delta = xyz_readings - xyz_voxels
        xyz_delta_norm = xyz_delta.mul(xyz_delta).sum(1).sqrt()


        angle_delta_normal = xyz_normals.mul(xyz_delta).sum(1)     

        # flip normal direction
        xyz_dot_sign = o3c.Tensor.ones(xyz_delta.shape[0], dtype = o3c.Dtype.Float32).to(o3c.Device(config.device))

        xyz_dot_sign[angle_delta_normal>0] *= -1

        sdf = xyz_dot_sign * xyz_delta_norm # xyz_dot / xyz_readings_norm 
        mask_inlier =  o3c.Tensor.ones(sdf.shape, dtype = o3c.Dtype.Bool).to(o3c.Device(config.device))

        sdf[sdf <= -trunc] = -trunc
        sdf[sdf >= trunc] = trunc
        sdf = (sdf / trunc).reshape((-1, 1))

        weight = vbg.attribute('weight').reshape((-1, 1))
        tsdf = vbg.attribute('tsdf').reshape((-1, 1))


        valid_voxel_indices = voxel_indices[mask_inlier]
        w = weight[valid_voxel_indices]

        wp = w + 1

        if i == 0:
            weight = o3d.core.Tensor.ones(weight.shape, dtype=o3c.Dtype.Float32).to(o3c.Device(config.device))
            weight *= -1

        
        update_delta = (tsdf[valid_voxel_indices] - sdf[mask_inlier]).abs()
        weight_mask = (weight[valid_voxel_indices]<0)
    
        # for reset case 1
        bool_mask_base = (update_delta >= threshold)
        # tensor_count_nonzero(bool_mask_base, 'base')

        bool_mask_reset = bool_mask_base.logical_and(sdf[mask_inlier].abs() <= tsdf[valid_voxel_indices].abs())
        bool_mask_reset = weight_mask.logical_or(bool_mask_reset)
        bool_mask_reset = bool_mask_reset.to(o3d.core.Dtype.Bool).reshape((-1,))
        tensor_count_nonzero(bool_mask_reset, 'reset')

        # for skipping case 2
        bool_mask_skip = bool_mask_base.logical_and(sdf[mask_inlier].abs() > tsdf[valid_voxel_indices].abs())
        bool_mask_skip = bool_mask_skip.to(o3d.core.Dtype.Bool).reshape((-1,))
        tensor_count_nonzero(bool_mask_skip , 'skip')

        # for filtering case 3
        bool_mask_filter = ((bool_mask_reset.logical_or(bool_mask_skip)) == False).to(o3d.core.Dtype.Bool).reshape((-1,))
        tensor_count_nonzero(bool_mask_filter , 'filter ')

        tsdf[valid_voxel_indices[bool_mask_reset] ] = sdf[mask_inlier][bool_mask_reset]
        weight[valid_voxel_indices[bool_mask_reset]] = 1 # -=1 #/=2 # = 1

        tsdf[valid_voxel_indices[bool_mask_filter]] \
            = (tsdf[valid_voxel_indices[bool_mask_filter]]  * w[bool_mask_filter]  +
               sdf[mask_inlier][bool_mask_filter]) / wp[bool_mask_filter] 
        weight[valid_voxel_indices[bool_mask_filter]] = wp[bool_mask_filter]

        if i % 50 == 49:
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
            vbg.prune(threshold=1, percentage=0.99)
            o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Info)

            o3d.core.cuda.release_cache()

    end = time.time()
    print('Average {} FPS'.format(len(range_names) / (end - start)))
    return vbg


if __name__ == '__main__':
    parser = ConfigParser()
    parser.add(
        '--config',
        is_config_file=True,
        help='YAML config file path. Please refer to default_config.yml as a '
        'reference. It overrides the default config file, but will be '
        'overridden by other command line inputs.')
    parser.add_argument('--device',
                        type=str,
                        default='CPU:0', #'CUDA:0',
                        choices=['CUDA:0', 'CPU:0'])
    parser.add_argument('--visualize', action='store_true')
    parser.add_argument('--raymarching', action='store_true')
    config = parser.get_config()
    print("Running on: ", config.device)

    device = o3d.core.Device(config.device)

    range_names = sorted(
        glob.glob('{}/VLP16/*.ply'.format(config.path_dataset)))
    poses = []
    for range_fname in range_names:
        seq_id = range_fname.split("/")[-1][:-4]
        pose_fname = '{}/pose/{}.txt'.format(config.path_dataset,
                                                    seq_id)
        pose = np.loadtxt(pose_fname)
        poses.append(pose)



    path_pcd = os.path.join(config.path_dataset, 'pcd.ply')
    mesh_name = 'mesh_' \
            + 'voxel' + str(config.voxel_size) \
            + 'block'+ str(config.block_resolution) + '.obj'
    path_mesh = os.path.join(config.path_dataset, mesh_name)
    path_vbg = os.path.join(config.path_dataset, 'vbg.npz')


    vbg = integrate(range_names[:], poses, config)
    vbg.save(path_vbg)
    # print("mesh", vbg.extract_triangle_mesh(0).shape)
    mesh = vbg.extract_triangle_mesh(0).to_legacy()
    o3d.visualization.draw([mesh])
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(path_mesh, mesh)
