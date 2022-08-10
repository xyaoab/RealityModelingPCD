import numpy as np
import open3d as o3d
import open3d.core as o3c


def load_ply_as_pcd_withnormals(fname):
    '''
    Load pointcloud in the format of .ply into field
    Data fields: pcd_x pcd_y pcd_z normal_x normal_y normal_z accruacy
    '''
    ply_pcd = o3d.t.io.read_point_cloud(fname)
    # points = np.asarray(ply_pcd.points, dtype=np.float32)
    # normals = np.asarray(ply_pcd.normals, dtype=np.float32)
    # pcd_obj = o3d.t.geometry.PointCloud(
    #         o3c.Tensor(np.zeros((points.shape[0], 3), dtype=np.float32)))
    
    # pcd_obj.point["normals"] = normals
    # pcd_obj.point["positions"] = points
    return ply_pcd


def tensor_count_nonzero(tensor, state):

    tmp = (tensor.to(o3c.Dtype.Float32).sum())
    # tmp /= tensor.shape[0]
    print("[{}] Out of {} tensor {:4f}".format(state, tensor.shape[0], tmp.cpu().numpy()))
