#include <string>


#include "open3d/Open3D.h"

using namespace open3d;

const std::string OPEN3D_DATA_DIR = "/home/abby/project/Open3D/examples/test_data/";

const std::string CAMPUS_DATA_DIR = "/home/abby/Data/nsh/";
const std::string PCD_FILE = "truncated_pc_sensorpose.ply";

int main(int argc, char *argv[]) {
    std::string calib_npz =
            utility::GetDataPathCommon(OPEN3D_DATA_DIR + "LiDARICP/ouster_calib.npz");
    const core::Device& dv = core::Device("CPU:0");
    t::pipelines::odometry::LiDARIntrinsic calib(calib_npz, dv);

    auto src_cloud_ptr = std::make_shared<geometry::PointCloud>();
    if (io::ReadPointCloud(CAMPUS_DATA_DIR + PCD_FILE, *src_cloud_ptr)) {
        utility::LogInfo("Successfully read {}", PCD_FILE);
        visualization::DrawGeometries({src_cloud_ptr});
    } else {
        utility::LogWarning("Failed to read {}", PCD_FILE);
        return 1;
    }
    auto t_src_cloud_ptr = std::make_shared<t::geometry::PointCloud>();
    *t_src_cloud_ptr = t::geometry::PointCloud::FromLegacy(*src_cloud_ptr);
    core::Tensor xyz = t_src_cloud_ptr->GetPointPositions();
    core::Tensor u, v, r, mask;
    std::tie(u, v, r, mask) = t::geometry::LiDARImage::Project(xyz, calib);

    core::Tensor simulated_r = core::Tensor::Zeros(
            core::SizeVector{calib.height_ / calib.down_factor_,
                             calib.width_ / calib.down_factor_},
            core::Dtype::Float32, xyz.GetDevice());
    auto u_mask = u.IndexGet({mask});
    auto v_mask = v.IndexGet({mask});
    auto r_mask = r.IndexGet({mask});
    simulated_r.IndexSet({v_mask, u_mask}, r_mask);

    t::geometry::LiDARImage src = t::geometry::LiDARImage(
            t::geometry::Image(simulated_r).To(core::UInt16, false, 1000, 0));
    auto vis = src.Visualize(calib);
    auto vis_ptr = std::make_shared<open3d::geometry::Image>(
                vis.ColorizeDepth(1000.0, 0.0, 30.0).ToLegacy());
    visualization::DrawGeometries({vis_ptr});
    return 0;
}