

#include <string>


#include "open3d/Open3D.h"

using namespace open3d;
const std::string DATA_DIR = "/home/abby/project/Open3D/examples/test_data/";
const std::string OUTPUT_DIR = "/home/abby/Data/open3d/";

int main(int argc, char *argv[]) {

    std::string calib_npz =
            utility::GetDataPathCommon(DATA_DIR + "LiDARICP/ouster_calib.npz");
    const core::Device& dv = core::Device("CPU:0");
    t::pipelines::odometry::LiDARIntrinsic calib(calib_npz, dv);


    t::geometry::LiDARImage src = t::io::CreateImageFromFile(
                utility::GetDataPathCommon(DATA_DIR + "LiDARICP/outdoor/000000.png"))
                ->To(dv);


    core::Tensor xyz_im, mask_im;
    core::Tensor transformation =
            core::Tensor::Eye(4, core::Dtype::Float64, dv);

    auto vis = src.Visualize(calib);
    auto vis_ptr = std::make_shared<open3d::geometry::Image>(
            vis.ColorizeDepth(1000.0, 0.0, 30.0).ToLegacy());
    visualization::DrawGeometries({vis_ptr});

    std::tie(xyz_im, mask_im) =
            src.Unproject(calib, transformation, 0.0, 100.0);

    auto pcd_ptr = std::make_shared<open3d::geometry::PointCloud>(
            t::geometry::PointCloud(xyz_im.IndexGet({mask_im})).ToLegacy());
    visualization::DrawGeometries({pcd_ptr});
    if (!io::WritePointCloud(OUTPUT_DIR +"outdoor_000000.ply", *pcd_ptr,
                            {true, true,false})) {
        utility::LogError("Failed to write to outdoor_000000");
    }


    return 0;
}