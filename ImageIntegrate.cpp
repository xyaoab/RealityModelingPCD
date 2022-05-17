
#include "open3d/Open3D.h"

using namespace open3d;

static core::Tensor GetIntrinsicTensor() {
    camera::PinholeCameraIntrinsic intrinsic = camera::PinholeCameraIntrinsic(
            camera::PinholeCameraIntrinsicParameters::PrimeSenseDefault);
    auto focal_length = intrinsic.GetFocalLength();
    auto principal_point = intrinsic.GetPrincipalPoint();
    return core::Tensor::Init<double>(
            {{focal_length.first, 0, principal_point.first},
             {0, focal_length.second, principal_point.second},
             {0, 0, 1}});
}

static std::vector<core::Tensor> GetExtrinsicTensors() {
    data::SampleRedwoodRGBDImages redwood_data;

    // Extrinsics
    auto trajectory = io::CreatePinholeCameraTrajectoryFromFile(
            redwood_data.GetOdometryLogPath());

    std::vector<core::Tensor> extrinsics;
    for (size_t i = 0; i < trajectory->parameters_.size(); ++i) {
        Eigen::Matrix4d extrinsic = trajectory->parameters_[i].extrinsic_;
        core::Tensor extrinsic_t =
                core::eigen_converter::EigenMatrixToTensor(extrinsic);
        extrinsics.emplace_back(extrinsic_t);
    }

    return extrinsics;
}
static std::vector<core::HashBackendType> EnumerateBackends(
        const core::Device &device, bool include_slab = true) {
    std::vector<core::HashBackendType> backends;
    if (device.GetType() == core::Device::DeviceType::CUDA) {
        if (include_slab) {
            backends.push_back(core::HashBackendType::Slab);
        }
        backends.push_back(core::HashBackendType::StdGPU);
    } else {
        backends.push_back(core::HashBackendType::TBB);
    }
    return backends;
}

static std::shared_ptr<t::geometry::VoxelBlockGrid> Integrate(
        const core::HashBackendType &backend,
        const core::Dtype &dtype,
        const core::Device &device,
        const int resolution,
        std::shared_ptr<t::geometry::VoxelBlockGrid> vbg_ptr = nullptr) {
    core::Tensor intrinsic = GetIntrinsicTensor();
    std::vector<core::Tensor> extrinsics = GetExtrinsicTensors();
    const float depth_scale = 1000.0;
    const float depth_max = 3.0;

    if (!vbg_ptr) {
        vbg_ptr = std::make_shared<t::geometry::VoxelBlockGrid>(
                std::vector<std::string>{"tsdf", "weight", "color"},
                std::vector<core::Dtype>{core::Float32, dtype, dtype},
                std::vector<core::SizeVector>{{1}, {1}, {3}}, 3.0 / 512,
                resolution, 10000, device, backend);
    }

    data::SampleRedwoodRGBDImages redwood_data;
    for (size_t i = 0; i < extrinsics.size(); ++i) {
        t::geometry::Image depth =
                t::io::CreateImageFromFile(redwood_data.GetDepthPaths()[i])
                        ->To(device);
        t::geometry::Image color =
                t::io::CreateImageFromFile(redwood_data.GetColorPaths()[i])
                        ->To(device);

        core::Tensor frustum_block_coords = vbg_ptr->GetUniqueBlockCoordinates(
                depth, intrinsic, extrinsics[i], depth_scale, depth_max,
                /*trunc_multiplier=*/4.0);
        vbg_ptr->Integrate(frustum_block_coords, depth, intrinsic,
                           extrinsics[i], depth_scale, depth_max,
                           /*trunc multiplier*/ resolution * 0.5);
    }

    return vbg_ptr;
}


int main(int argc, char *argv[]) {
    const core::Device& device = core::Device("CPU:0");
    std::vector<core::HashBackendType> backends = EnumerateBackends(device);
    for (auto backend : backends) {
        auto vbg_ptr = Integrate(backend, core::UInt16, device, 8);
        auto pcd = vbg_ptr->ExtractPointCloud();
        auto pcd_legacy =
                std::make_shared<open3d::geometry::PointCloud>(pcd.ToLegacy());
        // open3d::io::WritePointCloud("pcd_" + device.ToString() + ".ply",
        //                             *pcd_legacy);
        visualization::DrawGeometries({pcd_legacy});
        auto mesh = vbg_ptr->ExtractTriangleMesh(3.0f); 
        core::Tensor vertex_colors =
            core::Tensor::Ones({2, 3}, core::Float32, device) * 2;  
        mesh.SetVertexColors(vertex_colors);
        auto mesh_legacy =
                std::make_shared<geometry::TriangleMesh>(mesh.ToLegacy());
     
        visualization::DrawGeometries({mesh_legacy});
    }
    return 0;    
}