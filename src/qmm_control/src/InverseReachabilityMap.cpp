#include "qmm_control/InverseReachabilityMap.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

bool IRM::IsPointInCuboid(const Point3D& point, const Cuboid& cuboid) {
    return point.x >= cuboid.min_point.x && point.x <= cuboid.max_point.x &&
            point.y >= cuboid.min_point.y && point.y <= cuboid.max_point.y &&
            point.z >= cuboid.min_point.z && point.z <= cuboid.max_point.z;
}

double IRM::GetValue(const Eigen::Vector3d& point, const VoxelMap& voxel_map) {
    int x_idx = static_cast<int>((point.x() - (voxel_map.start_x - interval_ / 2)) / interval_);
    int y_idx = static_cast<int>((point.y() - (voxel_map.start_y - interval_ / 2)) / interval_);
    int z_idx = static_cast<int>((point.z() - (voxel_map.start_z - interval_ / 2)) / interval_);

    if (x_idx < 0 || x_idx >= voxel_map.x_length || y_idx < 0 || y_idx >= voxel_map.y_length || z_idx < 0 || z_idx >= voxel_map.z_length) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    return voxel_map.meas[x_idx][y_idx][z_idx];
}

double IRM::InterpolateVoxelValue(const Eigen::Vector3d& point, const VoxelMap& voxel_map) {
    double x_idx = (point.x() - (voxel_map.start_x - interval_ / 2)) / interval_;
    double y_idx = (point.y() - (voxel_map.start_y - interval_ / 2)) / interval_;
    double z_idx = (point.z() - (voxel_map.start_z - interval_ / 2)) / interval_;

    int x0 = static_cast<int>(x_idx);
    int y0 = static_cast<int>(y_idx);
    int z0 = static_cast<int>(z_idx);

    int x1 = x0 + 1;
    int y1 = y0 + 1;
    int z1 = z0 + 1;

    if (x0 < 0 || x1 >= voxel_map.x_length || y0 < 0 || y1 >= voxel_map.y_length || z0 < 0 || z1 >= voxel_map.z_length) {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double wx = x_idx - x0;
    double wy = y_idx - y0;
    double wz = z_idx - z0;

    // 트릴리니어 보간
    double c000 = voxel_map.meas[x0][y0][z0];
    double c001 = voxel_map.meas[x0][y0][z1];
    double c010 = voxel_map.meas[x0][y1][z0];
    double c011 = voxel_map.meas[x0][y1][z1];
    double c100 = voxel_map.meas[x1][y0][z0];
    double c101 = voxel_map.meas[x1][y0][z1];
    double c110 = voxel_map.meas[x1][y1][z0];
    double c111 = voxel_map.meas[x1][y1][z1];

    double c00 = c000 * (1 - wx) + c100 * wx;
    double c01 = c001 * (1 - wx) + c101 * wx;
    double c10 = c010 * (1 - wx) + c110 * wx;
    double c11 = c011 * (1 - wx) + c111 * wx;

    double c0 = c00 * (1 - wy) + c10 * wy;
    double c1 = c01 * (1 - wy) + c11 * wy;

    return c0 * (1 - wz) + c1 * wz;
}

VoxelMap IRM::CreateVoxelMap(const InverseReachabilityMap& reachMap) {
    // map 수정하면 -reachMap.voxelSize/2 빼기 
    double start_x = reachMap.xMin;
    double start_y = reachMap.yMin;
    double start_z = reachMap.zMin;

    int x_length = (reachMap.xMax - reachMap.xMin)/reachMap.voxelSize + 1;
    int y_length = (reachMap.yMax - reachMap.yMin)/reachMap.voxelSize + 1;
    int z_length = (reachMap.zMax - reachMap.zMin)/reachMap.voxelSize + 1;

    std::vector<std::vector<std::vector<double>>> array(
        x_length, std::vector<std::vector<double>>(y_length, std::vector<double>(z_length, 0))
    );

    for (const auto& data : reachMap.reachMap) {
        int x_idx = static_cast<int>((data.sphereCenter.x() - (start_x - reachMap.voxelSize/2)) / interval_);
        int y_idx = static_cast<int>((data.sphereCenter.y() - (start_y - reachMap.voxelSize/2)) / interval_);
        int z_idx = static_cast<int>((data.sphereCenter.z() - (start_z - reachMap.voxelSize/2)) / interval_);
        
        if (x_idx < x_length && y_idx < y_length && z_idx < z_length && x_idx >= 0 && y_idx >= 0 && z_idx >= 0) {
            array[x_idx][y_idx][z_idx] = data.measure;
            // std::cout << data.measure;
        }
        // else { 
        //     std::cout << "x y z : " << data.sphereCenter.x() << " " << data.sphereCenter.y() << " " << data.sphereCenter.z() << std::endl;
        //     std::cout << "x y z index : " << x_idx << " " << y_idx << " " << z_idx << std::endl;
        // }
    }
    return VoxelMap(start_x, start_y, start_z, array);
}

std::vector<std::vector<Cell>> IRM::Generate2dIRM(const Eigen::Matrix4d& T_world_ee, double z_offset) {
    Eigen::Vector4d ee_origin(0.0, 0.0, 0.0, 1.0);
    Eigen::Vector4d world_origin = T_world_ee * ee_origin;
    
    double center_u = world_origin.x();
    double center_v = world_origin.y();
    
    double half_width = (cols_ / 2.0) * interval_;
    double half_height = (rows_ / 2.0) * interval_;
    
    double min_u = center_u - half_width;
    double max_u = center_u + half_width;
    double min_v = center_v - half_height;
    double max_v = center_v + half_height;
    
    std::vector<std::vector<Cell>> grid;
    grid.reserve(rows_);
    
    for (int i = 0; i < rows_; i++) {
        std::vector<Cell> row;
        row.reserve(cols_);
        
        for (int j = 0; j < cols_; j++) {
            double u = min_u + j * interval_;
            double v = min_v + i * interval_;
            
            row.emplace_back(0, std::make_tuple(i, j), std::make_tuple(u, v));
            
            Eigen::Vector4d world_frame_point(u, v, z_offset, 1.0);
            Eigen::Vector4d ee_frame_point = T_world_ee.inverse() * world_frame_point;

            Point3D check_point(ee_frame_point.x(), ee_frame_point.y(), ee_frame_point.z());
            if (IsPointInCuboid(check_point, cuboid)) {
                // double value = InterpolateVoxelValue(Eigen::Vector3d(ee_frame_point.x(), ee_frame_point.y(), ee_frame_point.z()), voxel_map);
                double value = GetValue(Eigen::Vector3d(ee_frame_point.x(), ee_frame_point.y(), ee_frame_point.z()), voxel_map);
                if (!std::isnan(value)) {
                    row.back().score = value;  // 마지막으로 추가된 Cell의 score 업데이트
                }
            }
        }
        grid.push_back(std::move(row));
    }
    return grid;
}

std::vector<std::vector<Cell>> IRM::Get2dIRM(const Eigen::Matrix4d& T_world_ee, double z_offset) {
    return Generate2dIRM(T_world_ee, z_offset);
}

InverseReachabilityMap IRM::LoadInverseBinaryFile(const std::string& filename) {
    InverseReachabilityMap map;
    std::ifstream inFile(filename, std::ios::binary);

    if (!inFile) {
        throw std::runtime_error("CANT OPEN FILE: " + filename);
    }

    try {
        inFile.read(reinterpret_cast<char*>(&map.voxelSize), sizeof(double));

        inFile.read(reinterpret_cast<char*>(&map.xMin), sizeof(double));
        inFile.read(reinterpret_cast<char*>(&map.xMax), sizeof(double));
        inFile.read(reinterpret_cast<char*>(&map.yMin), sizeof(double));
        inFile.read(reinterpret_cast<char*>(&map.yMax), sizeof(double));
        inFile.read(reinterpret_cast<char*>(&map.zMin), sizeof(double));
        inFile.read(reinterpret_cast<char*>(&map.zMax), sizeof(double));

        uint64_t reachMapSize;
        inFile.read(reinterpret_cast<char*>(&reachMapSize), sizeof(uint64_t));

        if (reachMapSize == 0 || reachMapSize > 400000) {
            throw std::runtime_error("WRONG REACHABILITY MAP SIZE: " + std::to_string(reachMapSize));
        }

        map.reachMap.resize(reachMapSize);

        for (size_t i = 0; i < reachMapSize; ++i) {
            InverseReachabilityData data;

            double temp_sphereCenter[3];
            inFile.read(reinterpret_cast<char*>(temp_sphereCenter), 3 * sizeof(double));
            data.sphereCenter = Eigen::Vector3d(temp_sphereCenter[0], temp_sphereCenter[1], temp_sphereCenter[2]);

            inFile.read(reinterpret_cast<char*>(&data.measure), sizeof(double));

            map.reachMap[i] = data;
        }
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR OCCUR WHILE READING FILE : " << e.what() << std::endl;
        map.reachMap.clear();
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "VOXEL SIZE : " << map.voxelSize << std::endl;
    std::cout << "X Y Z MIN MAX : " << "min(" << map.xMin << ", " << map.yMin << ", " << map.zMin << ") max(" 
              << map.xMax << ", " << map.yMax << ", " << map.zMax << ")" << std::endl;
    std::cout << "X LENGTH : " << (map.xMax - map.xMin)/map.voxelSize + 1 << std::endl;
    std::cout << "Y LENGTH : " << (map.yMax - map.yMin)/map.voxelSize + 1 << std::endl;
    std::cout << "Z LENGTH : " << (map.zMax - map.zMin)/map.voxelSize + 1 << std::endl;
    std::cout << "CELL NUM : " << map.reachMap.size() << std::endl; 
    inFile.close();
    return map;
}

IRM::IRM(int rows, int cols) 
    : min_point(), max_point(), cuboid(), rows_(), cols_()
{
    // rows_ = 40;
    // cols_ = 40;
    rows_ = rows;
    cols_ = cols;

    InverseReachabilityMap irm;
    std::string package_path = ament_index_cpp::get_package_share_directory("qmm_control");
    std::string irm_file = package_path + "/map/inverse_reachability_map_sgr532_05_lma.bin";
    irm = LoadInverseBinaryFile(irm_file);
    min_point = Point3D(irm.xMin, irm.yMin, irm.zMin);
    max_point = Point3D(irm.xMax, irm.yMax, irm.zMax);
    cuboid = Cuboid(min_point, max_point);

    interval_ = irm.voxelSize;
    voxel_map = CreateVoxelMap(irm);
}
// g++ -std=c++17 -o test_save InverseReachabilityMap.cpp test_save.cpp -I /usr/include/eigen3