#ifndef IRM_H
#define IRM_H

#include <fstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <array>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <limits>
#include <Eigen/Dense>

struct Point3D 
{
    double x, y, z;
    Point3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
};

struct Plane 
{
    double a, b, c, d;  // ax + by + cz + d = 0

    Plane(Eigen::Matrix4d T) {
        Eigen::Vector3d normal = T.block<3,1>(0,2);
        Eigen::Vector3d point = T.block<3,1>(0,3);
        a = normal(0);
        b = normal(1);
        c = normal(2);
        d = -(a * point(0) + b * point(1) + c * point(2));
    }
};

struct Cuboid 
{
    Point3D min_point;
    Point3D max_point;
    
    Cuboid(const Point3D& min = Point3D(), const Point3D& max = Point3D())
        : min_point(min), max_point(max) {}
};

struct VoxelMap
{
    std::vector<std::vector<std::vector<double>>> meas;
    double start_x, start_y, start_z;
    int x_length, y_length, z_length;
    
    VoxelMap(double start_x = 0, double start_y = 0, double start_z = 0, std::vector<std::vector<std::vector<double>>> meas = {})
        : meas(meas), start_x(start_x), start_y(start_y), start_z(start_z)
    {
        if (!meas.empty()) 
        {
            x_length = meas.size();
            y_length = meas[0].size();
            z_length = meas[0][0].size();
        }
    }
};

struct InverseReachabilityData {
    Eigen::Vector3d sphereCenter;
    std::vector<Eigen::VectorXd> validConfigs;
    std::vector<double> manipValues;
    double measure;
};

struct InverseReachabilityMap {
    std::vector<InverseReachabilityData> reachMap;
    double voxelSize;
    double xMin, xMax, yMin, yMax, zMin, zMax;
};

class Cell {
public:
    double score;
    std::tuple<int, int> coordinates;
    std::tuple<double, double> absolute_coordinates;

    Cell(double score, std::tuple<int, int> coordinates, std::tuple<double, double> absolute_coordinates)
        : score(score), coordinates(coordinates), absolute_coordinates(absolute_coordinates) {}
};

class IRM {
private:
    Cuboid cuboid; // --> IRM에 외접하는 직육면체
    Point3D min_point, max_point; // --> IRM에 외접하는 직육면체의 최소점, 최대점 
    InverseReachabilityMap irm;
    VoxelMap voxel_map;
    
    std::array<std::pair<int, int>, 12> edges;
    std::array<Point3D, 8> vertices;
    
    bool IsPointInCuboid(const Point3D& point, const Cuboid& cuboid);
    double InterpolateVoxelValue(const Eigen::Vector3d& point, const VoxelMap& voxel_map);
    double GetValue(const Eigen::Vector3d& point, const VoxelMap& voxel_map);
    std::vector<std::vector<Cell>> Generate2dIRM(const Eigen::Matrix4d& T_world_ee, double z_offset);
    InverseReachabilityMap LoadInverseBinaryFile(const std::string& filename);
    VoxelMap CreateVoxelMap(const InverseReachabilityMap& reachMap);

public:
    int rows_, cols_; // --> mpc에서 사용할 맵의 크기(mpc 코드와 연동 위해 인자로 받음)
    double interval_;
    IRM(int rows, int cols);
    std::vector<std::vector<Cell>> Get2dIRM(const Eigen::Matrix4d& T_world_ee, double z_offset); // <-- 이 함수만 mpc에서 사용하면 됨 
};

#endif // IRM_H