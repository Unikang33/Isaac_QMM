#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

// 4x4 변환행렬을 입력받아 K1 매니퓰레이터의 IK를 계산하는 프로그램
// Chain: sgr532/base_link → sgr532/link6

std::string PACKAGE_PATH = ament_index_cpp::get_package_share_directory("qmm_control");
std::string robot_urdf = PACKAGE_PATH + "/urdf/k1.urdf";

KDL::Tree robot_tree;
KDL::Chain robot_chain;

// Roll, Pitch, Yaw를 Radians에서 Degrees로 변환
Eigen::Vector3d RadiansToDegrees(const Eigen::Vector3d& radians) {
    return radians * 180.0 / M_PI;
}

// 회전 행렬에서 Roll, Pitch, Yaw 추출 (ZYX Euler angles)
Eigen::Vector3d GetRollPitchYaw(const Eigen::Matrix4d& pose) {
    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    
    double roll, pitch, yaw;
    
    double sin_pitch = -R(0, 2);
    
    if (std::abs(sin_pitch) >= 1.0) {
        pitch = std::copysign(M_PI / 2.0, sin_pitch);
        roll = 0.0;
        yaw = std::atan2(-R(1, 0), R(1, 1));
    } else {
        pitch = std::asin(sin_pitch);
        roll = std::atan2(R(1, 2), R(2, 2));
        yaw = std::atan2(R(0, 1), R(0, 0));
    }
    
    return Eigen::Vector3d(roll, pitch, yaw);
}

// Forward Kinematics 계산
Eigen::Matrix4d CalculateFK(const KDL::JntArray& joint_positions) {
    KDL::ChainFkSolverPos_recursive fk_solver(robot_chain);
    KDL::Frame ee_frame;
    
    fk_solver.JntToCart(joint_positions, ee_frame);
    
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            pose(i, j) = ee_frame.M(i, j);
        }
        pose(i, 3) = ee_frame.p(i);
    }
    
    return pose;
}

// 정규화된 Manipulability 계산
double CalculateManipulability(const KDL::JntArray& joint_positions) {
    KDL::ChainJntToJacSolver jac_solver(robot_chain);
    KDL::Jacobian J(joint_positions.rows());
    jac_solver.JntToJac(joint_positions, J);
    Eigen::MatrixXd J_eigen = Eigen::Map<Eigen::MatrixXd>(J.data.data(), J.rows(), J.columns());
    
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eigen);
    Eigen::VectorXd singular_values = svd.singularValues();
    
    double manipulability = 0.0;
    double sigma_max = singular_values(0);
    
    if (sigma_max > 1e-10) {
        double product = 1.0;
        for (int i = 0; i < singular_values.size(); i++) {
            product *= singular_values(i);
        }
        double sigma_max_n = std::pow(sigma_max, singular_values.size());
        manipulability = product / sigma_max_n;
    }
    
    return manipulability;
}

// IK 계산 함수
bool SolveIK(const Eigen::Matrix4d& target_pose, KDL::JntArray& joint_solution, bool verbose = true) {
    // 변환행렬을 KDL Frame으로 변환
    Eigen::Vector3d translation = target_pose.block<3,1>(0,3);
    Eigen::Quaterniond quat(target_pose.block<3,3>(0,0));
    
    KDL::Frame desired_pose;
    desired_pose.p = KDL::Vector(translation.x(), translation.y(), translation.z());
    desired_pose.M = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
    
    if (verbose) {
        std::cout << "\n========== 입력 목표 자세 ==========" << std::endl;
        std::cout << "Position: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;
        std::cout << "Distance: " << translation.norm() << " m" << std::endl;
        
        Eigen::Vector3d rpy_rad = GetRollPitchYaw(target_pose);
        Eigen::Vector3d rpy_deg = RadiansToDegrees(rpy_rad);
        std::cout << "Orientation (RPY):" << std::endl;
        std::cout << "  Radians: [" << rpy_rad[0] << ", " << rpy_rad[1] << ", " << rpy_rad[2] << "]" << std::endl;
        std::cout << "  Degrees: [" << rpy_deg[0] << ", " << rpy_deg[1] << ", " << rpy_deg[2] << "]" << std::endl;
        std::cout << "\n변환 행렬:" << std::endl;
        std::cout << target_pose << std::endl;
        std::cout << "====================================\n" << std::endl;
    }
    
    // IK Solver 초기화
    double eps = 1e-5;
    int maxiter = 100;
    KDL::ChainIkSolverPos_LMA ik_solver(robot_chain, eps, maxiter);
    
    // 초기 추측값 (home position)
    KDL::JntArray initial_guess(robot_chain.getNrOfJoints());
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        initial_guess(i) = 0.0;
    }
    
    // IK 계산
    joint_solution.resize(robot_chain.getNrOfJoints());
    int ik_result = ik_solver.CartToJnt(initial_guess, desired_pose, joint_solution);
    
    if (ik_result < 0) {
        if (verbose) {
            std::cout << "❌ IK 계산 실패 (에러 코드: " << ik_result << ")" << std::endl;
            std::cout << "가능한 원인:" << std::endl;
            std::cout << "  - 목표 위치가 작업 공간 밖" << std::endl;
            std::cout << "  - Joint limits 초과" << std::endl;
            std::cout << "  - Singular configuration" << std::endl;
        }
        return false;
    }
    
    if (verbose) {
        std::cout << "✅ IK 계산 성공!" << std::endl;
    }
    
    return true;
}

int main() {
    // URDF 로드
    if (!kdl_parser::treeFromFile(robot_urdf, robot_tree)) {
        std::cerr << "❌ URDF 파싱 실패: " << robot_urdf << std::endl;
        return -1;
    }
    
    // KDL Chain 생성
    if (!robot_tree.getChain("sgr532/base_link", "sgr532/link6", robot_chain)) {
        std::cerr << "❌ KDL Chain 생성 실패" << std::endl;
        return -1;
    }
    
    std::cout << "✅ K1 매니퓰레이터 모델 로드 성공" << std::endl;
    std::cout << "   Chain: sgr532/base_link → sgr532/link6" << std::endl;
    std::cout << "   관절 수: " << robot_chain.getNrOfJoints() << std::endl;
    std::cout << std::endl;
    
    // ========================================
    // 목표 변환행렬 설정 (여기를 수정하세요!)
    // ========================================
    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
    
    // 예제 1: 간단한 자세
    target_pose << 1, 0, 0, 0.3,
                    0, 1, 0, 0.0,
                    0, 0, 1, 0.6,
                    0, 0, 0, 1;
    target_pose(2, 3) -= 0.295835;
    
    // 예제 2: 회전이 포함된 자세 (주석 해제하여 사용)
    // target_pose << 0.707107, -0.707107, 0, 0.3,
    //                0.707107,  0.707107, 0, 0.0,
    //                0,         0,        1, 0.4,
    //                0,         0,        0, 1;
    
    // ========================================
    // IK 계산
    // ========================================
    KDL::JntArray joint_solution;
    bool success = SolveIK(target_pose, joint_solution, true);
    
    if (!success) {
        return -1;
    }
    
    // ========================================
    // 결과 출력
    // ========================================
    std::cout << "\n========== IK 계산 결과 ==========" << std::endl;
    std::cout << "\n조인트 각도 (Radians):" << std::endl;
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        std::cout << "  joint" << (i+1) << ": " << joint_solution(i) << " rad" << std::endl;
    }
    
    std::cout << "\n조인트 각도 (Degrees):" << std::endl;
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        double degrees = joint_solution(i) * 180.0 / M_PI;
        std::cout << "  joint" << (i+1) << ": " << degrees << "°" << std::endl;
    }
    
    // ========================================
    // Forward Kinematics로 검증
    // ========================================
    std::cout << "\n========== FK 검증 ==========" << std::endl;
    Eigen::Matrix4d fk_result = CalculateFK(joint_solution);
    
    std::cout << "FK 결과 변환행렬:" << std::endl;
    std::cout << fk_result << std::endl;
    
    // 오차 계산
    Eigen::Matrix4d error = target_pose - fk_result;
    double position_error = error.block<3,1>(0,3).norm();
    double rotation_error = error.block<3,3>(0,0).norm();
    
    std::cout << "\n오차:" << std::endl;
    std::cout << "  위치 오차: " << position_error << " m (" << (position_error * 1000) << " mm)" << std::endl;
    std::cout << "  회전 오차: " << rotation_error << std::endl;
    
    // ========================================
    // Manipulability 계산
    // ========================================
    double manipulability = CalculateManipulability(joint_solution);
    std::cout << "\n========== Manipulability ==========" << std::endl;
    std::cout << "Normalized Manipulability: " << manipulability << " (0~1, 1이 최적)" << std::endl;
    
    if (manipulability > 0.5) {
        std::cout << "상태: ✅ 매우 좋음" << std::endl;
    } else if (manipulability > 0.1) {
        std::cout << "상태: ⚠️  보통" << std::endl;
    } else {
        std::cout << "상태: ❌ 나쁨 (singularity 근처)" << std::endl;
    }
    std::cout << "====================================\n" << std::endl;
    
    // ========================================
    // ROS2 Python 코드 생성
    // ========================================
    std::cout << "========== Python 코드 (irm_controller.py) ==========" << std::endl;
    std::cout << "self.k1_target_positions = {" << std::endl;
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        std::cout << "    'joint" << (i+1) << "': " << joint_solution(i);
        if (i < robot_chain.getNrOfJoints() - 1) {
            std::cout << ",";
        }
        std::cout << "  # " << (joint_solution(i) * 180.0 / M_PI) << "°" << std::endl;
    }
    std::cout << "}" << std::endl;
    std::cout << "====================================================\n" << std::endl;
    
    return 0;
}

