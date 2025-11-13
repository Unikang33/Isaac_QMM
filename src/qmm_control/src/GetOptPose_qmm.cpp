#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <string>
#include <utility>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <limits>
#include "qmm_control/InverseReachabilityMap.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <omp.h>

std::string PACKAGE_PATH = ament_index_cpp::get_package_share_directory("qmm_control");
std::string robot_urdf = PACKAGE_PATH + "/urdf/k1.urdf";

// 전역 변수로 KDL tree와 chain 선언
KDL::Tree robot_tree;
KDL::Chain robot_chain;

IRM irm_manager_(40, 40);

double INTERVAL = irm_manager_.interval_;
double RADIUS = irm_manager_.interval_/2.0;

// 전역 변수로 KDL solver들도 선언
KDL::ChainJntToJacSolver* global_jac_solver = nullptr;
KDL::ChainIkSolverPos_LMA* global_ik_solver = nullptr;

// IK 솔버 생성 함수
void initializeSolvers() {
    if (global_jac_solver == nullptr) {
        global_jac_solver = new KDL::ChainJntToJacSolver(robot_chain);
    }
    if (global_ik_solver == nullptr) {
        double eps = 1e-5;
        int maxiter = 100;
        global_ik_solver = new KDL::ChainIkSolverPos_LMA(robot_chain, eps, maxiter);
    }
}

// 메모리 해제 함수
void cleanupSolvers() {
    if (global_jac_solver) {
        delete global_jac_solver;
        global_jac_solver = nullptr;
    }
    if (global_ik_solver) {
        delete global_ik_solver;
        global_ik_solver = nullptr;
    }
}

void Get2dIRMS(std::vector<std::vector<std::vector<Cell>>>& irms, const Eigen::Matrix4d& ee_pose, const std::vector<double>& z_offsets) {
    // for (const auto& pose : ee_trajectory) {
    //     Eigen::Matrix4d T = pose;
    //     irms.push_back(irm_manager_.Get2dIRM(T, z_offset));
    // }
    for (const auto& z_offset : z_offsets) {
        Eigen::Matrix4d T = ee_pose;
        irms.push_back(irm_manager_.Get2dIRM(T, z_offset));
    }

}

std::vector<std::vector<Eigen::Matrix4d>> GetRandomPoses() {
    std::vector<std::vector<Eigen::Matrix4d>> poses_set;
    ////////////////////////////////////////
    // 저장되는 데이터 구조 확인한 후에 작성하기
    // 저장된 task pose 로드하는 부분
    ////////////////////////////////////////

    return poses_set;
}

Eigen::Matrix4d GetPoses() {
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose << 1, 0, 0, 0.0,
            0, 1, 0, 0.0,
            0, 0, 1, 0.4,
            0, 0, 0, 1;

    return pose;
}

// roll과 pitch 기준으로 자세 샘플링 (조합)
std::vector<Eigen::Matrix4d> SamplePoses(const std::vector<std::tuple<int, int, int>>& top_indices, 
                                          const std::vector<std::vector<std::vector<Cell>>>& irms,
                                          const std::vector<double>& z_offsets) {
    // roll과 pitch 샘플링 범위
    double roll_min = -0.2;   // rad
    double roll_max = 0.2;    // rad
    double pitch_min = -0.2;  // rad
    double pitch_max = 0.2;   // rad
    int roll_samples = 10;      // roll 샘플 개수
    int pitch_samples = 10;     // pitch 샘플 개수
    
    std::vector<Eigen::Matrix4d> sampled_poses;
    
    for (auto const& index : top_indices) {
        int x_idx = std::get<0>(index);
        int y_idx = std::get<1>(index);
        int z_idx = std::get<2>(index);
        
        // irms에서 직접 좌표 가져오기
        double x_center = std::get<0>(irms[z_idx][y_idx][x_idx].absolute_coordinates);
        double y_center = std::get<1>(irms[z_idx][y_idx][x_idx].absolute_coordinates);
        double z_center = z_offsets[z_idx];
        
        // 기본 방향 설정 (z축이 위를 향하는 방향)
        Eigen::Matrix3d base_rotation = Eigen::Matrix3d::Identity();
        
        Eigen::Vector3d position(x_center, y_center, z_center);
        
        // Roll과 Pitch의 모든 조합 샘플링
        for (int i = 0; i < roll_samples; ++i) {
            double roll = roll_min + (roll_max - roll_min) * i / (roll_samples - 1);
            
            for (int j = 0; j < pitch_samples; ++j) {
                double pitch = pitch_min + (pitch_max - pitch_min) * j / (pitch_samples - 1);
                
                // Roll-Pitch-Yaw 순서로 회전 행렬 생성
                Eigen::Matrix3d rotation = 
                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *      // Yaw = 0
                    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *    // Pitch 변화
                    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *     // Roll 변화
                    base_rotation;
                
                Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
                pose.block<3, 3>(0, 0) = rotation;
                pose.block<3, 1>(0, 3) = position;
                
                sampled_poses.push_back(pose);
            }
        }
    }
    
    return sampled_poses;
}

Eigen::Matrix4d GetOptimalPose(std::vector<Eigen::Matrix4d> base_poses, std::vector<Eigen::Matrix4d> ee_poses) {
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 성능 측정을 위한 카운터
    int ik_total_count = 0;
    int ik_failure_count = 0;
    int skipped_calculations = 0;

    struct PoseResult {
        double manipulability;
        Eigen::Matrix4d pose;
        int failures;
        int total;
        bool valid;
    };

    std::vector<PoseResult> results(base_poses.size());
    
    // 로봇 팔 길이 파라미터 (예상치)
    double max_reach = 1.0;  // 판다 로봇 최대 도달 거리 (미터)
    double min_reach = 0.1;  // 최소 작업 거리
    
    // 베이스 포즈와 엔드 이펙터 포즈 개수에 따라 적응형 정밀도 조정
    // 하지만 정확도는 유지
    double eps = 1e-5;
    int maxiter = 100;
    
    if (base_poses.size() * ee_poses.size() > 500) {
        // 대량 계산 시 약간 완화된 설정 (결과에 큰 영향 없음)
        eps = 2e-5;
        maxiter = 80;
    }
    
    // 빠른 필터링을 위한 유망한 베이스 포즈 표시
    std::vector<bool> promising_base_poses(base_poses.size(), true);
    
    // 최적화된 병렬 처리 설정
    #pragma omp parallel
    {
        // 스레드별 IK 솔버 생성 (스레드 간 경쟁 방지)
        KDL::ChainJntToJacSolver thread_jac_solver(robot_chain);
        KDL::ChainIkSolverPos_LMA thread_ik_solver(robot_chain, eps, maxiter);
        
        // 첫 번째 패스: 물리적으로 불가능한 베이스 포즈 빠르게 필터링
        #pragma omp for schedule(dynamic)
        for (size_t i = 0; i < base_poses.size(); i++) {
            const Eigen::Matrix4d& base_pose = base_poses[i];
            
            // 첫 번째 ee_pose만으로 빠른 테스트
            if (!ee_poses.empty()) {
                Eigen::Matrix4d base_to_ee_pose = base_pose.inverse() * ee_poses[0];
                Eigen::Vector3d translation = base_to_ee_pose.block<3,1>(0,3);
                
                // 물리적 제약 확인
                double distance = translation.norm();
                if (distance > max_reach || distance < min_reach) {
                    promising_base_poses[i] = false;
                    
                    #pragma omp atomic
                    skipped_calculations++;
                    
                    continue;
                }
            }
        }
        
        // 두 번째 패스: 유망한 베이스 포즈에 대해 모든 ee_poses 계산
        #pragma omp for schedule(dynamic)
        for (size_t i = 0; i < base_poses.size(); i++) {
            if (!promising_base_poses[i]) {
                continue;  // 필터링된 베이스 포즈는 건너뜀
            }
            
            const Eigen::Matrix4d& base_pose = base_poses[i];
            PoseResult& result = results[i];
            result.manipulability = 0.0;
            result.pose = base_pose;
            result.failures = 0;
            result.total = 0;
            result.valid = true;
            
            // 초기 추측값을 저장할 행렬 (캐싱)
            KDL::JntArray prev_joint_positions(robot_chain.getNrOfJoints());
            bool has_previous_solution = false;
            
            // 모든 ee_poses 처리 (샘플링 없음)
            for (const auto& ee_pose : ee_poses) {
                KDL::JntArray joint_positions(robot_chain.getNrOfJoints());
                KDL::Frame desired_pose;
                
                // 베이스에서 엔드 이펙터로의 변환
                Eigen::Matrix4d base_to_ee_pose = base_pose.inverse() * ee_pose;
                
                // 물리적 제약 확인
                Eigen::Vector3d translation = base_to_ee_pose.block<3,1>(0,3);
                double distance = translation.norm();
                if (distance > max_reach || distance < min_reach) {
                    result.total++;  // 총 시도 횟수에는 포함
                    result.failures++;  // 실패로 간주
                    continue;  // 이 엔드 이펙터 포즈는 건너뜀
                }
                
                Eigen::Quaterniond quat(base_to_ee_pose.block<3,3>(0,0));
                desired_pose.p = KDL::Vector(translation.x(), translation.y(), translation.z());
                desired_pose.M = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());

                result.total++;
                
                // 이전 솔루션을 초기값으로 사용 (수렴 속도 향상)
                int success;
                if (has_previous_solution) {
                    success = thread_ik_solver.CartToJnt(prev_joint_positions, desired_pose, joint_positions);
                } else {
                    // 첫 시도는 중립 자세에서 시작
                    KDL::JntArray initial_guess(robot_chain.getNrOfJoints());
                    for (unsigned int j = 0; j < robot_chain.getNrOfJoints(); j++) {
                        initial_guess(j) = 0.0;
                    }
                    success = thread_ik_solver.CartToJnt(initial_guess, desired_pose, joint_positions);
                }
                
                if (success >= 0) {
                    // 성공한 솔루션을 다음 초기값으로 저장
                    prev_joint_positions = joint_positions;
                    has_previous_solution = true;
                    
                    // 정규화된 조작성 계산
                    KDL::Jacobian J(joint_positions.rows());
                    thread_jac_solver.JntToJac(joint_positions, J);
                    Eigen::MatrixXd J_eigen = Eigen::Map<Eigen::MatrixXd>(J.data.data(), J.rows(), J.columns());
                    
                    // SVD를 통한 정규화된 manipulability 계산
                    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eigen);
                    Eigen::VectorXd singular_values = svd.singularValues();
                    
                    double sigma_max = singular_values(0);
                    if (sigma_max > 1e-10) {
                        double product = 1.0;
                        for (int i = 0; i < singular_values.size(); i++) {
                            product *= singular_values(i);
                        }
                        double sigma_max_n = std::pow(sigma_max, singular_values.size());
                        result.manipulability += product / sigma_max_n;
                    }
                } else {
                    result.failures++;
                }
            }
        }
    }
    
    // 결과 수집 및 최적의 포즈 찾기
    double max_manipulability = -100;
    Eigen::Matrix4d optimal_pose = Eigen::Matrix4d::Identity();
    int valid_results = 0;
    
    for (size_t i = 0; i < results.size(); i++) {
        const auto& result = results[i];
        
        // 원래 알고리즘과 동일하게 최대 조작성을 갖는 포즈 선택
        if (promising_base_poses[i] && result.manipulability > 0) {
            valid_results++;
            if (result.manipulability > max_manipulability) {
                max_manipulability = result.manipulability;
                optimal_pose = result.pose;
            }
            
            // 통계 수집
            ik_total_count += result.total;
            ik_failure_count += result.failures;
        }
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration = end_time - start_time;
    
    // 통계 및 결과 출력
    double success_rate = ik_total_count > 0 ? 100.0 * (ik_total_count - ik_failure_count) / ik_total_count : 0.0;
    std::cout << "IK 총 실행 횟수: " << ik_total_count << std::endl;
    std::cout << "IK 실패 횟수: " << ik_failure_count << std::endl;
    std::cout << "IK 성공률: " << success_rate << "%" << std::endl;
    std::cout << "필터링으로 건너뛴 포즈 수: " << skipped_calculations << "/" << base_poses.size() << std::endl;
    std::cout << "유효한 결과 수: " << valid_results << "/" << base_poses.size() << std::endl;
    std::cout << "GetOptimalPose 실행 시간: " << duration.count() << "초" << std::endl;
    
    return optimal_pose;
}

std::vector<std::tuple<int, int, int>> ExtractTopIndex(const std::vector<std::vector<std::vector<Cell>>>& irms, int top_count) {
    std::vector<std::tuple<int, int, int>> top_indices;
    
    struct CellScore {
        double score;
        int z_idx;
        int row_idx;
        int col_idx;
        
        bool operator<(const CellScore& other) const {
            return score > other.score;
        }
    };
    
    std::vector<CellScore> all_values;
    
    // 모든 z 레이어, row, col을 순회하며 점수 수집
    for (size_t z = 0; z < irms.size(); ++z) {
        for (size_t row = 0; row < irms[z].size(); ++row) {
            for (size_t col = 0; col < irms[z][row].size(); ++col) {
                const Cell& cell = irms[z][row][col];
                if (cell.score > 0) {
                    CellScore cs = {cell.score, static_cast<int>(z), static_cast<int>(row), static_cast<int>(col)};
                    all_values.push_back(cs);
                }
            }
        }
    }
    
    if (all_values.empty()) {
        std::cout << "No valid scores found in the IRMs." << std::endl;
        return top_indices;
    }
    
    std::sort(all_values.begin(), all_values.end());
    
    if (static_cast<size_t>(top_count) > all_values.size()) {
        top_count = static_cast<int>(all_values.size());
        std::cout << "Requested more items than available. Will return all " << top_count << " positive scores." << std::endl;
    }
    
    for (int i = 0; i < top_count; ++i) {
        top_indices.emplace_back(all_values[i].col_idx, all_values[i].row_idx, all_values[i].z_idx);
        std::cout << "Top " << (i+1) << ": Score = " << all_values[i].score 
                  << " at index (x=" << all_values[i].col_idx 
                  << ", y=" << all_values[i].row_idx 
                  << ", z=" << all_values[i].z_idx << ")" << std::endl;
    }
    
    std::cout << "Extracted top " << top_count << " indices out of " << all_values.size() << " positive values." << std::endl;
    
    return top_indices;
}

double GetScore(const Eigen::Matrix4d& opt_base_pose, const std::vector<Eigen::Matrix4d>& ee_poses) {
    double max_reach = 1.0;
    double min_reach = 0.1;
    double total_score = 0.0;
    
    for (const auto& ee_pose : ee_poses) {
        Eigen::Matrix4d base_to_ee_pose = opt_base_pose.inverse() * ee_pose;
        Eigen::Vector3d translation = base_to_ee_pose.block<3,1>(0,3);
        double distance = translation.norm();
        if (distance > max_reach || distance < min_reach) {
            return 0.0;
        }
        total_score += 1.0;  // 도달 가능한 포즈 개수
    }
    return total_score;
}

void SaveIntegratedIRM(const std::vector<std::vector<double>>& integrated_irm, const std::string& filename) {
    std::ofstream outfile(filename, std::ios::binary);
    
    if (!outfile) {
        std::cerr << "CANT OPEN FILE: " << filename << std::endl;
        return;
    }
    
    size_t rows = integrated_irm.size();
    size_t cols = rows > 0 ? integrated_irm[0].size() : 0;
    
    outfile.write(reinterpret_cast<const char*>(&rows), sizeof(size_t));
    outfile.write(reinterpret_cast<const char*>(&cols), sizeof(size_t));
    
    for (const auto& row : integrated_irm) {
        outfile.write(reinterpret_cast<const char*>(row.data()), cols * sizeof(double));
    }
    
    outfile.close();
    std::cout << "INTEGRATED IRM SAVED SUCCESSFULLY: " << filename << std::endl;
}

void SaveEEposes(const std::vector<Eigen::Matrix4d>& poses, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    
    if (!outFile) {
        std::cerr << "CANT OPEN FILE: " << filename << std::endl;
        return;
    }
    
    try {
        uint64_t pose_size = poses.size();
        outFile.write(reinterpret_cast<const char*>(&pose_size), sizeof(uint64_t));
        
        for (const auto& pose : poses) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    double element = pose(i, j);
                    outFile.write(reinterpret_cast<const char*>(&element), sizeof(double));
                }
            }
        }
        
        std::cout << "POSE DATA SAVED TO: " << filename << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR SAVING FILE: " << e.what() << std::endl;
    }
    
    outFile.close();
}

void SaveTopIndices3D(const std::vector<std::tuple<int, int, int>>& top_indices, const std::string& filename) {
    std::ofstream outfile(filename, std::ios::binary);
    
    if (!outfile) {
        std::cerr << "CANT OPEN FILE: " << filename << std::endl;
        return;
    }
    
    size_t count = top_indices.size();
    outfile.write(reinterpret_cast<const char*>(&count), sizeof(size_t));
    
    for (const auto& index_tuple : top_indices) {
        int x_idx = std::get<0>(index_tuple);
        int y_idx = std::get<1>(index_tuple);
        int z_idx = std::get<2>(index_tuple);
        outfile.write(reinterpret_cast<const char*>(&x_idx), sizeof(int));
        outfile.write(reinterpret_cast<const char*>(&y_idx), sizeof(int));
        outfile.write(reinterpret_cast<const char*>(&z_idx), sizeof(int));
    }
    
    outfile.close();
    std::cout << "TOP 3D INDICES SAVED SUCCESSFULLY: " << filename << std::endl;
}

void SaveBaseOptimalPose(const Eigen::Matrix4d& optimal_pose, const std::string& filename) {
    std::ofstream outFile(filename, std::ios::binary);
    
    if (!outFile) {
        std::cerr << "CANT OPEN FILE: " << filename << std::endl;
        return;
    }
    
    try {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                double element = optimal_pose(i, j);
                outFile.write(reinterpret_cast<const char*>(&element), sizeof(double));
            }
        }
        
        std::cout << "OPTIMAL POSE SAVED TO: " << filename << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR SAVING FILE: " << e.what() << std::endl;
    }
    
    outFile.close();
}

// 회전 행렬에서 Roll, Pitch, Yaw 추출 (ZYX Euler angles)
// R = Rz(yaw) * Ry(pitch) * Rx(roll)
Eigen::Vector3d GetRollPitchYaw(const Eigen::Matrix4d& pose) {
    Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
    
    double roll, pitch, yaw;
    
    // pitch 계산 (Y축 회전)
    // sin(pitch) = -R(0,2)
    double sin_pitch = -R(0, 2);
    
    // Gimbal lock 체크
    if (std::abs(sin_pitch) >= 1.0) {
        // Gimbal lock 발생 (pitch = ±90°)
        pitch = std::copysign(M_PI / 2.0, sin_pitch);
        
        // 이 경우 roll과 yaw 중 하나를 0으로 설정
        roll = 0.0;
        yaw = std::atan2(-R(1, 0), R(1, 1));
    } else {
        pitch = std::asin(sin_pitch);
        
        // roll 계산 (X축 회전)
        // tan(roll) = R(1,2) / R(2,2)
        roll = std::atan2(R(1, 2), R(2, 2));
        
        // yaw 계산 (Z축 회전)
        // tan(yaw) = R(0,1) / R(0,0)
        yaw = std::atan2(R(0, 1), R(0, 0));
    }
    
    return Eigen::Vector3d(roll, pitch, yaw);
}

// Roll, Pitch, Yaw를 도(degree)로 변환
Eigen::Vector3d RadiansToDegrees(const Eigen::Vector3d& radians) {
    return radians * 180.0 / M_PI;
}

// Forward Kinematics 계산 함수
// 입력: 조인트 각도
// 출력: end-effector pose (4x4 변환행렬)
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

// Manipulability 계산 함수
// 입력: 4x4 변환행렬 (sgr532/base_link 기준 sgr532/link6 pose)
// 출력: manipulability 값 (음수는 IK 실패를 의미)
// KDL chain: sgr532/base_link → sgr532/link6
double CalculateManipulability(const Eigen::Matrix4d& ee_pose, bool verbose = false) {
    // ee_pose를 KDL Frame으로 변환
    Eigen::Vector3d translation = ee_pose.block<3,1>(0,3);
    Eigen::Quaterniond quat(ee_pose.block<3,3>(0,0));
    
    if (verbose) {
        std::cout << "[DEBUG] CalculateManipulability 입력:" << std::endl;
        std::cout << "  Position: [" << translation.x() << ", " << translation.y() << ", " << translation.z() << "]" << std::endl;
        std::cout << "  Distance: " << translation.norm() << " m" << std::endl;
    }
    
    KDL::Frame desired_pose;
    desired_pose.p = KDL::Vector(translation.x(), translation.y(), translation.z());
    desired_pose.M = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
    
    // IK 계산
    KDL::JntArray joint_positions(robot_chain.getNrOfJoints());
    KDL::JntArray initial_guess(robot_chain.getNrOfJoints());
    
    // 초기 추측값 (home position)
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        initial_guess(i) = 0.0;
    }
    
    int ik_result = global_ik_solver->CartToJnt(initial_guess, desired_pose, joint_positions);
    
    // IK 실패 시 -1 반환
    if (ik_result < 0) {
        if (verbose) {
            std::cout << "[DEBUG] IK 실패! 에러 코드: " << ik_result << std::endl;
            std::cout << "  가능한 원인:" << std::endl;
            std::cout << "  - 목표가 도달 불가능한 위치" << std::endl;
            std::cout << "  - joint limits 초과" << std::endl;
            std::cout << "  - 수치적 불안정성" << std::endl;
        }
        return -1.0;
    }
    
    if (verbose) {
        std::cout << "[DEBUG] IK 성공! 조인트 값:" << std::endl;
        for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
            std::cout << "  joint" << (i+1) << ": " << joint_positions(i) 
                      << " rad (" << (joint_positions(i) * 180.0 / M_PI) << "°)" << std::endl;
        }
    }
    
    // Jacobian 계산
    KDL::Jacobian J(joint_positions.rows());
    global_jac_solver->JntToJac(joint_positions, J);
    Eigen::MatrixXd J_eigen = Eigen::Map<Eigen::MatrixXd>(J.data.data(), J.rows(), J.columns());
    
    // Singular Value Decomposition
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_eigen);
    Eigen::VectorXd singular_values = svd.singularValues();
    
    if (verbose) {
        std::cout << "[DEBUG] Jacobian 분석:" << std::endl;
        std::cout << "  크기: " << J_eigen.rows() << "x" << J_eigen.cols() << std::endl;
        
        std::cout << "  Singular Values:" << std::endl;
        for (int i = 0; i < singular_values.size(); i++) {
            std::cout << "    σ" << (i+1) << ": " << singular_values(i) << std::endl;
        }
        std::cout << "  Condition Number: " << singular_values(0) / singular_values(singular_values.size()-1) << std::endl;
    }
    
    // 정규화된 Manipulability 계산
    // w_normalized = (σ1 × σ2 × ... × σn) / (σ_max^n)
    // 이 값은 0~1 사이이며, 1에 가까울수록 좋은 자세
    double manipulability = 0.0;
    double sigma_max = singular_values(0);
    
    if (sigma_max > 1e-10) {  // 수치 안정성을 위한 체크
        // 모든 singular value의 곱 계산
        double product = 1.0;
        for (int i = 0; i < singular_values.size(); i++) {
            product *= singular_values(i);
        }
        
        // 정규화
        double sigma_max_n = std::pow(sigma_max, singular_values.size());
        manipulability = product / sigma_max_n;
    }
    
    if (verbose) {
        std::cout << "  σ_max: " << sigma_max << std::endl;
        std::cout << "  Normalized Manipulability: " << manipulability << " (0~1, 1이 최적)" << std::endl;
    }
    
    return manipulability;
}

int main() {
    if (!kdl_parser::treeFromFile(robot_urdf, robot_tree)) {
        std::cout << "메인: 로봇 URDF 파싱 실패" << std::endl;
        return -1;
    }

    if (!robot_tree.getChain("sgr532/base_link", "sgr532/link6", robot_chain)){
        std::cout << "메인: 체인 가져오기 실패" << std::endl;
        return -1;
    }
    std::cout << "로봇 모델 로드 성공 (sgr532/base_link → sgr532/link6): 관절 수 = " << robot_chain.getNrOfJoints() << std::endl;
    
    // 전역 솔버 초기화
    initializeSolvers();   
    
    Eigen::Matrix4d ee_pose = GetPoses();
    // std::vector<std::vector<Eigen::Matrix4d>> ee_poses_set = GetRandomPoses();
    auto start_time_total = std::chrono::high_resolution_clock::now();
    auto start_time_2dIRM = std::chrono::high_resolution_clock::now();
    
    std::vector<std::vector<std::vector<Cell>>> irms; 
    
    std::vector<double> z_offsets;
    double standard_z = 0.295835;
    double max_offset = 0.15; 
    double min_offset = 0.05;
    double voxel_size = irm_manager_.interval_;

    for (double z_offset = standard_z; z_offset < standard_z + max_offset; z_offset += voxel_size) {
        z_offsets.push_back(z_offset);
    }
    for (double z_offset = standard_z - voxel_size; z_offset > standard_z - min_offset; z_offset -= voxel_size) {
        z_offsets.push_back(z_offset);
    }
    
    Get2dIRMS(irms, ee_pose, z_offsets);
    
    auto end_time_2dIRM = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_2dIRM = end_time_2dIRM - start_time_2dIRM;
    std::cout << "duration 2dIRM: " << duration_2dIRM.count() << " (s)" << std::endl;
    
    // 상위 10개 인덱스 추출
    auto start_time_opt = std::chrono::high_resolution_clock::now();
    std::vector<std::tuple<int, int, int>> top_indices = ExtractTopIndex(irms, 10);
    std::cout << "top_indices : " << top_indices.size() << std::endl;

    // 상위 10개 위치에 대해 자세 샘플링 (irms와 z_offsets 전달)
    std::vector<Eigen::Matrix4d> sampled_base_poses = SamplePoses(top_indices, irms, z_offsets);
    std::cout << "sampled_base_poses : " << sampled_base_poses.size() << std::endl;
    
    // 샘플링된 자세 중 최적자세 찾기
    std::vector<Eigen::Matrix4d> ee_poses_vec = {ee_pose};  // 단일 pose를 벡터로 변환
    Eigen::Matrix4d optimal_pose = GetOptimalPose(sampled_base_poses, ee_poses_vec);
    
    // Optimal pose 출력
    std::cout << "\n========== Optimal Base Pose ==========" << std::endl;
    std::cout << "Position (x, y, z): " 
              << optimal_pose(0, 3) << ", "
              << optimal_pose(1, 3) << ", "
              << optimal_pose(2, 3) << std::endl;
    
    // Roll, Pitch, Yaw 계산 및 출력
    Eigen::Vector3d rpy_rad = GetRollPitchYaw(optimal_pose);
    Eigen::Vector3d rpy_deg = RadiansToDegrees(rpy_rad);
    
    std::cout << "Orientation (Roll, Pitch, Yaw):" << std::endl;
    std::cout << "  Radians: " << rpy_rad[0] << ", " << rpy_rad[1] << ", " << rpy_rad[2] << std::endl;
    std::cout << "  Degrees: " << rpy_deg[0] << ", " << rpy_deg[1] << ", " << rpy_deg[2] << std::endl;
    
    std::cout << "\nTransformation Matrix:" << std::endl;
    std::cout << optimal_pose << std::endl;
    std::cout << "======================================\n" << std::endl;

    std::cout << "Target world to EE Pose: \n" << ee_pose << std::endl;
    std::cout << "======================================\n" << std::endl;

    Eigen::Matrix4d base_to_ee_pose = optimal_pose.inverse() * ee_pose;
    std::cout << "Target base to EE Pose: \n" << base_to_ee_pose << std::endl;
    std::cout << "======================================\n" << std::endl;
    
    // K1 매니퓰레이터 IK 계산 및 조인트 각도 출력
    // KDL chain: sgr532/base_link → sgr532/link6
    std::cout << "\n========== K1 Inverse Kinematics ==========" << std::endl;
    std::cout << "Chain: sgr532/base_link → sgr532/link6" << std::endl;
    
    // base_to_ee_pose를 KDL Frame으로 변환
    Eigen::Vector3d translation = base_to_ee_pose.block<3,1>(0,3);
    Eigen::Quaterniond quat(base_to_ee_pose.block<3,3>(0,0));
    
    KDL::Frame desired_pose;
    desired_pose.p = KDL::Vector(translation.x(), translation.y(), translation.z());
    desired_pose.M = KDL::Rotation::Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
    
    // IK 계산
    KDL::JntArray joint_positions(robot_chain.getNrOfJoints());
    KDL::JntArray initial_guess(robot_chain.getNrOfJoints());
    
    // 초기 추측값 (home position)
    for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
        initial_guess(i) = 0.0;
    }
    
    int ik_result = global_ik_solver->CartToJnt(initial_guess, desired_pose, joint_positions);
    
    if (ik_result >= 0) {
        std::cout << "✅ IK 계산 성공!" << std::endl;
        std::cout << "\nK1 조인트 각도 (Radians):" << std::endl;
        for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
            std::cout << "  joint" << (i+1) << ": " << joint_positions(i) << " rad" << std::endl;
        }
        
        std::cout << "\nK1 조인트 각도 (Degrees):" << std::endl;
        for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
            double degrees = joint_positions(i) * 180.0 / M_PI;
            std::cout << "  joint" << (i+1) << ": " << degrees << "°" << std::endl;
        }
        
        // Manipulability 계산
        std::cout << "\n=== Manipulability 분석 ===" << std::endl;
        double manipulability = CalculateManipulability(base_to_ee_pose, false);
        std::cout << "Normalized Manipulability: " << manipulability << std::endl;
        std::cout << "===========================" << std::endl;
        
        std::cout << "\nirm_controller.py에 입력할 값:" << std::endl;
        std::cout << "self.k1_target_positions = {" << std::endl;
        for (unsigned int i = 0; i < robot_chain.getNrOfJoints(); i++) {
            std::cout << "    'joint" << (i+1) << "': " << joint_positions(i);
            if (i < robot_chain.getNrOfJoints() - 1) {
                std::cout << ",";
            }
            std::cout << "  # " << (joint_positions(i) * 180.0 / M_PI) << "°" << std::endl;
        }
        std::cout << "}" << std::endl;
    } else {
        std::cout << "❌ IK 계산 실패 (에러 코드: " << ik_result << ")" << std::endl;
        std::cout << "목표 위치가 도달 불가능한 위치일 수 있습니다." << std::endl;
    }
    std::cout << "==========================================\n" << std::endl;
    
    auto end_time_opt = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_opt = end_time_opt - start_time_opt;
    std::cout << "duration opt: " << duration_opt.count() << " (s)" << std::endl;
    
    auto end_time_total = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> duration_total = end_time_total - start_time_total;
    std::cout << "duration total: " << duration_total.count() << " (s)" << std::endl;

    cleanupSolvers();
    ////////////////////////////////////////////////////////////////////////////////////////
    
    return 0;
}