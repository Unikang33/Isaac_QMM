#include "qmm_control/InverseReachabilityMap.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== IRM 테스트 프로그램 ===" << std::endl;
    
    try {
        // IRM 객체 생성 (40x40 그리드)
        std::cout << "\n1. IRM 객체 생성 중..." << std::endl;
        IRM irm(40, 40);
        std::cout << "   - Rows: " << irm.rows_ << std::endl;
        std::cout << "   - Cols: " << irm.cols_ << std::endl;
        std::cout << "   - Interval: " << irm.interval_ << std::endl;
        
        // 테스트용 변환 행렬 생성 (단위 행렬)
        std::cout << "\n2. 변환 행렬 생성..." << std::endl;
        Eigen::Matrix4d T_world_ee = Eigen::Matrix4d::Identity();
        T_world_ee(0, 3) = 0.36;  // x = 0.5
        T_world_ee(1, 3) = 0.0;  // y = 0.3
        T_world_ee(2, 3) = 0.3;  // z = 0.4
        
        std::cout << "   변환 행렬:" << std::endl;
        std::cout << T_world_ee << std::endl;
        
        // 2D IRM 생성
        std::cout << "\n3. 2D IRM 생성 중..." << std::endl;
        double z_offset = 0.0;  // ee의 z 위치와 같게 설정
        std::cout << "   - z_offset: " << z_offset << std::endl;
        
        // 샘플 포인트로 변환 테스트
        std::cout << "\n※ 좌표 변환 확인:" << std::endl;
        std::cout << "   ee 원점 (0,0,0) → world: (0.36, 0.0, 0.3)" << std::endl;
        std::cout << "   world z=0 평면 → ee frame z=-0.3 (IRM 범위 밖!)" << std::endl;
        std::cout << "   world z=0.3 평면 → ee frame z=0.0 (IRM 범위 내부)" << std::endl;
        
        Eigen::Vector4d sample_world_0(0.36, 0.0, 0.0, 1.0);
        Eigen::Vector4d sample_ee_0 = T_world_ee.inverse() * sample_world_0;
        std::cout << "\n   검증: world (0.36, 0.0, 0.0) -> ee frame: (" 
                  << sample_ee_0.x() << ", " << sample_ee_0.y() << ", " << sample_ee_0.z() << ")" << std::endl;
        
        Eigen::Vector4d sample_world_z(0.36, 0.0, z_offset, 1.0);
        Eigen::Vector4d sample_ee_z = T_world_ee.inverse() * sample_world_z;
        std::cout << "   검증: world (0.36, 0.0, " << z_offset << ") -> ee frame: (" 
                  << sample_ee_z.x() << ", " << sample_ee_z.y() << ", " << sample_ee_z.z() << ")" << std::endl;
        
        auto grid = irm.Get2dIRM(T_world_ee, z_offset);
        
        std::cout << "   - 그리드 크기: " << grid.size() << " x " << grid[0].size() << std::endl;
        
        // 통계 정보 출력
        std::cout << "\n4. 그리드 통계 정보:" << std::endl;
        double max_score = 0.0;
        double min_score = std::numeric_limits<double>::max();
        int non_zero_count = 0;
        
        for (const auto& row : grid) {
            for (const auto& cell : row) {
                if (cell.score > 0.0) {
                    non_zero_count++;
                    max_score = std::max(max_score, cell.score);
                    min_score = std::min(min_score, cell.score);
                }
            }
        }
        
        std::cout << "   - 전체 셀 수: " << grid.size() * grid[0].size() << std::endl;
        std::cout << "   - Non-zero 셀 수: " << non_zero_count << std::endl;
        std::cout << "   - 최대 스코어: " << max_score << std::endl;
        std::cout << "   - 최소 스코어 (non-zero): " << min_score << std::endl;
        
        // 샘플 데이터 출력 (중앙 부근)
        std::cout << "\n5. 샘플 데이터 (중앙 5x5 그리드):" << std::endl;
        int center_row = grid.size() / 2;
        int center_col = grid[0].size() / 2;
        
        std::cout << std::fixed << std::setprecision(3);
        for (int i = center_row - 2; i <= center_row + 2; i++) {
            std::cout << "   ";
            for (int j = center_col - 2; j <= center_col + 2; j++) {
                std::cout << std::setw(8) << grid[i][j].score << " ";
            }
            std::cout << std::endl;
        }
        
        std::cout << "\n=== 테스트 완료 ===" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "에러 발생: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

