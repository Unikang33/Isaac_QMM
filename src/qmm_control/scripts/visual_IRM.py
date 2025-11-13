#!/usr/bin/env python3
import os
import struct
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import sys

class ReachabilityMap:
    """Reachability Map 데이터를 저장하는 클래스"""
    def __init__(self):
        self.voxel_size = 0.0
        self.x_range = []
        self.y_range = []
        self.z_range = []
        self.reach_map = []

class ReachabilityData:
    """각 도달성 데이터 포인트를 저장하는 클래스"""
    def __init__(self):
        self.sphere_center = np.zeros(3)
        self.valid_configs = []
        self.manip_values = []
        self.measure = 0.0

def load_from_binary_file(filename):
    """C++ 바이너리 파일을 Python에서 로드 (추가 디버깅 정보 포함)"""
    reachability_map = ReachabilityMap()
    
    # 파일 존재 확인
    if not os.path.exists(filename):
        print(f"오류: 파일을 찾을 수 없습니다 - {filename}")
        return reachability_map

    print(f"파일 로드 중: {filename}")
    print(f"파일 크기: {os.path.getsize(filename) / (1024*1024):.2f} MB")

    try:
        with open(filename, "rb") as file:
            # voxelSize 불러오기
            reachability_map.voxel_size = struct.unpack("d", file.read(8))[0]
            print(f"복셀 크기: {reachability_map.voxel_size}")
            
            # x, y, z 범위 불러오기
            x_min, x_max = struct.unpack("2d", file.read(16))
            y_min, y_max = struct.unpack("2d", file.read(16))
            z_min, z_max = struct.unpack("2d", file.read(16))
            
            print(f"X 범위: {x_min} ~ {x_max}")
            print(f"Y 범위: {y_min} ~ {y_max}")
            print(f"Z 범위: {z_min} ~ {z_max}")
            
            reachability_map.x_range = np.arange(x_min, x_max + reachability_map.voxel_size, reachability_map.voxel_size)
            reachability_map.y_range = np.arange(y_min, y_max + reachability_map.voxel_size, reachability_map.voxel_size)
            reachability_map.z_range = np.arange(z_min, z_max + reachability_map.voxel_size, reachability_map.voxel_size)
            
            # reachMap 불러오기
            reach_map_size = struct.unpack("Q", file.read(8))[0]
            print(f"도달성 맵 크기: {reach_map_size} 항목")
            
                
            reachability_map.reach_map = []
            valid_points = 0  # 유효한 데이터 포인트 수

            try:
                for i in range(reach_map_size):
                    if i % 1000 == 0:  # 진행 상태 표시
                        sys.stdout.write(f"\r항목 로드 중... {i}/{reach_map_size} ({i/reach_map_size*100:.1f}%)")
                        sys.stdout.flush()
                    
                    data = ReachabilityData()

                    # sphereCenter 불러오기
                    data.sphere_center = np.array(struct.unpack("3d", file.read(24)))

                    # validConfigs 처리
                    config_size = struct.unpack("Q", file.read(8))[0]

                    data.valid_configs = []
                    for _ in range(config_size):
                        vec_size = struct.unpack("Q", file.read(8))[0]

                        config = struct.unpack(f"{vec_size}d", file.read(vec_size * 8))
                        data.valid_configs.append(list(config))

                    # manipValues 처리
                    manip_size = struct.unpack("Q", file.read(8))[0]
                    data.manip_values = list(struct.unpack(f"{manip_size}d", file.read(manip_size * 8)))

                    # measure 읽기
                    data.measure = struct.unpack("d", file.read(8))[0]
                    
                    # measure > 0인 경우만 카운트
                    if data.measure > 0:
                        valid_points += 1

                    reachability_map.reach_map.append(data)
                
                print(f"\n로드 완료: 총 {len(reachability_map.reach_map)} 항목, 유효 포인트: {valid_points}")
                
                # 몇 개의 데이터 샘플 출력
                if len(reachability_map.reach_map) > 0:
                    print("\n데이터 샘플:")
                    for i in range(min(3, len(reachability_map.reach_map))):
                        data = reachability_map.reach_map[i]
                        print(f"  항목 {i}: 중심={data.sphere_center}, measure={data.measure}, "
                              f"configs={len(data.valid_configs)}, manips={len(data.manip_values)}")
            
            except struct.error as e:
                print(f"\n이진 데이터 파싱 중 오류 발생: {e}")
                print(f"현재 위치: 아이템 {i}/{reach_map_size}")
                
    except Exception as e:
        print(f"파일 읽기 중 오류 발생: {e}")
        import traceback
        traceback.print_exc()
        reachability_map.reach_map = []

    # 데이터 유효성 검사
    if len(reachability_map.reach_map) == 0:
        print("경고: 로드된 데이터가 없습니다!")
    
    return reachability_map


class ReachabilityVisualizer:
    """Matplotlib를 이용하여 3D 시각화"""
    def __init__(self, reachability_map):
        self.reachability_map = reachability_map

        # measure 값 범위 계산
        measures = [data.measure for data in reachability_map.reach_map if data.measure > 0]
        
        if not measures:
            print("오류: 유효한 측정값이 없습니다!")
            self.min_measure = 0
            self.max_measure = 1
        else:
            self.min_measure = min(measures)
            self.max_measure = max(measures)
            print(f"Measure 범위: {self.min_measure} ~ {self.max_measure}")

    def normalize(self, values):
        """값을 0~1 사이로 정규화"""
        if self.max_measure == self.min_measure:
            return [1.0 for _ in values]  # 모든 값이 같으면 1로 설정
        return [(v - self.min_measure) / (self.max_measure - self.min_measure) for v in values]

    def visualize(self, filter_z=None, filter_x=None, filter_y=None, z_layer=None, z_tolerance=0.01):
        """3D 시각화 - 추가 필터링 옵션
        
        Parameters:
        ----------
        filter_z : float, optional
            이 값보다 큰 z 좌표를 가진 데이터만 표시
        filter_x : float, optional
            이 값보다 큰 x 좌표를 가진 데이터만 표시
        filter_y : float, optional
            이 값보다 큰 y 좌표를 가진 데이터만 표시
        z_layer : float, optional
            이 z 값을 기준으로 특정 층만 표시 (z_tolerance와 함께 사용)
        z_tolerance : float, optional
            z_layer 주변의 허용 오차 범위 (기본값: 0.01)
        """
        fig = plt.figure(figsize=(12, 10))
        ax = fig.add_subplot(111, projection='3d')

        # 필터링된 데이터 준비
        filtered_data = [data for data in self.reachability_map.reach_map 
                        if data.measure > 0]  # measure가 0보다 큰 데이터만 선택
        
        # Z 좌표 필터링 (옵션)
        if filter_z is not None:
            filtered_data = [data for data in filtered_data 
                            if data.sphere_center[2] > filter_z]
        
        # 특정 Z 층 필터링 (옵션)
        if z_layer is not None:
            filtered_data = [data for data in filtered_data 
                            if abs(data.sphere_center[2] - z_layer) <= z_tolerance]
        
        # X 좌표 필터링 (옵션)
        if filter_x is not None:
            filtered_data = [data for data in filtered_data 
                            if data.sphere_center[0] > filter_x]
        # Y 좌표 필터링 (옵션)
        if filter_y is not None:
            filtered_data = [data for data in filtered_data 
                            if data.sphere_center[1] > filter_y]

        print(f"시각화 데이터 포인트 수: {len(filtered_data)}")
        
        if not filtered_data:
            print("오류: 표시할 데이터가 없습니다!")
            plt.title("No valid data to display")
            plt.show()
            return

        # 데이터 추출
        x, y, z, measures = [], [], [], []
        for data in filtered_data:
            x.append(data.sphere_center[0])
            y.append(data.sphere_center[1])
            z.append(data.sphere_center[2])
            measures.append(data.measure)

        # measure 값 통계
        print(f"measure 통계: 최소={min(measures)}, 최대={max(measures)}, 평균={np.mean(measures)}")

        # 색상 정규화
        colors = self.normalize(measures)
        
        # 점 크기 및 투명도 조정
        point_size = 15  # 더 큰 점 크기
        alpha = 0.2  # 불투명하게
        
        scatter = ax.scatter(x, y, z, c=colors, cmap="viridis", s=point_size, alpha=alpha)

        # 축 레이블 및 제목
        ax.set_xlabel("X-axis")
        ax.set_ylabel("Y-axis")
        ax.set_zlabel("Z-axis")
        
        # 제목에 현재 표시중인 층 정보 추가
        title = "Reachability Map Visualization"
        if z_layer is not None:
            title += f" (z = {z_layer} ± {z_tolerance})"
        ax.set_title(title)
        
        # 축 한계 설정
        ax.set_xlim([min(x)-0.1, max(x)+0.1])
        ax.set_ylim([min(y)-0.1, max(y)+0.1])
        ax.set_zlim([min(z)-0.1, max(z)+0.1])
        
        # 그리드 표시
        ax.grid(True)
        
        # 컬러바 추가
        cbar = fig.colorbar(scatter, ax=ax, label="Reachability Measure")
        cbar.set_label("Reachability Measure", fontsize=12)
        
        # 뷰 각도 설정 (더 나은 3D 시각화를 위해)
        ax.view_init(elev=30, azim=45)
        
        plt.tight_layout()
        plt.show()

def main():
    # 스크립트 파일의 디렉토리 경로 가져오기
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # 프로젝트 루트 디렉토리 (scripts의 부모 디렉토리)
    project_root = os.path.dirname(script_dir)
    
    # 맵 파일 경로 설정
    map_dir = os.path.join(project_root, "map")
    filename = os.path.join(map_dir, "inverse_reachability_map_sgr532_05_lma.bin")
    
    # 파일 존재 확인
    if not os.path.exists(filename):
        print(f"오류: 맵 파일을 찾을 수 없습니다! {filename}")
        # 대체 파일 시도 - k1 파일
        filename = os.path.join(map_dir, "k1_inverted_reachability_map.bin")
        if not os.path.exists(filename):
            print(f"오류: 원본 맵 파일도 찾을 수 없습니다! {filename}")
            # map 디렉토리의 모든 .bin 파일 출력
            if os.path.exists(map_dir):
                bin_files = [f for f in os.listdir(map_dir) if f.endswith('.bin')]
                if bin_files:
                    print(f"\n사용 가능한 맵 파일:")
                    for i, bin_file in enumerate(bin_files, 1):
                        print(f"  {i}. {bin_file}")
                    print(f"\n첫 번째 파일을 사용합니다: {bin_files[0]}")
                    filename = os.path.join(map_dir, bin_files[0])
                else:
                    print(f"map 디렉토리에 .bin 파일이 없습니다.")
                    return
            else:
                print(f"map 디렉토리가 존재하지 않습니다: {map_dir}")
                return
        else:
            print(f"대체 맵 파일을 사용합니다: {filename}")
    else:
        print(f"맵 파일 사용: {filename}")
    
    # 파일 로드
    reachability_map = load_from_binary_file(filename)
    
    if not reachability_map.reach_map:
        print("오류: 맵 데이터가 비어 있습니다. 시각화를 중단합니다.")
        return
    
    print("맵 로드 완료! 시각화 시작...")
    
    # 시각화 객체 생성
    visualizer = ReachabilityVisualizer(reachability_map)
    
    # 사용 가능한 z 값의 범위 계산
    z_values = [data.sphere_center[2] for data in reachability_map.reach_map if data.measure > 0]
    min_z, max_z = min(z_values), max(z_values)
    unique_z_values = sorted(set([round(z, 2) for z in z_values]))
    
    print(f"Z 좌표 범위: {min_z} ~ {max_z}")
    print(f"사용 가능한 Z 층 값 (반올림): {unique_z_values[:10]}... (총 {len(unique_z_values)}개)")
    
    # 사용자 입력 받기
    print("\n특정 층 시각화 옵션:")
    print("1. 모든 데이터 시각화")
    print("2. Z > 0 데이터만 시각화")
    print("3. 특정 Z 층만 시각화")
    
    choice = input("선택하세요 (1/2/3): ").strip()
    
    if choice == '1':
        # 모든 데이터 시각화
        print("모든 데이터를 시각화합니다...")
        visualizer.visualize()
    elif choice == '2':
        # Z > 0 데이터만 시각화
        print("Z > 0 데이터만 시각화합니다...")
        visualizer.visualize(filter_z=0.0)
    elif choice == '3':
        # 특정 Z 층만 시각화
        try:
            z_layer = float(input(f"시각화할 Z 층 값을 입력하세요 ({min_z} ~ {max_z}): "))
            z_tolerance = float(input(f"허용 오차를 입력하세요 (기본값: 0.01): ") or "0.01")
            
            print(f"Z = {z_layer} ± {z_tolerance} 층을 시각화합니다...")
            visualizer.visualize(z_layer=z_layer, z_tolerance=z_tolerance)
        except ValueError:
            print("유효하지 않은 입력입니다. 모든 데이터를 시각화합니다.")
            visualizer.visualize()
    else:
        # 기본적으로 모든 데이터 시각화
        print("잘못된 선택입니다. 모든 데이터를 시각화합니다...")
        visualizer.visualize()


if __name__ == "__main__":
    main()