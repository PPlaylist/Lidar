import socket
import numpy as np
import open3d as o3d
import time
import parse

# 만약 parse가 공용이라면 import 경로 주의

# 서버에서 전송되는 포인트 클라우드 byte 사이즈
DATA_RECV_BYTES = 1855521  # parse 결과로 나오는 numpy 배열 크기에 맞게 조정
LIDAR_DATA_SHAPE = (57984, 4)  # 사용자의 라이다별 포인트 개수 x 채널 (XYZI)

################################
# 유용한 헬퍼 함수들
################################
def rm_zero_point(point_cloud):
    """
    xyz가 전부 0이거나 Intensity가 0인 경우를 삭제
    """
    mask_arr1 = point_cloud[:, 0] == 0
    mask_arr2 = point_cloud[:, 1] == 0
    mask_arr3 = point_cloud[:, 2] == 0
    mask_arr4 = point_cloud[:, 3] == 0
    mask_all = np.logical_and(mask_arr1, mask_arr2)
    mask_all = np.logical_and(mask_all, mask_arr3)
    mask_all = np.logical_or(mask_all, mask_arr4)
    return point_cloud[~mask_all]

def trim_60degree(point_cloud):
    """
    전방 약 ±60도(FOV)로 잘라내는 예시
    """
    mask_positive_x = point_cloud[:, 0] >= 0
    mask_negative_x = point_cloud[:, 0] < 0
    mask_right_y = (point_cloud[:, 1] > point_cloud[:, 0] * 1.732)    # tan(60 deg) = 1.732
    mask_left_y  = (point_cloud[:, 1] > point_cloud[:, 0] * -1.732)
    mask_right = np.logical_and(mask_positive_x, mask_right_y)
    mask_left  = np.logical_and(mask_negative_x, mask_left_y)
    mask_all = np.logical_or(mask_left, mask_right)
    return point_cloud[mask_all]

def distance_filter(point_cloud, max_dist=50.0):
    """
    일정 거리(max_dist) 밖의 점들은 제거
    """
    dist = np.sqrt(point_cloud[:, 0]**2 + point_cloud[:, 1]**2 + point_cloud[:, 2]**2)
    mask = dist < max_dist
    return point_cloud[mask]

def intensity_to_color(intensity):
    """
    Intensity를 0~1 범위로 정규화하여 컬러로 매핑
    (원하는 매핑 함수로 변경 가능)
    """
    norm_intensity = (intensity - intensity.min()) / (intensity.ptp() + 1e-8)
    # 단순히 R~G~B로 그라데이션 주는 예시
    colors = np.zeros((intensity.shape[0], 3))
    colors[:, 0] = norm_intensity          # R
    colors[:, 1] = 1.0 - norm_intensity    # G
    colors[:, 2] = 0.5                     # B는 고정
    return colors

################################
# Open3D 실시간 시각화
################################

def main():
    # 1) TCP 클라이언트 연결
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    LOCALHOST = "127.0.0.1"
    PORT_NO = 999
    client.connect((LOCALHOST, PORT_NO))
    print("Connected to server!")

    # 2) Open3D 시각화 준비
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="LiDAR Stream", width=1280, height=720)
    
    # 초기 point cloud geometry
    pcd = o3d.geometry.PointCloud()
    vis.add_geometry(pcd)

    # 카메라 뷰를 어느 정도 설정해두면 편함
    ctr = vis.get_view_control()
    params = ctr.convert_to_pinhole_camera_parameters()
    # 예시로 z축 기준 멀리 보는 정도 조정
    params.extrinsic[2, 3] = 30  
    ctr.convert_from_pinhole_camera_parameters(params)

    fps_t0 = time.time()
    frame_count = 0

    while True:
        # 3) TCP로부터 데이터를 받음
        lidar_bytes = b""
        remain_bytes = DATA_RECV_BYTES
        # 수신 누락 방지(필수는 아님. 환경에 따라 바로 recv(DATA_RECV_BYTES)를 써도 됨)
        while remain_bytes > 0:
            chunk = client.recv(remain_bytes)
            lidar_bytes += chunk
            remain_bytes -= len(chunk)

        # 4) numpy array로 변환
        point_cloud = np.frombuffer(lidar_bytes, dtype=np.float64)
        point_cloud = np.reshape(point_cloud, LIDAR_DATA_SHAPE)

        # (선택) 불필요한 점 제거, ROI 등 전처리
        point_cloud = rm_zero_point(point_cloud)
        point_cloud = distance_filter(point_cloud, max_dist=30.0)
        # point_cloud = trim_60degree(point_cloud)

        # 5) Open3D용 포인트 클라우드 업데이트
        #    XYZ만 추출
        pcd_points = point_cloud[:, :3]  
        #    Intensity -> color 매핑
        pcd_colors = intensity_to_color(point_cloud[:, 3])

        # Open3D PointCloud 갱신
        pcd.clear()
        pcd.points = o3d.utility.Vector3dVector(pcd_points)
        pcd.colors = o3d.utility.Vector3dVector(pcd_colors)

        # 시각화 업데이트
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()

        # FPS 계산
        frame_count += 1
        if frame_count % 10 == 0:
            t1 = time.time()
            fps = 10.0 / (t1 - fps_t0)
            fps_t0 = t1
            print(f"FPS: {fps:.2f}")

if __name__ == "__main__":
    main()
