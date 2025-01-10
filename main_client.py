import socket
from matplotlib import pyplot as plt
import numpy as np
import color_arr
import time
import gc
import parse

DATA_RECV_BYTES = 1855488  # 수신할 데이터 크기
LIDAR_DATA_SHAPE = (57984, 4)  # LiDAR 데이터의 형상 (여기서는 예시로 57984x4)

def rm_zero_point(point_cloud):
    """xyz가 전부 0이거나 Intensity가 0인 경우 삭제"""
    mask_array1 = point_cloud[:, 0] == 0
    mask_array2 = point_cloud[:, 1] == 0
    mask_array3 = point_cloud[:, 2] == 0
    mask_array4 = point_cloud[:, 3] == 0
    mask_all = np.logical_and(mask_array1, mask_array2)
    mask_all = np.logical_and(mask_all, mask_array3)
    mask_all = np.logical_or(mask_all, mask_array4)
    point_cloud = point_cloud[~mask_all]
    return point_cloud

def view_pcl(client, time1):
    fig = plt.figure()
    plt.style.use(['dark_background'])
    ax = fig.add_subplot(111, projection='3d')

    while True:
        plt.cla()

        # 데이터 수신 (TCP 소켓에서 데이터 받기)
        lidar_bytes = b""
        remain_bytes = DATA_RECV_BYTES
        while remain_bytes > 0:
            chunk = client.recv(remain_bytes)
            lidar_bytes += chunk
            remain_bytes -= len(chunk)
        
        if len(lidar_bytes) < DATA_RECV_BYTES:
            print("데이터 수신 실패, 다시 시도...")
            continue  # 데이터가 부족하면 다시 시도

        # 수신된 데이터를 numpy 배열로 변환
        point_cloud = np.reshape(np.frombuffer(lidar_bytes, dtype=np.float64), LIDAR_DATA_SHAPE)
        
        # 포인트 클라우드 전처리
        point_cloud = rm_zero_point(point_cloud)

        # 3D 시각화
        ax.scatter(point_cloud[:, 0], point_cloud[:, 1], point_cloud[:, 2], s=0.2)
        ax.set_xlim(-10, 10)
        ax.set_ylim(-10, 10)
        ax.set_zlim(-10, 10)

        # FPS 표시
        time2 = time.time()
        ax.text(-10, 0, 7, 'FPS:' + str(round(1 / (time2 - time1), 2)))
        time1 = time.time()

        plt.axis('off')
        plt.pause(0.001)
        gc.collect()
    plt.show()

def save_pcd(client, i):
    lidar_bytes = b""
    remain_bytes = DATA_RECV_BYTES
    while remain_bytes > 0:
        chunk = client.recv(remain_bytes)
        lidar_bytes += chunk
        remain_bytes -= len(chunk)

    if len(lidar_bytes) < DATA_RECV_BYTES:
        print("데이터 수신 실패, 다시 시도...")
        return  # 데이터가 부족하면 이 시도는 취소
    
    point_cloud = np.reshape(np.frombuffer(lidar_bytes, dtype=np.float64), LIDAR_DATA_SHAPE)
    point_cloud = rm_zero_point(point_cloud)

    if i > 5:
        with open(f"point_cloud_{i}_{time.localtime().tm_hour}{time.localtime().tm_min}{time.localtime().tm_sec}.pcd", 'w') as f:
            f.write(parse.HEADER.format(len(point_cloud), len(point_cloud)))
            point_cloud = np.round(point_cloud, 4)
            for i in range(len(point_cloud)):
                f.write(f"{point_cloud[i][0]} {point_cloud[i][1]} {point_cloud[i][2]} {point_cloud[i][3]}\n")
            print(f"pcd file {i} saved")

def main():
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    LOCALHOST = "127.0.0.1"
    PORT_NO = 999
    client.connect((LOCALHOST, PORT_NO))
    print("Connected to server\n")

    time1 = time.time()
    for i in range(7):
        save_pcd(client, i)

    # 시각화 함수 호출
    view_pcl(client, time1)

if __name__ == "__main__":
    main()
