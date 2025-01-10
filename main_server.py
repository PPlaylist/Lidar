import socket
import numpy as np
import parse

LIDAR_UDP_PORT = 2368  # LiDAR에서 보내는 기본 포트
LIDAR_TCP_PORT = 999   # LiDAR 데이터를 보낼 TCP 포트
LIDAR_UDP_PACKET_SIZE = 1206  # UDP 패킷 크기 (예시로 1206 바이트)

# LiDAR UDP 데이터 수신 후 파싱하는 함수
def get_data_PCL(serverSocket_):
    count = 0
    while True:
        count += 1
        data = serverSocket_.recv(LIDAR_UDP_PACKET_SIZE)  # 1206바이트씩 데이터 수신
        get_data, point_cloud = parse.parse_pos(data, count)  # parse_pos 호출하여 포인트 클라우드 파싱
        if get_data:
            return point_cloud  # 데이터가 성공적으로 파싱되었으면 반환

def main():
    # UDP 소켓 생성 및 바인딩
    UDP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    UDP_server_socket.bind(("0.0.0.0", 2368))  # LiDAR 장치와 맞는 IP, 포트 설정
    print(f"UDP socket listening on port {2368}...")

    # TCP 소켓 설정 (수신 데이터를 클라이언트에 전달)
    TCP_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCP_server_socket.bind(("127.0.0.1", 999))  # 로컬호스트에 연결
    TCP_server_socket.listen()
    print(f"TCP server listening on port {999}...")

    # 클라이언트 연결 수락
    client_socket, addr = TCP_server_socket.accept()
    print(f"Connected to {addr}")

    while True:
        try:
            # LiDAR 데이터 수신 및 파싱
            point_cloud = get_data_PCL(UDP_server_socket)

            # 포인트 클라우드 데이터가 비어있지 않은지 확인
            if point_cloud is not None and len(point_cloud) > 0:
                print(f"Sending point cloud with {len(point_cloud)} points")
                # 받은 데이터를 TCP로 전송
                client_socket.send(np.ndarray.tobytes(point_cloud))
            else:
                print("No data received or data is empty, retrying...")

        except Exception as e:
            print(f"Error while processing data: {e}")
            continue  # 오류가 발생하면 다시 시도

if __name__ == "__main__":
    main()
