#!/usr/bin/env python
import socket
import parse
import numpy as np

# Velodyne Puck: 일반적으로 1206바이트씩 패킷이 옴
# 사용자 환경에 맞춰 조정
LIDAR_UDP_PACKET_SIZE = 1206
LIDAR_DATA_BYTES = 1855521  # 사용하시는 parse 결과 크기에 맞춰 조정

def get_data_PCL(serverSocket_):
    count = 0
    while True:
        count += 1
        # 라이다 데이터 패킷을 메뉴얼에 따라 파싱하여 리턴하는 함수
        data = serverSocket_.recv(LIDAR_UDP_PACKET_SIZE)
        get_data, point_cloud = parse.parse_pos(data, count)
        if get_data:  # 하나의 스캔 라운드가 끝난 경우
            break
    return point_cloud

def main():
    # 1) LiDAR(UDP) 세팅
    IP_ADDRESS = "192.168.0.100"  # 리스닝할 서버 IP
    PORT_NO = 2368               # Velodyne 기본 포트
    serverSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    serverSocket.bind((IP_ADDRESS, PORT_NO))
    print("UDP socket bind OK:", IP_ADDRESS, PORT_NO)

    # 2) Python TCP Send 세팅
    LOCALHOST = "127.0.0.1"
    PORT_NO2 = 999
    TCPserverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    TCPserverSocket.bind((LOCALHOST, PORT_NO2))
    TCPserverSocket.listen()
    print("Waiting for TCP connection...")

    clientSocket, addr = TCPserverSocket.accept()
    print("Connected by", addr, "\nStart streaming point cloud...")

    while True:
        # 서버: Velodyne UDP 패킷 수신 -> parse -> point cloud numpy array -> byte 변환 -> TCP로 전송
        point_cloud = get_data_PCL(serverSocket)

        # 원하는 전처리를 여기에 해도 됨 (ROI, trim, remove_zero_point 등)
        send_data = np.ndarray.tobytes(point_cloud)
        clientSocket.send(send_data)
        parse.clear_append_data()  # parse 모듈에 임시 저장된 점 누적 리스트 초기화

if __name__ == "__main__":
    main()
