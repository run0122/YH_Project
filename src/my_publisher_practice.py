import socket

# 전송할 데이터
data = "Hello, Subscriber!"

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Subscriber의 IP 주소와 포트 번호
subscriber_ip = "192.168.0.40"
subscriber_port = 12345

# Subscriber에 연결
sock.connect((subscriber_ip, subscriber_port))

# 데이터 전송
sock.sendall(data.encode())

# 소켓 닫기
sock.close()