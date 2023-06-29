import socket

# 소켓 생성
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# IP 주소와 포트 번호
ip = "192.168.0.40"
port = 12345

# 소켓을 IP와 포트 번호에 바인딩
sock.bind((ip, port))

# 소켓을 수신 상태로 대기
sock.listen()

# Publisher의 연결을 받아들임
conn, addr = sock.accept()

# 데이터 수신
data = conn.recv(1024).decode()
print("Received data:", data)

# 소켓 닫기
conn.close()
sock.close()