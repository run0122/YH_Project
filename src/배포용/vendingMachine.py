import cv2
import RPi.GPIO as GPIO
import time
import socket

motor_pin = 2

# socket creation
sock = socket.socker.socker(socket.AF_INET, socket.SOCK_STREAM)

# IP address & PORT address
ip = "192.168.0.71"
port = 12345

# binging socket ip & port
sock.bind((ip, port))

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin, GPIO.OUT)
pwm = GPIO.PWM(motor_pin, 50)
pwm.start(11.0)

# 서보 모터 동작 함수
def servo_action(scale):
    if scale == 1:
        angle = 90
    elif scale == 2:
        angle = 60
    elif scale == 3:
        angle = 30
    elif scale == 4:
        angle = 120
    elif scale == 5:
        angle = 90
    else:
        angle = 90
   
    duty_cycle = angle / 10
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(1.0)

# wait socket
sock.listen()
try:
    while True:
        # Publisher connect
        conn, addr = sock.accept()

        # data receive
        numbers = conn.recv(1024).decode()

        print(numbers)

except KeyboardInterrupt:
    print('terminated.')

# 카메라 캡처 객체 생성
cap = cv2.VideoCapture(0)

while True:
    # 프레임 읽기
    ret, frame = cap.read()

    # 프레임 읽기 실패 시 종료
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        break

    # 인식된 숫자가 있을 경우 서보 모터 동작
    if numbers:
        scale = int(numbers)
        servo_action(scale)

    # 화면에 인식된 숫자 출력
    cv2.putText(frame, numbers, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow('Number Recognition', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 종료 시 비디오 캡처 객체와 창 해제
cap.release()
cv2.destroyAllWindows()
GPIO.cleanup()