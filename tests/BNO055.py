import time
import csv
import math
import board
import busio
import adafruit_bno055

# I2C 버스 및 센서 초기화
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

csv_filename = f"damper_impact_test_{int(time.time())}.csv"
print(f"데이터 로깅 시작: {csv_filename}")
print("충돌 실험 대기 중... (종료하려면 Ctrl+C를 누르세요)")

with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Time(s)", "Acc_X(m/s^2)", "Acc_Y(m/s^2)", "Acc_Z(m/s^2)", "Acc_Mag(m/s^2)"])
    
    start_time = time.time()
    
    try:
        while True:
            try:
                # 센서 리딩 시도
                current_time = time.time() - start_time
                linear_acc = sensor.linear_acceleration
                
                # 데이터가 정상적으로 들어온 경우에만 기록
                if linear_acc[0] is not None:
                    ax_x, ax_y, ax_z = linear_acc
                    # 3축 가속도 합성 벡터 (충격량 분석용)
                    acc_mag = math.sqrt(ax_x**2 + ax_y**2 + ax_z**2)
                    
                    csv_writer.writerow([current_time, ax_x, ax_y, ax_z, acc_mag])
                    csv_file.flush() # 전원이 끊겨도 직전까지의 데이터 보존
                    
            except OSError:
                # [Errno 121] 또는 [Errno 5] 발생 시 튕기지 않고 무시(Pass)
                pass
            except RuntimeError:
                # Adafruit 라이브러리 내부의 일시적 레지스터 읽기 에러 방어
                pass
            
            # 10ms 대기 (약 100Hz 폴링)
            time.sleep(0.01)
            
    except KeyboardInterrupt:
        print(f"\n실험 종료. [{csv_filename}] 파일이 안전하게 저장되었습니다.")
