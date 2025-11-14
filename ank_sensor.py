import serial
from datetime import datetime
import csv
import matplotlib.pyplot as plt
import numpy as np
import time
from collections import deque

# =========================
# 시리얼 포트 설정
# =========================
com = "/dev/cu.usbmodem496D155F34411"
baud = 115200
ser = serial.Serial(com, baud, timeout=0.1)

# =========================
# 플롯/버퍼 설정
# =========================
frames_to_plot = 100

# 최근 frames_to_plot개만 유지하는 deque (자동으로 오래된 값 삭제)
data1 = deque([0.0] * frames_to_plot, maxlen=frames_to_plot)
data2 = deque([0.0] * frames_to_plot, maxlen=frames_to_plot)
data3 = deque([0.0] * frames_to_plot, maxlen=frames_to_plot)
x_vals = deque(range(frames_to_plot), maxlen=frames_to_plot)

count = frames_to_plot  # 이미 0~frames_to_plot-1까지 채워져 있으므로

data_splited_float = [0.0] * 8

# =========================
# CSV 파일 준비 (실시간 로깅)
# =========================
csv_filename = datetime.now().strftime("data_%Y%m%d_%H%M%S.csv")
csv_file = open(csv_filename, mode="w", newline="", buffering=1)  # line-buffered
csv_writer = csv.writer(csv_file)
csv_writer.writerow(["timestamp", "index", "data1", "data2", "data3"])

# =========================
# 플롯 초기화 (최적화된 방식)
# =========================
plt.ion()  # interactive mode on
fig, ax = plt.subplots()

# 배경 색상은 한 번만 설정
fig.patch.set_facecolor('black')
ax.set_facecolor('black')

# RGB 색상 정의
col_1 = (255/255, 64/255, 0/255)    # 주황-빨강
col_2 = (0/255, 255/255, 255/255)   # 청록색
col_3 = (0/255, 255/255, 0/255)     # 초록색

# 초기 라인 객체 3개 생성 (데이터는 나중에 set_data로 갱신)
line1, = ax.plot(x_vals, data1, color=col_1, linewidth=3)
line2, = ax.plot(x_vals, data2, color=col_2, linewidth=3)
line3, = ax.plot(x_vals, data3, color=col_3, linewidth=3)

# 축/그리드/틱 설정 (한 번만)
ax.set_xlim(x_vals[0], x_vals[-1])
ax.set_ylim(0, 6000)  # 필요하면 나중에 자동으로 조정 가능
ax.tick_params(axis='both', which='major', labelsize=12)
ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.3)
ax.tick_params(colors='white')
for spine in ax.spines.values():
    spine.set_color('white')

try:
    while ser.isOpen():
        # 한 줄 읽기
        raw = ser.readline().decode('utf-8', errors='ignore').strip()
        if not raw:
            continue

        time.sleep(0.001)

        data_splited = raw.split(" ")

        # 최소 8개 값이 있을 때만 처리
        if len(data_splited) > 7:
            try:
                for i in range(8):
                    data_splited_float[i] = float(data_splited[i])
            except ValueError:
                # 변환 실패 시 이 줄은 무시
                continue

            # 차분 계산
            data1_ = round(data_splited_float[3] - data_splited_float[4], 7)
            data2_ = round(data_splited_float[4] - data_splited_float[5], 7)
            data3_ = round(data_splited_float[5] - data_splited_float[6], 7)

            print("{:.3f}, {:.3f}, {:.3f}".format(data1_, data2_, data3_))

            # ===== CSV 로깅 =====
            timestamp = datetime.now().isoformat()
            csv_writer.writerow([timestamp, count, data1_, data2_, data3_])

            # ===== 버퍼 업데이트 =====
            data1.append(data1_)
            data2.append(data2_)
            data3.append(data3_)
            x_vals.append(count)
            count += 1

            # ===== 플롯 데이터만 업데이트 (최적화 핵심) =====
            line1.set_data(x_vals, data1)
            line2.set_data(x_vals, data2)
            line3.set_data(x_vals, data3)

            # x축 범위 갱신
            ax.set_xlim(x_vals[0], x_vals[-1])

            # 필요하면 y축 자동 조정 (주석 해제해서 사용)
            # ymin = min(min(data1), min(data2), min(data3)) - 100
            # ymax = max(max(data1), max(data2), max(data3)) + 100
            # ax.set_ylim(ymin, ymax)

            plt.pause(0.001)

except KeyboardInterrupt:
    print("사용자에 의해 종료되었습니다.")

finally:
    ser.close()
    csv_file.close()
    plt.ioff()
    plt.show()
