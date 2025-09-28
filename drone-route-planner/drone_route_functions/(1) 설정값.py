import math
import pandas as pd
import folium  # 좌표를 지도에 표시, 경로 시각화 가능
import heapq   # 가장 작은/큰 값 효율적 관리
from ortools.constraint_solver import pywrapcp, routing_enums_pb2   # Google OR-Tools - 최적화 문제 라이브러리 VRP (Vehicle Routing Problem, 차량 경로 문제) 사용

# -----------------------
# 설정값
# -----------------------

DRONE_COUNT = 3          # 드론 수
DRONE_CAPACITY = 2       # 드론 1회 배송 가능 수
MAX_TRIPS = 3            # 드론 1대당 최대 운행 횟수
MAX_DISTANCE = 1.1       # 단일 왕복 최대 거리 (km)
SPEED = 40               # 드론 속도 (km/h)
WAIT_TIME = 0.5          # 왕복 후 대기시간 (분)
CSV_FILE = "places.csv"  # 입력 데이터 파일
