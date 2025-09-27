"""
Drone Route Planner
-------------------
여러 대의 드론을 이용하여 배송 경로를 최적화하는 프로그램.
- OR-Tools로 최적 경로 탐색
- 드론 용량/최대 이동거리/대기시간 고려
- Folium을 이용한 지도 시각화
"""

import math
import pandas as pd
import folium
import heapq
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

# -----------------------
# 설정값
# -----------------------
DRONE_COUNT = 3
DRONE_CAPACITY = 2       # 드론 1회 배송 가능 수
MAX_TRIPS = 3            # 드론 1대당 최대 운행 횟수
MAX_DISTANCE = 1.1       # 단일 왕복 최대 거리 (km)
SPEED = 40               # 드론 속도 (km/h)
WAIT_TIME = 0.5          # 왕복 후 대기시간 (분)
CSV_FILE = "places.csv"  # 입력 데이터 파일

# -----------------------
# 거리 계산 (Haversine)
# -----------------------
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*(math.sin(dlambda/2)**2)
    return R * (2 * math.atan2(math.sqrt(a), math.sqrt(1-a)))

# -----------------------
# CSV 로드
# -----------------------
def load_locations(file_name):
    df = pd.read_csv(file_name)
    locations = list(zip(df['latitude'], df['longitude']))
    names = df['place_name'].tolist()
    return locations, names

# -----------------------
# 거리 행렬 생성
# -----------------------
def build_distance_matrix(locations):
    n = len(locations)
    D = [[0]*n for _ in range(n)]
    for i in range(n):
        for j in range(i+1, n):
            d = haversine(*locations[i], *locations[j])
            D[i][j] = D[j][i] = d
    return D

# -----------------------
# OR-Tools 최적 경로 탐색
# -----------------------
def find_best_route(subset, distances):
    if not subset:
        return [0, 0], 0
    subset = [0] + list(subset)
    m = len(subset)

    manager = pywrapcp.RoutingIndexManager(m, 1, 0)
    routing = pywrapcp.RoutingModel(manager)

    # 거리 콜백
    def dist_callback(f, t):
        f, t = manager.IndexToNode(f), manager.IndexToNode(t)
        return int(round(distances[subset[f]][subset[t]] * 1000))

    transit_index = routing.RegisterTransitCallback(dist_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_index)

    # 용량 제약 (각 고객은 1의 수요)
    def demand_callback(f):
        return 0 if manager.IndexToNode(f) == 0 else 1

    demand_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(demand_index, 0, [DRONE_CAPACITY], True, "Capacity")

    # 최대 이동 거리 제약
    routing.AddDimension(transit_index, 0, int(MAX_DISTANCE * 1000), True, "Distance")

    # 탐색
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    solution = routing.SolveWithParameters(params)
    if not solution:
        return None, None

    # 경로 복원
    index = routing.Start(0)
    route_local = []
    while not routing.IsEnd(index):
        route_local.append(manager.IndexToNode(index))
        index = solution.Value(routing.NextVar(index))
    route_local.append(0)

    route_full = [subset[i] for i in route_local]
    total_km = sum(distances[route_full[i]][route_full[i+1]] for i in range(len(route_full)-1))
    return route_full, total_km

# -----------------------
# 드론 경로 계획
# -----------------------
def plan_drone_routes(locations, names, distances):
    n = len(locations)
    unvisited = set(range(1, n))
    drone_routes = [[] for _ in range(DRONE_COUNT)]
    drone_trips = [0] * DRONE_COUNT
    drone_times = [0] * DRONE_COUNT
    pq = [(0, d) for d in range(DRONE_COUNT)]
    heapq.heapify(pq)

    # 일반 배달 (용량 단위로 묶어서)
    while unvisited and pq:
        avail_time, drone = heapq.heappop(pq)
        if drone_trips[drone] >= MAX_TRIPS:
            continue

        subset = []
        current = 0
        for _ in range(DRONE_CAPACITY):
            if not unvisited:
                break
            nearest = min(unvisited, key=lambda i: distances[current][i])
            if distances[current][nearest] + distances[nearest][0] > MAX_DISTANCE:
                continue
            subset.append(nearest)
            unvisited.remove(nearest)
            current = nearest

        if not subset:
            continue

        route, total_km = find_best_route(subset, distances)
        if route is None:
            unvisited.update(subset)
            continue

        start_time = max(avail_time, drone_times[drone])
        travel_time = total_km / SPEED * 60
        drone_trips[drone] += 1
        drone_times[drone] = start_time + travel_time + WAIT_TIME
        drone_routes[drone].append({
            "trip_no": drone_trips[drone],
            "route": route,
            "total_distance_km": total_km,
            "start_time_min": start_time,
            "travel_time_min": travel_time
        })

        if drone_trips[drone] < MAX_TRIPS:
            heapq.heappush(pq, (drone_times[drone], drone))

    # 장거리 단독 배달 (먼 곳부터)
    long_distance = sorted(unvisited, key=lambda idx: distances[0][idx], reverse=True)
    for idx in long_distance:
        rt = distances[0][idx] * 2
        if rt > MAX_DISTANCE:
            print(f"[경고] '{names[idx]}' 왕복({rt:.2f} km) 초과 → 스킵")
            continue

        drone_idx = drone_times.index(min(drone_times))
        start_time = drone_times[drone_idx]
        travel_time = rt / SPEED * 60
        drone_times[drone_idx] = start_time + travel_time + WAIT_TIME
        drone_routes[drone_idx].append({
            "trip_no": len(drone_routes[drone_idx]) + 1,
            "route": [0, idx, 0],
            "total_distance_km": rt,
            "start_time_min": start_time,
            "travel_time_min": travel_time
        })

    return drone_routes

# -----------------------
# 결과 출력
# -----------------------
def print_results(drone_routes, names, distances):
    for d, routes in enumerate(drone_routes):
        if not routes:
            print(f"드론 {d+1}: 운행 없음")
            continue

        print(f"\n드론 {d+1}")
        for trip in routes:
            route_names = " -> ".join(names[i] for i in trip["route"])
            print(f"  운행 {trip['trip_no']}: {route_names}, 거리 {trip['total_distance_km']:.2f} km, 시간 {trip['travel_time_min']:.1f}분")

            for i in range(len(trip["route"]) - 1):
                f, t = trip["route"][i], trip["route"][i+1]
                print(f"    {names[f]} → {names[t]}: {distances[f][t]:.2f} km")

        total_distance = sum(trip["total_distance_km"] for trip in routes)
        total_time = sum(trip["travel_time_min"] for trip in routes)
        print(f"총 거리: {total_distance:.2f} km, 총 시간: {total_time:.1f}분")

# -----------------------
# 지도 생성
# -----------------------
def create_map(locations, names, drone_routes, out_html="tsp_map.html"):
    m = folium.Map(location=locations[0], zoom_start=18)
    colors = ['red', 'blue', 'green', 'purple', 'orange']

    for d, routes in enumerate(drone_routes):
        color = colors[d % len(colors)]
        for trip in routes:
            coords = [locations[i] for i in trip["route"]]
            folium.PolyLine(coords, color=color, weight=3).add_to(m)
            for i in trip["route"]:
                folium.Marker(locations[i], popup=names[i]).add_to(m)

    m.save(out_html)
    print(f"지도 저장 완료 → {out_html}")

# -----------------------
# 메인 실행
# -----------------------
def main():
    locations, names = load_locations(CSV_FILE)
    distances = build_distance_matrix(locations)
    drone_routes = plan_drone_routes(locations, names, distances)
    print_results(drone_routes, names, distances)
    create_map(locations, names, drone_routes)

if __name__ == "__main__":
    main()
