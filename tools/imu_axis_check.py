# Airy IMU 축 확인: 한 자세에서 N초 동안 IMU 가속도 평균(중력 반력 방향)과
# LiDAR 포인트에서 찾은 가장 큰 평면의 법선을 함께 기록한다. 두 자세 이상 기록하면 IMU->LiDAR 회전을 계산.
# 실행(컨테이너, rslidar_sdk 노드가 /rslidar_points, /rslidar_imu_data 발행 중):
#   python3 imu_axis_check.py <자세이름> [초]     예) upright 10, side 10
#   python3 imu_axis_check.py solve
import json
import os
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2 as pc2

RECORD_FILE = "/tmp/imu_axis_check.json"


def largest_plane_normal(points, iters=300, thresh=0.03, rng=np.random.default_rng(0)):
    best_inliers = None
    for _ in range(iters):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-6:
            continue
        normal /= norm
        inliers = np.abs((points - sample[0]) @ normal) < thresh
        if best_inliers is None or inliers.sum() > best_inliers.sum():
            best_inliers = inliers
    plane = points[best_inliers]
    centroid = plane.mean(axis=0)
    normal = np.linalg.svd(plane - centroid)[2][-1]
    # point the normal from the plane toward the sensor (origin)
    if normal @ (-centroid) < 0:
        normal = -normal
    return normal, best_inliers.sum(), np.linalg.norm(centroid)


def record(name, seconds):
    rclpy.init()
    node = Node("imu_axis_check")
    accels, clouds = [], []
    node.create_subscription(
        Imu, "/rslidar_imu_data",
        lambda m: accels.append([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z]),
        qos_profile_sensor_data)
    node.create_subscription(PointCloud2, "/rslidar_points", lambda m: clouds.append(m), qos_profile_sensor_data)
    t0 = time.time()
    while time.time() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)
    rclpy.shutdown()
    if not accels or not clouds:
        print(f"데이터 없음: imu {len(accels)}, cloud {len(clouds)}")
        sys.exit(1)

    acc = np.array(accels)
    acc_mean = acc.mean(axis=0)
    s = pc2.read_points(clouds[-1], field_names=("x", "y", "z"), skip_nans=True)
    pts = np.stack([s["x"], s["y"], s["z"]], axis=1).astype(float)
    pts = pts[np.linalg.norm(pts, axis=1) > 0.3]
    pts = pts[np.random.default_rng(1).choice(len(pts), min(20000, len(pts)), replace=False)]
    normal, n_in, dist = largest_plane_normal(pts)

    print(f"[{name}] IMU 가속도 평균 [g]: {np.round(acc_mean, 3)}  |크기| {np.linalg.norm(acc_mean):.3f}  "
          f"흔들림 std {np.round(acc.std(axis=0), 3)}  ({len(acc)}개)")
    print(f"[{name}] LiDAR 최대 평면: 법선(평면→센서) {np.round(normal, 3)}  점 {n_in}개  거리 {dist:.2f} m")
    data = json.load(open(RECORD_FILE)) if os.path.exists(RECORD_FILE) else {}
    data[name] = {"acc": acc_mean.tolist(), "normal": normal.tolist()}
    json.dump(data, open(RECORD_FILE, "w"), indent=1)
    print(f"저장: {RECORD_FILE} ({', '.join(data)})")


def solve():
    data = json.load(open(RECORD_FILE))
    print("기록된 자세:", ", ".join(data))
    for name, d in data.items():
        a = np.array(d["acc"]) / np.linalg.norm(d["acc"])
        n = np.array(d["normal"])
        print(f"  {name}: IMU 단위 가속도 {np.round(a, 3)}  LiDAR 평면 법선 {np.round(n, 3)}  "
              f"(같은 축 가정 시 각도 {np.degrees(np.arccos(np.clip(abs(a @ n), -1, 1))):.1f}°, 내적 부호 {np.sign(a @ n):+.0f})")
    if len(data) < 2:
        print("회전 계산에는 서로 다른 자세 2개 이상 필요")
        return
    # 평면 법선은 부호가 모호(바닥/천장/벽)하므로 각 자세의 부호는 사용자가 확인한 결과로 판단
    A = np.array([np.array(d["acc"]) / np.linalg.norm(d["acc"]) for d in data.values()])
    N = np.array([d["normal"] for d in data.values()])
    u, _, vt = np.linalg.svd(N.T @ A)
    R = u @ np.diag([1, 1, np.sign(np.linalg.det(u @ vt))]) @ vt  # imu -> lidar
    print("IMU→LiDAR 회전 행렬 (법선 부호 그대로 가정):\n", np.round(R, 3))


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "solve":
        solve()
    elif len(sys.argv) >= 2:
        record(sys.argv[1], float(sys.argv[2]) if len(sys.argv) > 2 else 10.0)
    else:
        print(__doc__ or "usage: imu_axis_check.py <pose> [sec] | solve")
