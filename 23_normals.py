"""
2단계-3: 법선(normal) 추정

법선이란?
  표면 위 각 점에서 표면에 수직인 방향(화살표)이다. 포인트클라우드에는 점의 위치만 있고
  "표면이 어느 쪽을 향하는지"는 없다. 대부분의 재구성 알고리즘(특히 Poisson)은 이 화살표로
  표면의 안과 밖을 구분하므로 먼저 구해 둬야 한다.

두 단계로 구한다.
  1) 축 구하기 (PCA): 점 주변 이웃들에 평면을 맞추고, 그 평면에 수직인 축을 고른다.
     이웃 범위(NORMAL_RADIUS)가 작으면 노이즈에 흔들리고, 크면 모서리에서 두 면이 섞인다.
  2) 부호 정하기 (orientation): 축만으로는 n 인지 -n 인지 모른다.
     실내 맵에서는 모든 법선이 '빈 공간(방 안쪽)'을 향해야 한다.
       none    : 부호를 정하지 않음 (PCA 결과 그대로)
       sensor  : 가장 가까운 센서 쪽을 향하도록 뒤집음 (센서 위치를 알 때)
       tangent : 이웃끼리 방향이 이어지도록 퍼뜨림 (센서 위치를 모를 때)

정답 메시의 법선과 비교해서 축의 각도 오차를 재고, 법선을 세 가지로 나눈다.
  맞음        : 정답과 OK_ANGLE 이내
  옆으로 누움 : 정답과 거의 수직. 1단계(축)부터 틀려서 부호를 어떻게 정해도 소용없다
  뒤집힘      : 정답의 반대, 즉 벽이나 물체 안쪽을 향함. 2단계(부호)가 틀렸다

입력: data/synth_room_clean.ply, data/synth_room_gt.ply, data/synth_room_sensors.npy
출력: data/synth_room_normals.ply (점 + 법선, --orient 로 고른 방식)

실행:
  python 23_normals.py                   # 세 방식 비교 후 sensor 방식 결과를 저장하고 보기
  python 23_normals.py --orient tangent  # tangent 방식 결과를 저장하고 보기
  python 23_normals.py --hide-ceiling
뷰어: 초록 = 맞음, 노랑 = 옆으로 누움, 빨강 = 뒤집힘. 짧은 선이 법선 방향이다(일부 점만 표시).
"""
import argparse
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR, orient_by_propagation, orient_toward_viewpoints

# ---- 바꿔볼 만한 파라미터 ----
NORMAL_RADIUS = 0.1  # 평면을 맞출 이웃 범위 [m]
MAX_NN = 30           # 이웃을 최대 몇 개까지 쓸지
TANGENT_K = 15        # tangent 방식에서 방향을 퍼뜨릴 이웃 수
EDGE_ANGLE = 30.0     # 평가용: 이웃의 정답 법선이 이 각도 이상 다르면 '모서리 근처'로 본다 [deg]
OK_ANGLE = 45.0       # 평가용: 정답과 이 각도 이내면 맞음, (180 - 이 각도) 이상이면 뒤집힘 [deg]
SHOW_RATIO = 0.03     # 뷰어에 법선 선을 그릴 점의 비율
ARROW_LEN = 0.08      # 뷰어의 법선 선 길이 [m]

ROOM_TRIANGLES = 12   # 21번에서 방 상자(삼각형 12개)를 가장 먼저 합쳤다. 이 삼각형들의 법선은 방 바깥을 향한다


def gt_normals(gt, points):
    """각 점에서 가장 가까운 정답 표면의 법선. 방 안쪽(빈 공간)을 향하도록 맞춘다."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(gt))
    ans = scene.compute_closest_points(o3d.core.Tensor(points, dtype=o3d.core.float32))
    n = ans["primitive_normals"].numpy().astype(np.float64)
    n[ans["primitive_ids"].numpy() < ROOM_TRIANGLES] *= -1
    return n


def near_edge(points, n_gt):
    """NORMAL_RADIUS 안의 이웃 중 정답 법선이 크게 다른 점이 있으면 모서리 근처다."""
    nns = o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(points, dtype=o3d.core.float32))
    nns.hybrid_index(NORMAL_RADIUS)
    idx = nns.hybrid_search(o3d.core.Tensor(points, dtype=o3d.core.float32), NORMAL_RADIUS, MAX_NN)[0].numpy()
    valid = idx >= 0
    cos = np.einsum("ij,ikj->ik", n_gt, n_gt[np.where(valid, idx, 0)])
    cos[~valid] = 1.0
    return cos.min(axis=1) < np.cos(np.deg2rad(EDGE_ANGLE))


def categorize(n, n_gt):
    """0 = 맞음, 1 = 옆으로 누움, 2 = 뒤집힘."""
    angle = np.degrees(np.arccos(np.clip(np.einsum("ij,ij->i", n, n_gt), -1, 1)))
    return np.digitize(angle, [OK_ANGLE, 180 - OK_ANGLE])


def orient_sensor(pcd, sensors):
    orient_toward_viewpoints(pcd, sensors)


def orient_tangent(pcd):
    # 퍼뜨린 결과는 '전부 안쪽' 아니면 '전부 바깥쪽'이다. 다수가 방 중심을 향하도록 전체 부호를 고른다
    orient_by_propagation(pcd, TANGENT_K)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--orient", choices=["none", "sensor", "tangent"], default="sensor",
                        help="저장하고 뷰어로 볼 부호 결정 방식")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장 근처 점을 숨김")
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(str(DATA_DIR / "synth_room_clean.ply"))
    gt = o3d.io.read_triangle_mesh(str(DATA_DIR / "synth_room_gt.ply"))
    sensors = np.load(DATA_DIR / "synth_room_sensors.npy")
    points = np.asarray(pcd.points)
    n_gt = gt_normals(gt, points)
    edge = near_edge(points, n_gt)

    # 1) 축 구하기
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=NORMAL_RADIUS, max_nn=MAX_NN))
    cos = np.einsum("ij,ij->i", np.asarray(pcd.normals), n_gt)
    angle = np.degrees(np.arccos(np.clip(np.abs(cos), 0, 1)))  # 부호는 무시한 축의 오차
    print(f"[1] 축의 각도 오차 (NORMAL_RADIUS = {NORMAL_RADIUS * 100:.0f} cm)")
    print(f"    평평한 곳   ({(~edge).sum():>7,}점): 중앙값 {np.median(angle[~edge]):4.1f}°, "
          f"상위 10% {np.percentile(angle[~edge], 90):4.1f}°")
    print(f"    모서리 근처 ({edge.sum():>7,}점): 중앙값 {np.median(angle[edge]):4.1f}°, "
          f"상위 10% {np.percentile(angle[edge], 90):4.1f}°")

    # 2) 부호 정하기: 세 방식 비교
    print(f"\n[2] 부호 방식별 결과 (맞음 ≤{OK_ANGLE:.0f}°, 옆으로 누움, 뒤집힘 ≥{180 - OK_ANGLE:.0f}°)")
    results = {}
    for name in ["none", "sensor", "tangent"]:
        q = o3d.geometry.PointCloud(pcd)
        t0 = time.time()
        if name == "sensor":
            orient_sensor(q, sensors)
        elif name == "tangent":
            orient_tangent(q)
        cat = categorize(np.asarray(q.normals), n_gt)
        print(f"    {name:<8} 맞음 {np.mean(cat == 0):6.1%} | 옆으로 누움 {np.mean(cat == 1):5.1%} | "
              f"뒤집힘 {np.mean(cat == 2):5.1%}  ({time.time() - t0:.1f}초)")
        results[name] = (q, cat)

    out, cat = results[args.orient]
    o3d.io.write_point_cloud(str(DATA_DIR / "synth_room_normals.ply"), out)
    print(f"\n저장 ({args.orient} 방식): {DATA_DIR}/synth_room_normals.ply")

    if not args.no_view:
        keep = np.arange(len(points))
        if args.hide_ceiling:
            keep = np.flatnonzero(points[:, 2] < points[:, 2].max() - 0.15)
        colors = np.array([(0.1, 0.7, 0.2), (1.0, 0.7, 0.0), (1.0, 0.0, 0.0)])[cat]  # 초록, 노랑, 빨강
        view = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points[keep]))
        view.colors = o3d.utility.Vector3dVector(colors[keep])

        show = np.random.default_rng(0).choice(keep, int(len(keep) * SHOW_RATIO), replace=False)
        ends = points[show] + np.asarray(out.normals)[show] * ARROW_LEN
        arrows = o3d.geometry.LineSet(
            o3d.utility.Vector3dVector(np.vstack([points[show], ends])),
            o3d.utility.Vector2iVector(np.c_[np.arange(len(show)), np.arange(len(show)) + len(show)]),
        )
        arrows.colors = o3d.utility.Vector3dVector(colors[show])
        o3d.visualization.draw_geometries([view, arrows], window_name=f"normals ({args.orient}): yellow = sideways, red = flipped")


if __name__ == "__main__":
    main()
