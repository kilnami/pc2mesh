"""
2단계-1: 정답을 아는 가상의 방을 '가상 LiDAR'로 스캔해서 포인트클라우드를 만든다.

왜 합성 데이터부터?
  실제 데이터로 결과가 이상하면 알고리즘 문제인지 데이터 문제인지 구분하기 어렵다.
  여기서는 정답 메시(GT)를 알고 있으니 나중에 재구성 결과를 정답과 비교할 수 있다.

출력 (--clutter 면 synth_room 대신 synth_clutter):
  data/synth_room.ply          포인트클라우드 (재구성 입력). 점마다 정답 법선도 들어 있다:
                               레이가 맞은 면의 법선을, 그 레이를 쏜 센서 쪽을 향하게 맞춘 것 (이상치 점은 0)
                               31번은 입력의 법선을 버리므로 재구성에는 영향이 없고, 37번이 채점할 때만 쓴다
  data/synth_room_gt.ply       정답 메시 (비교용)
  data/synth_room_sensors.npy  스캔한 센서 위치들 (법선 방향을 정할 때 씀)

--clutter: 물체가 많은 방 (8 x 6 x 3 m). 다리 달린 책상 2개, 선반, 쌓인 상자, 의자 3개, 큰 기계,
  벽을 따라 높이 2.2 m와 2.6 m에 가로 배관, 세로 배관. tangent의 사슬이 끊길 곳(얇은 판, 가는 다리,
  배관)을 일부러 많이 넣었다. 방/열린 장면과 상관없이 '복잡한 장면'에서 부호 방식을 채점하려고 만든 것.

실행:
  python 21_make_synth_room.py            # 만들고 3D 뷰어로 보기
  python 21_make_synth_room.py --no-view  # 만들기만
  python 21_make_synth_room.py --hide-ceiling  # 뷰어에서 천장을 숨겨 방 안을 보기 (저장 파일은 그대로)
  python 21_make_synth_room.py --clutter --no-view
"""
import argparse

import numpy as np
import open3d as o3d

from common import DATA_DIR, height_colors

# ---- 바꿔볼 만한 파라미터 ----
ROOM_SIZE = (6.0, 4.0, 2.5)  # 방 크기 x, y, z [m]
SCAN_POSITIONS = [(1.5, 1.0, 0.5), (3.0, 2.0, 0.5), (4.5, 3.0, 0.5)]  # 로봇이 스캔한 위치 [m]
CLUTTER_ROOM = (8.0, 6.0, 3.0)  # --clutter 방 크기 [m]
CLUTTER_SCANS = [(1.2, 1.2, 0.5), (4.0, 2.5, 0.5), (6.0, 1.5, 0.5), (2.5, 4.8, 0.5), (6.5, 4.8, 0.5)]
N_CHANNELS = 64              # 세로 레이저 개수 (위아래 해상도)
N_AZIMUTH = 1024             # 한 바퀴에 쏘는 횟수 (좌우 해상도)
V_FOV_DEG = (-45.0, 45.0)    # 세로 시야각 [deg]
MAX_RANGE = 20.0             # 최대 측정 거리 [m]
RANGE_NOISE_STD = 0.01       # 거리 노이즈 표준편차 [m], 실제 LiDAR는 1~3 cm 정도
OUTLIER_RATIO = 0.005        # 표면이 아닌 허공에 찍히는 잘못된 점 비율 (먼지, 경계 반사 등)
SEED = 0


def build_room():
    """정답 메시: 방(속이 빈 상자) + 책상 같은 상자 + 기둥."""
    sx, sy, sz = ROOM_SIZE
    room = o3d.geometry.TriangleMesh.create_box(sx, sy, sz)
    table = o3d.geometry.TriangleMesh.create_box(1.2, 0.6, 0.75)
    table.translate((2.0, 0.3, 0.0))
    pillar = o3d.geometry.TriangleMesh.create_cylinder(radius=0.2, height=sz, resolution=32)
    pillar.translate((4.5, 1.2, sz / 2))  # 원기둥은 원점 중심으로 만들어지므로 바닥에 세운다
    return room + table + pillar


def box(size, at):
    """크기 size인 상자를 모서리가 at에 오게 놓는다."""
    m = o3d.geometry.TriangleMesh.create_box(*size)
    m.translate(at)
    return m


def cylinder(radius, length, center, axis="z"):
    """원기둥: center 중심, axis 방향으로 길이 length."""
    m = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=length, resolution=24)
    if axis != "z":
        rot = {"x": (0, np.pi / 2, 0), "y": (np.pi / 2, 0, 0)}[axis]
        m.rotate(o3d.geometry.get_rotation_matrix_from_xyz(rot), center=(0, 0, 0))
    m.translate(center)
    return m


def table(x, y, w, d, h, top=0.04, leg=0.05):
    """얇은 상판 + 다리 4개."""
    parts = [box((w, d, top), (x, y, h - top))]
    for dx in (0, w - leg):
        for dy in (0, d - leg):
            parts.append(box((leg, leg, h - top), (x + dx, y + dy, 0)))
    return parts


def chair(x, y):
    """좌판 + 등받이 + 가는 다리 4개."""
    return table(x, y, 0.45, 0.45, 0.45, top=0.03, leg=0.03) + [box((0.45, 0.03, 0.45), (x, y + 0.42, 0.45))]


def build_clutter_room():
    """물체가 많은 정답 메시. 얇은 판, 가는 다리, 배관처럼 tangent의 사슬이 끊기기 쉬운 것들을 넣었다."""
    sx, sy, sz = CLUTTER_ROOM
    parts = [o3d.geometry.TriangleMesh.create_box(sx, sy, sz)]
    parts += table(2.0, 0.3, 1.6, 0.8, 0.75) + table(5.2, 3.0, 1.2, 0.7, 0.75)
    parts += [box((0.03, 0.4, 2.0), (0.2, 2.0, 0)), box((0.03, 0.4, 2.0), (1.4, 2.0, 0))]   # 선반 옆판
    parts += [box((1.2, 0.4, 0.02), (0.23, 2.0, h)) for h in (0.1, 0.6, 1.1, 1.6, 1.98)]    # 선반 칸
    parts += [box((0.8, 0.8, 0.8), (6.9, 0.2 + 0.85 * i, 0)) for i in range(3)]              # 쌓인 상자
    parts += [box((0.7, 0.7, 0.7), (6.95, 0.25 + 0.85 * i, 0.8)) for i in range(2)]
    parts += chair(2.4, 1.3) + chair(3.2, 1.3) + chair(5.4, 2.4)
    parts += [box((1.5, 1.0, 1.2), (3.0, 4.6, 0))]                                           # 큰 기계
    parts += [cylinder(0.08, 7.6, (4.0, 5.75, 2.2), "x"), cylinder(0.12, 7.6, (4.0, 5.6, 2.6), "x")]  # 가로 배관
    parts += [cylinder(0.05, sz, (0.4, 5.7, sz / 2)), cylinder(0.05, sz, (7.6, 5.7, sz / 2))]          # 세로 배관
    mesh = parts[0]
    for p in parts[1:]:
        mesh += p
    return mesh


def lidar_directions():
    """회전형 LiDAR처럼 세로 N_CHANNELS x 가로 N_AZIMUTH 방향의 단위 벡터를 만든다."""
    elev = np.deg2rad(np.linspace(*V_FOV_DEG, N_CHANNELS))
    azim = np.linspace(0.0, 2 * np.pi, N_AZIMUTH, endpoint=False)
    el, az = np.meshgrid(elev, azim, indexing="ij")
    dirs = np.stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)], axis=-1)
    return dirs.reshape(-1, 3)


def scan(scene, origin, dirs, rng):
    """한 위치에서 레이를 쏘고, 맞은 거리에 노이즈를 더해 점으로 만든다. (점, 정답 법선)"""
    origin = np.asarray(origin)
    rays = np.hstack([np.broadcast_to(origin, dirs.shape), dirs]).astype(np.float32)
    ans = scene.cast_rays(o3d.core.Tensor(rays))
    t = ans["t_hit"].numpy()  # 방향이 단위벡터라 t = 거리
    hit = np.isfinite(t) & (t < MAX_RANGE)
    t, d = t[hit], dirs[hit]
    n = ans["primitive_normals"].numpy()[hit].astype(np.float64)
    n[np.einsum("ij,ij->i", n, d) > 0] *= -1  # 정답 법선 = 맞은 면의 법선을 센서 쪽(레이 반대 방향)으로

    # LiDAR 노이즈는 주로 레이 방향(거리)으로 생긴다
    t = t + rng.normal(0.0, RANGE_NOISE_STD, t.shape)

    # 일부 레이는 표면보다 앞쪽 허공에 찍힌다 (이상치)
    bad = rng.random(t.shape) < OUTLIER_RATIO
    t[bad] = rng.uniform(np.minimum(0.3, t[bad]), t[bad])  # 아주 가까운 면이면 0.3 m보다 앞
    n[bad] = 0.0  # 허공의 점에는 정답 법선이 없다

    return origin + d * t[:, None], n


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장 근처 점을 숨김")
    parser.add_argument("--clutter", action="store_true", help="물체가 많은 방을 만든다 (synth_clutter)")
    args = parser.parse_args()

    name = "synth_clutter" if args.clutter else "synth_room"
    positions = CLUTTER_SCANS if args.clutter else SCAN_POSITIONS
    room_h = CLUTTER_ROOM[2] if args.clutter else ROOM_SIZE[2]
    rng = np.random.default_rng(SEED)
    gt_mesh = build_clutter_room() if args.clutter else build_room()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(gt_mesh))

    dirs = lidar_directions()
    scans, normals = [], []
    for i, pos in enumerate(positions):
        pts, nrm = scan(scene, pos, dirs, rng)
        print(f"스캔 {i}: 위치 {pos} -> 점 {len(pts):,}개")
        scans.append(pts)
        normals.append(nrm)
    points = np.vstack(scans)

    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    pcd.normals = o3d.utility.Vector3dVector(np.vstack(normals))
    nn = np.asarray(pcd.compute_nearest_neighbor_distance())
    lo, hi = pcd.get_min_bound(), pcd.get_max_bound()
    print(f"\n전체 점: {len(points):,}개")
    print(f"범위: x {lo[0]:.2f}~{hi[0]:.2f}, y {lo[1]:.2f}~{hi[1]:.2f}, z {lo[2]:.2f}~{hi[2]:.2f} m")
    print(f"이웃 점 간격: 중앙값 {np.median(nn) * 100:.1f} cm, 상위 1% {np.percentile(nn, 99) * 100:.1f} cm")

    DATA_DIR.mkdir(exist_ok=True)
    o3d.io.write_point_cloud(str(DATA_DIR / f"{name}.ply"), pcd)
    o3d.io.write_triangle_mesh(str(DATA_DIR / f"{name}_gt.ply"), gt_mesh)
    np.save(DATA_DIR / f"{name}_sensors.npy", np.array(positions))
    print(f"\n저장: {DATA_DIR}/{name}.ply (정답 법선 포함), {name}_gt.ply, {name}_sensors.npy")

    if not args.no_view:
        view = o3d.geometry.PointCloud(pcd)
        view.colors = o3d.utility.Vector3dVector(height_colors(points))
        if args.hide_ceiling:
            view = view.select_by_index(np.flatnonzero(points[:, 2] < room_h - 0.1))
        sensors = []
        for pos in positions:
            s = o3d.geometry.TriangleMesh.create_sphere(radius=0.08)
            s.translate(pos)
            s.paint_uniform_color((1.0, 0.0, 1.0))
            sensors.append(s)
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
        o3d.visualization.draw_geometries([view, axes, *sensors], window_name="synth room scan")


if __name__ == "__main__":
    main()
