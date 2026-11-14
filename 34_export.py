"""
4단계-1: 메시 후처리 + 시뮬레이터용 내보내기

시뮬레이터는 같은 물체를 메시 두 벌로 다룬다.
  보기용(visual)   : 카메라 영상에 그려지는 모양. 촘촘하고 색이 있을수록 좋다. 물리 계산에는 안 쓴다
  충돌용(collision): 물리 엔진이 매 스텝 부딪힘을 계산하는 모양. 삼각형이 적을수록 빠르다.
                    바닥에 구멍이 있으면 로봇이 빠지고, 없는 면이 있으면 못 지나간다
그래서 정리한 메시 하나를 목표 삼각형 수만 다르게 두 번 줄인다.

  1) 재구성: 33번의 reconstruct()로 normals.ply(32번이 고른 법선)에서 메시를 만든다 (--source)
  2) 정리
     - 바닥 아래(z < -FLOOR_MARGIN)로 내려간 부분 자르기
     - 점에서 먼 삼각형 자르기: 무게중심이 가장 가까운 입력 점에서 TRIM_X x voxel 넘게 떨어진 삼각형.
       33번에서 Poisson이 틀린 면의 대부분이 책상 밑에 지어낸 막이었다. 점 밀도로 자르는 trim과 달리
       '점이 아예 없는 곳'을 직접 자른다. 대신 그만큼 구멍이 생긴다 (작은 틈은 메운 채로 남는다)
     - 떠 있는 작은 조각 제거: 이어진 삼각형 덩어리 중 MIN_PIECE개 미만
     - 넓이 0인 삼각형, 겹친 꼭짓점·삼각형, 비다양체 모서리(세 장 이상이 만나는 모서리) 제거
     구멍 검사 두 가지: 열린 가장자리(삼각형 하나에만 속한 모서리) 길이, 바닥 구멍 비율
     (입력 점에 바닥이 있는 FLOOR_CELL 칸마다 아래에서 위로 광선을 쏴서 바닥 높이에서 안 맞는 칸)
  3) 줄이기 (quadric decimation): 이웃한 꼭짓점 둘을 하나로 합치기를 되풀이하는데, 매번 모양이 가장 덜
     변하는 쌍부터 합친다. 평평한 바닥은 큰 삼각형 몇 장이 되고, 굴곡진 의자는 촘촘하게 남는다.
     TABLE_TRIANGLES 마다 정리한 메시에서 얼마나 벗어났는지 표로 보여 준다 (--gt 가 있으면 33번 지표도).
     삼각형 수는 숫자로 박지 않고 표에서 고른다 (장면 크기가 다르면 알맞은 수도 다르다. 4 m² 책상에 맞던
     1만 개가 190 m² 공장에서는 모양이 무너졌다):
       충돌용: 벗어난 거리 95%가 COLLISION_TOL 이하이고 바닥 구멍이 HOLE_TOL 넘게 늘지 않는 가장 작은 수
       보기용: 벗어난 거리 95%가 VISUAL_TOL 이하인 가장 작은 수. 단 VISUAL_MIN 이상 (색은 꼭짓점에 실려 있어서
               너무 줄이면 모양보다 색이 먼저 뭉개진다)
  4) 색을 텍스처로 굽기: 꼭짓점 색은 obj 규격에 없어서 시뮬레이터가 무시한다 (Gazebo Fortress는 흰 그림자처럼,
     PyBullet은 흰색으로 그렸다). 그래서 텍스처 이미지에 삼각형마다 작은 정사각형 칸을 하나씩 주고, 칸의 반쪽
     (직각삼각형)에 세 꼭짓점 색을 섞어 칠한 뒤 obj + mtl + png 로 저장한다. 이 조합은 Gazebo, PyBullet, MuJoCo,
     Isaac Sim이 모두 읽는다. 처음에는 메시를 종이처럼 펼치는 UVAtlas를 썼는데, 다양체이고 앞뒷면 방향이
     일관된 메시만 받아서 공장 메시에서는 실패했다 (삼각형을 18% 지워도 안 됨). 칸 나누기는 메시 모양과
     상관없이 되고 삼각형을 하나도 지우지 않는다.
     주의: Open3D의 obj 저장은 텍스처 png를 위아래 뒤집어 쓴다 (Open3D 자신의 읽기 함수와만 맞는다).
     그래서 저장한 뒤 png를 표준 규칙(v = 0이 이미지 아래)에 맞게 다시 덮어쓴다. 그 결과 Gazebo, PyBullet에서는
     맞게 보이고 Open3D 뷰어에서는 엉뚱하게 보인다 (obj를 직접 읽어 텍스처 색과 꼭짓점 색을 숫자로 비교해 확인).
  5) 내보내기 (data/<이름>/export/)
     visual.obj + visual.mtl + visual_0.png (텍스처), visual.glb (꼭짓점 색), collision.obj
     scene.urdf: PyBullet, Isaac Sim(URDF importer) 등. 바닥에 고정된 링크 하나 (질량 0 = 움직이지 않음)
     scene.sdf + model.config: Gazebo. static 모델 (폴더째 <include> 로 불러온다)
     좌표: 미터, z가 위, 바닥 z = 0 (31번에서 맞춤). glb만 glTF 규격대로 y가 위가 되게 돌려서 저장한다.
     glb는 Gazebo Fortress가 못 읽는다 (Collada, obj, stl만 읽음. glTF는 다음 버전 Garden부터). Isaac Sim, Blender용.
  MuJoCo는 충돌 계산 때 메시를 볼록 껍질(convex hull) 하나로 바꾼다. 장면 전체가 꽉 찬 덩어리가 되므로
  볼록 조각 여러 개로 나누는 단계(볼록 분해)가 따로 필요하다 → 다음 단계.

입력: data/<이름>/normals.ply, inspect.json (31, 32번 결과)
출력: data/<이름>/export/

실행:
  python 34_export.py hall101/ref --gt data/hall101/gt/prepared.ply   # 정답이 있으면 줄일 때마다 정확도도
  python 34_export.py hall101/ref --source bpa --no-view
  python 34_export.py redwood_bedroom --collision 5000   # 삼각형 수를 직접 정함 (기본: 표에서 자동으로 고름)
뷰어: 충돌용 메시를 삼각형 선과 함께 보여 준다 (--view visual 이면 보기용)
"""
import argparse
import importlib
import json

import numpy as np
import open3d as o3d

from common import DATA_DIR

m33 = importlib.import_module("33_reconstruct_eval")  # 파일 이름이 숫자로 시작해서 import 문 대신 씀

# ---- 바꿔볼 만한 파라미터 ----
FLOOR_MARGIN = 0.05        # 바닥에서 이만큼 아래까지는 남긴다 [m]
TRIM_X = 2.0               # 점에서 voxel x TRIM_X 넘게 떨어진 삼각형을 자른다 (voxel 2 cm면 4 cm). 0이면 안 자름
MIN_PIECE = 100            # 이보다 삼각형이 적은 떠 있는 조각은 지운다
TABLE_TRIANGLES = [1_000_000, 300_000, 100_000, 30_000, 10_000, 3_000, 1_000]  # 줄이기 표에 넣을 삼각형 수
COLLISION_TOL = 0.02       # 충돌용: 정리한 메시에서 벗어난 거리 95%가 이 이하 [m]
VISUAL_TOL = 0.005         # 보기용: 같은 기준 [m]
VISUAL_MIN = 100_000       # 보기용 최소 삼각형 수
HOLE_TOL = 0.01            # 줄이면서 바닥 구멍 비율이 이만큼(1%p)까지만 늘어도 된다
N_SAMPLES = 200_000        # 벗어난 거리를 잴 때 정리한 메시 위에서 뽑을 점 수
FLOOR_BAND = 0.05          # 바닥 구멍 검사: 이 높이 아래를 바닥으로 본다 [m]
FLOOR_CELL = 0.05          # 바닥 구멍 검사 칸 크기 [m]
TEXTURE_MAX = 4096         # 텍스처 이미지 한 변의 목표 최대 [px]. 삼각형 칸은 4~16 px (삼각형이 많으면 작아짐)


def clean(mesh, P, voxel):
    """2) 정리. 정리한 메시와 단계별 제거 개수를 돌려준다."""
    mesh = o3d.geometry.TriangleMesh(mesh)
    report = {}
    V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    below = V[T].mean(axis=1)[:, 2] < -FLOOR_MARGIN
    mesh.remove_triangles_by_mask(below)
    report["바닥 아래"] = int(below.sum())
    if TRIM_X > 0:
        V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
        centers = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(V[T].mean(axis=1)))
        far = np.asarray(centers.compute_point_cloud_distance(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))))
        far = far > TRIM_X * voxel
        mesh.remove_triangles_by_mask(far)
        report[f"점에서 {TRIM_X * voxel * 100:.0f} cm 넘게 떨어짐"] = int(far.sum())
    labels, counts, _ = mesh.cluster_connected_triangles()
    small = np.asarray(counts)[np.asarray(labels)] < MIN_PIECE
    mesh.remove_triangles_by_mask(small)
    report[f"{MIN_PIECE}장 미만 조각"] = int(small.sum())
    mesh.remove_unreferenced_vertices()
    mesh.remove_degenerate_triangles()
    mesh.remove_duplicated_triangles()
    mesh.remove_duplicated_vertices()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    return mesh, report


def open_edge_length(mesh):
    """삼각형 하나에만 속한 모서리(= 구멍이나 바깥 가장자리)의 총 길이 [m]."""
    T = np.asarray(mesh.triangles)
    edges = np.sort(np.vstack([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]), axis=1)
    uniq, count = np.unique(edges, axis=0, return_counts=True)
    V = np.asarray(mesh.vertices)
    e = uniq[count == 1]
    return float(np.linalg.norm(V[e[:, 0]] - V[e[:, 1]], axis=1).sum())


def floor_hole_ratio(mesh, P):
    """입력 점에 바닥이 있는 칸 중, 아래에서 쏜 광선이 바닥 높이에서 메시에 맞지 않는 칸의 비율."""
    cells = np.unique(np.floor(P[P[:, 2] < FLOOR_BAND, :2] / FLOOR_CELL).astype(int), axis=0)
    xy = (cells + 0.5) * FLOOR_CELL
    rays = np.c_[xy, np.full(len(xy), -0.5), np.zeros((len(xy), 2)), np.ones(len(xy))].astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    hit_z = -0.5 + scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()  # 못 맞으면 inf
    return float(np.mean(np.abs(hit_z) > FLOOR_BAND))


def decimate(mesh, n):
    if len(mesh.triangles) <= n:
        return o3d.geometry.TriangleMesh(mesh)
    out = mesh.simplify_quadric_decimation(n)
    out.compute_vertex_normals()
    return out


def deviation(samples, mesh):
    """정리한 메시 위 점들이 줄인 메시에서 떨어진 거리 (평균, 95%) [m]."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    d = scene.compute_distance(o3d.core.Tensor(samples, dtype=o3d.core.float32)).numpy()
    return d.mean(), np.percentile(d, 95)


def morton_order(xyz, bits=10):
    """3D 점을 Z자 곡선(Morton) 순서로 줄 세운다: 순서가 가까우면 공간에서도 가깝다."""
    q = ((xyz - xyz.min(axis=0)) / max(np.ptp(xyz, axis=0).max(), 1e-9) * (2 ** bits - 1)).astype(np.uint64)
    code = np.zeros(len(xyz), np.uint64)
    for b in range(bits):  # x, y, z 의 b번째 비트를 번갈아 끼워 넣는다
        for axis in range(3):
            code |= ((q[:, axis] >> np.uint64(b)) & np.uint64(1)) << np.uint64(3 * b + axis)
    return np.argsort(code)


def morton_cell(k, levels):
    """k번째 칸의 (열, 행): 2D Z자 곡선이라 순서가 가까운 칸은 이미지에서도 가까이 놓인다."""
    col, row = np.zeros_like(k), np.zeros_like(k)
    for b in range(levels):
        col |= ((k >> (2 * b)) & 1) << b
        row |= ((k >> (2 * b + 1)) & 1) << b
    return col, row


def bake_texture(mesh):
    """4) 삼각형마다 텍스처에 cell x cell px 칸을 하나씩 주고, 칸 안 직각삼각형 (A 왼쪽 위, B 오른쪽 위, C 왼쪽 아래)에
    세 꼭짓점 색을 무게를 달리해 섞어 칠한다. 칸 가장자리 1 px은 여백(가장 가까운 삼각형 색)으로 둔다.
    공간에서 가까운 삼각형을 이미지에서도 가까운 칸에 둔다. GPU는 멀리 있는 면을 그릴 때 텍스처를 작게 줄여(평균 내어)
    쓰는데, 이웃 칸이 엉뚱한 삼각형이면 그 색이 섞여 얼룩이 된다 (Gazebo에서 확인).
    (텍스처가 붙은 메시, 칸 크기 px, 이미지 한 변 px)를 돌려준다. 이미지 0행이 v = 1 (표준 규칙)."""
    T, C, V = np.asarray(mesh.triangles), np.asarray(mesh.vertex_colors), np.asarray(mesh.vertices)
    levels = int(np.ceil(np.log2(max(np.sqrt(len(T)), 1))))
    per_row = 2 ** levels  # Z자 배치를 위해 2의 거듭제곱
    cell = int(np.clip(TEXTURE_MAX // per_row, 4, 16))
    size = per_row * cell
    # 칸 안 픽셀 중심의 위치 → 세 꼭짓점 무게 (모든 칸이 같아서 한 번만 계산). 삼각형 밖 픽셀은 빗변으로 당긴다
    g = np.clip((np.arange(cell) + 0.5 - 1) / (cell - 2), 0, 1)
    s, t = np.meshgrid(g, g)  # s: 오른쪽(B 쪽), t: 아래쪽(C 쪽)
    over = s + t > 1
    s[over], t[over] = s[over] / (s + t)[over], t[over] / (s + t)[over]
    w = np.stack([1 - s - t, s, t], axis=-1).astype(np.float32)               # (cell, cell, 3)
    slot = np.empty(len(T), np.int64)
    slot[morton_order(V[T].mean(axis=1))] = np.arange(len(T))                # 삼각형 → 몇 번째 칸 (공간 순서)
    col, row = morton_cell(slot, levels)                                      # 그 칸의 이미지 위 자리
    cells = np.zeros((per_row * per_row, cell, cell, 3), np.float32)
    cells[row * per_row + col] = np.einsum("yxk,nkc->nyxc", w, C[T].astype(np.float32))
    img = cells.reshape(per_row, per_row, cell, cell, 3).transpose(0, 2, 1, 3, 4).reshape(size, size, 3)
    # 각 삼각형 꼭짓점의 이미지 위 위치 (px) → UV (u = x / size, v = 1 - y / size)
    x0, y0 = col * cell, row * cell
    px = np.stack([np.c_[x0 + 1, y0 + 1], np.c_[x0 + cell - 1, y0 + 1], np.c_[x0 + 1, y0 + cell - 1]], axis=1)
    uv = np.stack([px[..., 0] / size, 1 - px[..., 1] / size], axis=-1)        # (삼각형, 꼭짓점 3, 2)
    out = o3d.geometry.TriangleMesh(mesh)
    out.triangle_uvs = o3d.utility.Vector2dVector(uv.reshape(-1, 2))
    out.textures = [o3d.geometry.Image(np.ascontiguousarray((np.clip(img, 0, 1) * 255).astype(np.uint8)))]
    out.triangle_material_ids = o3d.utility.IntVector([0] * len(T))
    return out, cell, size


def write_glb(path, mesh):
    """glTF는 y가 위인 좌표계라서 (x, y, z) -> (x, z, -y)로 돌려 저장한다 (불러오는 쪽이 다시 z가 위로 바꾼다)."""
    m = o3d.geometry.TriangleMesh(mesh)
    m.rotate(np.array([[1.0, 0, 0], [0, 0, 1], [0, -1, 0]]), center=(0, 0, 0))
    o3d.t.io.write_triangle_mesh(str(path), o3d.t.geometry.TriangleMesh.from_legacy(m))  # 기존 API의 glb는 깨짐


URDF = """<?xml version="1.0"?>
<!-- 바닥에 고정된 장면 하나. 질량 0 = 움직이지 않는 물체 (PyBullet), Isaac Sim은 불러올 때 'Fix Base Link' -->
<robot name="{name}">
  <link name="scene">
    <inertial>
      <mass value="0"/>
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0"/>
    </inertial>
    <visual>
      <geometry><mesh filename="visual.obj"/></geometry>
    </visual>
    <!-- concave="yes": PyBullet 전용. 오목한 삼각형 메시를 볼록 껍질로 바꾸지 않고 그대로 충돌에 씀. 다른 프로그램은 무시 -->
    <collision concave="yes">
      <geometry><mesh filename="collision.obj"/></geometry>
    </collision>
  </link>
</robot>
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<!-- Gazebo가 이 폴더를 '모델'로 알아보게 하는 파일. 월드에서 <include><uri>이 폴더 경로</uri></include> 로 불러온다 -->
<model>
  <name>{name}</name>
  <version>1.0</version>
  <sdf version="1.9">scene.sdf</sdf>
  <description>pc2mesh 34_export.py</description>
</model>
"""

SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="scene">
      <visual name="visual">
        <geometry><mesh><uri>visual.obj</uri></mesh></geometry>
      </visual>
      <collision name="collision">
        <geometry><mesh><uri>collision.obj</uri></mesh></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (32번까지 마친 것)")
    parser.add_argument("--source", choices=["poisson_trim", "poisson_raw", "bpa"], default="poisson_trim",
                        help="재구성 방법 (기본: Poisson trim. 구멍이 적어 충돌용에 안전)")
    parser.add_argument("--visual", type=int, help="보기용 목표 삼각형 수 (기본: 표에서 VISUAL_TOL 기준으로 고름)")
    parser.add_argument("--collision", type=int, help="충돌용 목표 삼각형 수 (기본: 표에서 COLLISION_TOL 기준으로 고름)")
    parser.add_argument("--gt", help="정답 점 파일 (있으면 줄일 때마다 33번과 같은 정확도·완성도를 잼)")
    parser.add_argument("--view", choices=["collision", "visual"], default="collision", help="뷰어로 볼 메시")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    args = parser.parse_args()

    folder = DATA_DIR / args.name
    info = json.loads((folder / "inspect.json").read_text())
    voxel, depth = info["voxel"], info["recommended"]["poisson_depth"]
    pcd = o3d.io.read_point_cloud(str(folder / "normals.ply"))
    P = np.asarray(pcd.points)

    # 1) 재구성
    raw = m33.reconstruct(pcd, depth, voxel)[args.source][0]
    print(f"[재구성] {args.source}: 삼각형 {len(raw.triangles):,}개 | 열린 가장자리 {open_edge_length(raw):.1f} m, "
          f"바닥 구멍 {floor_hole_ratio(raw, P):.1%}")

    # 2) 정리
    mesh, report = clean(raw, P, voxel)
    print(f"[정리] 지운 삼각형: " + ", ".join(f"{k} {v:,}" for k, v in report.items()))
    print(f"  → 삼각형 {len(mesh.triangles):,}개 | 열린 가장자리 {open_edge_length(mesh):.1f} m, "
          f"바닥 구멍 {floor_hole_ratio(mesh, P):.1%}")

    # 3) 줄이기
    gt = None
    if args.gt:
        gt = o3d.io.read_point_cloud(args.gt)
        G = np.asarray(gt.points)
        inside, _ = m33.footprint(G)
        gt_eval = G[inside(G[:, :2])]          
    samples = np.asarray(mesh.sample_points_uniformly(N_SAMPLES).points)
    head = "  정확도  완성도      F" if gt else ""
    print(f"\n[줄이기] {'삼각형':>9} | 정리한 메시에서 벗어난 거리 평균 / 95% | 바닥 구멍{head}")
    rows, prev = [], mesh
    for label, n in [("정리 후", None)] + [(f"{n:,}", n) for n in TABLE_TRIANGLES if n < len(mesh.triangles)]:
        m = mesh if n is None else decimate(prev, n)  # 바로 앞 단계에서 이어서 줄인다 (큰 메시에서 빠름)
        prev = m
        mean, p95 = deviation(samples, m)
        holes = floor_hole_ratio(m, P)
        rows.append((m, p95, holes))
        line = f"  {label:>9} ({len(m.triangles):>9,}) | {mean * 1000:6.1f} mm / {p95 * 1000:6.1f} mm | {holes:7.1%}"
        if gt:
            acc, comp, f = m33.evaluate(m, gt, gt_eval, inside)
            line += f" {acc:>7.1%}{comp:>7.1%}{f:>7.1%}"
        print(line)

    def pick(tol, min_triangles=0):
        """벗어난 거리 95%가 tol 이하, 바닥 구멍이 HOLE_TOL 넘게 안 늘고, min_triangles 이상인 것 중 가장 작은 메시."""
        ok = [m for m, p95, holes in rows
              if p95 <= tol and holes <= rows[0][2] + HOLE_TOL and len(m.triangles) >= min(min_triangles, len(mesh.triangles))]
        return min(ok, key=lambda m: len(m.triangles))

    visual = decimate(mesh, args.visual) if args.visual else pick(VISUAL_TOL, VISUAL_MIN)
    collision = decimate(mesh, args.collision) if args.collision else pick(COLLISION_TOL)
    collision = o3d.geometry.TriangleMesh(collision)
    collision.vertex_colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))  # 충돌용은 색이 필요 없다
    print(f"[선택] 보기용 {len(visual.triangles):,}개 "
          f"({'직접 지정' if args.visual else f'벗어난 거리 95% ≤ {VISUAL_TOL * 1000:.0f} mm, {VISUAL_MIN:,}개 이상'}), "
          f"충돌용 {len(collision.triangles):,}개 "
          f"({'직접 지정' if args.collision else f'≤ {COLLISION_TOL * 100:.0f} cm, 바닥 구멍 +{HOLE_TOL:.0%}p 이내'})")

    # 4) 색을 텍스처로 굽기
    textured = None
    if visual.has_vertex_colors():
        textured, cell, size = bake_texture(visual)
        print(f"\n[텍스처] {size} x {size} px 이미지, 삼각형마다 {cell} x {cell} px 칸에 색을 칠함")

    # 5) 내보내기
    out = folder / "export"
    out.mkdir(exist_ok=True)
    for m in (visual, collision, textured):
        if m is not None:
            m.triangle_normals = o3d.utility.Vector3dVector(np.zeros((0, 3)))  # obj는 꼭짓점 법선만 담는다 (경고 방지)
    o3d.io.write_triangle_mesh(str(out / "visual.obj"), textured or visual)  # 텍스처가 있으면 visual.mtl, visual_0.png도
    if textured:  # Open3D가 뒤집어 쓴 png를 표준 규칙대로 다시 쓴다 (Gazebo, PyBullet이 읽는 방향)
        o3d.io.write_image(str(out / "visual_0.png"), textured.textures[0])
    write_glb(out / "visual.glb", visual)
    o3d.io.write_triangle_mesh(str(out / "collision.obj"), collision)
    name = args.name.replace("/", "_")
    (out / "scene.urdf").write_text(URDF.format(name=name))
    (out / "scene.sdf").write_text(SDF.format(name=name))
    (out / "model.config").write_text(MODEL_CONFIG.format(name=name))
    print(f"\n[내보내기] 보기용 삼각형 {len((textured or visual).triangles):,}개, 충돌용 {len(collision.triangles):,}개 → {out}/")
    print(f"  visual.obj{' + visual.mtl + visual_0.png' if textured else ''}, visual.glb, collision.obj, "
          "scene.urdf (PyBullet, Isaac Sim), scene.sdf + model.config (Gazebo)")
    print("  MuJoCo는 볼록 분해가 필요해서 아직 없음")

    if not args.no_view:
        m = visual if args.view == "visual" else collision
        if args.view == "collision":
            m.paint_uniform_color((0.75, 0.75, 0.8))
        o3d.visualization.draw_geometries([m], window_name=f"{args.name}: {args.view} ({len(m.triangles):,} triangles)",
                                          mesh_show_wireframe=args.view == "collision", mesh_show_back_face=True)


if __name__ == "__main__":
    main()
