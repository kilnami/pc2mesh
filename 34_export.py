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
     충돌용에만: 줄인 뒤 바닥을 패치로 덮는다 (--no-floor-patch 로 끈다). 두 가지 칸을 덮는다.
       구멍: 입력 점에 바닥이 있는데 메시가 없는 칸 (floor_hole_ratio 가 세는 칸)
       못 본 바닥: 바닥 점이 아예 없지만 점(어느 높이든)으로 둘러싸인 범위 안인 칸. 물체 밑, 센서가 못 본 곳. 바닥이 평평하다고 보고
         바닥이 보이는 곳에서 FILL_MAX 이내만 메운다. 물체를 따로 빼면 그 밑이 구멍이 되기 때문이다
       칸마다 사각형을 깔고 높이는 멀쩡한 칸의 메시 바닥 높이를 안쪽으로 번지게 한다. 큰 빈 자리는 큰 정사각형 하나로 깐다.
       바닥 점 높이는 안 쓴다 (벽 밑단·물체 아랫부분이 섞여 기존 바닥보다 최대 6 cm 높게 나왔다).
       받침 상자와 달리 기존 바닥 높이에 두께 없는 면을 얹으므로 메시의 최저 높이가 바뀌지 않는다
     충돌용·보기용 둘 다: 벽 치마 (--no-wall-skirt 로 끈다). 벽은 바닥까지 이어져 있는 게 당연한데, 스캔은 기계·팔레트 뒤의 벽 밑을 못 본다
       (공장 동쪽 벽은 원본에서도 밑단 칸의 33%에 점이 없다). 점군에서 큰 수직 벽 평면을 찾고(find_walls) 벽을 10 cm 칸으로 펼쳐서, 열마다
       벽이 처음 나오는 높이가 바닥에서 떠 있으면 그 사이를 바닥(z = 0)까지 얇은 면으로 채운다. 떠 있는 높이가 SKIRT_MAX(1 m)를 넘는 열은
       문 같은 열린 곳으로 보고 두고, 처음 나온 벽 위로 SKIRT_MIN_RUN 칸이 이어져야 벽으로 본다. 보기용에는 벽 가장자리 색을 입힌다
       (라이다는 충돌용이 아니라 보기용을 본다)
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
PATCH_RING = 1             # 바닥 패치: 구멍 칸 둘레로 이만큼 칸을 더 깐다 (기존 면과 겹쳐서 틈을 없앤다)
PATCH_BLOCKS = (8, 4)      # 바닥 패치: 큰 빈 자리는 이 칸 수의 정사각형으로 깐다 (삼각형 수를 줄인다)
FILL_MAX = 2.0             # 바닥 점이 없는 칸(물체 밑)은 바닥이 보이는 곳에서 이 거리 안까지만 메운다 [m]
FILL_ITER = int(FILL_MAX / FLOOR_CELL)
WALL_BAND = 0.06           # 벽 치마: 벽 평면에서 이 안의 점을 벽 점으로 본다 [m]
WALL_MIN_POINTS = 5000     # 벽 치마: 평면 하나로 인정하는 벽 점 수 (바닥 위 0.3~2 m, 법선이 수평)
WALL_MIN_WIDTH = 3.0       # 벽 치마: 평면의 폭이 이 이상이어야 벽으로 본다 [m]
WALL_MIN_FILL = 0.5        # 벽 치마: 벽 범위(폭 x 바닥 위 0.3~2 m)를 10 cm 칸으로 나눴을 때 점이 있는 칸이 이 비율 이상이어야 벽으로 본다
WALL_CELL = 0.10           # 벽 치마: 벽을 펼친 칸 크기 [m]
SKIRT_MAX = 1.0            # 벽 치마: 벽이 처음 나오는 높이가 이 이하일 때만 바닥까지 내린다 [m]. 더 높이 떠 있으면 문 같은 열린 곳으로 보고 둔다
SKIRT_MIN_RUN = 3          # 벽 치마: 처음 나온 벽 위로 이만큼(칸) 이어져야 벽으로 본다 (떨어진 조각 하나는 벽이 아니다)
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


def floor_cells(P):
    """입력 점에 바닥(FLOOR_BAND 아래)이 있는 칸. 칸 정수 좌표 (N, 2)와 칸 안 바닥 점 높이의 중앙값 (N,)을 돌려준다."""
    low = P[P[:, 2] < FLOOR_BAND]
    cells, inv = np.unique(np.floor(low[:, :2] / FLOOR_CELL).astype(int), axis=0, return_inverse=True)
    inv = inv.ravel()
    order = np.lexsort((low[:, 2], inv))             # 칸별로 모으고 칸 안에서는 높이순
    start = np.concatenate([[0], np.cumsum(np.bincount(inv))[:-1]])
    mid = start + np.bincount(inv) // 2
    return cells, low[order, 2][mid]


def floor_hit_z(mesh, cells):
    """칸 중심에서 아래로부터 쏜 광선이 메시에 처음 맞는 높이. 못 맞으면 inf."""
    xy = (cells + 0.5) * FLOOR_CELL
    rays = np.c_[xy, np.full(len(xy), -0.5), np.zeros((len(xy), 2)), np.ones(len(xy))].astype(np.float32)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return -0.5 + scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()


def floor_hole_ratio(mesh, P):
    """입력 점에 바닥이 있는 칸 중, 아래에서 쏜 광선이 바닥 높이에서 메시에 맞지 않는 칸의 비율."""
    return float(np.mean(np.abs(floor_hit_z(mesh, floor_cells(P)[0])) > FLOOR_BAND))


def unseen_floor_cells(P, seen):
    """점(어느 높이든)으로 둘러싸인 범위 안인데 바닥 점이 없는 칸 (물체 밑, 센서가 못 본 바닥). 칸 정수 좌표 (M, 2).
    점이 하나라도 있는 칸을 벽으로 보고, 격자 가장자리(점이 없는 여백)에서 4방향으로 번지는 빈 칸을 바깥으로 센다.
    바깥에 닿지 않는 빈 칸이 안쪽이다. 지붕·천장이 스캔돼 있으면 방 전체가 점으로 덮여서 안쪽이 곧 방 안이다.
    seen: floor_cells 가 돌려준 칸 (바닥 점이 있는 칸은 뺀다)."""
    ij = np.floor(P[:, :2] / FLOOR_CELL).astype(int)
    lo = ij.min(axis=0) - 1
    ij -= lo
    shape = ij.max(axis=0) + 2                        # 가장자리 한 칸은 늘 점이 없는 여백
    occupied = np.zeros(shape, bool)
    occupied[ij[:, 0], ij[:, 1]] = True
    outside = np.zeros(shape, bool)
    outside[[0, -1], :] = True
    outside[:, [0, -1]] = True
    while True:
        grown = outside.copy()
        grown[1:] |= outside[:-1]
        grown[:-1] |= outside[1:]
        grown[:, 1:] |= outside[:, :-1]
        grown[:, :-1] |= outside[:, 1:]
        grown &= ~occupied
        if (grown == outside).all():
            break
        outside = grown
    inside = ~outside
    inside[seen[:, 0] - lo[0], seen[:, 1] - lo[1]] = False
    return np.argwhere(inside) + lo


def full_blocks(mask, s, lo):
    """mask 에서 s x s 칸이 모두 True 인 정사각형의 왼쪽 아래 칸 (a, b). 칸 번호가 s 의 배수인 자리에서만 잡는다 (이웃 타일과 경계가 맞게)."""
    nx, ny = mask.shape
    I = np.zeros((nx + 1, ny + 1), int)
    I[1:, 1:] = mask.cumsum(0).cumsum(1)
    a = np.arange(0, nx - s + 1)
    b = np.arange(0, ny - s + 1)
    a, b = a[(lo[0] + a) % s == 0], b[(lo[1] + b) % s == 0]
    n = I[a[:, None] + s, b[None, :] + s] - I[a[:, None], b[None, :] + s] - I[a[:, None] + s, b[None, :]] + I[a[:, None], b[None, :]]
    ia, ib = np.nonzero(n == s * s)
    return np.c_[a[ia], b[ib]]


def floor_patch(mesh, P, bounds=None):
    """바닥을 덮는 패치 메시와 정보 {holes, unseen}를 돌려준다. 메시에 합치지는 않는다. 덮을 게 없으면 패치는 None.
    덮는 칸 두 가지 (칸 = FLOOR_CELL):
      구멍: 입력 점에 바닥이 있는데 메시가 없는 칸 (floor_hole_ratio 가 세는 칸)
      못 본 바닥: 바닥 점이 아예 없는데 점으로 둘러싸인 범위 안인 칸 (물체 밑 등. unseen_floor_cells).
                  바닥이 보이는 곳에서 FILL_MAX 이내만. 바닥이 평평하다고 보고 주변 높이를 이어 깐다
    구멍 칸과 못 본 칸의 둘레 PATCH_RING 칸 중 바닥 점이 있는 칸까지 같이 깐다.
    둘레까지 까는 이유: 구멍 판정은 칸 중심 한 점이라서 실제 구멍 가장자리는 칸 경계와 안 맞는다. 멀쩡한 면과 겹쳐 틈이 안 남게 한다.
    높이: 멀쩡한 칸에서 잰 메시 바닥 높이를 안쪽으로 번지게 한다 (칸 모서리마다 둘레 네 칸의 평균, 빈 모서리는 이웃 모서리에서 한 겹씩 채움).
    입력 점 높이는 쓰지 않는다: 바닥 점에 벽 밑단과 물체 아랫부분이 섞여 있어서 기존 바닥보다 최대 6 cm 높게 나왔다 (합성 방).
    삼각형 수: 큰 빈 자리는 PATCH_BLOCKS 칸 정사각형 하나(삼각형 2장)로, 나머지는 칸마다 깐다.
    bounds [x0, y0, x1, y1] 를 주면 그 안(과 한 칸 바깥)의 칸만 본다 (타일: 메시가 경계에서 잘려 있어서 그 밖은 구멍으로 잘못 잡힌다)."""
    r = PATCH_RING
    cells, _ = floor_cells(P)
    unseen = unseen_floor_cells(P, cells)

    def in_region(c):
        if bounds is None:
            return np.ones(len(c), bool)
        xy = (c + 0.5) * FLOOR_CELL
        return ((xy[:, 0] >= bounds[0] - FLOOR_CELL) & (xy[:, 0] <= bounds[2] + FLOOR_CELL) &
                (xy[:, 1] >= bounds[1] - FLOOR_CELL) & (xy[:, 1] <= bounds[3] + FLOOR_CELL))
    cells, unseen = cells[in_region(cells)], unseen[in_region(unseen)]
    hit = floor_hit_z(mesh, cells)
    hole = np.abs(hit) > FLOOR_BAND
    if not hole.any() and len(unseen) == 0:
        return None, dict(holes=0, unseen=0)
    both = np.vstack([cells, unseen])
    lo = both.min(axis=0)
    nx, ny = both.max(axis=0) - lo + 1
    shape = (nx + 2 * r, ny + 2 * r)                  # 둘레 r 칸 여백
    at = lambda c: (c[:, 0] - lo[0] + r, c[:, 1] - lo[1] + r)
    seen_mask, unseen_mask, cover = (np.zeros(shape, bool) for _ in range(3))
    seen_mask[at(cells)] = True
    unseen_mask[at(unseen)] = True
    cover[at(cells[hole])] = True
    cover[at(unseen)] = True
    H = np.full(shape, np.nan)                        # 메시가 바닥을 덮고 있는 칸의 바닥 높이 (구멍·못 본 칸은 NaN)
    H[at(cells[~hole])] = hit[~hole]
    grown = cover.copy()
    for dx in range(-r, r + 1):
        for dy in range(-r, r + 1):
            grown[r + dx:r + dx + nx, r + dy:r + dy + ny] |= cover[r:r + nx, r:r + ny]
    grown &= seen_mask | unseen_mask                  # 점이 없고 범위 밖인 칸(건물 밖)은 깔지 않는다

    known = ~np.isnan(H)
    S, N = np.where(known, H, 0.0), known.astype(float)
    corner = lambda A: A[:-1, :-1] + A[1:, :-1] + A[:-1, 1:] + A[1:, 1:]   # 모서리 둘레 네 칸의 합
    cz = np.where(corner(N) > 0, corner(S) / np.maximum(corner(N), 1), np.nan)[r - 1:r + nx, r - 1:r + ny]   # (nx + 1, ny + 1), 칸 (i, j) 의 왼쪽 아래 모서리 = (i, j)
    for _ in range(FILL_ITER):                        # 높이를 모르는 모서리는 이웃 모서리 평균으로 한 겹씩 채운다
        pad = np.pad(cz, 1, constant_values=np.nan)
        near = np.stack([pad[1 + dx:pad.shape[0] - 1 + dx, 1 + dy:pad.shape[1] - 1 + dy]
                         for dx in (-1, 0, 1) for dy in (-1, 0, 1) if (dx, dy) != (0, 0)])
        ok = ~np.isnan(near)
        new = np.isnan(cz) & ok.any(axis=0)
        if not new.any():
            break
        cz = np.where(new, np.where(ok, near, 0.0).sum(axis=0) / np.maximum(ok.sum(axis=0), 1), cz)
    good = ~np.isnan(cz)
    inner = grown[r:r + nx, r:r + ny] & good[:-1, :-1] & good[1:, :-1] & good[:-1, 1:] & good[1:, 1:]   # 네 모서리 높이를 아는 칸만
    info = dict(holes=int(hole.sum()), unseen=int((inner & unseen_mask[r:r + nx, r:r + ny]).sum()))
    if not inner.any():
        return None, info

    quads, left = [], inner.copy()
    for s in PATCH_BLOCKS:                            # 큰 정사각형부터 깔고 남은 칸은 칸마다
        blocks = full_blocks(left, s, lo)
        for a, b in blocks:
            left[a:a + s, b:b + s] = False
        quads.append((blocks, s))
    quads.append((np.argwhere(left), 1))
    origin = np.vstack([q for q, _ in quads])
    size = np.concatenate([np.full(len(q), s) for q, s in quads])
    corners = origin[:, None, :] + size[:, None, None] * np.array([[0, 0], [1, 0], [1, 1], [0, 1]])   # 위에서 보아 반시계
    flat = corners[..., 0] * (ny + 1) + corners[..., 1]
    used, quad = np.unique(flat, return_inverse=True)
    quad = quad.reshape(-1, 4)
    ca, cb = used // (ny + 1), used % (ny + 1)
    V = np.c_[(lo[0] + ca) * FLOOR_CELL, (lo[1] + cb) * FLOOR_CELL, cz[ca, cb]]
    patch = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V),
                                      o3d.utility.Vector3iVector(np.vstack([quad[:, [0, 1, 2]], quad[:, [0, 2, 3]]])))
    patch.compute_vertex_normals()
    return patch, info


def add_floor_patch(collision, P, bounds=None):
    """충돌용 메시에 바닥 패치를 합친다. (합친 메시, 패치 삼각형 수, floor_patch 의 정보)를 돌려준다. 덮을 곳이 없으면 그대로."""
    patch, info = floor_patch(collision, P, bounds)
    if patch is None:
        return collision, 0, info
    merged = o3d.geometry.TriangleMesh(collision)
    merged += patch
    return merged, len(patch.triangles), info


def find_walls(P, N):
    """점군에서 큰 수직 벽 평면을 찾는다: [{n, t, d, inward, u0, u1, top}].
    벽 점 = 법선이 거의 수평이고 바닥 위 0.3~2 m. 법선 방향의 가장 많은 쪽과 거기에 직교하는 쪽(직각 건물 가정)에서,
    평면까지의 거리를 2 cm 칸으로 세어 점이 몰린 곳(WALL_MIN_POINTS 이상)을 평면으로 본다. 폭이 WALL_MIN_WIDTH 이상이고, 벽 범위 중 점이 있는 칸이
    WALL_MIN_FILL 이상이어야 한다 (진짜 벽은 62~100%, 인천 지하철 역사의 기둥 줄과 공장의 기계 면은 9~29%였다).
    n: 법선(xy), t: 벽을 따라가는 방향, d: P·n 의 값, inward: 방 안쪽을 가리키는 부호, u0~u1: 벽의 범위, top: 벽 높이."""
    h = (np.abs(N[:, 2]) < 0.25) & (P[:, 2] > 0.3) & (P[:, 2] < 2.0)
    if h.sum() < WALL_MIN_POINTS:
        return []
    ang = np.degrees(np.arctan2(N[h, 1], N[h, 0])) % 180
    hist, _ = np.histogram(ang, bins=180, range=(0, 180))
    smooth = sum(np.roll(hist, k) for k in range(-2, 3))     # 0도와 180도는 같은 방향이라 원형으로 합친다
    a0 = float(np.argmax(smooth)) + 0.5
    walls = []
    for a in (a0, (a0 + 90) % 180):
        n = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        t = np.array([-n[1], n[0]])
        da = np.abs(((np.degrees(np.arctan2(N[:, 1], N[:, 0])) - a + 90) % 180) - 90)
        dist = P[h & (da < 8), :2] @ n
        if len(dist) < WALL_MIN_POINTS:
            continue
        counts, edges = np.histogram(dist, bins=np.arange(dist.min(), dist.max() + 0.02, 0.02))
        smooth = np.convolve(counts, np.ones(3), "same")
        peaks = []
        for i in np.argsort(smooth)[::-1]:
            if smooth[i] < WALL_MIN_POINTS:
                break
            c = (edges[i] + edges[i + 1]) / 2
            if all(abs(c - q) > 0.3 for q in peaks):
                peaks.append(c)
        for c in peaks:
            near = np.abs(P[:, :2] @ n - c) < WALL_BAND
            u, z = P[near, :2] @ t, P[near, 2]
            u0, u1, top = np.percentile(u, 0.5), np.percentile(u, 99.5), np.percentile(z, 99)
            if u1 - u0 < WALL_MIN_WIDTH or top < 1.5:
                continue
            nu, nv = int((u1 - u0) / WALL_CELL) + 1, int(1.7 / WALL_CELL)       # 벽 범위 중 점이 있는 칸의 비율: 같은 평면에 늘어선 기둥·기계 면은 듬성듬성하다
            filled = np.zeros((nu, nv), bool)
            iu, iv = ((u - u0) / WALL_CELL).astype(int), ((z - 0.3) / WALL_CELL).astype(int)
            ok = (iu >= 0) & (iu < nu) & (iv >= 0) & (iv < nv)
            filled[iu[ok], iv[ok]] = True
            if filled.mean() < WALL_MIN_FILL:
                continue
            d = float(np.mean(P[near, :2] @ n))
            inward = 1.0 if (P[:, :2].mean(axis=0) - d * n) @ n >= 0 else -1.0
            walls.append(dict(n=n, t=t, d=d, inward=inward, u0=float(u0), u1=float(u1), top=float(min(top, 4.0))))
    return walls


def wall_skirt(mesh, walls):
    """벽 밑이 떠 있는 곳을 바닥(z = 0)까지 내리는 얇은 면 (치마)과 정보 {columns, length, refused}를 돌려준다. 없으면 치마는 None.
    벽을 WALL_CELL 칸으로 펼쳐서, 평면 안쪽 15 cm에서 벽 쪽으로 광선을 쏴 메시가 벽 부근(-12~+10 cm)에 있는 칸을 찾는다.
    열마다 처음 나오는 칸(아래에서부터)이 바닥에서 떠 있고, 그 위로 SKIRT_MIN_RUN 칸 이어지고, 떠 있는 높이가 SKIRT_MAX 이하이면
    그 열을 바닥까지 채운다. 더 높이 떠 있는 열은 문 같은 열린 곳일 수 있어 두고 refused 로 센다.
    치마의 위치는 그 열에서 처음 나온 벽의 실제 위치를 따르고, 이웃한 열과 가장자리를 같이 쓴다 (틈 없이 이어짐)."""
    sc = o3d.t.geometry.RaycastingScene()
    sc.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    V, T = [], []
    cols = refused = 0
    length = 0.0
    for w in walls:
        n, t, d, inward = w["n"], w["t"], w["d"], w["inward"]
        us = np.arange(w["u0"], w["u1"], WALL_CELL) + WALL_CELL / 2
        vs = np.arange(0.0, w["top"], WALL_CELL) + WALL_CELL / 2
        U, Vv = np.meshgrid(us, vs, indexing="ij")
        xy = (d * n)[None, :] + inward * 0.15 * n[None, :] + (U.reshape(-1, 1) - (d * n) @ t) * t[None, :]
        rays = np.c_[xy, Vv.reshape(-1), np.tile(-inward * n, (U.size, 1)), np.zeros(U.size)].astype(np.float32)
        th = sc.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy().reshape(U.shape)
        cover = np.isfinite(th) & (th > 0.03) & (th < 0.25)
        info_col = {}                                           # 열 번호 -> (벽 위치 offset, 치마 위 높이)
        for k in range(len(us)):
            idx = np.flatnonzero(cover[k])
            if len(idx) == 0 or idx[0] == 0:                    # 벽이 없거나 바닥에서 이미 시작
                continue
            i0 = int(idx[0])
            if not cover[k, i0:i0 + SKIRT_MIN_RUN].all():
                continue
            if i0 * WALL_CELL > SKIRT_MAX:
                refused += 1
                continue
            info_col[k] = (d + inward * (0.15 - th[k, i0]), i0 * WALL_CELL + WALL_CELL / 2)   # 치마 윗면은 벽 안으로 반 칸 겹친다
        for k, (off, top) in info_col.items():
            def edge(j):                                        # 열 경계 위치: 이웃 치마 열이 있으면 평균
                offs = [info_col[q][0] for q in (j - 1, j) if q in info_col]
                return float(np.mean(offs))
            pts = []
            for j, z in ((k, 0.0), (k + 1, 0.0), (k + 1, top), (k, top)):
                u = w["u0"] + j * WALL_CELL
                xyz = edge(j) * n + u * t
                pts.append([xyz[0], xyz[1], z])
            base = len(V)
            V.extend(pts)
            quad = [0, 1, 2, 3] if inward > 0 else [1, 0, 3, 2]
            T.extend([[base + quad[0], base + quad[1], base + quad[2]], [base + quad[0], base + quad[2], base + quad[3]]])
            cols += 1
            length += WALL_CELL
    info = dict(columns=cols, length=length, refused=refused, walls=len(walls))
    if not V:
        return None, info
    skirt = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(np.array(V)), o3d.utility.Vector3iVector(np.array(T)))
    skirt.compute_vertex_normals()
    return skirt, info


def color_skirt(skirt, visual):
    """치마 꼭짓점에 보기용 메시의 벽 색을 입힌다. 치마는 열마다 꼭짓점 4개(왼쪽 아래, 오른쪽 아래, 오른쪽 위, 왼쪽 위)씩 이어져 있다.
    위쪽 꼭짓점(벽 안으로 반 칸 들어간 자리)에서 가장 가까운 보기용 꼭짓점의 색을 쓰고, 아래쪽 꼭짓점은 같은 열 위쪽 것을 따라간다
    (바닥 가까이에서 찾으면 바닥 색이 묻는다)."""
    V = np.asarray(skirt.vertices).reshape(-1, 4, 3)
    tree = o3d.geometry.KDTreeFlann(o3d.geometry.PointCloud(visual.vertices))
    vc = np.asarray(visual.vertex_colors)
    colors = np.zeros((len(V), 4, 3))
    for i, quad in enumerate(V):
        for j in (2, 3):
            colors[i, j] = vc[tree.search_knn_vector_3d(quad[j], 1)[1][0]]
        colors[i, 0], colors[i, 1] = colors[i, 3], colors[i, 2]
    skirt.vertex_colors = o3d.utility.Vector3dVector(colors.reshape(-1, 3))
    return skirt


def add_wall_skirt(collision, walls, visual=None):
    """충돌용 메시에 벽 치마를 합친다. visual 을 주면 같은 치마에 벽 색을 입혀 보기용에도 합친다.
    (합친 충돌용, 합친 보기용 또는 None, 치마 삼각형 수, wall_skirt 의 정보)를 돌려준다. 메울 곳이 없으면 그대로.
    라이다는 충돌용이 아니라 보기용 메시를 본다고 알고 있어서, 라이다에도 벽이 보이려면 보기용에도 필요하다."""
    skirt, info = wall_skirt(collision, walls)
    if skirt is None:
        return collision, visual, 0, info
    merged = o3d.geometry.TriangleMesh(collision)
    merged += skirt
    vis = None
    if visual is not None:
        colored = color_skirt(o3d.geometry.TriangleMesh(skirt), visual)
        vis = o3d.geometry.TriangleMesh(visual)
        vis += colored
    return merged, vis, len(skirt.triangles), info


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
    parser.add_argument("--no-floor-patch", action="store_true", help="충돌용 메시의 바닥 구멍 패치를 넣지 않는다")
    parser.add_argument("--no-wall-skirt", action="store_true", help="벽 밑을 바닥까지 내리는 벽 치마를 넣지 않는다")
    parser.add_argument("--gt", help="정답 점 파일 (있으면 줄일 때마다 33번과 같은 정확도·완성도를 잼)")
    parser.add_argument("--view", choices=["collision", "visual"], default="collision", help="뷰어로 볼 메시")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    args = parser.parse_args()
    o3d.utility.random.seed(0)   # 면 위에서 점을 뽑는 계산이 무작위라서, 같은 입력이면 표와 삼각형 수 선택이 같게 고정

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
    if not args.no_floor_patch:   # 줄인 뒤의 충돌용 메시에서 구멍을 다시 찾는다 (줄이면서 생긴 구멍도 덮는다)
        before = floor_hole_ratio(collision, P)
        collision, n_patch, info = add_floor_patch(collision, P)
        print(f"[바닥 패치] 충돌용 바닥 구멍 {before:.1%} → {floor_hole_ratio(collision, P):.1%}, "
              f"바닥 점이 없는 자리(물체 밑 등) {info['unseen'] * FLOOR_CELL ** 2:.1f} ㎡ 메움 "
              f"(삼각형 +{n_patch:,}개 → {len(collision.triangles):,}개). 보기용에는 넣지 않는다")
    if not args.no_wall_skirt:   # 벽 밑이 떠 있는 곳을 바닥까지 내린다. 충돌용과 보기용 둘 다 (라이다는 보기용을 본다)
        walls = find_walls(P, np.asarray(pcd.normals))
        collision, visual, n_skirt, winfo = add_wall_skirt(collision, walls, visual if visual.has_vertex_colors() else None)
        print(f"[벽 치마] 벽 {winfo['walls']}장에서 바닥에서 떠 있는 열 {winfo['columns']}개 (폭 {winfo['length']:.1f} m)를 바닥까지 채움, "
              f"{SKIRT_MAX:g} m 넘게 떠서 둔 열 {winfo['refused']}개 (문 같은 열린 곳). 삼각형 +{n_skirt:,}개 (충돌용·보기용 각각)")

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
