"""
3단계-2: 방 밖 점 자르기 + 이상치 제거 + 법선 (정답 없이 부호 검증)

31_prepare.py 결과를 이어받는다. 반경과 부호 방식은 inspect.json의 추천값을 쓴다.
  0) 장면 종류 판별 (--scene auto): 맨 위의 수평 면(천장 후보)이 바닥 면적의 ROOM_COVER 이상을 덮으면 '방',
     아니면 '열린 장면'(벽·천장 없이 잘라낸 조각, 문이 열린 공간 등)으로 본다.
     장면 종류는 1) 자르기와 가상 센서를 세울 범위에만 쓴다. 부호를 정하는 규칙은 방이든 열린 장면이든 같다
     ('면의 바깥은 스캐너가 본 쪽'은 어디서나 같고, 스캐너가 설 수 있는 곳은 '바닥 위'로 정했으므로).
  1) 방 밖 점 자르기: 천장 점들의 윤곽(볼록 껍질)을 방 범위로 보고, 그 바깥과 바닥 아래·천장 위를 자른다.
     창문 유리 너머, 열린 문 밖, 반사로 생긴 점들이 여기서 사라진다.
     (가구가 벽을 가려도 천장은 보통 다 보이므로 바닥보다 천장 윤곽이 믿을 만하다)
     열린 장면은 천장이 없으니 바닥 아래만 자른다.
  2) 통계적 이상치 제거: 2-2와 같은 방법. 지워진 점이 어디 몰렸는지 높이별로 보고한다.
  3) 법선: 추천 반경으로 축을 구하고 부호를 정한다.
  4) 정답 없는 부호 검증: 방 안 빈 공간(점에서 충분히 떨어진 곳)에 '가상 센서'를 세우고,
     가상 센서 기준(sensor 방식) 결과와 tangent 결과가 서로 다른 점을 의심 지역으로 표시한다.
     둘 중 하나는 틀렸다는 뜻이므로, 그 지역을 눈으로 확인하면 된다.
     sensor 방식은 '가장 가까운 센서'가 아니라 '그 점에서 보이는(시선이 다른 표면에 막히지 않는)
     가장 가까운 센서' 쪽으로 뒤집는다. 가림은 점들을 OCC_CELL 격자에 채워 넣고 선분이 채워진 칸을
     지나는지로 판단한다. 가장 가까운 센서만 쓰면 책상 위의 센서가 책상 옆면을 책상 안쪽으로 뒤집는다.
     가상 센서는 바로 아래(FLOOR_NEAR 안)에 바닥 점(높이 SENSOR_FLOOR_Z 아래의 수평면)이 있는 곳에만 세운다.
     스캐너는 보이는 바닥 위에 서 있다. (0.5 m 칸 안 어디든 바닥 점이 있으면 되던 예전 기준으로는 윗면이 안
      찍힌 책상 속에 센서가 서서 책상 옆면을 안쪽으로 뒤집었다)
     (처음에는 열린 장면이면 장면 둘레 1 m 바깥까지 세웠는데, 공장에서 벽 바깥에 선 센서 때문에 벽 전체가
      바깥을 향했다. 바닥 위로 제한하니 NavVis 법선 기준 뒤집힘이 가상 센서 20% → 13%, 혼합 19% → 10.5%로 줄었다)
     높이는 '스캔한 장비 높이'(--sensor-height, 로봇이면 LiDAR·카메라 높이)를 맨 위로, 그 아래 SENSOR_STEP마다
     SENSOR_LOW까지 둔다. 데이터에 있는 점은 실제 장비가 본 면뿐이라, 장비 높이 근처의 가상 센서는 자연히
     '실제로 보인 쪽'에 선다. 정답 법선으로 채점한 결과 (혼합 기준):
       합성 방 (0.5 m에서 스캔):        장비 높이 0.5 m → 0.4%,  예전 1.0·1.6 m 고정 → 2.8%
       가구 많은 합성 방 (0.5 m에서):   장비 높이 0.5 m → 0.9%,  예전 → 2.0%  (tangent 2.5%)
       공장 (사람이 메고 스캔):         장비 높이 2.0 m → 9.8%,  예전 → 10.5% (tangent 15.8%)
     높이를 천장까지 전부 쌓으면 오히려 나빴다 (공장 33.6%): 낮은 지붕 위(건물 밖)에 센서가 섰고, 지붕 밑에
     막아도 철망·골판 지붕처럼 성긴 면 너머 '안 보인 쪽'에 선 센서를 가림 검사가 걸러 내지 못했다.
     tangent의 전체 부호는 '가상 센서 결과와 더 많이 일치하는 쪽'으로 정한다. (예전에는 방이면 '방 중심을
     향하는 쪽이 다수'로 정했는데, 이 기준은 벽·바닥·천장에만 맞고 방 안 가구는 오히려 중심 반대쪽을 본다)
  4b) 혼합: 가상 센서 결과를 쓰되, 가까운 센서가 모두 가려져 '보이는 센서'를 못 찾은 점(주로 책상 밑)만
     tangent 부호를 따른다. 그런 점은 가장 가까운 센서(대개 책상 위쪽)로 대신 뒤집혀서 틀리기 쉽다.
     Hall 101 다섯 개 모두 Poisson 정확도가 가상 센서보다 1~2%p 올랐다 (33번).
     (시도했다가 버린 것: ① 같은 면 이웃을 이은 조각마다 tangent처럼 부호를 전달하고 조각 전체를 다수결로
      정하기 → 완만한 곳을 따라 조각 하나가 점의 83~91%를 삼키고 그 안에서 부호가 어긋났다. 사슬을 다시
      들이면 tangent의 약점도 다시 들어온다. ② 같은 면 이웃 과반과 반대인 점만 뒤집기 → 외톨이 뒤집힘은
      절반으로 줄었지만 메시 정확도는 그대로였다. 외톨이 뒤집힘은 메시 오류의 원인이 아니었다)
     정답 없는 지표로 '이웃 과반과 부호가 반대인 점'(외톨이 뒤집힘)도 세 방식 모두 출력한다.
  5) 저장할 법선 (--final auto): 실제 센서가 있으면 sensor. 없으면 혼합 (방이든 열린 장면이든).
     예전에는 방은 tangent였다. 37번으로 정답 법선과 채점해 보니 갈리는 기준은 방/열린 장면이 아니라 물체가
     얼마나 많은가였다: 단순한 합성 방(책상 1, 기둥 1)은 tangent 0.1% 대 혼합 2.8%였지만, 가구가 많은 합성 방은
     2.5% 대 2.0%, 공장(NavVis 법선 기준)은 15.8% 대 10.5%로 혼합이 나았다. 물체가 많으면 tangent의 전달
     사슬이 끊길 곳(얇은 판, 가는 다리, 배관)이 많아지고, 한 번 끊기면 물체가 통째로 뒤집힌다.

입력: data/<이름>/prepared.ply, inspect.json (+ sensors.npy 가 있으면 사용)
출력: data/<이름>/normals.ply (+ 비교용 normals_tangent.ply, normals_virtual.ply, normals_hybrid.ply), virtual_sensors.npy, disagree.npy
      virtual_choice.npz: 점마다 고른 가상 센서 번호(index)와, 시선이 막히지 않은 센서를 찾았는지(seen)
      (점 순서는 normals_virtual.ply 와 같음)

실행:
  python 32_clean_normals.py redwood_bedroom                 # 의심 지역(빨강)을 뷰어로 보기
  python 32_clean_normals.py redwood_bedroom --view removed  # 잘린 점(주황)과 이상치(빨강) 보기
  python 32_clean_normals.py hall101/ref --view choice       # 점마다 고른 가상 센서(같은 색), 검정 = 보이는 센서 없음
  python 32_clean_normals.py synth_room --radius 0.05        # 반경을 직접 지정
  python 32_clean_normals.py hall101/ref                     # 열린 장면 (자동 판별)
  python 32_clean_normals.py synth_clutter --sensor-height 0.5   # 0.5 m 높이 장비로 스캔한 데이터
"""
import argparse
import colorsys
import json
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR, orient_by_propagation

# ---- 바꿔볼 만한 파라미터 ----
CROP_MARGIN = 0.10      # 방 윤곽, 바닥, 천장에서 이만큼 바깥까지는 남긴다 [m]
CEILING_BAND = 0.15     # 가장 높은 곳에서 이만큼 아래까지를 천장 점으로 본다 [m]
ROOM_COVER = 0.5        # 천장 후보 윤곽이 바닥 윤곽 면적의 이 비율 이상이면 '방'으로 본다
SENSOR_FLOOR_Z = 0.1    # 이 높이 아래의 수평면 점을 바닥으로 본다 [m]
FLOOR_NEAR = 0.1        # 가상 센서 바로 아래(수평 거리) 이 안에 바닥 점이 있어야 한다 [m]
NB_NEIGHBORS = 20       # 이상치 판단 때 볼 이웃 수 (2-2와 같음)
STD_RATIO = 2.0
MAX_NN = 30
TANGENT_K = 15
FREE_GRID = 0.5         # 가상 센서 후보 격자 간격 [m]
SENSOR_HEIGHT = 2.0     # 가상 센서 맨 위 높이 = 스캔한 장비 높이 [m] (--sensor-height 로 바꿈. 로봇이면 LiDAR 높이)
SENSOR_STEP = 0.5       # 그 아래로 이 간격마다 높이를 하나씩 더 둔다 [m]
SENSOR_LOW = 0.5        # 가장 낮은 센서 높이 [m] (장비가 이보다 낮으면 장비 높이 하나만)
FREE_CLEARANCE = 0.4    # 가장 가까운 점에서 이만큼 떨어져야 빈 공간으로 본다 [m] (낮은 센서는 높이의 3/4까지만)
OCC_CELL = 0.05         # 가림 판정용 점유 격자 크기 [m]
OWN_SURFACE = 0.2       # 점에서 이 거리 안에서는 점 자신이 놓인 면에 걸리는 것을 막힘으로 치지 않는다 [m]
LOS_CANDIDATES = 8      # 가까운 순서로 시선이 막히지 않았는지 확인해 볼 센서 수
LOS_SAMPLES = 48        # 점과 센서 사이 선분에서 검사할 위치 수
LINE_SAMPLE = 300       # --view choice: 고른 센서까지 선을 그어 볼 점 개수


def convex_hull_2d(xy):
    """2D 볼록 껍질 (모노톤 체인). 반시계 방향 꼭짓점 배열을 돌려준다."""
    pts = np.unique(np.round(xy, 3), axis=0)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]
    cross = lambda o, a, b: (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower, upper = [], []
    for p in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    for p in pts[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return np.array(lower[:-1] + upper[:-1])


def inside_hull(xy, hull, margin):
    """점이 볼록 껍질을 margin [m]만큼 넓힌 영역 안에 있는가 (margin이 음수면 좁힌 영역)."""
    a, b = hull, np.roll(hull, -1, axis=0)
    e = b - a
    length = np.linalg.norm(e, axis=1)
    ok = np.ones(len(xy), dtype=bool)
    for s in range(0, len(xy), 200_000):
        q = xy[s:s + 200_000]
        cross = e[None, :, 0] * (q[:, None, 1] - a[None, :, 1]) - e[None, :, 1] * (q[:, None, 0] - a[None, :, 0])
        ok[s:s + 200_000] = np.all(cross >= -margin * length[None], axis=1)  # 반시계 껍질: 안쪽이면 cross > 0
    return ok


def hull_area(xy):
    """점들의 2D 볼록 껍질 넓이 (신발끈 공식)."""
    sample = xy[np.random.default_rng(0).choice(len(xy), min(50_000, len(xy)), replace=False)]
    h = convex_hull_2d(sample)
    return 0.5 * abs(np.dot(h[:, 0], np.roll(h[:, 1], -1)) - np.dot(h[:, 1], np.roll(h[:, 0], -1)))


def detect_scene(P):
    """맨 위 수평 면(천장 후보)이 바닥을 충분히 덮으면 방, 아니면 열린 장면. (판정, 덮는 비율)"""
    z_top = np.percentile(P[:, 2], 99.5)
    top, floor = P[P[:, 2] > z_top - CEILING_BAND], P[P[:, 2] < 0.1]
    if len(top) < 1000 or len(floor) < 1000:
        return "open", 0.0
    cover = hull_area(top[:, :2]) / hull_area(floor[:, :2])
    return ("room" if cover >= ROOM_COVER else "open"), cover


def open_bounds(P):
    """열린 장면: 장면 범위 직사각형 (반시계 꼭짓점)."""
    lo, hi = P[:, :2].min(axis=0), P[:, :2].max(axis=0)
    return np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])


def crop_to_room(P):
    z_top = np.percentile(P[:, 2], 99.5)
    ceiling = P[P[:, 2] > z_top - CEILING_BAND]
    if len(ceiling) < 1000:
        print("  천장 점이 너무 적어 방 윤곽 대신 전체 점의 윤곽을 씀 (가로 방향으로는 거의 안 자름)")
        ceiling = P
    sample = ceiling[np.random.default_rng(0).choice(len(ceiling), min(50_000, len(ceiling)), replace=False)]
    hull = convex_hull_2d(sample[:, :2])
    keep = inside_hull(P[:, :2], hull, CROP_MARGIN) & (P[:, 2] > -CROP_MARGIN) & (P[:, 2] < z_top + CROP_MARGIN)
    return keep, hull, z_top


def sensor_heights(top):
    """바닥과 장비 높이 사이를 나눈 가상 센서 높이: top, top - SENSOR_STEP, ... (SENSOR_LOW 이상)."""
    if top < SENSOR_LOW:
        return (round(float(top), 2),)
    return tuple(float(h) for h in np.round(np.arange(top, SENSOR_LOW - 1e-9, -SENSOR_STEP)[::-1], 2))


def virtual_sensors(pcd, hull, heights):
    """방 윤곽 안, heights 높이의 격자점 중 (1) 바로 아래 FLOOR_NEAR 안에 바닥 점이 있고 (2) 모든 점에서
    FREE_CLEARANCE 이상 떨어진 곳 = 센서가 있었을 법한 빈 공간.
    (1)은 '스캐너는 보이는 바닥 위에 서 있다'는 뜻이다. 벽 바깥, 잘라 낸 장면 밖, 그리고 윗면이 안 찍힌 물체 속
    (합성 방에서 0.5 m 높이 로봇은 책상 윗면을 못 봐서, 점으로는 책상 속이 빈 공간처럼 보였다)을 걸러 낸다."""
    P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
    floor = P[(P[:, 2] < SENSOR_FLOOR_Z) & (np.abs(N[:, 2]) > 0.8)]
    floor_xy = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[floor[:, :2], np.zeros(len(floor))]))
    lo, hi = hull.min(axis=0), hull.max(axis=0)
    gx, gy = np.meshgrid(np.arange(lo[0], hi[0], FREE_GRID), np.arange(lo[1], hi[1], FREE_GRID))
    cand = np.array([[x, y, h] for x, y in zip(gx.ravel(), gy.ravel()) for h in heights])
    cand = cand[inside_hull(cand[:, :2], hull, -0.3)]
    below = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.c_[cand[:, :2], np.zeros(len(cand))]))
    cand = cand[np.asarray(below.compute_point_cloud_distance(floor_xy)) <= FLOOR_NEAR]
    tree = o3d.geometry.KDTreeFlann(pcd)
    clear = [np.sqrt(tree.search_knn_vector_3d(c, 1)[2][0]) > min(FREE_CLEARANCE, 0.75 * c[2]) for c in cand]
    return cand[np.array(clear, dtype=bool)]


def occupancy_grid(P, cell):
    """점이 하나라도 있는 칸을 True로 채운 3D 격자와 격자의 시작 좌표."""
    lo = P.min(axis=0) - 2 * cell
    idx = np.floor((P - lo) / cell).astype(int)
    grid = np.zeros(idx.max(axis=0) + 3, dtype=bool)
    grid[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    return grid, lo


def is_visible(P, axis, V, grid, lo):
    """점 P에서 시점 V까지의 선분이 다른 표면(채워진 칸)에 막히지 않는가.
    점 자신이 놓인 면에 걸리지 않도록, 점에서 OWN_SURFACE 이내이면서 법선 축 방향으로 OCC_CELL 이내인 위치는
    검사하지 않는다. 한계: 스캔되지 않은 면(점이 없는 곳)은 격자에서 빈칸이라 시선을 막지 못한다."""
    t = np.linspace(0, 1, LOS_SAMPLES + 2)[1:-1]
    Q = P[:, None] + t[None, :, None] * (V - P)[:, None]
    off = np.abs(np.einsum("msi,mi->ms", Q - P[:, None], axis))
    own = (off <= OCC_CELL) & (np.linalg.norm(Q - P[:, None], axis=2) < OWN_SURFACE)
    idx = np.floor((Q - lo) / OCC_CELL).astype(int)
    inside = np.all((idx >= 0) & (idx < grid.shape), axis=2)
    idx = np.clip(idx, 0, np.array(grid.shape) - 1)
    blocked = grid[idx[..., 0], idx[..., 1], idx[..., 2]] & inside & ~own
    return ~blocked.any(axis=1)


def visible_viewpoints(pcd, V, chunk=20_000):
    """점마다 '보이는 시점 중 가장 가까운 것'의 번호를 고른다. (번호, 보였는지)
    가까운 LOS_CANDIDATES개가 모두 막혔으면 가장 가까운 것을 쓰고, '보였는지'는 False."""
    P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
    grid, lo = occupancy_grid(P, OCC_CELL)
    out = np.empty(len(P), dtype=int)
    seen = np.zeros(len(P), dtype=bool)
    for s in range(0, len(P), chunk):
        p, n = P[s:s + chunk], N[s:s + chunk]
        order = np.argsort(((p[:, None] - V[None]) ** 2).sum(axis=2), axis=1)[:, :LOS_CANDIDATES]
        chosen = order[:, 0].copy()
        todo = np.ones(len(p), dtype=bool)
        for j in range(order.shape[1]):  # 아직 못 정한 점만 다음 후보로 다시 검사
            if not todo.any():
                break
            cand = order[todo, j]
            ok = is_visible(p[todo], n[todo], V[cand], grid, lo)
            ids = np.flatnonzero(todo)[ok]
            chosen[ids] = cand[ok]
            todo[ids] = False
        out[s:s + chunk] = chosen
        seen[s:s + chunk] = ~todo
    return out, seen


def orient_toward(pcd, targets):
    """점마다 정해진 시점(targets[i]) 쪽을 향하도록 법선을 뒤집는다."""
    p, n = np.asarray(pcd.points), np.asarray(pcd.normals).copy()
    n[np.einsum("ij,ij->i", n, targets - p) < 0] *= -1
    pcd.normals = o3d.utility.Vector3dVector(n)


def lone_flip_ratio(P, N, k=8):
    """정답 없는 지표: 축이 비슷한(|cos| > 0.7) 이웃 k개 중 과반과 부호가 반대인 점의 비율 (외톨이 뒤집힘)."""
    nns = o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(P, dtype=o3d.core.float32))
    nns.knn_index()
    idx = nns.knn_search(o3d.core.Tensor(P, dtype=o3d.core.float32), k + 1)[0].numpy()[:, 1:]
    dots = np.einsum("ij,ikj->ik", N, N[idx])
    same_axis = np.abs(dots) > 0.7
    against = ((dots < 0) & same_axis).sum(axis=1)
    return np.mean((against * 2 >= same_axis.sum(axis=1)) & (same_axis.sum(axis=1) > 0))


def sensor_colors(n):
    """센서마다 구분되는 색 (색상환을 황금비 간격으로 돈다)."""
    return np.array([colorsys.hsv_to_rgb((i * 0.618) % 1, 0.75, 0.95) for i in range(n)])


def choice_lines(P, V, choice, palette):
    """표본 점 LINE_SAMPLE개에서 그 점이 고른 센서까지 선을 긋는다 (센서 색)."""
    pick = np.random.default_rng(0).choice(len(P), min(LINE_SAMPLE, len(P)), replace=False)
    lines = o3d.geometry.LineSet()
    lines.points = o3d.utility.Vector3dVector(np.vstack([P[pick], V[choice[pick]]]))
    lines.lines = o3d.utility.Vector2iVector(np.c_[np.arange(len(pick)), len(pick) + np.arange(len(pick))])
    lines.colors = o3d.utility.Vector3dVector(palette[choice[pick]])
    return lines


def band_report(z, height):
    bands = [("바닥 근처 (0.1 m 아래)", z < 0.1), ("중간", (z >= 0.1) & (z <= height - 0.3)),
             ("천장 근처 (위 0.3 m)", z > height - 0.3)]
    return ", ".join(f"{name} {m.mean():.0%}" for name, m in bands)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (31_prepare.py 의 --name)")
    parser.add_argument("--radius", type=float, help="법선 반경 [m] (기본: inspect.json 추천값)")
    parser.add_argument("--scene", choices=["auto", "room", "open"], default="auto", help="장면 종류 (기본: 자동 판별)")
    parser.add_argument("--sensor-height", type=float, default=SENSOR_HEIGHT,
                        help="스캔한 장비 높이 [m] = 가상 센서 맨 위 높이 (기본 2.0. 로봇이면 LiDAR·카메라 높이)")
    parser.add_argument("--final", choices=["auto", "tangent", "virtual", "hybrid"], default="auto",
                        help="센서가 없을 때 저장할 부호 방식 (기본: 혼합)")
    parser.add_argument("--view", choices=["check", "removed", "choice"], default="check",
                        help="뷰어에서 볼 것 (choice = 점마다 고른 가상 센서)")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장 근처를 숨김")
    args = parser.parse_args()

    folder = DATA_DIR / args.name
    info = json.loads((folder / "inspect.json").read_text())
    radius = args.radius or info["recommended"]["normal_radius"]
    sensors = np.load(folder / "sensors.npy") if (folder / "sensors.npy").exists() else None
    pcd = o3d.io.read_point_cloud(str(folder / "prepared.ply"))
    P = np.asarray(pcd.points)
    print(f"입력: {folder}/prepared.ply, 점 {len(P):,}개")

    # 0) 장면 종류
    scene, cover = detect_scene(P)
    if args.scene != "auto":
        scene = args.scene
    print(f"[장면] {'방' if scene == 'room' else '열린 장면'} (맨 위 수평 면이 바닥 면적의 {cover:.0%}를 덮음"
          f"{', 직접 지정' if args.scene != 'auto' else ''})")

    # 1) 방 밖 점 자르기
    if scene == "room":
        keep, hull, z_top = crop_to_room(P)
        cut = np.flatnonzero(~keep)
        print(f"[방 밖 자르기] 천장 윤곽(꼭짓점 {len(hull)}개) 바깥과 바닥 아래·천장 위: {len(cut):,}개 ({len(cut) / len(P):.2%})")
    else:
        z_top = np.percentile(P[:, 2], 99.5)
        keep, hull = P[:, 2] > -CROP_MARGIN, open_bounds(P)
        cut = np.flatnonzero(~keep)
        print(f"[방 밖 자르기] 열린 장면이라 천장 윤곽으로는 자르지 않고 바닥 아래만: {len(cut):,}개 ({len(cut) / len(P):.2%})")
    room = pcd.select_by_index(np.flatnonzero(keep))

    # 2) 이상치 제거
    clean, kept = room.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO)
    removed = np.setdiff1d(np.arange(len(room.points)), kept)
    zr = np.asarray(room.points)[removed, 2]
    print(f"[이상치 제거] {len(removed):,}개 ({len(removed) / len(room.points):.2%}) | 높이 분포: {band_report(zr, z_top)}")
    print(f"  (참고: 전체 점의 높이 분포는 {band_report(np.asarray(room.points)[:, 2], z_top)})")

    # 3) 법선 축
    C = np.asarray(clean.points)
    clean.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius, max_nn=MAX_NN))
    print(f"[법선] 반경 {radius * 100:.1f} cm, 점 {len(C):,}개")

    # 4) 부호: tangent, 가상 센서, (있으면) 실제 센서
    tan = o3d.geometry.PointCloud(clean)
    t0 = time.time()
    orient_by_propagation(tan, TANGENT_K)  # 전체 부호는 아래에서 가상 센서 다수결로 다시 정한다
    print(f"  tangent: {time.time() - t0:.1f}초")
    heights = sensor_heights(args.sensor_height)
    vs = virtual_sensors(clean, hull, heights)
    t0 = time.time()
    virt = o3d.geometry.PointCloud(clean)
    choice, seen = visible_viewpoints(clean, vs)  # 점마다 '어디서 보였을까'의 추측 (센서 번호)
    orient_toward(virt, vs[choice])
    n_tan, n_virt = np.asarray(tan.normals), np.asarray(virt.normals)
    disagree = np.einsum("ij,ij->i", n_tan, n_virt) < 0
    if disagree.mean() > 0.5:  # tangent 전체 부호: 가상 센서와 더 많이 맞는 쪽으로
        tan.normals = o3d.utility.Vector3dVector(-n_tan)
        n_tan, disagree = -n_tan, ~disagree
        print("  tangent 전체 부호: 가상 센서와 반대인 점이 더 많아서 전부 뒤집음")
    print(f"  가상 센서 {len(vs)}개 (바닥 위, 높이 {heights} m = 장비 {args.sensor_height} m 아래로 {SENSOR_STEP} m마다, "
          f"점에서 {FREE_CLEARANCE} m 이상 떨어진 빈 공간), "
          f"보이는 센서 찾기 {time.time() - t0:.1f}초")
    print(f"  점마다 고른 센서: 보이는 센서를 찾음 {seen.mean():.1%}, 가까운 {LOS_CANDIDATES}개가 모두 막혀 "
          f"가장 가까운 센서로 대신함 {(~seen).mean():.1%} | 한 번이라도 고른 센서 {len(np.unique(choice))}/{len(vs)}개")
    print(f"[정답 없는 검증] tangent와 가상 센서 결과가 다른 점: {disagree.mean():.2%} ({disagree.sum():,}개)")

    # 4b) 혼합: 가상 센서 결과에서, 보이는 센서를 못 찾은 점만 tangent 부호로
    n_hyb = n_virt.copy()
    n_hyb[~seen] = n_tan[~seen]
    hyb = o3d.geometry.PointCloud(clean)
    hyb.normals = o3d.utility.Vector3dVector(n_hyb)
    print(f"[혼합] 보이는 센서를 못 찾은 점 {(~seen).mean():.1%} 중 tangent를 따라 가상 센서와 부호가 달라진 점 "
          f"{(np.einsum('ij,ij->i', n_hyb, n_virt) < 0).mean():.2%}")
    print(f"  이웃 과반과 부호가 반대인 점: tangent {lone_flip_ratio(C, n_tan):.2%}, 가상 센서 {lone_flip_ratio(C, n_virt):.2%}, "
          f"혼합 {lone_flip_ratio(C, n_hyb):.2%}")
    print(f"  혼합과 다른 점: tangent {(np.einsum('ij,ij->i', n_hyb, n_tan) < 0).mean():.2%}, "
          f"가상 센서 {(np.einsum('ij,ij->i', n_hyb, n_virt) < 0).mean():.2%}")
    if sensors is not None:
        real = o3d.geometry.PointCloud(clean)
        real_choice, _ = visible_viewpoints(clean, sensors)
        orient_toward(real, sensors[real_choice])
        n_real = np.asarray(real.normals)
        diff = lambda n: (np.einsum("ij,ij->i", n, n_real) < 0).mean()
        print(f"  (실제 센서와 다른 점: tangent {diff(n_tan):.2%}, 가상 센서 {diff(n_virt):.2%}, 혼합 {diff(n_hyb):.2%})")

    use = args.final if args.final != "auto" else "hybrid"
    if sensors is not None:
        final, label = real, "sensor"
    else:
        final, label = {"virtual": (virt, "가상 센서"), "tangent": (tan, "tangent"), "hybrid": (hyb, "혼합")}[use]
    o3d.io.write_point_cloud(str(folder / "normals.ply"), final)
    o3d.io.write_point_cloud(str(folder / "normals_tangent.ply"), tan)  # 33번에서 방식끼리 메시로 비교할 때 쓴다
    o3d.io.write_point_cloud(str(folder / "normals_virtual.ply"), virt)
    o3d.io.write_point_cloud(str(folder / "normals_hybrid.ply"), hyb)
    np.save(folder / "virtual_sensors.npy", vs)
    np.savez(folder / "virtual_choice.npz", index=choice, seen=seen)
    np.save(folder / "disagree.npy", disagree)
    print(f"\n저장 ({label} 방식): {folder}/normals.ply (비교용 normals_tangent.ply, normals_virtual.ply, normals_hybrid.ply), "
          "virtual_sensors.npy, virtual_choice.npz, disagree.npy")

    if args.no_view:
        return
    ball_colors = np.tile((1.0, 0.0, 1.0), (len(vs), 1))
    if args.view == "removed":
        view = o3d.geometry.PointCloud(pcd)
        col = np.full((len(P), 3), 0.75)
        col[cut] = (1.0, 0.55, 0.0)
        col[np.flatnonzero(keep)[removed]] = (0.9, 0.1, 0.1)
        title = f"{args.name}: orange = outside room, red = outliers"
    elif args.view == "choice":
        view = o3d.geometry.PointCloud(virt)
        palette = sensor_colors(len(vs))
        col = palette[choice]
        col[~seen] = 0.0
        used = np.isin(np.arange(len(vs)), choice)
        ball_colors = np.where(used[:, None], palette, 0.6)  # 아무 점도 고르지 않은 센서는 회색
        title = f"{args.name}: point color = chosen virtual sensor, black = no visible sensor"
    else:
        view = o3d.geometry.PointCloud(final)
        col = np.full((len(C), 3), 0.75)
        col[disagree] = (0.9, 0.1, 0.1)
        title = f"{args.name}: red = tangent and virtual sensors disagree"
    view.colors = o3d.utility.Vector3dVector(col)
    Q = np.asarray(view.points)
    if args.hide_ceiling:
        view = view.select_by_index(np.flatnonzero(Q[:, 2] < z_top - 0.25))
    geoms = [view]
    if args.view == "choice":
        geoms.append(choice_lines(C, vs, choice, palette))
    for s, c in zip(vs, ball_colors):
        ball = o3d.geometry.TriangleMesh.create_sphere(radius=0.05)
        ball.translate(s)
        ball.paint_uniform_color(c)
        geoms.append(ball)
    o3d.visualization.draw_geometries(geoms, window_name=title)


if __name__ == "__main__":
    main()
