"""
5단계-1: 넓은 현장을 타일로 나눠 내보내기 (메시 분할 1·2단계)

34번은 장면 전체를 메시 하나로 만든다. 현장이 넓어지면 메시와 텍스처가 면적에 비례해 커진다
(공장 190 m²에서 이미 삼각형 100만 개, 텍스처 4096²). 그래서 바닥을 격자 타일로 나눠 타일마다
보기용 메시 + 텍스처 + 충돌용 메시를 따로 만든다. 현장이 넓어져도 타일 하나의 크기는 예산 안에 머문다.
  예산 (타일 하나): 보기용 삼각형 VISUAL_MAX 이하, 텍스처 JPEG 8192² 이하 (보통 4096²), 충돌용 삼각형 COLLISION_MAX 이하
         (월드 하나: 보기용 삼각형 합 3,000만 이하)

  1) 나누기: normals.ply(32번 결과)의 점을 바닥 (x, y) 격자 타일로 나눈다. 칸 수는 --grid 로 정하거나
     --tile-size 로 칸 크기를 준다. 타일마다 경계 바깥 OVERLAP 만큼의 점을 더 가져와 재구성한다.
     (Poisson은 점이 끊기는 가장자리에서 면이 부정확하다. 그 가장자리가 타일 밖에 떨어지게 한다)
     층 나누기(1F, 2F)는 아직 없다: 한 층짜리 현장만 된다 (LEVEL)
  2) 타일마다 34번과 같은 과정: Poisson 재구성 → 정리 (34번의 clean)
     Poisson depth는 점 묶음이 작아지면 같이 줄여서 칸 크기를 voxel 근처로 유지한다 (poisson_trim 참고)
  3) 줄이기와 자르기: 순서가 중요하다.
     경계 바깥까지 포함한 메시를 먼저 줄이고, 그다음 타일 경계에서 자른다.
     자른 뒤에 줄이면 줄이기가 경계 꼭짓점을 움직여서 이웃 타일과 틈이 생긴다.
     자르기: 경계선(축에 평행한 평면)에 걸친 삼각형을 평면에서 잘라 새 꼭짓점을 만든다 (색은 섞어서 이음).
     두 타일이 같은 선에서 자르므로 겹치지도 비지도 않는다.
     삼각형 수는 34번처럼 표에서 고른다 (벗어난 거리 95%, 바닥 구멍. 표는 자른 메시로 잰다). 예산이 상한이다.
  4) 텍스처: 타일마다 34번의 칸 나누기로 굽고 JPEG (품질 JPEG_QUALITY)로 저장한다. 메시는 OBJ 그대로.
     파일 이름은 texture_<타일>.jpg: 모델마다 같은 이름(texture.jpg)이면 Gazebo가 텍스처를 섞어 쓴다 (TEXTURE_FILE 참고)
  5) 내보내기: models/<타일>/ 폴더 (visual.obj + visual.mtl + texture_<타일>.jpg (+ 같은 내용의 texture.jpg), collision.obj, model.config, model.sdf),
     model.sdf 에는 충돌 메시만 둔다. 바닥 받침 상자(floor_support: 스캔이 바닥을 못 본 구멍에서 빠지지 않게)는
     --floor-support 를 줄 때만 넣는다. 기본은 뺀다: 플랫폼(메시 올리기)이 받침 아랫면(윗면 - 두께 0.1 m)을 건물 바닥으로
     읽어 월드를 0.1 m 올렸고, 그 월드에서 Go2 가 걷지 못했다 (받침을 빼니 바닥 z = 0 으로 읽고 걸었다). 구멍은 올리는 쪽이 바닥 평면으로 메운다
     world.sdf (타일 전부 include + 월드 플러그인), tiles.json (타일 경계)
     충돌 OBJ에는 꼭짓점 법선(vn)을 넣는다. 없으면 Gazebo 물리가 기동 중에 죽는 환경이 있다.
  6) 이음매 확인: 이웃한 두 타일의 경계선 위 점이 상대 타일 메시에서 얼마나 떨어져 있는지 잰다.

입력: data/<이름>/normals.ply, inspect.json (31, 32번 결과)
출력: data/<이름>/tiles/ (world.sdf, tiles.json, models/), data/<이름>/tiles_report.json

실행:
  python 38_tiles.py factory --grid 2x2 --gt data/factory/gt/prepared.ply
  python 38_tiles.py factory --grid 6x9 --only f1_struct_x2y3   # 한 타일만 (시험용)
"""
import argparse
import importlib
import json
import shutil
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from common import DATA_DIR

m33 = importlib.import_module("33_reconstruct_eval")  # 파일 이름이 숫자로 시작해서 import 문 대신 씀
m34 = importlib.import_module("34_export")

# ---- 바꿔볼 만한 파라미터 ----
TILE_SIZE = 15.0           # --grid 가 없을 때 타일 한 변의 목표 [m] (공장 밀도에서 예산에 맞는 크기)
OVERLAP = 1.0              # 타일 바깥으로 이만큼 더 가져와 재구성 [m]
EDGE_PAD = 0.05            # 점이 있는 범위보다 이만큼 더 넓게 타일 바깥 경계를 잡는다 [m]
VISUAL_MAX = 1_000_000     # 예산: 타일 하나의 보기용 삼각형
VISUAL_DENSITY = 4400      # 보기용 최소 밀도 [삼각형/m²]: 예산 100만 개 ÷ (공장 밀도에서 알맞은 타일 15 x 15 m).
                           # 색은 꼭짓점에 실려 있어서 34번의 '10만 개 이상'(4 m² 책상 장면 기준)은 넓은 타일에서 색을 뭉갠다.
                           # 면적당으로 정하면 타일 크기와 상관없이 밀도가 일정하다
COLLISION_MAX = 100_000    # 예산: 타일 하나의 충돌용 삼각형
TEXTURE_PX = 4096          # 텍스처 한 변 목표 [px] (예산 상한 8192)
JPEG_QUALITY = 90
TEXTURE_COPY = "texture.jpg"          # 규격상의 이름. 같은 내용의 사본으로만 둔다 (visual.mtl 은 가리키지 않아 Gazebo 는 안 읽는다)
TEXTURE_FILE = "texture_{tile}.jpg"   # 텍스처 파일 이름. 타일마다 달라야 한다: 이름이 같으면(texture.jpg) Gazebo(ogre2)가
                                      # 모델이 달라도 같은 이름의 텍스처 하나를 모든 타일에 쓴다 (벽에 엉뚱한 색 얼룩, 확인함)
LEVEL, PREFIX = "1F", "f1_struct"   # tiles.json 의 level, 모델 폴더 이름 앞부분 (<PREFIX>_x0y0)
SEAM_OK = 0.01             # 이음매: 경계 점이 상대 타일 메시에서 이 이내면 맞닿은 것 [m]
SEAM_GAP = 0.05            # 이보다 멀면 건너편에 면이 없는 것 (구멍)이라 따로 센다 [m]
WORLD = "{name}_world"

MODEL_CONFIG = m34.MODEL_CONFIG.replace("scene.sdf", "model.sdf").replace("34_export.py", "38_tiles.py")
FLOOR_BAND = 0.15          # 바닥 높이를 재는 점의 높이 상한 [m]
FLOOR_PCT = 5              # 받침 윗면 = 타일 바닥 점 높이의 이 백분위 (스캔한 바닥 밑에 숨도록 낮게)
SUPPORT_THICK = 0.1        # 바닥 받침 상자 두께 [m]. 받침은 --floor-support 를 줄 때만 넣는다 (아래 SUPPORT 와 docstring 5 참고)
SUPPORT = """      <!-- 바닥 받침: 스캔에 바닥이 없는 자리(물체에 가려 못 본 곳)에서 로봇이 빠지지 않게 하는 보이지 않는 얇은 상자.
           윗면이 스캔한 바닥보다 약간 낮아서, 바닥이 있는 곳은 충돌 메시가 그대로 받친다 -->
      <collision name="floor_support">
        <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
        <geometry><box><size>{sx:.3f} {sy:.3f} {sz:g}</size></box></geometry>
      </collision>
"""
MODEL_SDF_SUPPORT = m34.SDF.replace("    </link>", SUPPORT + "    </link>")

PLUGINS = """    <!-- 월드 플러그인 (Gazebo Fortress 이름). Garden 이상이면 gz-sim-<이름>-system / gz::sim::systems::<이름> -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
"""

WORLD_SDF = """<?xml version="1.0"?>
<!-- 타일로 나눈 월드. 타일 모델은 models/ 폴더에 있다 (README.txt 의 실행 방법 참고) -->
<sdf version="1.9">
  <world name="{world}">
{plugins}    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.75 0.85 0.95 1</background>
    </scene>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <cast_shadows>true</cast_shadows>
    </light>
{includes}  </world>
</sdf>
"""

README = """{name} 타일 월드 (점군 → 메시 변환, 타일 {n}개)

구성
  world.sdf        월드 (플러그인 + 조명 + 타일 전부 include)
  tiles.json       타일 경계 (world_tiles/v1)
  models/<타일>/    타일 모델: visual.obj + visual.mtl + texture_<타일>.jpg (+ texture.jpg 사본), collision.obj, model.config, model.sdf
  좌표: 미터, z축 위, 바닥 z = 0. 타일 메시는 월드 좌표 그대로라서 include 에 pose 가 없다.

실행 (압축을 푼 폴더에서)
  Gazebo Fortress (ROS 2 Humble):   IGN_GAZEBO_RESOURCE_PATH=$PWD/models ign gazebo world.sdf
  Gazebo Garden 이상:               GZ_SIM_RESOURCE_PATH=$PWD/models gz sim world.sdf  (world.sdf 의 플러그인 이름도 바꿔야 함)

참고
  - 타일 경계에서 삼각형을 잘라 맞췄다. 두 타일은 따로 재구성했기 때문에 경계에서 표면이 수 mm 어긋날 수 있다.
  - 충돌 OBJ 에는 꼭짓점 법선(vn)이 들어 있다.
{support}  - 텍스처 파일 이름이 타일마다 다르다 (texture_<타일>.jpg). 모델마다 texture.jpg 로 같으면 Gazebo 가 한 텍스처를 모든 타일에 쓴다.
    visual.mtl 은 texture_<타일>.jpg 를 가리킨다. 같은 내용의 texture.jpg 사본을 폴더마다 함께 두었다 (플랫폼이 이 이름을 찾을 때용).
  - 층 나누기는 아직 없다 (level 은 전부 {level}).
"""


README_SUPPORT = """  - model.sdf 에 바닥 받침(floor_support, 보이지 않는 얇은 상자)이 있다. 스캔이 바닥을 못 본 구멍(물체에 가려진 곳)에서 로봇이 빠지지 않게 한다.
    윗면은 스캔한 바닥보다 약간 낮다. 바닥이 평평한 현장용이다 (구덩이나 경사가 있으면 받침을 빼야 한다).
    받침 아랫면을 건물 바닥으로 읽는 도구는 월드를 받침 두께만큼 올린다. 그런 곳에 올릴 패키지에는 쓰지 않는다.
"""
README_NO_SUPPORT = """  - 바닥 받침(floor_support)은 없다. 스캔이 바닥을 못 본 구멍(물체에 가려진 곳)에는 충돌 바닥이 없어 로봇이 빠질 수 있다.
    올리는 도구가 바닥 평면을 넣어 주면 괜찮다. 바닥이 평평한 현장에서 Gazebo 로 바로 쓰려면 --floor-support 로 다시 내보낸다.
"""


def make_tiles(P, tile_size, grid):
    """점이 있는 바닥 범위를 같은 크기 타일로 나눈다. 경계는 cm 단위로 반올림."""
    lo, hi = P[:, :2].min(axis=0) - EDGE_PAD, P[:, :2].max(axis=0) + EDGE_PAD
    n = np.array(grid) if grid else np.ceil((hi - lo) / tile_size).astype(int)
    xs, ys = (np.round(np.linspace(lo[a], hi[a], n[a] + 1), 2) for a in (0, 1))
    return [dict(name=f"{PREFIX}_x{i}y{j}", ix=i, iy=j, bounds=[xs[i], ys[j], xs[i + 1], ys[j + 1]])
            for j in range(n[1]) for i in range(n[0])]


def in_bounds(P, b, margin=0.0):
    return ((P[:, 0] >= b[0] - margin) & (P[:, 0] <= b[2] + margin) &
            (P[:, 1] >= b[1] - margin) & (P[:, 1] <= b[3] + margin))


def floor_level(P):
    """타일 안 점의 바닥 높이: FLOOR_BAND 아래 점 높이의 FLOOR_PCT 백분위 (mm 단위로 반올림). 바닥 점이 없으면 0"""
    z = P[P[:, 2] < FLOOR_BAND, 2]
    return round(float(np.percentile(z, FLOOR_PCT)), 3) if len(z) else 0.0


def poisson_trim(pcd, voxel, depth=None):
    """Poisson 재구성 + 밀도 trim (33번과 같은 방법). depth는 이 점 묶음에서 칸 크기(= 묶음 크기 x 1.1 / 2^depth)가
    voxel에 가장 가깝게 정한다. 31번은 '칸이 voxel 이하'가 되게 올려 잡지만, 작은 점 묶음(타일)에서 voxel보다 촘촘한
    칸을 쓰면 점이 닿지 않는 칸에서 꼭짓점 색이 어두워졌다 (공장 타일 하나에서 색 밝기 중앙값: 점 0.366,
    칸 1.1 cm → 0.303, 칸 2.1 cm → 0.380)."""
    extent = pcd.get_axis_aligned_bounding_box().get_extent().max()
    depth = depth or int(np.round(np.log2(extent * 1.1 / voxel)))
    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    density = np.asarray(density)
    mesh.remove_vertices_by_mask(density < np.quantile(density, m33.DENSITY_QUANTILE))
    return mesh, depth, extent * 1.1 / 2 ** depth


def clip_plane(V, C, T, axis, c, keep_above):
    """축 axis의 값이 c 이상(keep_above) 또는 이하인 쪽만 남긴다.
    평면에 걸친 삼각형은 평면에서 잘라 새 꼭짓점을 만든다 (모서리 위에서 선형으로, 색도 같은 비율로 섞음).
    같은 모서리를 쓰는 이웃 삼각형은 새 꼭짓점을 함께 쓴다."""
    nv = len(V)
    d = (V[:, axis] - c) * (1 if keep_above else -1)        # d >= 0 이면 남는 쪽
    inside = (d >= 0)[T]                                    # (삼각형, 3)
    n_in = inside.sum(axis=1)
    whole = T[n_in == 3]
    mixed = np.flatnonzero((n_in == 1) | (n_in == 2))
    if len(mixed) == 0:
        return V, C, whole
    Tm, im = T[mixed], inside[mixed]
    k = np.argmax(im == (n_in[mixed] == 1)[:, None], axis=1)  # 혼자 다른 꼭짓점 (1개만 안이면 그 점, 2개가 안이면 밖의 점)
    r = np.arange(len(mixed))
    a, b, c2 = Tm[r, k], Tm[r, (k + 1) % 3], Tm[r, (k + 2) % 3]  # 돌려도 감는 방향은 그대로
    eu, ev = np.concatenate([a, a]), np.concatenate([b, c2])    # 평면과 만나는 모서리 (a-b, a-c2)
    key = np.minimum(eu, ev) * nv + np.maximum(eu, ev)
    uniq, inv = np.unique(key, return_inverse=True)
    lo, hi = uniq // nv, uniq % nv
    t = (d[lo] / (d[lo] - d[hi]))[:, None]
    newV = V[lo] + t * (V[hi] - V[lo])
    newV[:, axis] = c                                       # 경계선 위에 정확히 놓는다
    newC = C[lo] + t * (C[hi] - C[lo])
    p_ab, p_ac = nv + inv[:len(mixed)], nv + inv[len(mixed):]
    one = n_in[mixed] == 1                                  # 안쪽 꼭짓점이 a 하나: 삼각형 하나가 남는다
    two = ~one                                              # 안쪽이 b, c2 둘: 사각형 (p_ab, b, c2, p_ac) → 삼각형 둘
    tris = [whole,
            np.c_[a[one], p_ab[one], p_ac[one]],
            np.c_[p_ab[two], b[two], c2[two]],
            np.c_[p_ab[two], c2[two], p_ac[two]]]
    return np.vstack([V, newV]), np.vstack([C, newC]), np.vstack(tris)


def clip_mesh(mesh, bounds):
    """메시를 [x0, y0, x1, y1] 안으로 자른다 (z는 그대로)."""
    V, C = np.asarray(mesh.vertices), np.asarray(mesh.vertex_colors)
    T = np.asarray(mesh.triangles).astype(np.int64)
    x0, y0, x1, y1 = bounds
    for axis, c, above in [(0, x0, True), (0, x1, False), (1, y0, True), (1, y1, False)]:
        V, C, T = clip_plane(V, C, T, axis, c, above)
    out = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(T.astype(np.int32)))
    out.vertex_colors = o3d.utility.Vector3dVector(np.clip(C, 0, 1))
    out.remove_unreferenced_vertices()
    out.remove_degenerate_triangles()
    out.remove_unreferenced_vertices()
    out.compute_vertex_normals()
    return out


def tile_floor_patch(collision, P, bounds):
    """타일 충돌 메시에 바닥 패치를 합친다 (34번의 floor_patch). (합친 메시, 패치 삼각형 수, 정보)를 돌려준다.
    점은 타일 밖까지 전부 넘긴다: 물체 밑이 타일 경계에 걸쳐 있어도 범위 안인지(점으로 둘러싸였는지) 알아야 한다.
    경계에 걸친 칸은 메시가 경계에서 잘려 있어 구멍으로 잡히므로, 패치는 경계에서 잘라 쓴다. 이웃 타일도 같은 선에서 자르므로 겹치지도 비지도 않는다."""
    patch, info = m34.floor_patch(collision, P, bounds)
    if patch is None:
        return collision, 0, info
    patch.vertex_colors = o3d.utility.Vector3dVector(np.zeros((len(patch.vertices), 3)))   # clip_mesh 가 색도 같이 자른다
    patch = clip_mesh(patch, bounds)
    patch.vertex_colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))
    merged = o3d.geometry.TriangleMesh(collision)
    merged += patch
    return merged, len(patch.triangles), info


def make_table(mesh_ext, bounds, P_floor, extra=()):
    """34번의 줄이기 표를 타일 단위로: [(자른 메시, 벗어난 거리 95% [m], 바닥 구멍, 표의 삼각형 수)]. 앞 행에서 이어서 줄인다.
    줄이기는 경계 바깥을 포함한 메시에, 표의 값은 경계에서 자른 메시로 센다.
    extra 는 표에 더 넣을 삼각형 수 (타일 몫). 표의 삼각형 수는 줄 이름이다. 자른 뒤 실제 개수는 그보다 조금 적을 수 있다 (예산 상한은 실제 개수로 본다)."""
    base = clip_mesh(mesh_ext, bounds)
    samples = np.asarray(base.sample_points_uniformly(m34.N_SAMPLES).points)
    rows = [(base, 0.0, m34.floor_hole_ratio(base, P_floor), len(base.triangles))]
    cur, cur_clip = mesh_ext, base
    print(f"  {'정리 후':>9} ({len(base.triangles):>9,}) | 0.0 mm | 바닥 구멍 {rows[0][2]:.1%}")
    ladder = sorted({n for n in m34.TABLE_TRIANGLES if n <= VISUAL_MAX} | set(extra), reverse=True)
    for n in [n for n in ladder if n < len(base.triangles)]:
        target = int(n * len(cur.triangles) / len(cur_clip.triangles))   # 바깥 몫까지 합친 목표
        for _ in range(3):  # 자른 뒤 n을 넘으면 목표를 낮춰 다시 (예산 상한)
            nxt = m34.decimate(cur, target)
            nxt_clip = clip_mesh(nxt, bounds)
            if len(nxt_clip.triangles) <= n:
                break
            target = int(target * n / len(nxt_clip.triangles) * 0.97)
        cur, cur_clip = nxt, nxt_clip
        mean, p95 = m34.deviation(samples, cur_clip)
        holes = m34.floor_hole_ratio(cur_clip, P_floor)
        rows.append((cur_clip, p95, holes, n))
        print(f"  {n:>9,} ({len(cur_clip.triangles):>9,}) | {mean * 1000:5.1f} / {p95 * 1000:5.1f} mm | 바닥 구멍 {holes:.1%}")
    return rows


def pick(rows, tol, min_triangles, cap):
    """벗어난 거리 95%가 tol 이하, 바닥 구멍이 안 늘고, 표의 삼각형 수가 min_triangles 이상, 실제 개수가 cap 이하인
    가장 작은 메시. 없으면 cap 이하 중 가장 큰 메시 (기준을 못 맞췄다고 알린다)."""
    hole0, n0 = rows[0][2], rows[0][3]
    ok = [m for m, p95, h, n in rows
          if p95 <= tol and h <= hole0 + m34.HOLE_TOL and min(min_triangles, n0) <= n and len(m.triangles) <= cap]
    if ok:
        return min(ok, key=lambda m: len(m.triangles)), True
    return max((m for m, *_ in rows if len(m.triangles) <= cap), key=lambda m: len(m.triangles)), False


def model_sdf(tile, floor_z=None):
    """타일 모델 SDF: 충돌 메시. floor_z 를 주면 타일 크기의 바닥 받침 상자(윗면 z = floor_z)도 넣는다."""
    if floor_z is None:
        return m34.SDF.format(name=tile["name"])
    x0, y0, x1, y1 = tile["bounds"]
    return MODEL_SDF_SUPPORT.format(name=tile["name"], cx=(x0 + x1) / 2, cy=(y0 + y1) / 2,
                                    cz=floor_z - SUPPORT_THICK / 2, sx=x1 - x0, sy=y1 - y0, sz=SUPPORT_THICK)


def write_tile(folder, tile, visual, collision, floor_z=None):
    """models/<타일>/ 에 파일을 쓰고 (텍스처 한 변 px, 파일 크기 MB 딕셔너리)를 돌려준다. floor_z 는 바닥 받침 윗면 높이 (None 이면 받침 없음)."""
    folder.mkdir(parents=True, exist_ok=True)
    textured, cell, size = m34.bake_texture(visual)
    collision = o3d.geometry.TriangleMesh(collision)
    collision.vertex_colors = o3d.utility.Vector3dVector(np.zeros((0, 3)))  # 충돌용은 색이 필요 없다
    for m in (textured, collision):
        m.triangle_normals = o3d.utility.Vector3dVector(np.zeros((0, 3)))   # obj는 꼭짓점 법선만 담는다
    o3d.io.write_triangle_mesh(str(folder / "visual.obj"), textured, write_vertex_colors=False)  # 색은 텍스처에 있다
    texture = TEXTURE_FILE.format(tile=tile["name"])
    o3d.io.write_image(str(folder / texture), textured.textures[0], quality=JPEG_QUALITY)
    shutil.copyfile(folder / texture, folder / TEXTURE_COPY)   # 플랫폼 규격의 texture.jpg (mtl 은 위의 고유 이름을 가리킨다)
    (folder / "visual_0.png").unlink(missing_ok=True)      # Open3D가 같이 쓴 (뒤집힌) png는 안 쓴다
    mtl = folder / "visual.mtl"
    mtl.write_text(mtl.read_text().replace("visual_0.png", texture))
    o3d.io.write_triangle_mesh(str(folder / "collision.obj"), collision)   # 꼭짓점 법선(vn) 포함
    (folder / "model.config").write_text(MODEL_CONFIG.format(name=tile["name"]))
    (folder / "model.sdf").write_text(model_sdf(tile, floor_z))
    sizes = {f.name: f.stat().st_size / 2 ** 20 for f in folder.iterdir()}
    return size, sizes


def line_points(mesh, axis, c, lo, hi):
    """타일 경계선(축 axis의 값 = c)에 놓인 꼭짓점과 그 모서리의 중점 (경계선 위 구간 [lo, hi]만)."""
    V, T = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
    on = np.abs(V[:, axis] - c) < 1e-6
    E = np.sort(np.vstack([T[:, [0, 1]], T[:, [1, 2]], T[:, [2, 0]]]), axis=1)
    E = np.unique(E[on[E[:, 0]] & on[E[:, 1]]], axis=0)
    pts = np.vstack([V[on], (V[E[:, 0]] + V[E[:, 1]]) / 2])
    along = pts[:, 1 - axis]
    return pts[(along >= lo) & (along <= hi)]


def seam_report(tiles, meshes):
    """이웃한 타일 쌍마다, 한쪽 경계선 위 점이 상대 타일 메시에서 떨어진 거리. 양쪽 방향을 합친다.
    SEAM_GAP 넘게 먼 점은 상대 쪽에 면이 없는 곳(구멍)이라 이음매 어긋남에서 빼고 따로 센다."""
    by = {(t["ix"], t["iy"]): t for t in tiles if t["name"] in meshes}
    rows = []
    for (ix, iy), a in by.items():
        for (nx, ny, axis) in [(ix + 1, iy, 0), (ix, iy + 1, 1)]:
            b = by.get((nx, ny))
            if b is None:
                continue
            c = a["bounds"][2 + axis]                       # a 의 위쪽(x1 또는 y1) 경계 = b 의 아래쪽 경계
            lo = max(a["bounds"][1 - axis], b["bounds"][1 - axis])
            hi = min(a["bounds"][3 - axis], b["bounds"][3 - axis])
            for kind in ("visual", "collision"):
                ma, mb = meshes[a["name"]][kind], meshes[b["name"]][kind]
                d = np.concatenate([_dist(line_points(m1, axis, c, lo, hi), m2) for m1, m2 in [(ma, mb), (mb, ma)]])
                near = d[d <= SEAM_GAP]
                rows.append(dict(pair=f"{a['name']} | {b['name']}", kind=kind, points=len(d),
                                 touch=float(np.mean(d <= SEAM_OK)), hole=float(np.mean(d > SEAM_GAP)),
                                 median_mm=float(np.median(near) * 1000) if len(near) else None,
                                 p95_mm=float(np.percentile(near, 95) * 1000) if len(near) else None))
    return rows


def _dist(points, mesh):
    if len(points) == 0:
        return np.zeros(0)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene.compute_distance(o3d.core.Tensor(points, dtype=o3d.core.float32)).numpy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (32번까지 마친 것)")
    parser.add_argument("--grid", help="타일 칸 수 가로x세로 (예: 2x2). 없으면 --tile-size 로 계산")
    parser.add_argument("--tile-size", type=float, default=TILE_SIZE, help=f"타일 한 변 목표 [m] (기본 {TILE_SIZE:g})")
    parser.add_argument("--overlap", type=float, default=OVERLAP, help=f"재구성할 때 타일 바깥으로 더 가져올 점 [m] (기본 {OVERLAP:g})")
    parser.add_argument("--depth", type=int, help="Poisson depth (기본: 칸 크기가 voxel에 가장 가깝게 자동)")
    parser.add_argument("--gt", help="정답 점 파일 (있으면 타일을 합친 보기용 메시의 정확도·완성도를 33번과 같게 잼)")
    parser.add_argument("--only", help="이 타일만 만든다 (쉼표로 여러 개). 시험용")
    parser.add_argument("--out", help="출력 폴더 (기본: data/<이름>/tiles)")
    parser.add_argument("--visual-total", type=int, help="타일 전체의 보기용 삼각형 목표. 타일 면적에 비례해 나눈다 (예: 단일 파일과 같게 1000000)")
    parser.add_argument("--collision-total", type=int, help="타일 전체의 충돌용 삼각형 상한. 타일 면적에 비례해 나눈다 (예: 100000)")
    parser.add_argument("--no-floor-patch", action="store_true", help="충돌용 메시의 바닥 구멍 패치를 넣지 않는다 (34번의 floor_patch)")
    parser.add_argument("--floor-support", action="store_true",
                        help="타일마다 바닥 받침 상자(floor_support)를 넣는다. 기본은 뺀다: 받침 아랫면을 바닥으로 읽는 도구는 월드를 올려서 로봇이 못 걷는다")
    parser.add_argument("--texture-px", type=int, default=TEXTURE_PX, help=f"타일 텍스처 한 변 목표 [px] (기본 {TEXTURE_PX}). 전체 픽셀을 단일 파일과 맞추려면 4096 / 타일 수의 제곱근")
    args = parser.parse_args()
    o3d.utility.random.seed(0)   # 면 위에서 점을 뽑는 계산이 무작위라서, 같은 입력이면 표와 삼각형 수 선택이 같게 고정
    m34.TEXTURE_MAX = args.texture_px

    folder = DATA_DIR / args.name
    voxel = json.loads((folder / "inspect.json").read_text())["voxel"]
    pcd = o3d.io.read_point_cloud(str(folder / "normals.ply"))
    P = np.asarray(pcd.points)
    grid = [int(v) for v in args.grid.lower().split("x")] if args.grid else None
    tiles = make_tiles(P, args.tile_size, grid)
    todo = [t for t in tiles if not args.only or t["name"] in args.only.split(",")]
    out = Path(args.out) if args.out else folder / "tiles"
    print(f"[나누기] 점 {len(P):,}개, 타일 {len(tiles)}개 (만들 것 {len(todo)}개), 재구성에는 경계 바깥 {args.overlap:g} m까지 사용")
    for t in tiles:
        x0, y0, x1, y1 = t["bounds"]
        print(f"  {t['name']}: x {x0:.2f} ~ {x1:.2f}, y {y0:.2f} ~ {y1:.2f} ({x1 - x0:.1f} x {y1 - y0:.1f} m)")

    area = {t['name']: (t['bounds'][2] - t['bounds'][0]) * (t['bounds'][3] - t['bounds'][1]) for t in tiles}
    share = lambda total, name: int(total * area[name] / sum(area.values()))   # 타일 몫 (면적 비례)
    meshes, report = {}, dict(tiles=[], seams=[])
    t_all = time.time()
    for t in todo:
        t0 = time.time()
        b = t["bounds"]
        print(f"\n===== {t['name']} =====")
        pts_tile = P[in_bounds(P, b)]
        ext = pcd.select_by_index(np.flatnonzero(in_bounds(P, b, args.overlap)))
        P_ext = np.asarray(ext.points)
        # 바닥 구멍 검사에 쓸 점: 경계 칸이 반쯤 타일 밖으로 나가 구멍으로 잘못 세이지 않게 안쪽만
        P_floor = P[in_bounds(P, [b[0] + m34.FLOOR_CELL, b[1] + m34.FLOOR_CELL, b[2] - m34.FLOOR_CELL, b[3] - m34.FLOOR_CELL])]

        raw, depth, cell = poisson_trim(ext, voxel, args.depth)
        print(f"[재구성] 점 {len(P_ext):,}개 (타일 안 {len(pts_tile):,}개), depth {depth} (칸 {cell * 100:.1f} cm): "
              f"삼각형 {len(raw.triangles):,}개 ({time.time() - t0:.0f}초)")
        mesh, rep = m34.clean(raw, P_ext, voxel)
        print("[정리] 지운 삼각형: " + ", ".join(f"{k} {v:,}" for k, v in rep.items()) + f" → {len(mesh.triangles):,}개")
        del raw

        print(f"\n[줄이기] {'삼각형(자른 뒤)':>16} | 정리한 메시에서 벗어난 거리 평균 / 95% | 바닥 구멍")
        vis_cap, col_cap = VISUAL_MAX, COLLISION_MAX
        vis_min = min(VISUAL_MAX, max(m34.VISUAL_MIN, int(VISUAL_DENSITY * (b[2] - b[0]) * (b[3] - b[1]))))
        why = f"{VISUAL_DENSITY:,}개/m² x 타일 면적"
        if args.visual_total:   # 전체 목표를 면적 비례로 나눈 몫이 최소이자 상한
            vis_cap = vis_min = min(VISUAL_MAX, share(args.visual_total, t["name"]))
            why = f"전체 {args.visual_total:,}개 중 면적 몫"
        if args.collision_total:
            col_cap = min(COLLISION_MAX, share(args.collision_total, t["name"]))
        rows = make_table(mesh, b, P_floor, extra=[n for n in (vis_cap, col_cap) if n < VISUAL_MAX])
        visual, v_ok = pick(rows, m34.VISUAL_TOL, vis_min, vis_cap)
        collision, c_ok = pick(rows, m34.COLLISION_TOL, 0, col_cap)
        print(f"[선택] 보기용 {len(visual.triangles):,}개 (최소 {vis_min:,}개 = {why}){'' if v_ok else ' (기준 5 mm를 못 맞춤: 예산 안에서 가장 큰 것)'}, "
              f"충돌용 {len(collision.triangles):,}개{'' if c_ok else ' (기준 2 cm를 못 맞춤: 예산 안에서 가장 큰 것)'}")
        if not args.no_floor_patch:   # 고른 충돌용 메시에서 구멍을 다시 찾는다. 패치 삼각형은 예산(col_cap) 밖에 더해진다
            before = m34.floor_hole_ratio(collision, P_floor)
            collision, n_patch, info = tile_floor_patch(collision, P, b)
            print(f"[바닥 패치] 충돌용 바닥 구멍 {before:.1%} → {m34.floor_hole_ratio(collision, P_floor):.1%}, "
                  f"바닥 점이 없는 자리(물체 밑 등) {info['unseen'] * m34.FLOOR_CELL ** 2:.1f} ㎡ 메움 "
                  f"(삼각형 +{n_patch:,}개 → {len(collision.triangles):,}개). 보기용에는 넣지 않는다")

        floor_z = floor_level(pts_tile) if args.floor_support else None
        if floor_z is None:
            print("[바닥 받침] 넣지 않음 (--floor-support 로 넣는다)")
        else:
            print(f"[바닥 받침] 타일 전체 크기 상자, 윗면 z = {floor_z:+.3f} m (바닥 점 높이의 {FLOOR_PCT}% 지점)")
        size, sizes = write_tile(out / "models" / t["name"], t, visual, collision, floor_z)
        print(f"[내보내기] 텍스처 {size} x {size} px, 파일 " + ", ".join(f"{k} {v:.1f} MB" for k, v in sorted(sizes.items())))
        meshes[t["name"]] = dict(visual=visual, collision=collision)
        report["tiles"].append(dict(name=t["name"], bounds=b, visual_triangles=len(visual.triangles),
                                    collision_triangles=len(collision.triangles), texture_px=size, floor_support_z=floor_z,
                                    visual_ok=v_ok, collision_ok=c_ok, mb=sizes, seconds=round(time.time() - t0)))
        print(f"  ({time.time() - t0:.0f}초)")

    # 이음매 확인
    if len(meshes) > 1:
        report["seams"] = seam_report(tiles, meshes)
        print("\n[이음매] 이웃 타일 경계선 위 점이 상대 타일 메시에서 떨어진 거리")
        for s in report["seams"]:
            mm = f"중앙값 {s['median_mm']:.1f} mm, 95% {s['p95_mm']:.1f} mm" if s["median_mm"] is not None else "-"
            print(f"  {s['pair']:34s} {s['kind']:9s} 점 {s['points']:>6,}개: {SEAM_OK * 100:g} cm 이내 {s['touch']:.0%}, "
                  f"{SEAM_GAP * 100:g} cm 넘게 먼 곳(건너편이 빔) {s['hole']:.0%} | 나머지의 {mm}")

    if args.gt and meshes:  # 타일로 나눠도 모양이 맞는지: 보기용 메시를 합쳐 34번과 같은 지표로
        gt = o3d.io.read_point_cloud(args.gt)
        G = np.asarray(gt.points)
        inside, _ = m33.footprint(G)
        merged = o3d.geometry.TriangleMesh()
        for m in meshes.values():
            merged += m["visual"]
        acc, comp, f = m33.evaluate(merged, gt, G[inside(G[:, :2])], inside)
        print(f"\n[정답 비교] 타일 보기용 메시를 합친 것: 정확도 {acc:.1%}, 완성도 {comp:.1%}, F {f:.1%}")
        report["gt"] = dict(accuracy=float(acc), completeness=float(comp), f=float(f))

    # 월드와 타일 경계 파일
    out.mkdir(parents=True, exist_ok=True)
    includes = "".join(f'    <include>\n      <uri>model://{t["name"]}</uri>\n      <name>{t["name"]}</name>\n    </include>\n'
                       for t in todo)
    (out / "world.sdf").write_text(WORLD_SDF.format(world=WORLD.format(name=args.name.replace("/", "_")),
                                                    plugins=PLUGINS, includes=includes))
    lines = ",\n".join("    " + json.dumps(dict(model=t["name"], level=LEVEL, bounds=[float(v) for v in t["bounds"]]))
                       for t in todo)  # 타일 하나가 한 줄
    (out / "tiles.json").write_text('{\n  "schema": "world_tiles/v1",\n  "tiles": [\n' + lines + "\n  ]\n}\n")
    (out / "README.txt").write_text(README.format(name=args.name, n=len(todo), level=LEVEL,
                                                  support=README_SUPPORT if args.floor_support else README_NO_SUPPORT))
    vis_sum = sum(t["visual_triangles"] for t in report["tiles"])
    col_sum = sum(t["collision_triangles"] for t in report["tiles"])
    print(f"\n[월드] 타일 {len(todo)}개: 보기용 삼각형 합 {vis_sum:,}개 (예산 3,000만), 충돌용 합 {col_sum:,}개 → {out}/")
    print(f"  world.sdf, tiles.json, models/ ({time.time() - t_all:.0f}초)")
    report["totals"] = dict(visual_triangles=vis_sum, collision_triangles=col_sum)
    (folder / "tiles_report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
