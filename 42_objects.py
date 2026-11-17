"""
6단계-1: 점군에서 구조물(바닥·벽·지붕)을 빼고, 남은 점을 물체별 덩어리로 나눈다

왜?
  타일(38번)은 구조물만 담고, 물체는 따로 모델을 만들어 월드에 <include> 하는 것이 최종 목표다.
  그러려면 점군에서 '이 점들이 물체 하나'를 먼저 가려내야 한다. 이 파일은 그 첫 단계(점 나누기)만 한다.
  물체별 메시와 모델 폴더는 다음 단계에서 이 결과를 읽어 만든다.

하는 일
  1) 구조물 점 표시: 바닥(높이 FLOOR_BAND 아래), 큰 수직 벽 평면(34번 find_walls), 지붕·천장(find_roofs)
  2) 남은 점을 OBJ_GAP 이내로 이어진 덩어리로 묶는다 (DBSCAN). 점이 MIN_POINTS 보다 적거나 가장 긴 변이
     MIN_SIZE 보다 짧은 덩어리는 작은 조각으로 버린다.
  3) 덩어리마다 바닥에 서 있는지(standing) 떠 있는지(hanging, 가장 낮은 점이 STAND_Z 위)를 적는다.

'물체 하나'의 기준 (정해야 하는 부분)
  이어진 덩어리 = 한 물체로 본다. 쌓인 상자 5개도 붙어 있으면 한 물체다. 이 기준은 쓰임새(움직이는지,
  로봇이 무엇을 목표로 하는지)가 정하는 것이라 --gap 으로 바꿀 수 있게 두었다.
  점을 구조물에서 떼어 낸 만큼 물체 밑바닥(FLOOR_BAND)도 같이 잘려 나간다. 물체 모델을 만들 때 바닥까지 이어 줘야 한다.

시험한 결과
  합성 방 (synth_clutter). 정답 물체 9개: 책상 2, 선반, 쌓인 상자, 의자 3, 기계, 배관 4개 묶음.
  정답 단위는 '닿아 있으면 한 물체'다 (21번 clutter_objects). 세로 배관이 가로 배관을 꿰뚫고 지나가서 배관 4개가 한 물체다.
    gap  0.04 0.06 0.08 0.10 0.12 0.15 0.20 0.25
    맞음   5    7    8    8    8    6    5    4     (9개 중)
  gap 이 작으면 상자·배관이 조각나고, 크면 가까운 책상과 의자가 한 덩어리가 된다 (상판과 의자 사이가 15 cm). 그래서 0.10 으로 두었다.
  못 맞춘 하나(table_0)는 gap 을 어떻게 해도 안 된다. 상판 점이 거의 없어서(스캐너가 0.5 m 높이에서 밑면을 비스듬히만 본 탓으로 보인다)
  다리 두 묶음이 이어지지 않는다. 점에 없는 윗면은 gap 으로 이을 수 없다.
  물체로 나온 점 33,350개 중 99.9% 가 정답 물체 점이고, 정답 물체 점의 96.6% 를 물체로 찾았다.

  공장 (gap 0.10)은 아직 안 된다:
  - 지붕 규칙(find_roofs)이 틀렸었다. 처음에는 칸 수만 세서 높이 5곳(2.4~5.5 m)을 지붕으로 봤고, 그 때문에 2.4 m 위가 전부 구조물로
    빠져 큰 기계가 z 2.37 에서 잘렸다. 높이별 수평면을 보니 (아래향 수평 점의 덮은 칸 비율):
      5.2~5.6 m 91%   지붕. 진짜
      2.4~3.1 m 19~28%  북쪽 구역(y 6~9 m) 전체를 덮는 천장. 진짜 (폭 3 m, 34 ㎡)
      3.4~4.1 m, 4.5~4.9 m 19~21%  배관·보의 밑면. 길게 늘어선 것이라 칸 수는 많지만 폭이 1.5 m 도 안 된다. 지붕이 아니다
    그래서 점이 닿은 칸에서 폭 1.5 m 미만을 깎아 낸 뒤 ROOF_COVER 를 본다 (깎은 뒤 가짜 층은 9% 이하, 진짜는 19%·91%).
    고친 뒤 지붕·천장은 2.80~2.97 m (34 ㎡)와 5.15~5.47 m (163 ㎡) 둘이다. 지붕 점은 덮인 칸 위만 빼서, 같은 높이의 다른 곳 배관은 물체로 남는다.
  - 지붕이 맞게 잡히자 2.4 m 에서 안 잘려서 배관이 바닥에서 지붕까지 이어진다. 가장 큰 덩어리가 27만 점(2.4 m 높이)에서
    53만 점(11.2 x 9.8 x 5.3 m)이 됐다. 이어진 것은 gap 만으로 갈라지지 않는다. 다음에 풀 것이다.
  - 지붕 트러스 보(길이 10.5 m, 높이 4.8~5.15 m)와 지붕 위 환기구(5.5~6.7 m)가 '떠 있는 물체'(54개)로 나온다. 구조물에 넣을지 정해야 한다.
  인천 지하철: 지붕은 2.94~3.09 m 하나(49 ㎡)만 잡힌다. 높이 7.5 m 의 진짜 천장은 아래향 수평 점이 2.3만 개뿐이라 바닥 칸의 12% 에
    못 미쳐서 안 잡힌다 (성긴 천장은 못 잡는 한계). 그래서 기둥 6개가 7.5 m 높이의 서 있는 물체로 나온다.

  S3DIS 방 5개 (44번으로 변환, --score-s3dis. Area_1: office_1, conferenceRoom_1, office_31, hallway_6, pantry_1. 데이터는 data/ 아래에만 둔다):
    물체 = table·chair·sofa·bookcase. 점수에서 뺀 것 = beam·column·window·door·board·clutter. 정답 단위는 인스턴스 하나씩과
    '맞닿은 가구를 묶은 것' 두 가지로 센다. 천장·바닥·벽 중 구조물로 뺀 비율 / 인스턴스별 ok / 묶음 ok:
      conferenceRoom_1  100%    9/15   10/13   (의자 8/13, 탁자 1/1)
      office_31         100%    5/31    6/22   (의자 4/14, 탁자 1/13)
      office_1         93.6%    0/8     1/6    (벽 폭 기준을 고친 뒤. 고치기 전 92.9%)
      hallway_6        81.3%    1/10    1/10   (채움 기준을 낮춘 뒤. 고치기 전 75.3%)
      pantry_1         92.4%    1/3     1/3
    깨지는 곳:
    - 벽 폭 기준 3 m 는 큰 공장에 맞춘 값이라 작은 방의 짧은 벽(office_1 서쪽 벽 2.95 m)을 놓쳤다. 지금은 min(3 m, 0.4 x 그 방향 장면 크기).
      고친 뒤 합성 방, 공장, 인천, 침실의 벽 수는 그대로이고 office_1 만 2장 → 3장이 됐다 (창 점이 물체로 새던 것이 22% → 0%).
    - 그래도 office_1 서쪽 벽 점 8천 개가 샌다. 창(−2.85)이 벽면(−2.70)보다 15 cm 안쪽에 있어서, 점이 더 많은 창 평면이 벽으로 뽑혀
      벽면이 띠(±8 cm) 밖에 남는다. 평행한 두 평면이 가까우면 하나의 두꺼운 벽으로 묶어야 한다.
    - hallway_6 의 진짜 긴 벽(34 m)은 채움 비율 35% 라서 WALL_MIN_FILL 0.5 에 걸렸다 (문이 많은 복도 벽). 0.325 로 낮추니 이 벽만 늘고
      (벽 1장 → 2장) 합성 방, 공장, 인천, 침실, 나머지 S3DIS 방은 그대로였다. 후보 평면 35개의 채움 비율이 가짜 10~29%, 진짜 35~100% 라서
      그 틈(29~35%)의 가운데를 골랐다 (여유가 한쪽 3%p 뿐이라 다른 현장에서는 다시 재야 한다).
      hallway_6: 구조물로 뺀 비율 75.3% → 81.3%, 물체 점 중 벽 비율 33% → 30% (벽 점 11만 개는 아직 샌다, 인스턴스 ok 는 1/10 그대로).
      점 수 기준 5000 을 2000 으로 낮추는 건 복도에 가짜 벽 7장이 늘어서 안 쓴다.
    - office_31 의 짧은 벽(7 m)은 점이 3,848개뿐이라 점 수 기준 5000 에 걸린다.
    - 천장 밑 보(beam)와 기둥(column)이 물체 점의 7~19% 를 차지한다 (구조물로 안 빠진다). 칠판은 벽 띠로 100% 빠진다.
    - gap 0.10 이 정답 접촉 거리 0.04 보다 커서, 가까이 놓인 의자와 책상이 한 덩어리가 되면 '섞임'으로 센다 (office_31).

입력: data/<이름>/normals.ply (32번 결과. 법선은 벽·지붕을 찾는 데 쓴다)
출력: data/<이름>/objects.npz   labels: 점마다 -1 바닥, -2 벽, -3 지붕, -4 작은 조각, 0 이상은 물체 번호 (큰 순서)
      data/<이름>/objects.json  물체 목록 (번호, 점 수, 범위, 종류)
      data/<이름>/objects_top.png  위에서 본 그림

실행:
  python 42_objects.py synth_clutter --score-synth     # 합성 방은 정답 물체(21번 clutter_objects)로 채점
  python 42_objects.py factory --gap 0.10
"""
import argparse
import importlib
import json
import re

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from common import DATA_DIR

# 파일 이름이 숫자로 시작해서 import 문 대신 씀
m21 = importlib.import_module("21_make_synth_room")
m34 = importlib.import_module("34_export")

# ---- 바꿔볼 만한 파라미터 ----
FLOOR_BAND = m34.FLOOR_BAND  # 이 높이 아래는 바닥 [m]
WALL_MARGIN = 0.08           # 벽 평면에서 이 안의 점을 벽으로 뺀다 [m]
WALL_PAD = 0.10              # 벽의 양 끝에서 이만큼 더 벽으로 본다 [m]
ROOF_MIN_Z = 2.0             # 지붕·천장은 이 높이 위에서만 찾는다 [m]
ROOF_CELL = 0.5              # 지붕이 덮은 넓이를 셀 때 칸 크기 [m]. 폭 3 x 3 칸(1.5 m)보다 좁은 것은 지붕이 아니라고 본다
ROOF_BIN = 0.10              # 지붕 높이를 찾을 때 높이 칸 [m]
ROOF_COVER = 0.12            # 폭 1.5 m 이상인 수평면이 바닥이 보이는 칸의 이 비율 이상을 덮은 높이를 지붕으로 본다.
                             # 공장에서 진짜 천장은 19%(북쪽 구역)와 91%(지붕), 배관·덕트 밑면은 9% 이하였다. 그 사이 값이다
ROOF_PAD = 0.05              # 지붕 점의 높이 범위 위아래로 더 빼는 두께 [m]
OBJ_GAP = 0.10               # 이 거리 안에 이웃이 있으면 같은 물체 [m]. 합성 방에서 0.08~0.12 가 가장 좋았다
DBSCAN_MIN = 5               # DBSCAN 핵심점 기준 이웃 수 (자기 포함)
MIN_POINTS = 100             # 덩어리가 이보다 점이 적으면 작은 조각
MIN_SIZE = 0.15              # 덩어리의 가장 긴 변이 이보다 짧으면 작은 조각 [m]
STAND_Z = 0.3                # 가장 낮은 점이 이 높이 이하면 바닥에 서 있는 물체 [m]

FLOOR, WALL, ROOF, SMALL = -1, -2, -3, -4   # labels 의 구조물·조각 표시
TRUE_DIST = 0.05             # 채점: 정답 표면에서 이 안의 점만 그 정답 물체의 점으로 센다 [m]
S3DIS_OBJECTS = ("table", "chair", "sofa", "bookcase")             # S3DIS 채점: 물체로 세는 클래스
S3DIS_BASE = ("ceiling", "floor", "wall")                          # 구조물로 빠져야 하는 클래스
S3DIS_IGNORE = ("beam", "column", "window", "door", "board", "clutter")   # 점수에서 뺀다: 벽에 붙은 것(보·기둥·문·창·칠판)은 우리 구조물 제거 대상이 아니거나 벽 평면으로 빠지고, 잡동사니는 정답 경계가 모호함
CONTACT_DIST = 0.04          # 채점: 정답 인스턴스끼리 점이 이 안에 닿으면 한 단위 (닿아 있으면 한 물체)
MIN_UNIT_POINTS = 300        # 채점: 처리된 점이 이보다 적은 정답 단위는 세지 않는다


def _grid(xy, origin, shape):
    """(x, y) 점이 닿은 ROOF_CELL 칸을 True 로 표시한 격자."""
    ij = np.floor((xy - origin) / ROOF_CELL).astype(int)
    ok = ((ij >= 0) & (ij < shape)).all(axis=1)
    M = np.zeros(shape, bool)
    M[ij[ok, 0], ij[ok, 1]] = True
    return M


def _lookup(M, xy, origin):
    """점 (x, y)가 격자 M 에서 True 인 칸에 들어 있나."""
    ij = np.floor((xy - origin) / ROOF_CELL).astype(int)
    ok = ((ij >= 0) & (ij < M.shape)).all(axis=1)
    out = np.zeros(len(xy), bool)
    out[ok] = M[ij[ok, 0], ij[ok, 1]]
    return out


def _spread(M, all_of):
    """3 x 3 칸 이웃 전부(all_of=True, 깎기) 또는 하나라도(False, 부풀리기)가 True 인 칸. 격자 가장자리는 비어 있다고 본다."""
    out = M.copy()
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            shifted = np.roll(np.roll(M, dx, 0), dy, 1)
            out = out & shifted if all_of else out | shifted
    return out


def find_roofs(P, N):
    """지붕·천장 [{z: (낮은 쪽, 높은 쪽), cells: 덮은 칸 격자, origin, area: 덮은 넓이 ㎡}].
    법선이 수직인 점을 높이 0.2 m 창으로 보고, 그 점이 닿은 ROOF_CELL 칸에서 폭 3 칸(1.5 m)보다 좁은 것을 지운다(깎았다 부풀리기).
    남은 칸이 바닥이 보이는 칸의 ROOF_COVER 이상이면 지붕으로 본다. 배관과 덕트 밑면은 길어도 폭이 좁아서 빠지고,
    공장의 지붕(91%)과 북쪽 구역 천장(19%)은 남는다 (이전에는 칸 수만 세서 배관 줄이 모여 2.4~4.8 m 에 지붕이 4개 더 나왔다).
    지붕 점은 높이 범위 안이면서 덮은 칸 위에 있는 점만 뺀다: 같은 높이의 덮이지 않은 곳의 배관은 물체로 남는다."""
    origin = P[:, :2].min(axis=0) - 2 * ROOF_CELL
    shape = tuple(np.ceil((P[:, :2].max(axis=0) + 2 * ROOF_CELL - origin) / ROOF_CELL).astype(int))
    n_floor = _grid(P[P[:, 2] < FLOOR_BAND, :2], origin, shape).sum()
    if n_floor == 0:
        return []
    up = (np.abs(N[:, 2]) > 0.9) & (P[:, 2] > ROOF_MIN_Z)
    Pu = P[up]
    b = np.floor(Pu[:, 2] / ROOF_BIN).astype(int)
    flagged = {}
    for bi in np.unique(b):
        M = _spread(_spread(_grid(Pu[(b == bi) | (b == bi + 1), :2], origin, shape), True), False)
        if M.sum() / n_floor >= ROOF_COVER:
            flagged[bi] = M
    runs, run = [], []
    for bi in sorted(flagged):
        if run and bi > run[-1] + 1:
            runs.append(run)
            run = []
        run.append(bi)
    if run:
        runs.append(run)
    roofs = []
    for run in runs:
        cover = np.logical_or.reduce([flagged[bi] for bi in run])
        sel = (b >= run[0]) & (b <= run[-1] + 1) & _lookup(cover, Pu[:, :2], origin)
        z = Pu[sel, 2]
        roofs.append(dict(z=(float(np.percentile(z, 1)) - ROOF_PAD, float(np.percentile(z, 99)) + ROOF_PAD),
                          cells=_spread(cover, False), origin=origin, area=float(cover.sum() * ROOF_CELL ** 2)))
    return roofs


def split_structure(P, N):
    """구조물 점 표시 (바닥, 벽, 지붕 불리언 배열)와 찾은 벽·지붕 정보."""
    walls = m34.find_walls(P, N)
    roofs = find_roofs(P, N)
    floor = P[:, 2] < FLOOR_BAND
    wall = np.zeros(len(P), bool)
    for w in walls:
        u = P[:, :2] @ w["t"]
        wall |= (np.abs(P[:, :2] @ w["n"] - w["d"]) < WALL_MARGIN) & (u > w["u0"] - WALL_PAD) & (u < w["u1"] + WALL_PAD)
    roof = np.zeros(len(P), bool)
    for r in roofs:
        roof |= (P[:, 2] >= r["z"][0]) & (P[:, 2] <= r["z"][1]) & _lookup(r["cells"], P[:, :2], r["origin"])
    return floor, wall, roof, walls, roofs


def group_objects(Q, gap=OBJ_GAP):
    """구조물이 아닌 점 Q를 gap 이내로 이어진 덩어리로 묶는다. 점마다 덩어리 번호 (노이즈 -1)."""
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(Q))
    return np.asarray(pcd.cluster_dbscan(eps=gap, min_points=DBSCAN_MIN))


def find_objects(P, N, gap=OBJ_GAP):
    """점군에서 물체를 나눈다. (labels, 물체 목록, 구조물 정보)를 돌려준다.
    labels: 점마다 FLOOR / WALL / ROOF / SMALL 이거나 물체 번호(0 이상, 점이 많은 순서)."""
    floor, wall, roof, walls, roofs = split_structure(P, N)
    labels = np.full(len(P), SMALL, np.int32)
    labels[roof] = ROOF
    labels[wall] = WALL
    labels[floor] = FLOOR
    labels, objects = cluster_nonstructure(P, labels, gap)
    return labels, objects, dict(walls=walls, roofs=roofs)


def cluster_nonstructure(P, structure_labels, gap=OBJ_GAP):
    """SMALL 로 남겨 둔 점에 같은 DBSCAN·작은 조각 필터를 적용한다. 입력 라벨은 보존한다.

    46번 비교 실험에서 구조물 마스크만 정답으로 바꾸고 이 단계를 그대로 재사용한다.
    """
    labels = np.asarray(structure_labels, dtype=np.int32).copy()
    rest = np.flatnonzero(labels == SMALL)
    if not len(rest):
        return labels, []
    raw = group_objects(P[rest], gap)
    k = raw.max() + 1 if len(raw) else 0
    sizes = np.bincount(raw[raw >= 0], minlength=k)
    objects, new_id = [], np.full(k, -1)
    for c in np.argsort(sizes)[::-1]:
        q = P[rest[raw == c]]
        lo, hi = q.min(axis=0), q.max(axis=0)
        if len(q) < MIN_POINTS or (hi - lo).max() < MIN_SIZE:
            continue
        new_id[c] = len(objects)
        objects.append(dict(id=len(objects), points=int(len(q)), min=lo.round(3).tolist(), max=hi.round(3).tolist(),
                            kind="standing" if lo[2] <= STAND_Z else "hanging"))
    keep = raw >= 0
    labels[rest[keep]] = np.where(new_id[raw[keep]] >= 0, new_id[raw[keep]], SMALL)
    return labels, objects


def gt_object_meshes(name):
    """합성 방 정답 물체 메시 (31번이 점군에 적용한 변환을 같이 적용한 것). (이름 목록, 메시 목록)"""
    T = np.loadtxt(DATA_DIR / name / "prepared_transform.txt")
    names, meshes = [], []
    for obj_name, parts in m21.clutter_objects():
        mesh = parts[0]
        for p in parts[1:]:
            mesh += p
        names.append(obj_name)
        meshes.append(mesh.transform(T))
    return names, meshes


def gt_labels(P, name):
    """합성 방 정답: 점마다 가장 가까운 정답 물체 번호 (방 껍데기·먼 점은 -1). (번호, 이름 목록)"""
    T = np.loadtxt(DATA_DIR / name / "prepared_transform.txt")
    shell = o3d.geometry.TriangleMesh.create_box(*m21.CLUTTER_ROOM)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(shell.transform(T)))   # geometry id 0 = 방 껍데기
    names, meshes = gt_object_meshes(name)
    for mesh in meshes:
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    ans = scene.compute_closest_points(o3d.core.Tensor(P.astype(np.float32)))
    gid = ans["geometry_ids"].numpy().astype(int) - 1
    dist = np.linalg.norm(ans["points"].numpy() - P, axis=1)
    gid[dist > TRUE_DIST] = -1
    return gid, names


def score_synth(labels, gid, names):
    """나눈 결과를 정답 물체로 채점해 출력한다.
    물체마다: 점이 가장 많이 든 덩어리(best)가 그 물체 점을 몇 % 담았나(recall), 그 덩어리가 그 물체 점으로 몇 % 차 있나(purity),
    그 물체 점의 10% 이상을 가진 덩어리가 몇 개인가(조각)."""
    is_obj, found = gid >= 0, labels >= 0
    print(f"\n정답 물체 점 {is_obj.sum():,}개 중 물체로 나온 것 {found[is_obj].mean():.1%}, "
          f"물체로 나온 점 {found.sum():,}개 중 정답 물체 점 {is_obj[found].mean():.1%}")
    print(f"{'정답 물체':10s} {'점':>6s}  {'best':>4s} {'recall':>7s} {'purity':>7s} {'조각':>4s}  판정")
    n_ok = 0
    for k, name in enumerate(names):
        mine = gid == k
        counts = np.bincount(labels[mine & found])
        if len(counts) == 0:
            print(f"{name:10s} {mine.sum():6d}  못 찾음")
            continue
        best = int(np.argmax(counts))
        recall = counts[best] / mine.sum()
        purity = counts[best] / (labels == best).sum()
        frags = int((counts >= 0.1 * mine.sum()).sum())
        verdict = "ok" if recall >= 0.8 and purity >= 0.8 else ("섞임" if purity < 0.8 else "쪼개짐")
        n_ok += verdict == "ok"
        print(f"{name:10s} {mine.sum():6d}  {best:4d} {recall:7.1%} {purity:7.1%} {frags:4d}  {verdict}")
    n_clusters = labels.max() + 1
    mostly = sum(1 for c in range(n_clusters) if np.mean(gid[labels == c] >= 0) < 0.5)
    print(f"→ 정답 물체 {len(names)}개 중 ok {n_ok}개. 물체 덩어리 {n_clusters}개 중 대부분이 정답 물체가 아닌 것 {mostly}개")


def s3dis_labels(P, name):
    """S3DIS 정답을 처리된 점 P 로 옮긴다: 44번이 저장한 정답 점에 31번의 변환을 적용하고, P 의 점마다 가장 가까운 정답 점의 라벨을 준다
    (TRUE_DIST 안에 정답 점이 없으면 -1). (클래스 이름 목록, 인스턴스 이름 목록, 점별 클래스 번호, 점별 인스턴스 번호)"""
    g = np.load(DATA_DIR / name / "raw" / "gt_points.npz")
    T = np.loadtxt(DATA_DIR / name / "prepared_transform.txt")
    xyz = g["xyz"].astype(np.float64) @ T[:3, :3].T + T[:3, 3]
    nns = o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(xyz))
    nns.knn_index()
    idx, d2 = nns.knn_search(o3d.core.Tensor(P), 1)              # 돌려주는 거리는 제곱이다
    idx, ok = idx.numpy().ravel(), d2.numpy().ravel() <= TRUE_DIST ** 2
    cls = np.where(ok, g["cls"][idx], -1)
    inst = np.where(ok, g["inst"][idx], -1)
    return [str(c) for c in g["cls_names"]], [str(n) for n in g["inst_names"]], cls, inst


def contact_components(P, inst, members):
    """members 에 든 정답 인스턴스끼리 점이 CONTACT_DIST 안에서 맞닿으면 한 단위로 묶는다. {인스턴스 번호: 단위 번호}.
    (점을 CONTACT_DIST 칸에 넣고, 한 인스턴스의 칸을 이웃 27칸으로 부풀려서 다른 인스턴스의 칸과 겹치면 맞닿은 것)"""
    offs = np.array([(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1)])
    cells, grown = {}, {}
    for a in members:
        c = np.unique(np.floor(P[inst == a] / CONTACT_DIST).astype(np.int64), axis=0)
        cells[a] = c[:, 0] * 1_000_003 * 1_000_003 + c[:, 1] * 1_000_003 + c[:, 2]
        grown[a] = np.unique((c[:, None, :] + offs[None]).reshape(-1, 3) @ np.array([1_000_003 ** 2, 1_000_003, 1]))
    parent = {a: a for a in members}

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for i, a in enumerate(members):
        for b in members[i + 1:]:
            if find(a) != find(b) and np.isin(cells[b], grown[a]).any():
                parent[find(a)] = find(b)
    roots = sorted({find(a) for a in members})
    return {a: roots.index(find(a)) for a in members}


def score_units(labels, unit, ignore, unit_name):
    """정답 단위(점마다 unit 번호, -1 = 단위 아님)마다 채점한다. recall = 단위의 점 중 점이 가장 많이 든 덩어리(best)에 든 비율,
    purity = best 덩어리의 점(ignore 는 뺀 것) 중 그 단위의 점 비율, 조각 = 그 단위 점의 10% 이상을 가진 덩어리 수.
    recall, purity 둘 다 0.8 이상이면 ok. 점이 MIN_UNIT_POINTS 보다 적은 단위는 뺀다. [{unit, points, best, recall, purity, fragments, verdict}]"""
    rows = []
    for u in np.unique(unit[unit >= 0]):
        mine = unit == u
        if mine.sum() < MIN_UNIT_POINTS:
            continue
        counts = np.bincount(labels[mine & (labels >= 0)])
        if len(counts) == 0 or counts.max() == 0:
            rows.append(dict(unit=int(u), name=unit_name(u), points=int(mine.sum()), best=-1, recall=0.0, purity=0.0, fragments=0,
                             verdict="못 찾음"))
            continue
        best = int(np.argmax(counts))
        in_best = (labels == best) & ~ignore
        recall, purity = counts[best] / mine.sum(), (in_best & mine).sum() / max(in_best.sum(), 1)
        verdict = "ok" if recall >= 0.8 and purity >= 0.8 else ("섞임" if purity < 0.8 else "쪼개짐")
        rows.append(dict(unit=int(u), name=unit_name(u), points=int(mine.sum()), best=best, recall=float(recall), purity=float(purity),
                         fragments=int((counts >= 0.1 * mine.sum()).sum()), verdict=verdict))
    return rows


def print_units(rows, title, show=30):
    print(f"\n{title}")
    print(f"{'정답':26s} {'점':>7s}  {'best':>4s} {'recall':>7s} {'purity':>7s} {'조각':>4s}  판정")
    for r in sorted(rows, key=lambda r: -r["points"])[:show]:
        print(f"{r['name'][:26]:26s} {r['points']:7d}  {r['best']:4d} {r['recall']:7.1%} {r['purity']:7.1%} {r['fragments']:4d}  {r['verdict']}")
    n_ok = sum(r["verdict"] == "ok" for r in rows)
    print(f"→ 정답 단위 {len(rows)}개 중 ok {n_ok}개 (쪼개짐 {sum(r['verdict'] == '쪼개짐' for r in rows)}, "
          f"섞임 {sum(r['verdict'] == '섞임' for r in rows)}, 못 찾음 {sum(r['verdict'] == '못 찾음' for r in rows)})")


def score_s3dis(P, labels, name):
    """S3DIS 방 채점. 물체 클래스(S3DIS_OBJECTS)를 정답으로 본다. 기둥·보·문·창·잡동사니는 점수에서 뺀다 (S3DIS_IGNORE).
    1) 구조물 제거: 천장·바닥·벽 점이 구조물로 빠졌나, 물체 점이 구조물로 잘못 빠지지 않았나
    2) 물체 덩어리 구성: 물체로 나온 점이 무엇의 점인가 (구조물이 물체로 샌 정도)
    3) 인스턴스마다 채점, 4) 맞닿은 인스턴스를 묶은 단위마다 채점 ('닿아 있으면 한 물체' 기준)"""
    cls_names, inst_names, cls, inst = s3dis_labels(P, name)
    cn = np.array(cls_names + ["(정답 없음)"])[cls]                  # 점별 클래스 이름 (-1 은 맨 뒤)
    struct = (labels == FLOOR) | (labels == WALL) | (labels == ROOF)
    found = labels >= 0
    base, obj = np.isin(cn, S3DIS_BASE), np.isin(cn, S3DIS_OBJECTS)
    print(f"\n[S3DIS 정답 옮기기] 점 {len(P):,}개 중 정답 라벨이 붙은 점 {(cls >= 0).mean():.1%}")
    print(f"[구조물 제거] 천장·바닥·벽 점 {base.sum():,}개 중 구조물로 뺀 것 {struct[base].mean():.1%}, 물체 덩어리로 샌 것 {found[base].mean():.1%}")
    print("  물체 클래스 점 중 구조물로 잘못 빠진 비율: " + ", ".join(
        f"{c} {struct[cn == c].mean():.0%}" for c in S3DIS_OBJECTS if (cn == c).any()))
    comp = {c: int((found & (cn == c)).sum()) for c in sorted(set(cn[found]))}
    tot = sum(comp.values())
    print(f"[물체 덩어리 구성] 물체로 나온 점 {tot:,}개: " + ", ".join(f"{c} {v / tot:.0%}" for c, v in sorted(comp.items(), key=lambda kv: -kv[1])))
    leak = sum(v for c, v in comp.items() if c in S3DIS_BASE)
    print(f"  → 천장·바닥·벽이 물체로 샌 점 {leak:,}개 ({leak / tot:.1%})")

    ignore = np.isin(cn, S3DIS_IGNORE)
    members = sorted({int(i) for i in np.unique(inst[obj])})
    inst_unit = np.where(obj, inst, -1)
    rows = score_units(labels, inst_unit, ignore, lambda u: inst_names[u])
    print_units(rows, "[인스턴스마다] 가구 한 개씩을 정답 단위로 (붙어 있는 의자와 책상도 따로)")
    by_cls = {}
    for r in rows:
        c = re.sub(r"_?\d+$", "", r["name"])
        by_cls.setdefault(c, []).append(r["verdict"] == "ok")
    print("  클래스별 ok: " + ", ".join(f"{c} {sum(v)}/{len(v)}" for c, v in sorted(by_cls.items())))

    if members:
        comp_of = contact_components(P, inst, members)
        unit = np.full(len(P), -1)
        for a, u in comp_of.items():
            unit[inst == a] = u
        groups = {}
        for a, u in comp_of.items():
            groups.setdefault(u, []).append(inst_names[a])
        rows2 = score_units(labels, unit, ignore, lambda u: "+".join(groups[u])[:60])
        print_units(rows2, f"[맞닿은 것 묶음] 점이 {CONTACT_DIST * 100:g} cm 안에서 닿은 가구를 한 단위로 (닿아 있으면 한 물체)")


def draw(P, labels, objects, out):
    fig, ax = plt.subplots(1, 2, figsize=(16, 9))
    for a in ax:
        a.set_aspect("equal")
    ax[0].set_title("floor(gray) / wall(blue) / roof(green) / object(red) / small(orange)")
    colors = {FLOOR: "0.8", WALL: "tab:blue", ROOF: "tab:green", SMALL: "tab:orange"}
    for lab, c in colors.items():
        m = labels == lab
        ax[0].scatter(P[m][::4, 0], P[m][::4, 1], s=0.2, c=c)
    m = labels >= 0
    ax[0].scatter(P[m][::2, 0], P[m][::2, 1], s=0.3, c="tab:red")
    ax[1].set_title(f"objects ({len(objects)})")
    rng = np.random.default_rng(0)
    pal = rng.random((max(len(objects), 1), 3)) * 0.8
    ax[1].scatter(P[m, 0], P[m, 1], s=0.3, c=pal[labels[m]])
    for o in objects:
        if o["points"] >= 300:
            c = (np.array(o["min"]) + np.array(o["max"])) / 2
            ax[1].text(c[0], c[1], str(o["id"]), fontsize=7, ha="center")
    fig.savefig(out, dpi=70, bbox_inches="tight")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/<이름>/normals.ply 를 읽는다")
    parser.add_argument("--gap", type=float, default=OBJ_GAP, help=f"같은 물체로 보는 이웃 거리 [m] (기본 {OBJ_GAP})")
    parser.add_argument("--score-synth", action="store_true", help="합성 방(synth_clutter)이면 정답 물체로 채점")
    parser.add_argument("--score-s3dis", action="store_true", help="S3DIS 방(44번이 만든 gt_points.npz)이면 정답 라벨로 채점")
    args = parser.parse_args()

    out_dir = DATA_DIR / args.name
    pcd = o3d.io.read_point_cloud(str(out_dir / "normals.ply"))
    P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
    labels, objects, info = find_objects(P, N, args.gap)

    n = lambda lab: int((labels == lab).sum())
    print(f"점 {len(P):,}개: 바닥 {n(FLOOR):,}  벽 {n(WALL):,}  지붕 {n(ROOF):,}  작은 조각 {n(SMALL):,}  "
          f"물체 {int((labels >= 0).sum()):,} ({(labels >= 0).mean():.1%})")
    roofs = [(round(r["z"][0], 2), round(r["z"][1], 2), round(r["area"])) for r in info["roofs"]]
    print(f"벽 {len(info['walls'])}장, 지붕·천장 (낮은 쪽 m, 높은 쪽 m, 넓이 ㎡) {roofs}")
    standing = sum(o["kind"] == "standing" for o in objects)
    print(f"물체 덩어리 {len(objects)}개 (바닥에 선 것 {standing}, 떠 있는 것 {len(objects) - standing}), gap {args.gap} m")
    for o in objects[:15]:
        size = np.array(o["max"]) - np.array(o["min"])
        print(f"  #{o['id']:3d} {o['points']:7d}점  크기 {np.round(size, 2)}  z {o['min'][2]:.2f}~{o['max'][2]:.2f}  {o['kind']}")

    if args.score_synth:
        gid, names = gt_labels(P, args.name)
        score_synth(labels, gid, names)

    if args.score_s3dis:
        score_s3dis(P, labels, args.name)

    np.savez_compressed(out_dir / "objects.npz", labels=labels)
    (out_dir / "objects.json").write_text(json.dumps(dict(gap=args.gap, objects=objects), indent=1))
    draw(P, labels, objects, out_dir / "objects_top.png")
    print(f"\n저장: {out_dir}/objects.npz, objects.json, objects_top.png")


if __name__ == "__main__":
    main()
