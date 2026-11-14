"""
38번이 만든 타일 메시를 위에서 내려다본 nav2 지도(yaml + pgm)로 바꾼다. 플랫폼 "맵 등록"에 올리는 파일.

nav2 지도는 흑백 이미지 한 장이다: 한 칸 = RES m, 검정 = 막힘, 흰색 = 비어 있음, 회색 = 모름.
메시를 일정 높이 구간(band)으로 잘라, 그 구간에 면이 있는 칸을 '막힘'으로 칠한다. 구간이 다른 지도를 두 장 만든다:
  _loc: 위치 추정(AMCL)용. 라이다가 볼 수 있는 높이 전부 (LOC_BAND). 스캔 끝점이 지도에 없는 벽에 닿으면 감점되므로 넓게.
  _obs: 장애물용. 로봇 몸통이 부딪힐 높이 (OBS_BAND): 계단턱보다 높고 몸 높이 이하. 천장 배관·보는 지나다니는 데 상관없다.
        스캔 범위 가장자리를 막고, 윤곽선만 남은 물체 속을 메운다 (solidify).
두 장은 크기·해상도·원점이 같다 (플랫폼 규칙). 원점 yaw는 0.

입력: data/<이름>/tiles/ (38번 결과: tiles.json, models/<타일>/visual.obj)
출력: data/<이름>/nav2/<이름>_loc.{yaml,pgm}, <이름>_obs.{yaml,pgm}, <이름>_nav2_preview.png
실행: python 40_nav2_map.py factory
      python 40_nav2_map.py factory --loc-band 0.3,1.0 --obs-band 0.1,0.5   # 로봇 라이다·몸 높이에 맞춰 바꾸기
"""
import argparse
import json
from collections import deque
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from common import DATA_DIR

# ---- 바꿔볼 만한 파라미터 ----
RES = 0.05                 # 한 칸 크기 [m]
MARGIN = 0.5               # 메시 범위 바깥으로 더 잡는 여백 [m]
LOC_BAND = (0.20, 1.50)    # 위치 추정용으로 자르는 높이 구간 [m] (바닥 z = 0)
OBS_BAND = (0.15, 0.60)    # 장애물용 높이 구간 [m]
SAMPLE = 0.02              # 면 위에 찍는 점 간격 [m]. 칸(5 cm)보다 훨씬 촘촘해야 얇은 벽도 칸을 채운다
MIN_HITS = 2               # 칸에 이만큼 이상 점이 있어야 '막힘' (면 조각 하나가 스치는 잡음 제거)
SEAL = 2                   # 장애물용: 스캔 범위 가장자리 이 칸 수를 막힘으로 막는다 (밖은 바닥이 없다)
CLOSE = 3                  # 비어 있는 곳을 이어 붙이는 칸 수 (3 = 15 cm): 메시 틈 때문에 바닥이 군데군데 빠지는 걸 메운다
OCC, FREE, UNKNOWN = 0, 254, 205   # pgm 값 (nav2 trinary: 검정 막힘, 흰색 빔, 회색 모름)


def load_tiles(pkg):
    """모든 타일의 보기용 메시를 하나로 합쳐 (정점, 삼각형) 배열로 돌려준다."""
    tiles = json.loads((pkg / "tiles.json").read_text())["tiles"]
    verts, tris, n = [], [], 0
    for t in tiles:
        m = o3d.io.read_triangle_mesh(str(pkg / "models" / t["model"] / "visual.obj"))
        verts.append(np.asarray(m.vertices))
        tris.append(np.asarray(m.triangles) + n)
        n += len(m.vertices)
    return np.vstack(verts), np.vstack(tris), tiles


def surface_points(V, T, band, step):
    """높이 구간 band 에 걸친 삼각형 면 위에 step 간격으로 점을 찍고, 구간 안에 있는 점만 돌려준다.
    (큰 삼각형은 꼭짓점만으로는 칸을 못 채우므로 면 위에 직접 찍는다)"""
    z = V[T][:, :, 2]
    if band is not None:
        T = T[(z.max(axis=1) >= band[0]) & (z.min(axis=1) <= band[1])]
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(V), o3d.utility.Vector3iVector(T))
    n = int(min(max(mesh.get_surface_area() / step ** 2, 1000), 8_000_000))
    P = np.asarray(mesh.sample_points_uniformly(n).points)
    return P if band is None else P[(P[:, 2] >= band[0]) & (P[:, 2] <= band[1])]


def to_cells(P, origin, shape):
    """월드 점 → (이미지 행, 열). 이미지 맨 윗줄이 y가 큰 쪽이다."""
    col = np.floor((P[:, 0] - origin[0]) / RES).astype(int)
    row = shape[0] - 1 - np.floor((P[:, 1] - origin[1]) / RES).astype(int)
    ok = (col >= 0) & (col < shape[1]) & (row >= 0) & (row < shape[0])
    return row[ok], col[ok]


def shift_or(a, n, combine):
    """a 를 상하좌우 n 칸 이내로 밀어서 합친다 (combine = np.logical_or 이면 부풀리기, and 이면 깎기)"""
    pad = np.pad(a, n, constant_values=(combine is np.logical_and))
    out = np.ones_like(a) if combine is np.logical_and else np.zeros_like(a)
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            out = combine(out, pad[n + dy:n + dy + a.shape[0], n + dx:n + dx + a.shape[1]])
    return out


def make_map(V, T, band, known, origin, shape):
    """pgm 배열 한 장: 높이 구간에 면이 있는 칸 = 막힘, 면이 있는 곳(known)의 나머지 = 빔, 그 밖 = 모름"""
    hits = np.zeros(shape, int)
    r, c = to_cells(surface_points(V, T, band, SAMPLE), origin, shape)
    np.add.at(hits, (r, c), 1)
    occupied = hits >= MIN_HITS
    img = np.full(shape, UNKNOWN, np.uint8)
    img[known & ~occupied] = FREE
    img[occupied] = OCC
    return img


def main_free_region(free):
    """빈 칸 중 가장 큰 덩어리 (4방향으로 이어진 것). 나머지는 물체 속이나 닫힌 틈이다"""
    seen = np.zeros_like(free)
    best = []
    for r0, c0 in zip(*np.nonzero(free)):
        if seen[r0, c0]:
            continue
        seen[r0, c0] = True
        comp, q = [(r0, c0)], deque([(r0, c0)])
        while q:
            r, c = q.popleft()
            for rr, cc in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
                if 0 <= rr < free.shape[0] and 0 <= cc < free.shape[1] and free[rr, cc] and not seen[rr, cc]:
                    seen[rr, cc] = True
                    comp.append((rr, cc))
                    q.append((rr, cc))
        if len(comp) > len(best):
            best = comp
    keep = np.zeros_like(free)
    keep[tuple(np.array(best).T)] = True
    return keep


def solidify(img, known):
    """장애물용 보정 두 가지. (1) 스캔 범위 가장자리를 막힘으로: 밖은 바닥이 없어 로봇이 떨어진다.
    (2) 바깥과 이어지지 않은 빈 칸을 막힘으로: 면만 스캔돼서 윤곽선만 남은 물체 속이다 (어차피 로봇이 못 들어간다)."""
    out = img.copy()
    out[known & ~erode(known, SEAL)] = OCC
    free = out == FREE
    out[free & ~main_free_region(free)] = OCC
    return out


def erode(a, n):
    return shift_or(a, n, np.logical_and)


def write_map(folder, name, img, origin):
    """name.pgm (P5, 8비트) + name.yaml (map_server 설정)"""
    with open(folder / f"{name}.pgm", "wb") as f:
        f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
        f.write(img.tobytes())
    (folder / f"{name}.yaml").write_text(
        f"image: {name}.pgm\nresolution: {RES}\norigin: [{origin[0]:.3f}, {origin[1]:.3f}, 0.0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


def parse_band(text, default):
    return tuple(float(v) for v in text.split(",")) if text else default


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (38번까지 마친 것)")
    parser.add_argument("--loc-band", help=f"위치 추정용 높이 구간 'lo,hi' [m] (기본 {LOC_BAND[0]:g},{LOC_BAND[1]:g})")
    parser.add_argument("--out", help="출력 폴더 (기본: data/<이름>/nav2)")
    parser.add_argument("--obs-band", help=f"장애물용 높이 구간 'lo,hi' [m] (기본 {OBS_BAND[0]:g},{OBS_BAND[1]:g})")
    args = parser.parse_args()
    loc_band, obs_band = parse_band(args.loc_band, LOC_BAND), parse_band(args.obs_band, OBS_BAND)
    o3d.utility.random.seed(0)   # 면 위 점 찍기가 무작위라서, 같은 입력이면 같은 지도가 나오게 고정

    folder = DATA_DIR / args.name
    V, T, tiles = load_tiles(folder / "tiles")
    print(f"[읽기] 타일 {len(tiles)}개, 정점 {len(V):,}개, 삼각형 {len(T):,}개")

    # 지도 범위: 타일 경계 + 여백, 원점은 칸에 맞게 내림. 원점 yaw = 0
    x0, y0 = min(t["bounds"][0] for t in tiles) - MARGIN, min(t["bounds"][1] for t in tiles) - MARGIN
    x1, y1 = max(t["bounds"][2] for t in tiles) + MARGIN, max(t["bounds"][3] for t in tiles) + MARGIN
    origin = (np.floor(x0 / RES) * RES, np.floor(y0 / RES) * RES)
    shape = (int(np.ceil((y1 - origin[1]) / RES)), int(np.ceil((x1 - origin[0]) / RES)))   # (행, 열)
    print(f"[범위] {shape[1]} x {shape[0]} 칸 ({shape[1] * RES:.1f} x {shape[0] * RES:.1f} m), 원점 ({origin[0]:.2f}, {origin[1]:.2f})")

    # 메시가 있는 곳 (바닥·벽·천장 아무 높이): 여기만 '비어 있음'으로 칠하고 나머지는 '모름'
    cover = np.zeros(shape, bool)
    r, c = to_cells(surface_points(V, T, None, RES), origin, shape)
    cover[r, c] = True
    known = shift_or(shift_or(cover, CLOSE, np.logical_or), CLOSE, np.logical_and)    # 닫기: 부풀렸다 깎기

    out = Path(args.out) if args.out else folder / "nav2"
    out.mkdir(parents=True, exist_ok=True)
    maps = {}
    for tag, band in (("loc", loc_band), ("obs", obs_band)):
        img = make_map(V, T, band, known, origin, shape)
        if tag == "obs":
            img = solidify(img, known)
        write_map(out, f"{args.name}_{tag}", img, origin)
        maps[tag] = img
        print(f"[{tag}] 높이 {band[0]:g} ~ {band[1]:g} m: 막힘 {(img == OCC).sum():,}칸, 빔 {(img == FREE).sum():,}칸 ({(img == FREE).sum() * RES ** 2:.0f} m²), "
              f"모름 {(img == UNKNOWN).sum():,}칸 -> {out}/{args.name}_{tag}.yaml, .pgm")

    plt.rcParams["font.family"] = "Noto Sans CJK JP"
    fig, ax = plt.subplots(1, 2, figsize=(14, 11), dpi=80)
    ext = (origin[0], origin[0] + shape[1] * RES, origin[1], origin[1] + shape[0] * RES)
    for a, (tag, title, band) in zip(ax, (("loc", "위치 추정용", loc_band), ("obs", "장애물용", obs_band))):
        a.imshow(maps[tag], cmap="gray", vmin=0, vmax=255, extent=ext)
        a.set_title(f"{args.name}_{tag} ({title}, 높이 {band[0]:g} ~ {band[1]:g} m)", fontsize=13)
        a.set_xlabel("x [m]"); a.set_ylabel("y [m]")
    fig.suptitle(f"nav2 지도 (한 칸 {RES * 100:g} cm): 검정 = 막힘, 흰색 = 빔, 회색 = 모름", fontsize=14)
    plt.tight_layout()
    fig.savefig(out / f"{args.name}_nav2_preview.png", facecolor="white", bbox_inches="tight")
    print(f"[그림] {out}/{args.name}_nav2_preview.png")


if __name__ == "__main__":
    main()
