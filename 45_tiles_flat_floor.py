"""
5단계-3: 38번이 만든 타일 패키지에서 충돌 바닥을 z = 0 평면으로 평평하게 만든 시험용 사본을 만든다

왜: 인천 월드(플랫폼 이름 subway)에서 Go2 가 못 갔다. 같은 월드·지도·경로에서 TurtleBot 은 완주했다.
    원인 후보 하나는 충돌 바닥이 평평하지 않다는 것이다. 충돌 바닥은 점군을 재구성한 표면이라 높이가 -0.05 ~ +0.05 m 사이에서
    오르내리고, 점이 드문 곳은 큰 삼각형끼리 기울어져 있다. 다리 달린 로봇은 발이 닿는 면의 높이와 기울기에 민감할 수 있다.
    바닥만 평평하게 하고 나머지는 그대로 둔 사본을 올려 Go2 가 걷는지 보면 이 후보를 확인하거나 버릴 수 있다.
    (아직 확인한 게 아니다. 충돌 바닥 울퉁불퉁함이 원인이라는 증거는 없다.)

방법: 충돌 OBJ 에서 바닥 삼각형의 꼭짓점 높이를 0 으로 맞춘다. 삼각형을 지우거나 새로 깔지 않으므로
    바닥이 덮은 범위, 벽 밑단과의 이음매, 타일 경계는 그대로다.
    * 바닥 삼각형 = 거의 수평(법선 z 성분 0.8 이상)이고 세 꼭짓점이 모두 -0.15 m ~ FLOOR_TOP(0.08 m) 사이인 삼각형
    * 그보다 높은 면(물체 윗면, 바닥에 깔린 낮은 덩어리)은 건드리지 않는다. 낮은 덩어리의 가장자리는 바닥과 만나는 꼭짓점이
      0 으로 내려가서 가파르게 이어진다
    * 세 꼭짓점이 다 같은 높이로 내려와 넓이가 0 이 된 삼각형(수직 벽 밑단)이 생기면 지운다
    * 바닥과 만나는 꼭짓점의 법선(vn)은 이웃 면으로 다시 계산한다 (충돌 OBJ 에는 꼭 vn 이 있어야 한다)
    * 보기용 메시, 텍스처, world.sdf 는 그대로다. 센서가 보는 보기용 바닥은 예전 그대로이므로 바닥 하나만 바뀐 비교가 된다
    * 물체 모델(f1_obj_*)이 있는 패키지에서는 타일 구조물(tiles.json 에 적힌 모델)만 바꾼다

시험한 결과 (인천, 타일 4개):
    * 바닥 삼각형 21,851개 (381 m²)의 꼭짓점 17,352개를 0 으로 맞췄다. 가장 많이 움직인 꼭짓점은 7.9 cm. 넓이 0 이 된 삼각형은 없었다
    * 5도 넘게 기운 바닥 면적은 타일별 4.1 / 1.9 / 1.2 / 8.6 % → 0 %. 바닥 높이(면적 가중 5~95%)는 -0.6 ~ +3.8 cm 등 → 전부 0
    * 아래로 쏜 광선(0.1 m 간격)이 바닥 높이에 닿는 칸은 원본과 같다 (36,091칸). 그중 96.3 % 가 z = 0 에 닿는다.
      나머지는 바닥 위의 낮은 덩어리다 (북쪽 x 2~8, y 5.5~8.5 m 에 높이 0.1~0.25 m 덩어리. 점군에도 0.1~0.5 m 에 점이 많다). 그대로 두었다
    * PyBullet: 경로(w1~w5) 주변에 공 240개를 떨어뜨리면 원본은 서는 높이가 -0.7 ~ +1.9 cm 로 흩어지고 2초 동안 중앙값 6 cm 굴러간다.
      평평한 사본은 전부 같은 높이에 서고 굴러가지 않는다. 빠진 공은 둘 다 0개
    * Gazebo Fortress: 충돌 메시만 넣은 월드를 1500 스텝 돌려 오류 없이 끝났다 (보기용 메시를 넣은 전체 월드로는 돌리지 않았다)
    * 알아 둘 것: 경로 주변 원래 바닥은 이미 꽤 평평했다 (공이 서는 높이 표준편차 0.36 cm). 그래서 이 사본으로도 Go2 가 못 가면
      충돌 바닥의 울퉁불퉁함은 원인에서 빠진다. 반대로 걸으면 원인 후보가 맞는 것이다 (보기용 바닥은 그대로이므로 센서 쪽은 따로 봐야 한다)

입력: data/<이름>/tiles/ (38번 결과)
출력: data/<이름>/tiles_flatfloor/ , data/<이름>/tiles_flatfloor.zip , data/<이름>/tiles_flatfloor_report.json

실행:
  python 45_tiles_flat_floor.py incheon_subway
"""
import argparse
import json
import re
import shutil
import zipfile

import numpy as np

from common import DATA_DIR

# ---- 바꿔볼 만한 파라미터 ----
FLOOR_Z = 0.0           # 평평하게 만든 바닥 높이 [m] (타일은 바닥 z = 0 으로 만든다)
FLOOR_TOP = 0.08        # 바닥 삼각형의 꼭짓점이 이 높이 이하여야 바닥으로 본다 [m]
FLOOR_LOW = -0.15       # 이 높이 이상이어야 한다 [m]
FLAT_NZ = 0.8           # 법선의 z 성분이 이 이상이면 수평으로 본다 (약 37도 이내)
MIN_AREA2 = 1e-10       # 외적 크기(넓이의 2배)가 이보다 작으면 넓이 0 으로 보고 지운다 [m²]
TILT_DEG = 5.0          # 보고용: 이 각도보다 기운 바닥 면적 비율을 센다

README_NOTE = """  - 시험용 사본: 충돌 OBJ 의 바닥(거의 수평이고 높이 -0.15 ~ 0.08 m 인 삼각형)을 z = 0 평면으로 평평하게 맞췄다 (45_tiles_flat_floor.py).
    삼각형을 지우거나 더하지 않고 꼭짓점 높이만 바꿨다. 보기용 메시와 world.sdf 는 그대로다. 바닥보다 높은 면은 그대로 둔다.
"""


def read_obj(path):
    """Open3D 가 쓴 형식(v / vn 번갈아, f a//a b//b c//c)의 OBJ 를 읽는다. 줄 번호를 함께 돌려줘서 일부만 고쳐 쓸 수 있다."""
    lines = path.read_text().splitlines(keepends=True)
    vi = [i for i, line in enumerate(lines) if line.startswith("v ")]
    ni = [i for i, line in enumerate(lines) if line.startswith("vn ")]
    fi = [i for i, line in enumerate(lines) if line.startswith("f ")]
    V = np.array([lines[i].split()[1:4] for i in vi], dtype=float)
    N = np.array([lines[i].split()[1:4] for i in ni], dtype=float)
    corners = [[t.split("//") for t in lines[i].split()[1:4]] for i in fi]
    assert len(V) == len(N) and all(len(c) == 2 for f in corners for c in f), f"{path}: v//vn 형식이 아니다"
    F = np.array([[int(c[0]) for c in f] for f in corners]) - 1
    FN = np.array([[int(c[1]) for c in f] for f in corners]) - 1
    assert (F == FN).all(), f"{path}: 면의 v 번호와 vn 번호가 다르다"
    return lines, vi, ni, fi, V, N, F


def face_normals(V, F):
    """면 법선 (길이 = 넓이의 2배)과 그 길이."""
    t = V[F]
    n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
    return n, np.linalg.norm(n, axis=1)


def floor_stats(V, F, floor):
    """바닥 삼각형의 면적 가중 높이 분포와 기운 면적 비율 (보고용)."""
    n, a2 = face_normals(V, F[floor])
    w = a2 / 2
    zc = V[F[floor]][:, :, 2].mean(axis=1)
    order = np.argsort(zc)
    cum = np.cumsum(w[order]) / w.sum()
    p05, p50, p95 = (float(zc[order][np.searchsorted(cum, q)]) for q in (0.05, 0.5, 0.95))
    tilt = np.degrees(np.arccos(np.clip(np.abs(n[:, 2]) / np.maximum(a2, 1e-18), 0, 1)))
    return {"area": float(w.sum()), "z_p05": p05, "z_p50": p50, "z_p95": p95,
            "tilt_area_ratio": float(w[tilt > TILT_DEG].sum() / w.sum())}


def flatten(path, floor_top):
    """path 의 충돌 OBJ 를 제자리에서 고친다. 통계를 돌려준다."""
    lines, vi, ni, fi, V, N, F = read_obj(path)
    n, a2 = face_normals(V, F)
    nz = np.abs(n[:, 2]) / np.maximum(a2, 1e-18)
    z = V[F][:, :, 2]
    floor = (nz > FLAT_NZ) & (z.max(axis=1) <= floor_top) & (z.min(axis=1) >= FLOOR_LOW)
    assert floor.any(), f"{path}: 바닥 삼각형이 없다"
    before = floor_stats(V, F, floor)

    flat_v = np.unique(F[floor])
    V2 = V.copy()
    V2[flat_v, 2] = FLOOR_Z
    n2, a2_2 = face_normals(V2, F)
    keep = a2_2 > MIN_AREA2
    dropped = int((~keep & (a2 > MIN_AREA2)).sum())      # 원래 있던 넓이 0 삼각형은 세지 않는다 (남겨 둔다)
    keep |= a2 <= MIN_AREA2

    # 바닥과 만나는 꼭짓점의 법선을 이웃 면으로 다시 구한다 (면적 가중 평균)
    touched = np.unique(F[keep & np.isin(F, flat_v).any(axis=1)])
    acc = np.zeros_like(V2)
    for k in range(3):
        np.add.at(acc, F[keep][:, k], n2[keep])
    length = np.linalg.norm(acc, axis=1)
    upd = touched[length[touched] > 1e-12]
    N2 = N.copy()
    N2[upd] = acc[upd] / length[upd, None]

    flat_set = set(flat_v.tolist())
    drop_line = {fi[k] for k in np.flatnonzero(~keep)}
    upd_set = set(upd.tolist())
    new = list(lines)      # 줄 번호가 밀리지 않게 원래 줄 번호로 먼저 바꿔 놓고 면 줄은 마지막에 지운다
    for k, i in enumerate(vi):
        if k in flat_set:
            t = lines[i].split()
            new[i] = f"v {t[1]} {t[2]} {FLOOR_Z:g}\n"
    for k, i in enumerate(ni):
        if k in upd_set:
            new[i] = f"vn {N2[k, 0]:.6g} {N2[k, 1]:.6g} {N2[k, 2]:.6g}\n"
    n_tri = int(keep.sum())
    text = "".join(line for i, line in enumerate(new) if i not in drop_line)
    text = re.sub(r"# number of triangles: \d+", f"# number of triangles: {n_tri}", text, count=1)
    path.write_text(text)

    F_kept = F[keep]
    floor_kept = floor[keep]
    after = floor_stats(V2, F_kept, floor_kept)
    return {"vertices": len(V), "triangles_before": len(F), "triangles_after": n_tri,
            "floor_triangles": int(floor.sum()), "flattened_vertices": len(flat_v), "dropped_degenerate": dropped,
            "normals_recomputed": len(upd), "max_vertex_shift": float(np.abs(V[flat_v, 2] - FLOOR_Z).max()),
            "before": before, "after": after}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (38번까지 마친 것)")
    parser.add_argument("--floor-top", type=float, default=FLOOR_TOP, help="바닥으로 볼 꼭짓점 최대 높이 [m]")
    args = parser.parse_args()

    src = DATA_DIR / args.name / "tiles"
    dst = DATA_DIR / args.name / "tiles_flatfloor"
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)

    tiles = json.loads((dst / "tiles.json").read_text())["tiles"]
    report = {}
    for t in tiles:
        s = flatten(dst / "models" / t["model"] / "collision.obj", args.floor_top)
        report[t["model"]] = s
        b, a = s["before"], s["after"]
        print(f"  {t['model']}: 바닥 삼각형 {s['floor_triangles']}개 ({b['area']:.1f} m²), 꼭짓점 {s['flattened_vertices']}개를 z={FLOOR_Z:g} 으로, "
              f"최대 {s['max_vertex_shift'] * 100:.1f} cm 이동")
        print(f"      바닥 높이 5/50/95% {b['z_p05'] * 100:+.1f}/{b['z_p50'] * 100:+.1f}/{b['z_p95'] * 100:+.1f} cm → "
              f"{a['z_p05'] * 100:+.1f}/{a['z_p50'] * 100:+.1f}/{a['z_p95'] * 100:+.1f} cm, "
              f"{TILT_DEG:g}도 넘게 기운 바닥 {b['tilt_area_ratio'] * 100:.1f}% → {a['tilt_area_ratio'] * 100:.1f}%, "
              f"지운 넓이 0 삼각형 {s['dropped_degenerate']}개, 법선 다시 계산 {s['normals_recomputed']}개")
    (DATA_DIR / args.name / "tiles_flatfloor_report.json").write_text(json.dumps(report, indent=1, ensure_ascii=False))

    readme = dst / "README.txt"
    text = readme.read_text()
    readme.write_text(text + ("" if text.endswith("\n") else "\n") + README_NOTE)

    # zip: 압축 풀면 world.sdf, tiles.json, models/ 가 바로 나오게 (tiles.zip 과 같은 구조)
    out = DATA_DIR / args.name / "tiles_flatfloor.zip"
    out.unlink(missing_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(dst.rglob("*")):
            if f.is_file():
                z.write(f, f.relative_to(dst))
    print(f"→ {dst}/ , {out} ({out.stat().st_size / 2 ** 20:.1f} MB)")


if __name__ == "__main__":
    main()
