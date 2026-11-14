"""
3단계-1: 실제 데이터 준비와 진단

어떤 데이터든 같은 파이프라인에 넣기 위한 첫 단계다. 파라미터를 숫자로 박아 두지 않고
데이터를 재서 추천하는 것이 목적이다.
  1) 불러오기: 형식과 좌표계를 공통 형태(미터, z가 위, 바닥 z = 0)로 맞춘다 (loaders.py)
  2) 다운샘플: 용도(시뮬레이터)에 필요한 해상도 --voxel 로 줄인다
  3) 진단:
     - 법선 반경: 반경마다 법선 방향이 '불확실'하거나 이웃이 너무 적은 점의 비율(나쁨 비율)을 재고,
       가장 좋은 값에서 KNEE_TOL 이내로 들어오는 가장 작은 반경을 고른다 (반경이 크면 모서리가 뭉개지므로).
       나쁨 비율은 합성 데이터에서 정답으로 쟀던 '옆으로 누움'을 정답 없이 재는 대체 지표다.
       · 듬성한 스캔(합성 방, 로봇 LiDAR): 반경을 키우면 나쁨 비율이 뚝 떨어지다가 평평해진다
       · 촘촘하지만 형상이 복잡한 곳(가구 장식, 천): 반경과 상관없이 남는다. 반경을 키우면 오히려 는다
       불확실 = PCA 고유값 λ1 ≥ λ2 ≥ λ3 에서 λ3 / λ2 > AMBIGUOUS_RATIO.
       법선은 가장 덜 퍼진 방향(λ3)이라서, 그다음 방향(λ2)과 퍼짐이 비슷하면 둘 중 어느 쪽인지 흔들린다.
       (처음에는 '선 모양' λ2 / λ1 을 썼는데, 합성 데이터 정답과 맞춰 보니 이 기준이 훨씬 잘 맞았다)
     - Poisson depth: (방 크기 x 1.1) / 2^depth 가 voxel 이하가 되는 가장 작은 depth
     - 부호 방식: 스캔 위치(--sensors)가 있으면 sensor, 없으면 tangent

출력: data/<이름>/prepared.ply, prepared_transform.txt, inspect.json

실행:
  python 31_prepare.py data/redwood_bedroom/raw/bedroom.ply --name redwood_bedroom
  python 31_prepare.py data/synth_room.ply --name synth_room --sensors data/synth_room_sensors.npy
"""
import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d

from common import DATA_DIR, height_colors
from loaders import level_floor, load_cloud

# ---- 바꿔볼 만한 파라미터 ----
RADII = [0.03, 0.05, 0.075, 0.10, 0.15]  # 법선 반경 후보 [m]
MAX_NN = 30            # 23_normals.py 와 같은 이웃 상한
MIN_NEIGHBORS = 4      # 이웃이 이보다 적으면 평면을 맞출 수 없다고 본다
AMBIGUOUS_RATIO = 0.3  # λ3 > AMBIGUOUS_RATIO x λ2 이면 법선 방향이 불확실
KNEE_TOL = 0.005       # 가장 좋은 나쁨 비율에서 이만큼(0.5%p) 이내면 충분하다고 본다 (합성 데이터 정답으로 맞춘 값)
BAD_THRESHOLD = 0.025  # 가장 좋은 나쁨 비율도 이보다 크면, 반경으로는 해결되지 않는 영역이 있다고 알린다
N_QUERY = 50_000       # 진단에 쓸 표본 점 수


def nn_spacing(pcd, n=3000, seed=0):
    """점에서 가장 가까운 다른 점까지의 거리 분포 (표본)."""
    P = np.asarray(pcd.points)
    q = np.random.default_rng(seed).choice(len(P), min(n, len(P)), replace=False)
    tree = o3d.geometry.KDTreeFlann(pcd)
    d = [np.sqrt(tree.search_knn_vector_3d(P[i], 2)[2][1]) for i in q]
    return np.percentile(d, [50, 90, 99])


def neighborhood_quality(P, radius, seed=0):
    """표본 점마다 반경 안 이웃의 퍼짐을 PCA로 재서 (이웃 부족 비율, 불확실 비율, 표본 번호, 점별 나쁨 여부)를 돌려준다."""
    q = np.random.default_rng(seed).choice(len(P), min(N_QUERY, len(P)), replace=False)
    nns = o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(P, dtype=o3d.core.float32))
    nns.hybrid_index(radius)
    idx, _, cnt = nns.hybrid_search(o3d.core.Tensor(P[q], dtype=o3d.core.float32), radius, MAX_NN)
    idx, cnt = idx.numpy(), cnt.numpy()
    w = (idx >= 0)[..., None].astype(np.float64)
    nb = P[np.where(idx >= 0, idx, 0)]
    k = np.maximum(w.sum(axis=1), 1)                       # 실제 이웃 수, (Q, 1)
    d = (nb - ((nb * w).sum(axis=1) / k)[:, None, :]) * w  # 이웃 평균을 뺀 좌표 (빈 칸은 0)
    ev = np.linalg.eigvalsh(np.einsum("qki,qkj->qij", d, d) / k[:, :, None])  # 오름차순: λ3, λ2, λ1
    sparse = cnt < MIN_NEIGHBORS
    ambiguous = ~sparse & (ev[:, 0] > AMBIGUOUS_RATIO * ev[:, 1])
    return sparse.mean(), ambiguous.mean(), q, ambiguous | sparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("input", help="포인트클라우드 파일")
    parser.add_argument("--name", help="결과를 넣을 data/ 아래 폴더 이름 (기본: 파일 이름)")
    parser.add_argument("--up", default="z", help="원본에서 위쪽 축: x, y, z, -x, -y, -z")
    parser.add_argument("--scale", type=float, default=1.0, help="단위 변환 배수 (mm 데이터면 0.001)")
    parser.add_argument("--voxel", type=float, default=0.02, help="다운샘플 크기 [m] = 목표 해상도")
    parser.add_argument("--sensors", help="스캔 위치 .npy (원본 좌표계, N x 3). 있으면 함께 변환해 저장")
    parser.add_argument("--transform", help="다른 데이터에서 구한 4x4 변환(prepared_transform.txt)을 그대로 씀. "
                                            "같은 좌표계에 정렬된 여러 클라우드를 정답과 같은 좌표로 맞출 때")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장 근처를 숨김")
    args = parser.parse_args()

    name = args.name or Path(args.input).stem
    out_dir = DATA_DIR / name
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1) 불러오기 + 좌표 맞추기
    raw = load_cloud(args.input, args.scale)
    print(f"[불러오기] {args.input}: 점 {len(raw.points):,}개, 색 {'있음' if raw.has_colors() else '없음'}")
    sp_raw = nn_spacing(raw)
    if args.transform:
        T = np.loadtxt(args.transform)
        leveled = o3d.geometry.PointCloud(raw)
        leveled.transform(T)
        info = {"tilt_deg": None, "floor_inlier_ratio": None}
        print(f"[바닥 맞추기] 생략: {args.transform}의 변환을 그대로 적용")
    else:
        leveled, T, info = level_floor(raw, up=args.up)
        print(f"[바닥 맞추기] 기울기 {info['tilt_deg']:.2f}° 보정, 바닥을 z = 0으로 이동 "
              f"(바닥 후보 중 평면 비율 {info['floor_inlier_ratio']:.0%})")

    # 2) 다운샘플
    pcd = leveled.voxel_down_sample(args.voxel)
    P = np.asarray(pcd.points)
    lo, hi = np.percentile(P, 0.5, axis=0), np.percentile(P, 99.5, axis=0)
    size = hi - lo
    sp = nn_spacing(pcd)
    print(f"[다운샘플 {args.voxel * 100:g} cm] 점 {len(P):,}개 | 크기 {size[0]:.2f} x {size[1]:.2f} m, "
          f"높이 {hi[2]:.2f} m (상하위 0.5% 제외)")
    print(f"  이웃 점 간격 중앙값: 원본 {sp_raw[0] * 1000:.1f} mm → 다운샘플 후 {sp[0] * 1000:.1f} mm")

    # 3) 진단
    print(f"\n[법선 반경 진단] 법선 방향이 불확실하거나 이웃이 {MIN_NEIGHBORS}개 미만인 점의 비율 (표본 {min(N_QUERY, len(P)):,}점)")
    table = []
    for r in RADII:
        sparse, ambiguous, _, _ = neighborhood_quality(P, r)
        table.append({"radius": r, "sparse": float(sparse), "ambiguous": float(ambiguous), "bad": float(sparse + ambiguous)})
    best = min(t["bad"] for t in table)
    recommended = next(t["radius"] for t in table if t["bad"] - best <= KNEE_TOL)
    for t in table:
        mark = "  ← 추천" if t["radius"] == recommended else ""
        print(f"  반경 {t['radius'] * 100:4.1f} cm: 이웃 부족 {t['sparse']:6.1%} + 불확실 {t['ambiguous']:6.1%} "
              f"= {t['bad']:6.1%}{mark}")
    if best > BAD_THRESHOLD:
        print(f"  ※ 어떤 반경에서도 {best:.1%}가 남음 → 스캔 간격이 아니라 형상 자체가 평면이 아닌 곳"
              "(장식, 천, 식물, 모서리)일 가능성이 큼. 반경을 키워도 해결되지 않음")

    depth = int(np.ceil(np.log2(size.max() * 1.1 / args.voxel)))
    sensors = None
    if args.sensors:
        S = np.load(args.sensors) * args.scale
        sensors = S @ T[:3, :3].T + T[:3, 3]
        np.save(out_dir / "sensors.npy", sensors)
    orient = "sensor (tangent와 비교 권장)" if sensors is not None else "tangent (스캔 위치 없음)"
    print(f"\n[추천] 법선 반경 {recommended * 100:.1f} cm | Poisson depth {depth} "
          f"(격자 {size.max() * 1.1 / 2 ** depth * 100:.1f} cm) | 부호 방식 {orient}")

    o3d.io.write_point_cloud(str(out_dir / "prepared.ply"), pcd)
    np.savetxt(out_dir / "prepared_transform.txt", T, fmt="%.9f")
    report = {
        "source": str(args.input), "points_raw": len(raw.points), "points": len(P), "voxel": args.voxel,
        "transform": T.tolist(), "tilt_deg": info["tilt_deg"], "size_m": size.tolist(), "height_m": float(hi[2]),
        "spacing_raw_m": sp_raw.tolist(), "spacing_m": sp.tolist(), "has_colors": pcd.has_colors(),
        "has_sensors": sensors is not None, "radius_table": table,
        "recommended": {"normal_radius": recommended, "poisson_depth": depth,
                        "orient": "sensor" if sensors is not None else "tangent"},
    }
    (out_dir / "inspect.json").write_text(json.dumps(report, indent=2, ensure_ascii=False))
    print(f"\n저장: {out_dir}/prepared.ply, prepared_transform.txt, inspect.json")

    if not args.no_view:
        view = o3d.geometry.PointCloud(pcd)
        if not view.has_colors():
            view.colors = o3d.utility.Vector3dVector(height_colors(P))
        if args.hide_ceiling:
            view = view.select_by_index(np.flatnonzero(P[:, 2] < hi[2] - 0.25))
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)  # 원점 = 방 가운데 바닥
        o3d.visualization.draw_geometries([view, axes], window_name=f"{name}: prepared")


if __name__ == "__main__":
    main()
