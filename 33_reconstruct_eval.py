"""
3단계-3: 표면 재구성 + 정답 스캔과 비교

24번(합성 방)에서 했던 재구성을 실제 데이터(32번 결과)에 돌리고, 더 좋은 스캐너로 찍은 정답 스캔과 비교한다.
  재구성: Poisson raw / Poisson trim / Ball Pivoting. 해상도는 inspect.json 의 voxel, depth 를 따른다
  부호 비교: --orient tangent virtual hybrid 이면 32번이 저장한 법선마다 메시를 만들어 비교한다
  남의 메시: --mesh 로 다른 프로그램이 만든 메시(원본 좌표)에 prepared_transform.txt 를 적용해 같은 기준으로 잰다

정답이 '측정한 점'일 때 주의할 점
  - 정답 = 정답 스캐너가 '본 곳'이지 '진짜 표면 전체'가 아니다. 정답 스캐너가 못 본 곳(가구 밑, 가려진 면)에
    만든 면은 맞더라도 '정확하지 않음'으로 잡힌다. 그래서 절대값보다 같은 기준에서의 상대 비교로 읽는다.
    정답 스캔 자체를 입력으로 넣은 결과(Hall 101의 ref)가 이 기준에서 낼 수 있는 상한선이다.
  - 정답이 큰 공간에서 잘라낸 조각이면, 위에서 내려다본 정답 윤곽 안에서만 잰다 (가장자리 EDGE_CELLS칸 제외).
    조각 밖까지 있는 메시(예: Matterport 메시)를 공정하게 비교하기 위해서다. 정답이 방 전체면 --no-crop.

지표 (24번과 같음. 정답이 메시에서 점으로 바뀌었을 뿐)
  정확도: 만든 메시 위에서 고르게 뽑은 점 중 정답 점에서 OK_DIST 이내인 비율 (지어낸 면이 많으면 낮아짐)
  완성도: 정답 점 중 만든 메시에서 OK_DIST 이내인 비율 (구멍이 많으면 낮아짐)
  F     : 둘의 조화평균. 둘 중 하나만 높으면 낮게 나온다 (한 숫자로 순위를 볼 때)

입력: data/<이름>/normals*.ply, inspect.json, prepared_transform.txt (31, 32번 결과) + 정답 점 (--gt)
출력: data/<이름>/mesh_<방법>_<부호>.ply

실행:
  # 정답 준비 (한 번): 정답 스캔을 ref와 같은 좌표로, 5 mm 간격으로
  python 31_prepare.py data/hall101/raw/Hall_101_segment/Hall_101_segment_RTC360_reference.e57 \
      --name hall101/gt --voxel 0.005 --transform data/hall101/ref/prepared_transform.txt --no-view
  python 33_reconstruct_eval.py hall101/blk_leica --gt data/hall101/gt/prepared.ply --orient tangent virtual
  python 33_reconstruct_eval.py hall101/blk_mp --gt data/hall101/gt/prepared.ply \
      --mesh data/hall101/raw/Hall_101_segment/Hall_101_segment_BLK360_mesh.ply
뷰어 색: 정답까지의 거리. 파랑 = 정확, 빨강 = 2 x OK_DIST 이상 떨어짐, 회색 = 평가 영역 밖
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import open3d as o3d

from common import DATA_DIR

# ---- 바꿔볼 만한 파라미터 ----
DENSITY_QUANTILE = 0.05    # Poisson trim: 점 밀도 하위 몇 %의 정점을 잘라낼지 (24번과 같음)
BPA_RADII_X = [1.5, 3, 6]  # Ball Pivoting 공 반지름 = voxel x 이 값들 (voxel 2 cm면 24번과 같은 3, 6, 12 cm)
OK_DIST = 0.03             # 평가: 정답에서 이 거리 이내면 맞는 표면으로 본다 [m] (24번과 같음)
N_SAMPLES = 200_000        # 평가: 메시 위에서 고르게 뽑을 점 개수
FOOTPRINT_CELL = 0.05      # 평가 영역: 위에서 내려다본 정답 윤곽을 이 격자로 잰다 [m]
EDGE_CELLS = 2             # 평가 영역: 윤곽 가장자리에서 이만큼의 칸은 뺀다 (조각을 잘라낸 경계라서)
ORIENT_FILES = {"final": "normals.ply", "tangent": "normals_tangent.ply", "virtual": "normals_virtual.ply",
                "hybrid": "normals_hybrid.ply"}


def footprint(G):
    """위에서 내려다본 정답 윤곽 (점이 하나라도 있는 칸에서 가장자리를 깎은 것). (xy가 안에 있는지 판단하는 함수, 넓이)"""
    lo = G[:, :2].min(axis=0)
    ij = np.floor((G[:, :2] - lo) / FOOTPRINT_CELL).astype(int)
    grid = np.zeros(ij.max(axis=0) + 1, dtype=bool)
    grid[ij[:, 0], ij[:, 1]] = True
    for _ in range(EDGE_CELLS):  # 깎기: 상하좌우 칸이 모두 채워진 칸만 남긴다
        g = np.pad(grid, 1)
        grid = g[1:-1, 1:-1] & g[:-2, 1:-1] & g[2:, 1:-1] & g[1:-1, :-2] & g[1:-1, 2:]

    def inside(xy):
        ij = np.floor((xy - lo) / FOOTPRINT_CELL).astype(int)
        ok = np.all((ij >= 0) & (ij < grid.shape), axis=1)
        ok[ok] = grid[ij[ok, 0], ij[ok, 1]]
        return ok
    return inside, grid.sum() * FOOTPRINT_CELL ** 2


def reconstruct(pcd, depth, voxel):
    """24번과 같은 세 가지 메시. {이름: (메시, 걸린 시간)}"""
    out = {}
    t0 = time.time()
    raw, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=depth)
    density = np.asarray(density)
    out["poisson_raw"] = (raw, time.time() - t0)
    trimmed = o3d.geometry.TriangleMesh(raw)
    trimmed.remove_vertices_by_mask(density < np.quantile(density, DENSITY_QUANTILE))
    out["poisson_trim"] = (trimmed, time.time() - t0)
    t0 = time.time()
    radii = o3d.utility.DoubleVector([voxel * x for x in BPA_RADII_X])
    out["bpa"] = (o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, radii), time.time() - t0)
    return out


def nearest_distance(points, gt):
    """점마다 가장 가까운 정답 점까지의 거리."""
    return np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)).compute_point_cloud_distance(gt))


def evaluate(mesh, gt, gt_eval, inside):
    """(정확도, 완성도, F). gt = 정답 점 전체, gt_eval = 평가 영역 안의 정답 점 (N x 3)."""
    S = np.asarray(mesh.sample_points_uniformly(N_SAMPLES).points)
    acc = np.mean(nearest_distance(S[inside(S[:, :2])], gt) < OK_DIST)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    comp = np.mean(scene.compute_distance(o3d.core.Tensor(gt_eval, dtype=o3d.core.float32)).numpy() < OK_DIST)
    return acc, comp, 2 * acc * comp / max(acc + comp, 1e-9)


def error_colors(mesh, gt, inside):
    """정점마다 정답까지의 거리로 색칠: 0 이면 파랑, 2 x OK_DIST 이상이면 빨강, 평가 영역 밖은 회색."""
    V = np.asarray(mesh.vertices)
    s = np.clip(nearest_distance(V, gt) / (2 * OK_DIST), 0, 1)
    col = np.stack([s, 0.2 * (1 - s), 1 - s], axis=1)
    col[~inside(V[:, :2])] = 0.7
    return col


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (32번까지 마친 것)")
    parser.add_argument("--gt", required=True, help="정답 점 파일. 입력과 같은 좌표계여야 함 (31_prepare.py --transform 으로 준비)")
    parser.add_argument("--orient", nargs="+", choices=list(ORIENT_FILES), default=["final"],
                        help="어떤 법선으로 재구성할지 (final = 32번이 고른 normals.ply)")
    parser.add_argument("--mesh", nargs="*", default=[], help="같이 비교할 외부 메시 (원본 좌표)")
    parser.add_argument("--no-crop", action="store_true", help="정답이 장면 전체일 때: 정답 윤곽으로 자르지 않음")
    parser.add_argument("--view", help="뷰어로 볼 결과 (표 맨 앞의 이름. 기본: 첫 번째 poisson_trim)")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    args = parser.parse_args()

    folder = DATA_DIR / args.name
    info = json.loads((folder / "inspect.json").read_text())
    voxel, depth = info["voxel"], info["recommended"]["poisson_depth"]
    gt = o3d.io.read_point_cloud(args.gt)
    G = np.asarray(gt.points)
    if args.no_crop:
        inside, area = (lambda xy: np.ones(len(xy), dtype=bool)), None
    else:
        inside, area = footprint(G)
    gt_eval = G[inside(G[:, :2])]
    print(f"정답: {args.gt}, 점 {len(G):,}개 | 평가 영역 안 {len(gt_eval):,}개"
          + (f" (위에서 본 넓이 {area:.1f} m²)" if area else ""))
    print(f"재구성: voxel {voxel * 100:.0f} cm, Poisson depth {depth}, "
          f"BPA 공 {', '.join(f'{voxel * x * 100:.0f}' for x in BPA_RADII_X)} cm | 평가 기준 OK_DIST {OK_DIST * 100:.0f} cm")

    results = {}
    for orient in args.orient:
        pcd = o3d.io.read_point_cloud(str(folder / ORIENT_FILES[orient]))
        for method, (mesh, dt) in reconstruct(pcd, depth, voxel).items():
            results[f"{method}_{orient}"] = (mesh, dt)
            o3d.io.write_triangle_mesh(str(folder / f"mesh_{method}_{orient}.ply"), mesh)
    T = np.loadtxt(folder / "prepared_transform.txt")
    for path in args.mesh:
        mesh = o3d.io.read_triangle_mesh(path)
        mesh.transform(T)  # 원본 좌표 -> 31번에서 맞춘 좌표
        results[Path(path).stem] = (mesh, None)

    width = max(len(k) for k in results) + 2
    print(f"\n{'':<{width}}{'삼각형':>8}{'시간':>7}  정확도  완성도      F")
    for key, (mesh, dt) in results.items():
        acc, comp, f = evaluate(mesh, gt, gt_eval, inside)
        dt_text = f"{dt:5.1f}s" if dt is not None else "     -"
        print(f"{key:<{width}}{len(mesh.triangles):>10,}{dt_text:>7} {acc:>7.1%}{comp:>7.1%}{f:>7.1%}")
    print(f"\n저장: {folder}/mesh_<방법>_<부호>.ply")

    if not args.no_view:
        key = args.view or f"poisson_trim_{args.orient[0]}"
        mesh = o3d.geometry.TriangleMesh(results[key][0])
        mesh.vertex_colors = o3d.utility.Vector3dVector(error_colors(mesh, gt, inside))
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh], window_name=f"{key}: blue = accurate, red = invented, gray = not evaluated",
                                          mesh_show_back_face=True)


if __name__ == "__main__":
    main()
