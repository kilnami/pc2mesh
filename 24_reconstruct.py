"""
2단계-4: 표면 재구성 (점 + 법선 -> 삼각형 메시)

대표적인 두 방법을 비교한다.
  Poisson      : 법선을 이용해 공간 전체에 '벽 속 / 빈 공간' 함수를 만들고, 그 경계면을 메시로 뽑는다.
                 매끈하고 구멍을 메워 주지만, 점이 없는 곳에도 그럴듯한 막을 지어낸다.
                 그래서 보통 '점 밀도가 낮은 곳(= 지어낸 곳)'의 정점을 잘라내는 후처리를 같이 한다.
                 자르기 전(raw)과 후(trim)를 둘 다 만든다.
  Ball Pivoting: 반지름 r인 공을 점들 위로 굴려서, 공이 세 점에 동시에 닿으면 삼각형을 만든다.
                 점에 충실하지만, 점 간격이 공보다 넓은 곳은 구멍으로 남는다.

정답 메시와 비교해서 잰다.
  정확도   : 만든 메시 위의 점 중 정답 표면에서 OK_DIST 이내인 비율 (지어낸 면이 많으면 낮아짐)
  완성도   : 정답 표면 중 만든 메시가 OK_DIST 이내로 덮은 비율 (구멍이 많으면 낮아짐). 부위별로도 본다
  책상 윗면: 스캔에 점이 하나도 없던 곳. 메시가 어느 높이에 덮개를 만들었는지 (정답 0.75 m)

입력: data/synth_room_normals.ply, data/synth_room_gt.ply
출력: data/synth_room_poisson_raw.ply, synth_room_poisson.ply (trim), synth_room_bpa.ply

실행:
  python 24_reconstruct.py                           # 셋 다 만들고 Poisson(trim) 결과 보기
  python 24_reconstruct.py --view poisson_raw --hide-ceiling
  python 24_reconstruct.py --no-view
뷰어 색: 정답 표면까지의 거리. 파랑 = 정확, 빨강 = 2 x OK_DIST 이상 떨어짐(지어낸 면)
"""
import argparse
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR

# ---- 바꿔볼 만한 파라미터 ----
POISSON_DEPTH = 9               # 옥트리 깊이. 1 늘면 격자가 절반으로 촘촘해진다. 9면 약 1.3 cm
DENSITY_QUANTILE = 0.05         # Poisson trim: 점 밀도 하위 몇 %의 정점을 잘라낼지
BPA_RADII = [0.03, 0.06, 0.12]  # Ball Pivoting: 굴릴 공의 반지름들 [m]. 작은 공부터 굴리고 남은 틈을 큰 공으로
OK_DIST = 0.03                  # 평가: 정답 표면에서 이 거리 이내면 맞는 표면으로 본다 [m]
N_SAMPLES = 200_000             # 평가: 메시 위에서 고르게 뽑을 점 개수
CEILING_Z = 2.35                # --hide-ceiling: 중심이 이보다 높은 삼각형을 숨긴다 [m]

# 21번에서 방(삼각형 0~11) -> 책상(12~23) -> 기둥 순서로 합쳤다. 방 안에서는 벽 0~7, 천장 8~9, 바닥 10~11
REGIONS = [("바닥", 10, 12), ("벽", 0, 8), ("천장", 8, 10), ("책상", 12, 24), ("기둥", 24, 10**9)]
TABLE_TOP = ((2.1, 3.1), (0.4, 0.8))  # 책상 윗면 안쪽 x, y 범위 [m] (가장자리 제외)


def poisson(pcd):
    """자르기 전 메시와 정점별 밀도를 돌려준다."""
    mesh, density = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=POISSON_DEPTH)
    return mesh, np.asarray(density)


def trim(mesh, density):
    out = o3d.geometry.TriangleMesh(mesh)
    out.remove_vertices_by_mask(density < np.quantile(density, DENSITY_QUANTILE))
    return out


def ball_pivoting(pcd):
    return o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(BPA_RADII))


def scene_of(mesh):
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    return scene


def distance(scene, points):
    return scene.compute_distance(o3d.core.Tensor(points, dtype=o3d.core.float32)).numpy()


def table_top_height(pts):
    """책상 윗면 위쪽 공간에 메시가 있다면 그 높이의 중앙값. 없으면 None."""
    (x0, x1), (y0, y1) = TABLE_TOP
    x, y, z = pts.T
    above = (x > x0) & (x < x1) & (y > y0) & (y < y1) & (z > 0.3) & (z < 2.3)
    return np.median(z[above]) if above.sum() > 50 else None


def evaluate(mesh, gt_scene, gt_pts, gt_region):
    pts = np.asarray(mesh.sample_points_uniformly(N_SAMPLES).points)
    acc = np.mean(distance(gt_scene, pts) < OK_DIST)
    ok = distance(scene_of(mesh), gt_pts) < OK_DIST
    by_region = [np.mean(ok[gt_region == name]) for name, _, _ in REGIONS]
    return acc, np.mean(ok), by_region, table_top_height(pts)


def error_colors(mesh, gt_scene):
    """정점마다 정답 표면까지의 거리로 색칠: 0 이면 파랑, 2 x OK_DIST 이상이면 빨강."""
    s = np.clip(distance(gt_scene, np.asarray(mesh.vertices)) / (2 * OK_DIST), 0, 1)
    return np.stack([s, 0.2 * (1 - s), 1 - s], axis=1)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--view", choices=["poisson_raw", "poisson", "bpa"], default="poisson",
                        help="뷰어로 볼 결과 (poisson = 자른 뒤)")
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장을 숨김")
    args = parser.parse_args()

    pcd = o3d.io.read_point_cloud(str(DATA_DIR / "synth_room_normals.ply"))
    gt = o3d.io.read_triangle_mesh(str(DATA_DIR / "synth_room_gt.ply"))
    gt_scene = scene_of(gt)
    print(f"입력: 점 {len(pcd.points):,}개 (법선 포함)")

    # 정답 표면에서 고르게 뽑은 점과, 각 점이 어느 부위인지
    gt_pts = np.asarray(gt.sample_points_uniformly(N_SAMPLES).points)
    ids = gt_scene.compute_closest_points(o3d.core.Tensor(gt_pts, dtype=o3d.core.float32))["primitive_ids"].numpy()
    gt_region = np.full(len(gt_pts), "", dtype=object)
    for name, lo, hi in REGIONS:
        gt_region[(ids >= lo) & (ids < hi)] = name

    meshes = {}
    t0 = time.time()
    raw, density = poisson(pcd)
    meshes["poisson_raw"] = (raw, time.time() - t0)
    meshes["poisson"] = (trim(raw, density), time.time() - t0)
    t0 = time.time()
    meshes["bpa"] = (ball_pivoting(pcd), time.time() - t0)

    labels = {"poisson_raw": "Poisson raw", "poisson": f"Poisson trim{DENSITY_QUANTILE:.0%}", "bpa": "BallPivoting"}
    region_head = "".join(f"{name:>6}" for name, _, _ in REGIONS)
    print(f"\n{'':<17}{'삼각형':>9}{'시간':>7}  정확도  완성도 |{region_head} | 책상 윗면 높이")
    for key, (mesh, dt) in meshes.items():
        acc, comp, by_region, top = evaluate(mesh, gt_scene, gt_pts, gt_region)
        regions = "".join(f"{v:>8.0%}" for v in by_region)
        top_text = f"{top:.2f} m" if top is not None else "덮개 없음"
        print(f"{labels[key]:<17}{len(mesh.triangles):>10,}{dt:>6.1f}s {acc:>7.1%}{comp:>7.1%} |{regions} | {top_text}")
        o3d.io.write_triangle_mesh(str(DATA_DIR / f"synth_room_{key}.ply"), mesh)
    print(f"\n저장: {DATA_DIR}/synth_room_poisson_raw.ply, synth_room_poisson.ply, synth_room_bpa.ply")

    if not args.no_view:
        mesh = o3d.geometry.TriangleMesh(meshes[args.view][0])
        mesh.vertex_colors = o3d.utility.Vector3dVector(error_colors(mesh, gt_scene))
        if args.hide_ceiling:
            v, t = np.asarray(mesh.vertices), np.asarray(mesh.triangles)
            mesh.remove_triangles_by_mask(v[t].mean(axis=1)[:, 2] > CEILING_Z)
        mesh.compute_vertex_normals()
        o3d.visualization.draw_geometries([mesh], window_name=f"{args.view}: blue = accurate, red = invented",
                                          mesh_show_back_face=True)


if __name__ == "__main__":
    main()
