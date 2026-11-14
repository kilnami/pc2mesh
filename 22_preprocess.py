"""
2단계-2: 전처리 = 복셀 다운샘플링 + 통계적 이상치 제거

왜 하나?
  1) 점 밀도가 센서 근처는 빽빽하고 먼 곳은 듬성듬성하다. 밀도가 들쭉날쭉하면
     재구성 파라미터를 하나로 정하기 어렵다. 복셀 다운샘플링으로 밀도를 고르게 만든다.
  2) 허공에 뜬 점은 재구성할 때 가짜 면을 만든다. 통계적 이상치 제거로 걸러낸다.

합성 데이터라서 정답 메시까지의 거리로 '진짜 이상치'가 무엇인지 안다.
그래서 파라미터를 바꿀 때마다 얼마나 잘 걸렀는지 숫자로 확인할 수 있다.
(실제 데이터에는 정답이 없어서 눈으로 볼 수밖에 없다.)

입력: data/synth_room.ply, data/synth_room_gt.ply
출력: data/synth_room_clean.ply

실행:
  python 22_preprocess.py                 # 결과를 뷰어로 보기 (빨간 점 = 제거된 점)
  python 22_preprocess.py --hide-ceiling  # 천장을 숨기고 보기
  python 22_preprocess.py --no-view
"""
import argparse
import unicodedata

import numpy as np
import open3d as o3d

from common import DATA_DIR, height_colors

# ---- 바꿔볼 만한 파라미터 ----
VOXEL_SIZE = 0.02   # 복셀(정육면체) 한 변 [m]. 한 복셀 안의 점들을 평균 1개로 합친다
NB_NEIGHBORS = 20   # 이상치 판단 때 볼 이웃 점 개수
STD_RATIO = 2.0     # 이웃까지 평균 거리가 '전체 평균 + STD_RATIO x 표준편차'보다 크면 이상치로 본다
OUTLIER_DIST = 0.05  # 평가용: 정답 표면에서 이 거리 이상 떨어진 점을 '진짜 이상치'로 친다 [m]


def dist_to_gt(scene, pcd):
    """각 점에서 정답 메시 표면까지의 최단 거리."""
    pts = o3d.core.Tensor(np.asarray(pcd.points), dtype=o3d.core.float32)
    return scene.compute_distance(pts).numpy()


def report(name, d):
    bad = d > OUTLIER_DIST
    rms = np.sqrt(np.mean(d[~bad] ** 2))
    width = sum(2 if unicodedata.east_asian_width(c) == "W" else 1 for c in name)  # 한글은 2칸
    print(f"{name}{' ' * (12 - width)}점 {len(d):>8,}개 | 진짜 이상치 {bad.sum():>4,}개 | "
          f"표면 노이즈(RMS) {rms * 100:.2f} cm")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--no-view", action="store_true", help="3D 뷰어를 띄우지 않음")
    parser.add_argument("--hide-ceiling", action="store_true", help="뷰어에서 천장 근처 점을 숨김")
    args = parser.parse_args()

    raw = o3d.io.read_point_cloud(str(DATA_DIR / "synth_room.ply"))
    gt = o3d.io.read_triangle_mesh(str(DATA_DIR / "synth_room_gt.ply"))
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(gt))

    report("원본", dist_to_gt(scene, raw))

    # 1) 복셀 다운샘플링: 공간을 VOXEL_SIZE 격자로 나누고, 칸마다 점들의 평균 1개만 남긴다
    down = raw.voxel_down_sample(VOXEL_SIZE)
    d_down = dist_to_gt(scene, down)
    report("다운샘플", d_down)

    # 2) 통계적 이상치 제거: 이웃과 유난히 멀리 떨어진 점을 지운다
    clean, keep = down.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO)
    report("이상치제거", d_down[keep])

    removed = np.ones(len(down.points), dtype=bool)
    removed[keep] = False
    bad = d_down > OUTLIER_DIST
    print(f"\n진짜 이상치 {bad.sum():,}개 중 {(removed & bad).sum():,}개를 제거 "
          f"({(removed & bad).sum() / max(bad.sum(), 1):.0%})")
    print(f"멀쩡한 점 {(~bad).sum():,}개 중 {(removed & ~bad).sum():,}개를 잘못 제거 "
          f"({(removed & ~bad).sum() / (~bad).sum():.1%})")

    o3d.io.write_point_cloud(str(DATA_DIR / "synth_room_clean.ply"), clean)
    print(f"\n저장: {DATA_DIR}/synth_room_clean.ply")

    if not args.no_view:
        pts = np.asarray(down.points)
        view = o3d.geometry.PointCloud(down)
        colors = height_colors(pts) * 0.6 + 0.4  # 남은 점은 옅게
        colors[removed] = (1.0, 0.0, 0.0)        # 제거된 점은 빨강
        view.colors = o3d.utility.Vector3dVector(colors)
        if args.hide_ceiling:
            view = view.select_by_index(np.flatnonzero(pts[:, 2] < pts[:, 2].max() - 0.15))
        o3d.visualization.draw_geometries([view], window_name="preprocess: red = removed")


if __name__ == "__main__":
    main()
