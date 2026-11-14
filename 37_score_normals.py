"""
3단계-2b: 법선 부호 채점 (정답 법선이 있을 때만)

32번이 저장한 법선(tangent / 가상 센서 / 혼합 / 최종 normals.ply)을 정답 법선과 비교한다.
  정답 법선: 21번 합성 방(레이가 맞은 면의 법선을 센서 쪽으로) 또는 NavVis E57(스캐너가 넣어 둔 법선)
  점마다 가장 가까운 정답 점의 법선과 내적을 구해서
    - 축 맞음: |cos| > 0.7 (45° 이내). 축이 다르면 부호를 따질 수 없어서 따로 비율만 보여 준다
    - 뒤집힘: 축이 맞는 점 중 cos < 0 인 비율. 수직면(벽, 옆면)과 수평·비스듬한 면으로 나눠서도 본다
  --map 이면 최종 법선(normals.ply)의 뒤집힘을 빨강으로 칠한 그림을 저장한다 (천장 근처는 숨김)

입력: data/<이름>/normals*.ply, prepared_transform.txt (31, 32번 결과) + --ref 정답 법선 파일 (원본 좌표)
출력: (--map) data/<이름>/normals_score.png

실행:
  python 37_score_normals.py synth_clutter --ref data/synth_clutter.ply
  python 37_score_normals.py factory --ref data/factory/raw/Factory-small.e57 --map
"""
import argparse

import numpy as np
import open3d as o3d

from common import DATA_DIR
from loaders import load_reference_normals

AXIS_COS = 0.7       # 축이 이만큼(45° 이내) 맞아야 부호를 채점한다
VERTICAL_NZ = 0.3    # |법선 z| 가 이보다 작으면 수직면
FILES = [("tangent", "normals_tangent.ply"), ("가상 센서", "normals_virtual.ply"),
         ("혼합", "normals_hybrid.ply"), ("최종 normals.ply", "normals.ply")]


def save_map(path, P, flipped):
    col = np.full((len(P), 3), 0.8)
    col[flipped] = (0.9, 0.1, 0.1)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(P))
    pcd.colors = o3d.utility.Vector3dVector(col)
    top = np.percentile(P[:, 2], 99)
    pcd = pcd.select_by_index(np.flatnonzero(P[:, 2] < 0.8 * top))  # 천장을 숨겨 안을 보이게
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=1200, height=850, visible=False)
    vis.add_geometry(pcd)
    vis.get_render_option().point_size = 1.5
    ctr = vis.get_view_control()
    ctr.set_up((0, 0, 1))
    ctr.set_front((-0.5, -0.7, 0.5))
    ctr.set_lookat(pcd.get_center())
    ctr.set_zoom(0.55)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(str(path), do_render=True)
    vis.destroy_window()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (32번까지 마친 것)")
    parser.add_argument("--ref", required=True, help="정답 법선이 든 점 파일 (.e57 또는 법선 있는 .ply), 원본 좌표")
    parser.add_argument("--map", action="store_true", help="최종 법선의 뒤집힘 그림을 저장")
    args = parser.parse_args()

    folder = DATA_DIR / args.name
    T = np.loadtxt(folder / "prepared_transform.txt")
    R, RN = load_reference_normals(args.ref)
    R, RN = R @ T[:3, :3].T + T[:3, 3], RN @ T[:3, :3].T  # 31번이 맞춘 좌표로
    pcd = o3d.io.read_point_cloud(str(folder / "normals_tangent.ply"))
    P, axis = np.asarray(pcd.points), np.asarray(pcd.normals)
    nns = o3d.core.nns.NearestNeighborSearch(o3d.core.Tensor(R, dtype=o3d.core.float32))
    nns.knn_index()
    idx, d2 = nns.knn_search(o3d.core.Tensor(P, dtype=o3d.core.float32), 1)
    ref = RN[idx.numpy()[:, 0]]
    ok = np.abs(np.einsum("ij,ij->i", axis, ref)) > AXIS_COS
    vert = np.abs(axis[:, 2]) < VERTICAL_NZ
    print(f"[{args.name}] 점 {len(P):,}개, 가장 가까운 정답 점까지 중앙값 {np.median(np.sqrt(d2.numpy())) * 1000:.1f} mm, "
          f"축이 맞는 점 {ok.mean():.1%} (나머지는 축부터 다름)")
    print(f"  뒤집힘 비율        {'전체':>8}{'수직면':>9}{'수평·비스듬':>11}")
    for label, fname in FILES:
        n = np.asarray(o3d.io.read_point_cloud(str(folder / fname)).normals)
        c = np.einsum("ij,ij->i", n, ref)
        print(f"  {label:<16}{np.mean(c[ok] < 0):>9.1%}{np.mean(c[ok & vert] < 0):>10.1%}{np.mean(c[ok & ~vert] < 0):>11.1%}")
        if args.map and fname == "normals.ply":
            save_map(folder / "normals_score.png", P, ok & (c < 0))
            print(f"  저장: {folder / 'normals_score.png'} (빨강 = 최종 법선이 정답과 반대)")


if __name__ == "__main__":
    main()
