"""
.ply 파일(포인트클라우드 또는 메시)을 3D 뷰어로 본다. 여러 파일을 겹쳐 볼 수도 있다.

실행:
  python view.py data/synth_room_gt.ply                     # 정답 메시
  python view.py data/synth_room_gt.ply --wireframe         # 삼각형 경계선까지 보기
  python view.py data/synth_room_gt.ply data/synth_room_clean.ply --hide-ceiling  # 메시 + 점 겹쳐 보기
  python view.py data/synth_room_gt.ply --save gt.png       # 창 없이 이미지로 저장

뷰어 조작: 드래그 회전, 휠 확대/축소, Ctrl+드래그 이동, +/- 점 크기, W 와이어프레임 켜고 끄기, Q 닫기
"""
import argparse

import numpy as np
import open3d as o3d

from common import height_colors

CEILING_MARGIN = 0.15  # --hide-ceiling: 가장 높은 곳에서 이만큼 아래부터 숨긴다 [m]


def load(path):
    """삼각형이 있으면 메시로, 없으면 포인트클라우드로 읽는다."""
    mesh = o3d.io.read_triangle_mesh(path)
    if len(mesh.triangles) > 0:
        return mesh
    return o3d.io.read_point_cloud(path)


def prepare_mesh(mesh, hide_ceiling):
    v = np.asarray(mesh.vertices)
    t = np.asarray(mesh.triangles)
    print(f"  메시: 정점 {len(v):,}개, 삼각형 {len(t):,}개")
    if hide_ceiling:
        t = t[v[t].mean(axis=1)[:, 2] < v[:, 2].max() - CEILING_MARGIN]  # 삼각형 중심 높이로 판단
    # 삼각형마다 정점을 따로 갖게 풀어 주면 면이 각지게 음영 처리되어 모양이 잘 보인다
    flat = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(v[t].reshape(-1, 3)),
        o3d.utility.Vector3iVector(np.arange(len(t) * 3).reshape(-1, 3)),
    )
    flat.compute_vertex_normals()
    flat.paint_uniform_color((0.75, 0.75, 0.72))
    return flat


def prepare_points(pcd, hide_ceiling):
    p = np.asarray(pcd.points)
    print(f"  포인트클라우드: 점 {len(p):,}개")
    if not pcd.has_colors():
        pcd.colors = o3d.utility.Vector3dVector(height_colors(p))
    if hide_ceiling:
        pcd = pcd.select_by_index(np.flatnonzero(p[:, 2] < p[:, 2].max() - CEILING_MARGIN))
    return pcd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("files", nargs="+", help="볼 .ply 파일들")
    parser.add_argument("--hide-ceiling", action="store_true", help="천장 근처를 숨김")
    parser.add_argument("--wireframe", action="store_true", help="메시의 삼각형 경계선 표시")
    parser.add_argument("--save", metavar="PNG", help="창을 띄우지 않고 이 경로에 이미지로 저장")
    args = parser.parse_args()
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)  # 점 파일을 메시로 읽어볼 때 나는 경고 숨김

    geoms = []
    for path in args.files:
        print(path)
        g = load(path)
        if isinstance(g, o3d.geometry.TriangleMesh):
            geoms.append(prepare_mesh(g, args.hide_ceiling))
        else:
            geoms.append(prepare_points(g, args.hide_ceiling))
    geoms.append(o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5))  # 빨강 x, 초록 y, 파랑 z

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=" + ".join(args.files), width=1280, height=800, visible=args.save is None)
    for g in geoms:
        vis.add_geometry(g)
    opt = vis.get_render_option()
    opt.mesh_show_back_face = True  # 방 안쪽에서 봐도 벽이 보이도록 뒷면도 그린다
    opt.mesh_show_wireframe = args.wireframe
    opt.point_size = 2.0
    ctr = vis.get_view_control()
    ctr.set_up((0, 0, 1))              # z축이 위
    ctr.set_front((-0.3, -0.5, 1.0))   # 앞쪽 벽에 가리지 않도록 높은 곳에서 비스듬히 내려다보기
    ctr.set_zoom(0.7)

    if args.save:
        vis.poll_events()
        vis.update_renderer()
        vis.capture_screen_image(args.save, do_render=True)
        print(f"저장: {args.save}")
    else:
        vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
