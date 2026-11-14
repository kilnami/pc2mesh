"""
4단계-2: PyBullet에 불러와 확인하기

34번이 만든 scene.urdf를 실제 물리 엔진에 넣어 본다. 화면 없이(DIRECT 모드) 돌린다.
  1) 불러오기: 충돌 모양이 만들어지는지, 크기가 메시와 같은지
  2) 광선 검사: 위에서 아래로 GRID 간격 격자 광선을 쏴서, 물리 엔진이 아는 충돌 표면의 높이를
     34번 충돌 메시(Open3D로 같은 광선을 쏜 결과)와 비교한다. 같으면 메시가 그대로 들어간 것이다
  3) 떨어뜨리기: 표면이 있는 곳에서, 위(RAY_TOP)에서 쏜 광선이 처음 맞은 표면보다 DROP_ABOVE 위에
     공(반지름 BALL_R) N_BALLS개를 놓고 SIM_SECONDS초 굴린다. (높이를 하나로 박으면 공장처럼 키 큰 물체가 많은
     곳에서 공이 물체 속에 생긴다)
     - 표면 위에 멈춘 공: 멈춘 높이가 '공 바로 아래 메시 표면 + 반지름'과 맞는지 (안 맞으면 메시와 다른 것에 얹힌 것)
     - 뚫고 빠진 공: 아래에 메시가 있는데 그 아래로 떨어진 공 (충돌 메시의 구멍)
     - 가장자리 밖으로 떨어진 공: 잘라 낸 장면의 바깥으로 굴러 나간 공 (Hall 101은 조각이라 바깥에 바닥이 없다)
  4) 사진: 공이 멈춘 모습을 PyBullet 내장 렌더러로 찍는다 (export/pybullet.png)

--convex: URDF에서 concave="yes"를 뺀 채로 불러온다. 그러면 PyBullet은 충돌 메시를 볼록 껍질 하나로 바꿔서
  장면 전체가 꽉 찬 덩어리가 된다. 공이 바닥까지 못 내려오고 덩어리 위에 얹힌다 (MuJoCo는 늘 이렇게 한다).
  concave="yes"는 <collision> 태그에 붙어야 먹는다. <link>에 붙이면 무시된다 (직접 확인함).

입력: data/<이름>/export/scene.urdf, collision.obj (34번 결과)
출력: data/<이름>/export/pybullet.png (--convex 면 pybullet_convex.png)

실행:
  python 35_check_pybullet.py hall101/ref
  python 35_check_pybullet.py hall101/ref --convex
  python 35_check_pybullet.py factory --eye=-4.5,-7.5,3.5 --target=1,2,0.8   # 사진 시점 (음수는 = 로 붙여 씀)
"""
import argparse
import os
import tempfile

import numpy as np
import open3d as o3d
import pybullet as p

from common import DATA_DIR

# ---- 바꿔볼 만한 파라미터 ----
GRID = 0.1             # 광선 검사 격자 간격 [m]
RAY_TOP = 3.0          # 광선을 쏘기 시작하는 높이 [m]. 천장이 있으면 천장보다 낮아야 한다
RAY_BATCH = 10_000     # PyBullet은 한 번에 쏠 수 있는 광선 수에 한도가 있어서 나눠 쏜다
BALL_R = 0.05          # 떨어뜨릴 공 반지름 [m]
N_BALLS = 60           # 떨어뜨릴 공 개수 (광선이 표면에 맞은 격자점 중에서 고름)
DROP_ABOVE = 0.5       # 아래 표면에서 이만큼 위에서 떨어뜨린다 [m]
SIM_SECONDS = 3.0      # 굴리는 시간 [s] (PyBullet 기본 1/240초 간격)
REST_TOL = 0.01        # 멈춘 높이가 '표면 + 반지름'에서 이 이내면 맞게 얹힌 것 [m]
IMAGE_SIZE = (1000, 750)
CAMERA_EYE, CAMERA_TARGET = (-1.9, -3.2, 2.8), (0.0, 0.0, 0.4)  # 사진 시점 (기본: Hall 101 책상에 맞춤)


def mesh_heights(mesh, xy, z_from=RAY_TOP):
    """Open3D로 z_from 높이(하나 또는 점마다)에서 아래로 광선을 쏴서 처음 맞는 표면 높이. 안 맞으면 nan."""
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    z0 = np.broadcast_to(z_from, len(xy))
    rays = np.c_[xy, z0, np.zeros((len(xy), 2)), -np.ones(len(xy))].astype(np.float32)
    t = scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy()
    return np.where(np.isfinite(t), z0 - t, np.nan)


def ray_grid(V):
    """충돌 메시 범위를 덮는 GRID 간격 격자점 (x, y)."""
    gx, gy = np.meshgrid(np.arange(V[:, 0].min(), V[:, 0].max(), GRID), np.arange(V[:, 1].min(), V[:, 1].max(), GRID))
    return np.c_[gx.ravel(), gy.ravel()]


def drop_positions(mesh):
    """공을 놓을 (x, y, z): 격자점 중 아래에 메시 표면이 있는 곳에서 N_BALLS개, 그 표면보다 DROP_ABOVE 위
    (난수 고정 → 36번 Gazebo와 같은 자리)."""
    xy = ray_grid(np.asarray(mesh.vertices))
    z = mesh_heights(mesh, xy)
    pick = np.random.default_rng(0).choice(np.flatnonzero(np.isfinite(z)), min(N_BALLS, np.isfinite(z).sum()), replace=False)
    return np.c_[xy[pick], z[pick] + DROP_ABOVE]


def report_balls(mesh, pos):
    """공이 끝난 위치 pos (N x 3)를 판정해 출력한다."""
    below = pos[:, 2] < -BALL_R
    # 남은 공은 중심에서 아래로 쏜 광선 = 받치고 있는 표면 (책상 밑으로 굴러간 공은 책상 윗면이 아니라 바닥)
    # 빠진 공은 위(RAY_TOP)에서 쏴서, 그 자리에 메시가 있었는지만 본다
    under = mesh_heights(mesh, pos[:, :2], np.where(below, RAY_TOP, pos[:, 2]))
    through = below & np.isfinite(under)    # 아래에 메시가 있는데 그 밑으로 빠짐
    off_edge = below & ~np.isfinite(under)  # 장면 바깥으로 굴러 떨어짐
    stay = ~below
    fits = stay & (np.abs(pos[:, 2] - (under + BALL_R)) < REST_TOL)
    print(f"[떨어뜨리기] 공 {len(pos)}개를 표면 {DROP_ABOVE} m 위에서 {SIM_SECONDS:.0f}초: 표면 위에 남음 {stay.sum()}개 "
          f"(그중 높이가 '아래 메시 표면 + 반지름'과 맞음 {fits.sum()}개, 바닥 {np.sum(stay & (pos[:, 2] < 0.1 + BALL_R))}개), "
          f"뚫고 빠짐 {through.sum()}개, 가장자리 밖으로 떨어짐 {off_edge.sum()}개")
    if (stay & ~fits).any():
        gap = pos[stay & ~fits, 2] - (under[stay & ~fits] + BALL_R)
        print(f"  높이가 안 맞는 공 {np.sum(stay & ~fits)}개: 아래 메시 표면 + 반지름보다 중앙값 {np.nanmedian(gap) * 100:.1f} cm "
              f"{'위' if np.nanmedian(gap) > 0 else '아래'} (모서리에 걸쳤거나, 메시에 없는 것 위에 얹혔거나, 파묻힘)")


def bullet_heights(xy):
    """같은 광선을 PyBullet 충돌 모양에 쏜 결과."""
    hits = []
    for s in range(0, len(xy), RAY_BATCH):
        q = xy[s:s + RAY_BATCH]
        hits += p.rayTestBatch([(x, y, RAY_TOP) for x, y in q], [(x, y, -1.0) for x, y in q])
    return np.array([h[3][2] if h[0] >= 0 else np.nan for h in hits])


def parse_xyz(text, default):
    return tuple(float(v) for v in text.split(",")) if text else default


def save_image(path, eye=CAMERA_EYE, target=CAMERA_TARGET):
    w, h = IMAGE_SIZE
    view = p.computeViewMatrix(cameraEyePosition=eye, cameraTargetPosition=target, cameraUpVector=(0, 0, 1))
    proj = p.computeProjectionMatrixFOV(fov=50, aspect=w / h, nearVal=0.1, farVal=50)
    rgb = p.getCameraImage(w, h, view, proj, renderer=p.ER_TINY_RENDERER)[2]
    img = np.reshape(np.asarray(rgb, dtype=np.uint8), (h, w, 4))[:, :, :3]
    o3d.io.write_image(str(path), o3d.geometry.Image(np.ascontiguousarray(img)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (34번까지 마친 것)")
    parser.add_argument("--convex", action="store_true", help='concave="yes"를 빼고 불러와 볼록 껍질 하나가 되는 것을 확인')
    parser.add_argument("--no-image", action="store_true", help="사진을 찍지 않음")
    parser.add_argument("--eye", help="사진 카메라 위치 x,y,z [m] (기본: Hall 101 책상에 맞춘 값)")
    parser.add_argument("--target", help="사진 카메라가 바라보는 점 x,y,z [m]")
    args = parser.parse_args()

    out = DATA_DIR / args.name / "export"
    urdf = out / "scene.urdf"
    path = str(urdf)
    if args.convex:  # 메시 경로가 URDF 기준 상대 경로라서 같은 폴더에 임시 파일로 만든다
        tmp = tempfile.NamedTemporaryFile("w", dir=out, suffix=".urdf", delete=False)
        tmp.write(urdf.read_text().replace('<collision concave="yes">', "<collision>"))
        tmp.close()
        path = tmp.name
    p.connect(p.DIRECT)
    body = p.loadURDF(path, useFixedBase=True)
    if args.convex:
        os.remove(path)

    # 1) 불러오기
    mesh = o3d.io.read_triangle_mesh(str(out / "collision.obj"))
    V = np.asarray(mesh.vertices)
    lo, hi = p.getAABB(body)
    print(f"[불러오기] {urdf.name}{' (concave 속성 뺌)' if args.convex else ''}: 충돌 모양 "
          f"{len(p.getCollisionShapeData(body, -1))}개, 크기 {np.round(np.subtract(hi, lo), 2)} m "
          f"(충돌 메시 {np.round(np.ptp(V, axis=0), 2)} m)")

    # 2) 광선 검사
    xy = ray_grid(V)
    z_mesh, z_bullet = mesh_heights(mesh, xy), bullet_heights(xy)
    on = np.isfinite(z_mesh)
    diff = np.abs(z_bullet[on] - z_mesh[on])
    print(f"[광선 검사] 메시에 맞은 광선 {on.sum()}개 중 PyBullet도 같은 높이(1 cm 이내)에서 맞은 비율 {np.mean(diff < 0.01):.1%}, "
          f"높이 차이 중앙값 {np.nanmedian(diff) * 100:.1f} cm")

    # 3) 떨어뜨리기
    p.setGravity(0, 0, -9.81)
    col = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_R)
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=BALL_R, rgbaColor=(0.9, 0.15, 0.1, 1))
    balls = [p.createMultiBody(0.1, col, vis, basePosition=tuple(xyz)) for xyz in drop_positions(mesh)]
    for _ in range(int(SIM_SECONDS * 240)):
        p.stepSimulation()
    report_balls(mesh, np.array([p.getBasePositionAndOrientation(b)[0] for b in balls]))

    # 4) 사진
    if not args.no_image:
        img = out / ("pybullet_convex.png" if args.convex else "pybullet.png")
        save_image(img, parse_xyz(args.eye, CAMERA_EYE), parse_xyz(args.target, CAMERA_TARGET))
        print(f"\n저장: {img}")
    p.disconnect()


if __name__ == "__main__":
    main()
