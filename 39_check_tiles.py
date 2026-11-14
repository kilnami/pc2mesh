"""
5단계-2: 타일 월드를 Gazebo에서 확인하기

38번이 만든 tiles/world.sdf를 파일 그대로 Gazebo(Fortress)에 넣어 본다. 화면 없이 서버만 돌린다.
  1) 기동 시간: 서버를 띄운 뒤 (a) 시뮬레이션 시간이 흐르기 시작할 때까지 (b) 카메라 센서가 첫 장면을 그릴 때까지(= 보기용
     메시를 다 불러온 시점) 걸린 시간과, 서버의 최대 메모리. 같은 월드를 '메시 파일 하나짜리'(34번 export)로도 똑같이 재서 비교한다.
     플러그인, 조명, 카메라는 같고 장면 메시만 다르다 (월드 파일에서 include 부분만 바꿈)
  2) 공 떨어뜨리기: 35·36번과 같은 무작위 공 + 타일 경계선 위에 SEAM_STEP 간격으로 놓은 공.
     경계에 틈이 있으면 경계 위 공이 빠진다. 판정은 35번과 같다 (충돌 메시는 타일 전부를 합친 것으로 판정).
     빠진 공은 이유를 가른다: 경계 틈 / 경계 양쪽에 원래 면이 없는 곳(스캔 안 된 구멍) / 굴러가서 다른 곳에서 빠짐.
     같은 공을 파일 하나짜리 월드에도 떨어뜨려 비교한다 (공장에는 원래 스캔 안 된 바닥 구멍이 있다)
  3) 사진: 월드에 카메라 센서 두 대 (전체, 경계 교차점 가까이)를 두고 Gazebo가 직접 그린 장면을 저장한다 (하나짜리도 같은 시점)

입력: data/<이름>/tiles/ (38번 결과), 비교용으로 data/<이름>/export/ (34번 결과, 없으면 비교 생략)
출력: data/<이름>/tiles_check/ (check_world*.sdf, 사진)

실행:
  python 39_check_tiles.py factory
  python 39_check_tiles.py factory --no-compare      # 메시 하나짜리와의 비교를 생략
"""
import argparse
import importlib
import json
import os
import re
import subprocess
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR

m35 = importlib.import_module("35_check_pybullet")  # 공 위치, 판정, 높이 계산
m36 = importlib.import_module("36_check_gazebo")    # ign 호출, 카메라 메시지, 공 SDF

# ---- 바꿔볼 만한 파라미터 ----
SEAM_STEP = 0.25            # 경계선 위에 공을 놓는 간격 [m]
SEAM_SIDE = 0.05            # 틈 판정: 경계에서 양쪽으로 이만큼 떨어진 곳에 면이 있는지 본다 [m]
STRAIGHT = 0.15             # 놓은 자리에서 이 이내에서 빠진 공은 '곧장 아래로' 빠진 것으로 본다 [m]
TIMEOUT = 600               # 서버가 준비되지 않으면 이 시간 [s] 뒤 포기
EYE, TARGET = (-4.5, -7.5, 3.5), (1.0, 2.0, 0.8)   # 전체 사진 시점 (공장 36번과 같음)
SEAM_EYE_OFFSET = (-2.5, -2.5, 1.6)                # 경계 교차점에서 이만큼 떨어진 곳에서 교차점을 본다

CAMERA = """    <model name="{name}">
      <static>true</static>
      <pose>{pose}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <topic>{name}</topic><update_rate>2</update_rate><always_on>1</always_on>
          <camera>
            <horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height></image>
            <clip><near>0.1</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
"""


def peak_memory_gb(pid):
    """프로세스와 그 자식들이 쓴 최대 상주 메모리(VmHWM)의 합 [GB]."""
    total, stack = 0.0, [pid]
    while stack:
        p = stack.pop()
        try:
            for line in open(f"/proc/{p}/status"):
                if line.startswith("VmHWM"):
                    total += int(line.split()[1]) / 2 ** 20
            for tid in os.listdir(f"/proc/{p}/task"):
                stack += [int(c) for c in open(f"/proc/{p}/task/{tid}/children").read().split()]
        except OSError:
            pass
    return total


def start_server(world, resource_path, engine, tag):
    env = dict(os.environ, IGN_PARTITION=f"pc2mesh_{tag}_{os.getpid()}", IGN_GAZEBO_RESOURCE_PATH=str(resource_path))
    server = subprocess.Popen(["ign", "gazebo", "-s", "-r", "-v", "1", "--headless-rendering",
                               "--physics-engine", m36.ENGINES[engine], str(world)],
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    return server, env


def stop(server):
    server.terminate()
    try:
        server.wait(timeout=10)
    except subprocess.TimeoutExpired:
        server.kill()


def startup(world, world_name, resource_path, engine, cam_topic, tag):
    """(시뮬레이션 시간이 흐르기 시작할 때까지 [s], 카메라 첫 장면까지 [s], 서버 최대 메모리 [GB])."""
    t0 = time.time()
    server, env = start_server(world, resource_path, engine, tag)
    t_sim = t_cam = None
    try:
        while t_cam is None:
            if server.poll() is not None:
                raise RuntimeError(f"Gazebo 서버가 멈춤:\n{server.stderr.read()[-2000:]}")
            if time.time() - t0 > TIMEOUT:
                raise RuntimeError(f"{TIMEOUT}초 안에 준비되지 않음")
            if t_sim is None:
                stats = m36.topic_once(f"/world/{world_name}/stats", env)
                if stats and m36.sim_seconds(stats) > 0:
                    t_sim = time.time() - t0
            elif m36.topic_once(f"/{cam_topic}", env):
                t_cam = time.time() - t0
        return t_sim, t_cam, peak_memory_gb(server.pid)
    finally:
        stop(server)


def single_text(world_text, single):
    """같은 월드 파일에서 타일 include만 빼고 34번 export 모델 하나를 넣는다 (나머지 플러그인·조명은 그대로)."""
    one = re.sub(r"<include>.*?</include>\s*", "", world_text, flags=re.S)
    return one.replace("</world>", f"    <include><uri>{single.resolve()}</uri><name>factory_single</name></include>\n  </world>")


def drop_test(world, world_name, resource_path, engine, cam_names, tag):
    """월드를 시뮬레이션 SIM_SECONDS까지 돌려 모든 물체의 위치와 카메라 사진을 받아 온다. (위치 메시지, 사진, 걸린 시간)"""
    t0 = time.time()
    server, env = start_server(world, resource_path, engine, tag)
    frames = {}
    try:
        sim_t = 0.0
        while sim_t < m35.SIM_SECONDS:
            if server.poll() is not None:
                raise RuntimeError(f"Gazebo 서버가 멈춤:\n{server.stderr.read()[-2000:]}")
            if time.time() - t0 > TIMEOUT:
                raise RuntimeError(f"{TIMEOUT}초 안에 시뮬레이션 {m35.SIM_SECONDS}초에 못 도달함 (현재 {sim_t:.2f}초)")
            stats = m36.topic_once(f"/world/{world_name}/stats", env)
            sim_t = m36.sim_seconds(stats) if stats else 0.0
        poses = m36.topic_once(f"/world/{world_name}/pose/info", env)
        for n in cam_names:
            frames[n] = m36.topic_once(f"/{n}", env)
    finally:
        stop(server)
    return poses, frames, time.time() - t0


def ball_positions(poses, n):
    found = {p["name"]: p.get("position", {}) for p in poses["pose"]}
    return np.array([[found[f"ball_{i}"].get(k, 0.0) for k in "xyz"] for i in range(n)])


def explain_fallen(start, end, axis, mesh):
    """경계선 위에서 놓은 공 중 바닥 아래로 간 공이 왜 빠졌는지 센다.
    곧장 아래로 빠졌으면(놓은 자리에서 STRAIGHT 이내) 놓은 자리의 경계 양쪽 SEAM_SIDE 떨어진 곳에 같은 높이의 면이
    둘 다 있는지 본다: 둘 다 있는데 빠졌으면 경계 틈, 한쪽이라도 없으면 원래 면이 없던 곳(스캔 안 된 구멍)."""
    fell = np.flatnonzero(end[:, 2] < -m35.BALL_R)
    gap = hole = rolled = 0
    for i in fell:
        if np.linalg.norm(end[i, :2] - start[i, :2]) > STRAIGHT:
            rolled += 1
            continue
        surface = start[i, 2] - m35.DROP_ABOVE
        step = np.zeros(2)
        step[axis[i]] = SEAM_SIDE
        sides = m35.mesh_heights(mesh, np.array([start[i, :2] - step, start[i, :2] + step]))
        if np.all(np.isfinite(sides) & (np.abs(sides - surface) < 0.2)):
            gap += 1
        else:
            hole += 1
    print(f"  빠진 공 {len(fell)}개: 경계 틈 {gap}개, 경계 양쪽에 원래 면이 없는 곳 {hole}개, 굴러가서 다른 곳에서 빠짐 {rolled}개")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (38번까지 마친 것)")
    parser.add_argument("--engine", choices=list(m36.ENGINES), default="dartsim")
    parser.add_argument("--no-compare", action="store_true", help="메시 하나짜리와 비교하지 않음")
    parser.add_argument("--repeat", type=int, default=2, help="기동 시간을 몇 번 재서 가장 짧은 값을 쓸지 (디스크 캐시 영향)")
    parser.add_argument("--eye", help="전체 사진 카메라 위치 x,y,z [m]")
    parser.add_argument("--target", help="전체 사진 카메라가 바라보는 점 x,y,z [m]")
    args = parser.parse_args()
    eye, target = m35.parse_xyz(args.eye, EYE), m35.parse_xyz(args.target, TARGET)

    folder = DATA_DIR / args.name
    pkg, single = folder / "tiles", folder / "export"
    out = folder / "tiles_check"
    out.mkdir(exist_ok=True)
    tiles = json.loads((pkg / "tiles.json").read_text())["tiles"]
    world_text = (pkg / "world.sdf").read_text()
    world_name = re.search(r'<world name="([^"]+)"', world_text).group(1)
    models = pkg / "models"
    compare = not args.no_compare and (single / "model.config").exists()

    # 1) 문법 검사 (world.sdf 그대로). ign sdf 는 Gazebo가 아니라서 모델 찾는 경로를 SDF_PATH 로 따로 줘야 한다
    env = dict(os.environ, IGN_PARTITION=f"pc2mesh_sdf_{os.getpid()}", SDF_PATH=str(models))
    r = m36.ign(["sdf", "-k", str(pkg / "world.sdf")], env)
    print(f"[문법 검사] ign sdf -k world.sdf: {r.stdout.strip() or r.stderr.strip()}")

    # 카메라 두 대: 전체, 경계 교차점 가까이
    xs = sorted({v for t in tiles for v in (t["bounds"][0], t["bounds"][2])})[1:-1]
    ys = sorted({v for t in tiles for v in (t["bounds"][1], t["bounds"][3])})[1:-1]
    w, h = m35.IMAGE_SIZE
    hfov = 2 * np.arctan(np.tan(m36.CAMERA_VFOV / 2) * w / h)
    cams = {"cam_overview": (eye, target)}
    if xs and ys:
        c = np.array([xs[0], ys[0], 0.5])
        cams["cam_seam"] = (tuple(c + SEAM_EYE_OFFSET), tuple(c))
    cam_xml = "".join(CAMERA.format(name=n, pose=m36.camera_pose(e, t), hfov=hfov, w=w, h=h) for n, (e, t) in cams.items())

    # 2) 기동 시간: 타일 월드 vs 메시 하나짜리 (카메라는 전체 시점 하나만)
    one_cam = CAMERA.format(name="cam_overview", pose=m36.camera_pose(eye, target), hfov=hfov, w=w, h=h)
    worlds = {"타일 월드": ("startup_tiles.sdf", world_text.replace("</world>", one_cam + "  </world>"), models)}
    if compare:
        worlds["메시 하나짜리"] = ("startup_single.sdf", single_text(world_text, single).replace("</world>", one_cam + "  </world>"), single)
    print("\n[기동 시간] (서버 시작 → 시뮬레이션 시간이 흐름 / 카메라 첫 장면 = 메시를 다 불러옴 | 서버 최대 메모리)")
    for label, (fname, text, res) in worlds.items():
        (out / fname).write_text(text)
        runs = [startup(out / fname, world_name, res, args.engine, "cam_overview", f"s{i}") for i in range(args.repeat)]
        t_sim, t_cam, mem = min(runs, key=lambda r: r[1])
        print(f"  {label:10s}: {t_sim:5.1f} s / {t_cam:5.1f} s | {mem:.1f} GB   (실행 {args.repeat}번 중 가장 짧은 값; "
              f"카메라 첫 장면 " + ", ".join(f"{r[1]:.1f}" for r in runs) + " s)")

    # 3) 공 떨어뜨리기: 타일 충돌 메시를 합친 것으로 위치와 판정을 정한다
    merged = o3d.geometry.TriangleMesh()
    for t in tiles:
        merged += o3d.io.read_triangle_mesh(str(models / t["model"] / "collision.obj"))
    rand_xyz = m35.drop_positions(merged)
    x_lo, x_hi = min(t["bounds"][0] for t in tiles), max(t["bounds"][2] for t in tiles)
    y_lo, y_hi = min(t["bounds"][1] for t in tiles), max(t["bounds"][3] for t in tiles)
    seam_xy = [(c, y) for c in xs for y in np.arange(y_lo, y_hi, SEAM_STEP)] + \
              [(x, c) for c in ys for x in np.arange(x_lo, x_hi, SEAM_STEP)]
    seam_axis = np.array([0] * (len(xs) * len(np.arange(y_lo, y_hi, SEAM_STEP))) +
                         [1] * (len(ys) * len(np.arange(x_lo, x_hi, SEAM_STEP))))  # 경계선과 수직인 축 (x = c 선이면 0)
    seam_xy = np.array(seam_xy).reshape(-1, 2)
    z = m35.mesh_heights(merged, seam_xy)
    ok = np.isfinite(z)
    seam_xyz, seam_axis = np.c_[seam_xy[ok], z[ok] + m35.DROP_ABOVE], seam_axis[ok]
    xyz = np.vstack([rand_xyz, seam_xyz])
    nr = len(rand_xyz)
    balls = "".join(m36.BALL.format(i=i, x=x, y=y, z=zz, r=m35.BALL_R) for i, (x, y, zz) in enumerate(xyz))
    check = out / "check_world.sdf"
    check.write_text(world_text.replace("</world>", cam_xml + balls + "  </world>"))
    print(f"\n[월드] {check} (타일 {len(tiles)}개 + 무작위 공 {nr}개 + 경계선 위 공 {len(seam_xyz)}개)")

    poses, frames, secs = drop_test(check, world_name, models, args.engine, list(cams), "run")
    print(f"[실행: 타일 월드] 엔진 {args.engine}, 시뮬레이션 {m35.SIM_SECONDS:.0f}초를 {secs:.1f}초에 (서버 시작 포함)")
    pos = ball_positions(poses, len(xyz))
    print("  무작위 공:")
    m35.report_balls(merged, pos[:nr])
    print("  타일 경계선 위 공:")
    m35.report_balls(merged, pos[nr:])
    explain_fallen(seam_xyz, pos[nr:], seam_axis, merged)
    pictures = {"": frames}

    if compare:  # 대조: 같은 공을 메시 하나짜리 월드에 (판정은 그 월드의 충돌 메시로)
        ctrl = out / "check_world_single.sdf"
        ctrl.write_text(single_text(world_text, single).replace("</world>", cam_xml + balls + "  </world>"))
        poses1, frames1, secs1 = drop_test(ctrl, world_name, single, args.engine, list(cams), "ctrl")
        pos1 = ball_positions(poses1, len(xyz))
        one = o3d.io.read_triangle_mesh(str(single / "collision.obj"))
        print(f"\n[대조: 메시 하나짜리 월드, 같은 공] 시뮬레이션을 {secs1:.1f}초에")
        print("  무작위 공:")
        m35.report_balls(one, pos1[:nr])
        print("  타일 경계선 자리에 놓은 공 (하나짜리에는 경계가 없다):")
        m35.report_balls(one, pos1[nr:])
        pictures["single_"] = frames1

    # 4) 사진
    for prefix, frames in pictures.items():
        for n, msg in frames.items():
            if msg:
                m36.save_camera_image(out / f"gazebo_{prefix}{n}.png", msg)
                print(f"저장 (Gazebo 카메라 사진): {out / f'gazebo_{prefix}{n}.png'}")
            else:
                print(f"카메라 {prefix}{n}: 사진을 받지 못함")
    print(f"화면으로 보기: IGN_GAZEBO_RESOURCE_PATH={models} ign gazebo {pkg / 'world.sdf'}")


if __name__ == "__main__":
    main()
