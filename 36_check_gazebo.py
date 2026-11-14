"""
4단계-3: Gazebo에 불러와 확인하기

35번(PyBullet)과 같은 공 떨어뜨리기를 Gazebo에서 한다. ROS 2 Humble과 짝인 Ignition Gazebo 6 (Fortress, 'ign' 명령)을 쓴다.
  1) 문법 검사: ign sdf -k 로 34번 scene.sdf 확인
  2) 월드 만들기 (export/gazebo_check.sdf): 34번 export 폴더를 모델로 <include>하고, 35번과 같은 자리에 공을 놓는다
     (같은 격자, 같은 난수 → 같은 위치). 공은 지름 10 cm, 0.1 kg
  3) 화면 없이 서버만 돌린다 (ign gazebo -s -r). 시뮬레이션 시간이 SIM_SECONDS를 넘으면 모든 물체의 위치를
     한 번 받아 오고(ign topic -e) 서버를 끈다. 다른 Gazebo와 섞이지 않게 IGN_PARTITION을 따로 쓴다
  4) 판정은 35번과 같다: 표면 위에 남음 / 높이가 '아래 메시 표면 + 반지름'과 맞음 / 뚫고 빠짐 / 가장자리 밖
  5) 사진: 월드에 카메라 센서를 하나 두고, 화면 없는 렌더링(--headless-rendering)으로 Gazebo가 직접 그린 장면을
     받아 저장한다 (export/gazebo.png). 보기용 메시의 텍스처가 실제로 어떻게 보이는지 확인하는 용도.
     렌더링이 안 되는 환경이면 공 위치만 Open3D로 대신 그린다

물리 엔진: Fortress 기본은 DART(--engine dartsim). --engine bullet 으로 Bullet 플러그인도 써 볼 수 있다.

화면으로 직접 보려면 (왼쪽 아래 ▶ 를 누르면 공이 떨어진다):
  ign gazebo data/<이름>/export/gazebo_check.sdf

입력: data/<이름>/export/ (34번 결과)
출력: data/<이름>/export/gazebo_check.sdf, gazebo.png

실행:
  python 36_check_gazebo.py hall101/ref
  python 36_check_gazebo.py hall101/ref --engine bullet
  python 36_check_gazebo.py factory --eye=-4.5,-7.5,3.5 --target=1,2,0.8   # 사진 시점 (음수는 = 로 붙여 씀)
"""
import argparse
import base64
import importlib
import json
import os
import subprocess
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR

m35 = importlib.import_module("35_check_pybullet")  # 공 위치, 판정, 높이 계산을 35번과 똑같이 쓴다

WORLD = "pc2mesh_check"
ENGINES = {"dartsim": "ignition-physics-dartsim-plugin", "bullet": "ignition-physics-bullet-plugin"}
TIMEOUT = 300          # 서버가 SIM_SECONDS까지 못 가면 이 시간 [s] 뒤 포기한다 (큰 메시는 불러오는 데만 수십 초)
CAMERA_VFOV = np.radians(50)                                     # 세로 화각 (35번과 같음)


def camera_pose(eye, target):
    """눈 위치에서 목표를 바라보는 SDF pose 문자열 (카메라는 자기 x축 방향을 본다)."""
    d = np.subtract(target, eye)
    yaw, pitch = np.arctan2(d[1], d[0]), np.arctan2(-d[2], np.hypot(d[0], d[1]))  # pitch가 양수면 아래를 본다
    return f"{eye[0]} {eye[1]} {eye[2]} 0 {pitch:.4f} {yaw:.4f}"


BALL = """    <model name="ball_{i}">
      <pose>{x:.4f} {y:.4f} {z:.3f} 0 0 0</pose>
      <link name="link">
        <inertial><mass>0.1</mass><inertia><ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz></inertia></inertial>
        <collision name="collision"><geometry><sphere><radius>{r}</radius></sphere></geometry></collision>
        <visual name="visual">
          <geometry><sphere><radius>{r}</radius></sphere></geometry>
          <material><ambient>0.9 0.15 0.1 1</ambient><diffuse>0.9 0.15 0.1 1</diffuse></material>
        </visual>
      </link>
    </model>
"""

WORLD_SDF = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="{world}">
    <physics name="1ms" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.75 0.85 0.95 1</background></scene>
    <model name="camera">
      <static>true</static>
      <pose>{cam_pose}</pose>
      <link name="link">
        <sensor name="camera" type="camera">
          <topic>camera</topic><update_rate>2</update_rate><always_on>1</always_on>
          <camera>
            <horizontal_fov>{hfov:.4f}</horizontal_fov>
            <image><width>{width}</width><height>{height}</height></image>
            <clip><near>0.1</near><far>50</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose><direction>-0.5 0.1 -0.9</direction>
      <diffuse>0.9 0.9 0.9 1</diffuse><specular>0.2 0.2 0.2 1</specular><cast_shadows>true</cast_shadows>
    </light>
    <include>
      <uri>{model_dir}</uri>
      <name>scene</name>
    </include>
{balls}  </world>
</sdf>
"""


def ign(args, env, timeout=30):
    return subprocess.run(["ign", *args], env=env, capture_output=True, text=True, timeout=timeout)


def topic_once(topic, env):
    """토픽 메시지 하나를 JSON으로 받는다 (아직 안 오면 None)."""
    try:
        r = ign(["topic", "-e", "-n", "1", "--json-output", "-t", topic], env, timeout=10)
    except subprocess.TimeoutExpired:
        return None
    lines = [l for l in r.stdout.splitlines() if l.strip().startswith("{")]
    return json.loads(lines[0]) if lines else None


def sim_seconds(stats):
    t = stats.get("simTime", {})
    return int(t.get("sec", 0)) + int(t.get("nsec", 0)) * 1e-9


def save_camera_image(path, msg):
    """Gazebo 카메라 메시지(JSON, 픽셀은 base64)를 png로 저장한다."""
    w, h = int(msg["width"]), int(msg["height"])
    img = np.frombuffer(base64.b64decode(msg["data"]), dtype=np.uint8).reshape(h, w, 3)  # RGB_INT8
    o3d.io.write_image(str(path), o3d.geometry.Image(np.ascontiguousarray(img)))


def save_image(path, visual, pos):
    """(카메라 사진을 못 받았을 때) 보기용 메시 + 공(빨강)을 Open3D로 비슷한 시점에서 그린다."""
    geoms = [visual]
    for x, y, z in pos:
        if z > -m35.BALL_R:  # 장면 아래로 떨어진 공은 그리지 않는다
            b = o3d.geometry.TriangleMesh.create_sphere(radius=m35.BALL_R)
            b.translate((x, y, z))
            b.paint_uniform_color((0.9, 0.15, 0.1))
            b.compute_vertex_normals()
            geoms.append(b)
    vis = o3d.visualization.Visualizer()
    vis.create_window(width=m35.IMAGE_SIZE[0], height=m35.IMAGE_SIZE[1], visible=False)
    for g in geoms:
        vis.add_geometry(g)
    ctr = vis.get_view_control()
    ctr.set_up((0, 0, 1))
    ctr.set_front((-0.45, -0.75, 0.6))
    ctr.set_lookat((0, 0, 0.4))
    ctr.set_zoom(0.55)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(str(path), do_render=True)
    vis.destroy_window()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (34번까지 마친 것)")
    parser.add_argument("--engine", choices=list(ENGINES), default="dartsim", help="Gazebo 물리 엔진 (기본: DART)")
    parser.add_argument("--no-image", action="store_true", help="사진을 찍지 않음")
    parser.add_argument("--eye", help="사진 카메라 위치 x,y,z [m] (기본: 35번과 같은 Hall 101용 시점)")
    parser.add_argument("--target", help="사진 카메라가 바라보는 점 x,y,z [m]")
    args = parser.parse_args()
    eye, target = m35.parse_xyz(args.eye, m35.CAMERA_EYE), m35.parse_xyz(args.target, m35.CAMERA_TARGET)

    out = DATA_DIR / args.name / "export"
    env = dict(os.environ, IGN_PARTITION=f"pc2mesh_{os.getpid()}")

    # 1) 문법 검사
    r = ign(["sdf", "-k", str(out / "scene.sdf")], env)
    print(f"[문법 검사] ign sdf -k scene.sdf: {r.stdout.strip() or r.stderr.strip()}")

    # 2) 월드 만들기
    mesh = o3d.io.read_triangle_mesh(str(out / "collision.obj"))
    xyz = m35.drop_positions(mesh)
    balls = "".join(BALL.format(i=i, x=x, y=y, z=z, r=m35.BALL_R) for i, (x, y, z) in enumerate(xyz))
    world = out / "gazebo_check.sdf"
    w, h = m35.IMAGE_SIZE
    hfov = 2 * np.arctan(np.tan(CAMERA_VFOV / 2) * w / h)
    world.write_text(WORLD_SDF.format(world=WORLD, model_dir=out.resolve(), balls=balls, cam_pose=camera_pose(eye, target),
                                      hfov=hfov, width=w, height=h))
    print(f"[월드] {world} (34번 모델 + 공 {len(xyz)}개)")

    # 3) 서버 돌리기
    t0 = time.time()
    server = subprocess.Popen(["ign", "gazebo", "-s", "-r", "-v", "1", "--headless-rendering",
                               "--physics-engine", ENGINES[args.engine], str(world)],
                              env=env, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, text=True)
    camera = None
    try:
        sim_t = 0.0
        while sim_t < m35.SIM_SECONDS:
            if server.poll() is not None:
                raise RuntimeError(f"Gazebo 서버가 멈춤:\n{server.stderr.read()[-2000:]}")
            if time.time() - t0 > TIMEOUT:
                raise RuntimeError(f"{TIMEOUT}초 안에 시뮬레이션 {m35.SIM_SECONDS}초에 못 도달함 (현재 {sim_t:.2f}초)")
            stats = topic_once(f"/world/{WORLD}/stats", env)
            sim_t = sim_seconds(stats) if stats else 0.0
        poses = topic_once(f"/world/{WORLD}/pose/info", env)
        if not args.no_image:
            camera = topic_once("/camera", env)
    finally:
        server.terminate()
        try:
            server.wait(timeout=10)
        except subprocess.TimeoutExpired:
            server.kill()
    print(f"[실행] 엔진 {args.engine}, 시뮬레이션 {sim_t:.1f}초를 {time.time() - t0:.1f}초에 (서버 시작 포함)")

    # 4) 판정
    found = {p["name"]: p.get("position", {}) for p in poses["pose"]}
    pos = np.array([[found[f"ball_{i}"].get(k, 0.0) for k in "xyz"] for i in range(len(xyz))])
    m35.report_balls(mesh, pos)

    # 5) 사진
    if not args.no_image:
        img = out / ("gazebo.png" if args.engine == "dartsim" else f"gazebo_{args.engine}.png")
        if camera:
            save_camera_image(img, camera)
            print(f"\n저장 (Gazebo 카메라가 찍은 사진): {img}")
        else:
            visual = o3d.io.read_triangle_mesh(str(out / "visual.obj"))
            visual.compute_vertex_normals()
            save_image(img, visual, pos)
            print(f"\n카메라 사진을 못 받아서 공 위치를 Open3D로 대신 그림: {img}")
        print(f"화면으로 보기: ign gazebo {world}")


if __name__ == "__main__":
    main()
