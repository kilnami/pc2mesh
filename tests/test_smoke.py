"""
스모크 테스트: 합성 방 하나로 파이프라인을 처음부터 끝까지 돌려서, 죽지 않고 대충 맞는 결과가 나오는지만 본다.

합성 방은 정답을 알고 크기가 작아서(점 13만 개) 전체가 몇 분 안에 끝난다. 정확한 숫자는 Poisson이 스레드 때문에 매번
조금 달라서 맞추지 않고, 기준선(이 값보다 나빠지면 무언가 깨진 것)만 둔다. 기준선은 합성 방에서 잰 값에 여유를 둔 것이다.
공장 같은 큰 현장의 문제는 여기서 못 잡는다.

스크립트는 임시 폴더에 복사해서 돌린다 (스크립트가 자기 옆의 data/ 에 쓰기 때문에, 이렇게 해야 내 data/ 를 건드리지 않는다).

실행:
  pytest                    # 전부. Gazebo(ign)가 없으면 그 부분만 건너뛴다
  pytest -m "not gazebo"    # Gazebo 없이
  pytest -k determin        # 이름에 determin 이 든 것만
  pytest -x                 # 처음 실패에서 멈춤
"""
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
HAS_GAZEBO = shutil.which("ign") is not None
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", PYTHONIOENCODING="utf-8", MPLBACKEND="Agg")


def gazebo(test):
    test = pytest.mark.gazebo(test)
    return pytest.mark.skipif(not HAS_GAZEBO, reason="ign(Gazebo Fortress)이 없다")(test)


class Workspace:
    """스크립트를 복사해 둔 임시 폴더. run()으로 스크립트를 돌리고 출력을 받는다."""

    def __init__(self, path):
        self.path = path

    def data(self, *parts):
        return self.path.joinpath("data", *parts)

    def run(self, script, *args, timeout=600):
        proc = subprocess.run([sys.executable, script, *args], cwd=self.path, env=ENV,
                              capture_output=True, text=True, timeout=timeout)
        assert proc.returncode == 0, (f"{script} {' '.join(args)} 실패 (종료 코드 {proc.returncode})\n"
                                      f"--- 출력 끝부분\n{proc.stdout[-2500:]}\n--- 오류\n{proc.stderr[-2500:]}")
        return proc.stdout


def grab(text, pattern, label):
    """출력에서 pattern 첫 줄을 찾는다. 못 찾으면 출력 형식이 바뀐 것이니 그 사실을 알려 준다."""
    m = re.search(pattern, text, re.M)
    assert m, f"출력에서 '{label}' 줄을 못 찾았다 (출력 형식이 바뀌었나?)\n--- 출력 끝부분\n{text[-2000:]}"
    return m


def number(s):
    return float(s.replace(",", ""))


def md5(path):
    return hashlib.md5(path.read_bytes()).hexdigest()


# ── 단계별 준비물. 앞 단계가 실패하면 뒤 단계 시험은 '오류'로 표시된다 (어디서 깨졌는지 보인다) ──────────────

@pytest.fixture(scope="session")
def ws(tmp_path_factory):
    path = tmp_path_factory.mktemp("pc2mesh")
    for script in ROOT.glob("*.py"):
        shutil.copy(script, path)
    return Workspace(path)


@pytest.fixture(scope="session")
def scan(ws):
    return ws.run("21_make_synth_room.py", "--no-view")


@pytest.fixture(scope="session")
def normals(ws, scan):
    ws.run("22_preprocess.py", "--no-view")
    return ws.run("23_normals.py", "--no-view")


@pytest.fixture(scope="session")
def recon(ws, normals):
    return ws.run("24_reconstruct.py", "--no-view")


@pytest.fixture(scope="session")
def prepared(ws, scan):
    return ws.run("31_prepare.py", "data/synth_room.ply", "--name", "synth_room",
                  "--sensors", "data/synth_room_sensors.npy", "--no-view")


@pytest.fixture(scope="session")
def cleaned(ws, prepared):
    return ws.run("32_clean_normals.py", "synth_room", "--no-view")


@pytest.fixture(scope="session")
def scored(ws, cleaned):
    return ws.run("37_score_normals.py", "synth_room", "--ref", "data/synth_room.ply")


@pytest.fixture(scope="session")
def export(ws, cleaned):
    return ws.run("34_export.py", "synth_room", "--no-view")


@pytest.fixture(scope="session")
def pybullet(ws, export):
    return ws.run("35_check_pybullet.py", "synth_room")


@pytest.fixture(scope="session")
def pybullet_convex(ws, export):
    return ws.run("35_check_pybullet.py", "synth_room", "--convex")


@pytest.fixture(scope="session")
def gazebo_check(ws, export):
    return ws.run("36_check_gazebo.py", "synth_room")


@pytest.fixture(scope="session")
def tiles(ws, cleaned):
    return ws.run("38_tiles.py", "synth_room", "--grid", "2x1")


@pytest.fixture(scope="session")
def tiles_check(ws, tiles, export):
    return ws.run("39_check_tiles.py", "synth_room")


@pytest.fixture(scope="session")
def nav2(ws, tiles):
    return ws.run("40_nav2_map.py", "synth_room")


@pytest.fixture(scope="session")
def support_variant(ws, cleaned):
    """받침을 넣어 만든 타일 패키지와, 거기서 받침을 뺀 사본. tiles 와 폴더가 겹치지 않게 따로 만든다."""
    dst = ws.data("synth_room_support")
    dst.mkdir(parents=True)
    for name in ("normals.ply", "inspect.json"):
        shutil.copy(ws.data("synth_room", name), dst / name)
    ws.run("38_tiles.py", "synth_room_support", "--grid", "2x1", "--floor-support")
    return ws.run("41_tiles_variant.py", "synth_room_support")


# ── 시험 ─────────────────────────────────────────────────────────────────────────────────────

def test_scan_writes_cloud_ground_truth_and_sensors(ws, scan):
    for name in ("synth_room.ply", "synth_room_gt.ply", "synth_room_sensors.npy"):
        assert ws.data(name).stat().st_size > 0, f"{name} 이 비었다"


def test_basic_sign_methods_match_ground_truth(normals):
    """23번이 스스로 채점한 뒤집힘 비율. 합성 방에서 sensor 0.3 %, tangent 0.0 %"""
    for method in ("sensor", "tangent"):
        flipped = number(grab(normals, rf"^\s*{method}\s+맞음\s+[\d.]+%\s+\|\s+옆으로 누움\s+[\d.]+%\s+\|\s+뒤집힘\s+([\d.]+)%",
                              f"{method} 뒤집힘").group(1))
        assert flipped <= 2, f"{method} 방식이 법선을 {flipped}% 뒤집었다"


def test_reconstruction_is_accurate(recon):
    """합성 방에서 Poisson raw 93.7 / 92.0 %, Ball Pivoting 99.7 / 87.0 % (정확도 / 완성도).
    정확도는 표면까지의 거리로 재서 법선이 통째로 뒤집혀도 거의 안 변한다. 부호 오류는 위 시험과 아래 채점 시험이 본다."""
    acc, comp = (number(g) for g in grab(recon, r"^\s*Poisson raw\s+[\d,]+\s+[\d.]+s\s+([\d.]+)%\s+([\d.]+)%", "Poisson raw").groups())
    assert acc >= 90 and comp >= 88, f"Poisson raw 정확도 {acc}% / 완성도 {comp}%"
    acc, comp = (number(g) for g in grab(recon, r"^\s*BallPivoting\s+[\d,]+\s+[\d.]+s\s+([\d.]+)%\s+([\d.]+)%", "BallPivoting").groups())
    assert acc >= 95 and comp >= 80, f"BallPivoting 정확도 {acc}% / 완성도 {comp}%"


def test_normal_signs_are_mostly_right(scored):
    """정답 법선과 비교해 최종 법선의 뒤집힘 0.3 %, 축이 맞는 점 99.2 %"""
    flipped = number(grab(scored, r"최종 normals\.ply\s+([\d.]+)%", "최종 법선 뒤집힘").group(1))
    axis = number(grab(scored, r"축이 맞는 점 ([\d.]+)%", "축이 맞는 점").group(1))
    assert flipped <= 1.5, f"최종 법선이 {flipped}% 뒤집혔다"
    assert axis >= 97, f"축이 맞는 점이 {axis}% 뿐이다"


def test_prepare_is_deterministic(ws, scan):
    """같은 입력을 두 번 준비하면 파일이 똑같아야 한다 (바닥 맞추기의 RANSAC 시드 고정을 지킨다)."""
    for tag in ("a", "b"):
        ws.run("31_prepare.py", "data/synth_room.ply", "--name", f"determ_{tag}",
               "--sensors", "data/synth_room_sensors.npy", "--no-view")
    a, b = ws.data("determ_a"), ws.data("determ_b")
    assert md5(a / "prepared.ply") == md5(b / "prepared.ply"), "같은 입력인데 prepared.ply 가 다르다"
    tilt = [json.loads((d / "inspect.json").read_text())["tilt_deg"] for d in (a, b)]
    assert tilt[0] == tilt[1], f"바닥 기울기가 다르다: {tilt}"


def test_export_writes_simulator_files(ws, export):
    folder = ws.data("synth_room", "export")
    for name in ("visual.obj", "visual.mtl", "visual_0.png", "visual.glb", "collision.obj",
                 "scene.urdf", "scene.sdf", "model.config"):
        assert (folder / name).stat().st_size > 0, f"export/{name} 이 없거나 비었다"
    m = grab(export, r"\[선택\] 보기용 ([\d,]+)개.*?충돌용 ([\d,]+)개", "삼각형 수 선택")
    visual, collision = number(m.group(1)), number(m.group(2))
    assert collision < visual, f"충돌용({collision:.0f})이 보기용({visual:.0f})보다 작아야 한다"


def test_urdf_keeps_concave_on_collision(ws, export):
    """concave="yes" 는 <collision> 에 붙어야 PyBullet이 먹는다. <link> 에 붙으면 무시된다."""
    urdf = ws.data("synth_room", "export", "scene.urdf").read_text()
    assert re.search(r'<collision\b[^>]*concave="yes"', urdf), "<collision> 에 concave=\"yes\" 가 없다"


def test_export_patches_floor_holes_in_collision_mesh(export):
    """충돌용 바닥 구멍 12 % → 0.0 %. 칸 중심 한 점으로 재서 패치를 깐 칸은 거의 0 이 나온다 (그래서 아주 낮게만 본다)."""
    before, after = (number(g) for g in grab(export, r"\[바닥 패치\] 충돌용 바닥 구멍 ([\d.]+)% → ([\d.]+)%", "바닥 패치").groups())
    assert after <= 0.5, f"패치를 깔았는데도 바닥 구멍이 {after}% 남았다 (전 {before}%)"
    unseen = number(grab(export, r"바닥 점이 없는 자리\(물체 밑 등\) ([\d.]+) ㎡ 메움", "못 본 바닥").group(1))
    assert unseen >= 3, f"물체 밑 바닥을 {unseen} ㎡ 만 메웠다 (합성 방은 상자와 둥근 물체 밑 등 약 6.8 ㎡)"


def test_export_adds_wall_skirt_to_both_meshes(export):
    """합성 방에서 벽 4장을 찾고, 바닥에서 떠 있는 열 10개(폭 1 m)를 바닥까지 채운다. 라이다가 보기용을 보므로 충돌용·보기용 둘 다에 넣는다."""
    m = grab(export, r"\[벽 치마\] 벽 (\d+)장에서 바닥에서 떠 있는 열 (\d+)개", "벽 치마")
    walls, columns = number(m.group(1)), number(m.group(2))
    assert walls == 4, f"합성 방의 벽은 4장이다: {walls:.0f}장을 찾았다"
    assert columns >= 3, f"떠 있는 벽 열을 {columns:.0f}개만 채웠다 (합성 방은 약 10개)"


def test_pybullet_loads_the_collision_mesh(pybullet):
    """광선 일치 100 %, 60개 중 뚫고 빠진 공 3개 (합성 방은 책상 밑 같은 바닥 구멍이 있다)"""
    match = number(grab(pybullet, r"맞은 비율 ([\d.]+)%", "광선 일치율").group(1))
    fell = number(grab(pybullet, r"뚫고 빠짐 (\d+)개", "뚫고 빠진 공").group(1))
    assert match >= 99, f"PyBullet 높이가 메시와 {match}% 만 맞는다"
    assert fell <= 10, f"공 {fell:.0f}개가 뚫고 빠졌다"


def test_without_concave_the_scene_becomes_one_blob(pybullet_convex):
    """concave 를 빼면 볼록 껍질 하나가 돼서 높이가 메시와 안 맞는다 (5.5 %). 이 차이가 사라지면 concave 가 안 먹는 것이다."""
    match = number(grab(pybullet_convex, r"맞은 비율 ([\d.]+)%", "광선 일치율").group(1))
    assert match < 50, f"concave 를 뺐는데도 {match}% 가 맞는다"


@gazebo
def test_gazebo_accepts_the_sdf_and_balls_land(gazebo_check):
    """sdf 문법 검사 통과, 60개 중 42개가 표면 위에 남음 (DART)"""
    assert re.search(r"ign sdf -k scene\.sdf: Valid", gazebo_check), "ign sdf -k 가 Valid 를 안 냈다"
    stay = number(grab(gazebo_check, r"표면 위에 남음 (\d+)개", "표면 위에 남은 공").group(1))
    assert stay >= 30, f"표면 위에 남은 공이 {stay:.0f}개 뿐이다"


def test_tile_package_layout(ws, tiles):
    folder = ws.data("synth_room", "tiles")
    world = (folder / "world.sdf").read_text()
    names = [t["model"] for t in json.loads((folder / "tiles.json").read_text())["tiles"]]
    assert len(names) == 2, f"타일이 2개여야 한다: {names}"
    for name in names:
        model = folder / "models" / name
        assert name in world, f"world.sdf 가 {name} 을 불러오지 않는다"
        assert "floor_support" not in (model / "model.sdf").read_text(), "받침은 기본으로 넣지 않는다"
        mtl = (model / "visual.mtl").read_text()
        assert f"map_Kd texture_{name}.jpg" in mtl, "텍스처 이름이 타일마다 달라야 한다 (같으면 Gazebo가 섞어 쓴다)"
        assert (model / f"texture_{name}.jpg").stat().st_size > 0
        assert re.search(r"^vn ", (model / "collision.obj").read_text(), re.M), "충돌 OBJ 에 꼭짓점 법선(vn)이 없다"


def test_tiles_patch_floor_holes_in_collision_mesh(tiles):
    """타일마다 충돌용 바닥 구멍이 패치로 덮인다 (타일 경계에서 잘린 칸도 구멍으로 잡히므로 경계에서 자른 패치를 쓴다)."""
    after = [number(g) for g in re.findall(r"\[바닥 패치\] 충돌용 바닥 구멍 [\d.]+% → ([\d.]+)%", tiles)]
    assert len(after) == 2, f"타일 2개에서 바닥 패치 줄이 나와야 한다: {after}"
    assert max(after) <= 0.5, f"타일 충돌 메시에 바닥 구멍이 남았다: {after}"
    unseen = [number(g) for g in re.findall(r"바닥 점이 없는 자리\(물체 밑 등\) ([\d.]+) ㎡ 메움", tiles)]
    assert sum(unseen) >= 3, f"타일에서 물체 밑 바닥을 {sum(unseen):.1f} ㎡ 만 메웠다 (합성 방은 약 6.8 ㎡): {unseen}"


def test_tiles_add_wall_skirt(tiles):
    """벽은 점 전체에서 한 번 찾고(4장), 타일마다 치마를 넣는다. 타일 경계에서 잘라 쓴다."""
    walls = number(grab(tiles, r"\[벽\] 큰 수직 벽 평면 (\d+)장", "벽 찾기").group(1))
    assert walls == 4, f"합성 방의 벽은 4장이다: {walls:.0f}장"
    columns = [number(g) for g in re.findall(r"\[벽 치마\] 바닥에서 떠 있는 열 (\d+)개", tiles)]
    assert len(columns) == 2 and sum(columns) >= 3, f"타일 2개에서 치마 줄이 나와야 하고 열이 있어야 한다: {columns}"


def test_tile_seams_are_closed(tiles):
    """경계선 위 점이 상대 타일 메시에서 1 cm 이내인 비율. 합성 방에서 보기용 99 %, 충돌용 93 %"""
    visual = number(grab(tiles, r"visual\s+점\s+[\d,]+개: 1 cm 이내 (\d+)%", "보기용 이음매").group(1))
    collision = number(grab(tiles, r"collision\s+점\s+[\d,]+개: 1 cm 이내 (\d+)%", "충돌용 이음매").group(1))
    assert visual >= 90 and collision >= 80, f"이음매 1 cm 이내: 보기용 {visual}%, 충돌용 {collision}%"


@gazebo
def test_no_ball_falls_through_a_tile_seam(tiles_check):
    gaps = re.findall(r"경계 틈 (\d+)개", tiles_check)
    assert gaps, "경계 틈 줄을 못 찾았다 (출력 형식이 바뀌었나?)\n" + tiles_check[-2000:]
    assert all(g == "0" for g in gaps), f"타일 경계 틈으로 공이 빠졌다: {gaps}"


def test_nav2_maps_follow_platform_rules(ws, nav2):
    """위치 추정용과 장애물용 지도는 크기, 해상도, 원점이 같아야 하고 원점 yaw 는 0 이어야 한다."""
    folder = ws.data("synth_room", "nav2")
    sizes, origins, resolutions = [], [], []
    for kind in ("loc", "obs"):
        with open(folder / f"synth_room_{kind}.pgm", "rb") as f:
            assert f.readline().strip() == b"P5"
            sizes.append(f.readline().split())
        yaml = (folder / f"synth_room_{kind}.yaml").read_text()
        origins.append(grab(yaml, r"^origin:\s*\[(.*)\]", "origin").group(1))
        resolutions.append(grab(yaml, r"^resolution:\s*([\d.]+)", "resolution").group(1))
    assert sizes[0] == sizes[1], f"두 지도의 크기가 다르다: {sizes}"
    assert origins[0] == origins[1] and resolutions[0] == resolutions[1], "두 지도의 원점이나 해상도가 다르다"
    assert float(origins[0].split(",")[2]) == 0.0, "원점 yaw 는 0 이어야 한다"


def test_variant_removes_only_the_floor_support(ws, support_variant):
    base = ws.data("synth_room_support")
    for model in sorted((base / "tiles" / "models").glob("*/model.sdf")):
        assert "floor_support" in model.read_text(), "--floor-support 로 만든 패키지에는 받침이 있어야 한다"
    for model in sorted((base / "tiles_nosupport" / "models").glob("*/model.sdf")):
        assert "floor_support" not in model.read_text(), "41번이 받침을 못 뺐다"
    with zipfile.ZipFile(base / "tiles_nosupport.zip") as z:
        top = {name.split("/")[0] for name in z.namelist()}
    assert {"world.sdf", "tiles.json", "models"} <= top, f"zip 맨 위에 world.sdf 등이 있어야 한다: {sorted(top)}"
