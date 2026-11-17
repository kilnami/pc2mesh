"""
6단계-2: 물체마다 모델 하나씩 만들기

42번이 점군을 구조물과 물체 덩어리로 나눴다. 이 파일은 물체 덩어리마다 Gazebo 모델 폴더를 만든다.
타일(38번 --with-objects)이 이 모델들을 월드에 <include> 한다. 책임님이 바라는 '물체를 따로 객체로' 의 모양이다.

하는 일 (물체마다)
  1) 점 모으기: 42번 labels 에서 그 물체의 점. 점이 MODEL_MIN_POINTS 보다 적으면 모델로 만들지 않고 구조물(타일)에 둔다.
  2) 재구성과 정리: 38번의 Poisson(밀도 trim)과 34번의 clean 을 그대로 쓴다. 작은 물체의 가는 다리가 지워지지 않게
     떠 있는 조각 기준만 MIN_PIECE 장으로 낮춘다 (34번은 100장).
  3) 발 늘이기 (extend_feet): 42번이 바닥 점(FLOOR_BAND 아래)을 구조물로 뺐으므로 바닥에 선 물체는 바닥에서 FLOOR_BAND 만큼
     떠 있다. 바닥 위 FLOOR_BAND ~ FLOOR_BAND + FOOT_BLEND 구간을 늘려서 아래쪽 가장자리가 z = 0 에 닿게 한다.
     (바닥 점을 물체에 같이 넣고 Poisson 을 돌리면 바닥의 위쪽 법선이 다리 속을 '바깥'이라고 해서 막이 생긴다)
  4) 줄이기: 보기용은 면적 x VISUAL_DENSITY 개 (38번과 같은 밀도, 최소 VISUAL_MIN 개). 충돌용은 34번처럼 줄이되
     벗어난 거리 95% 가 COLLISION_TOL 이하인 가장 작은 것 (작은 물체라 34번의 2 cm 보다 엄격하게).
  5) 모델 폴더: 물체 바닥 중심(xy 범위의 가운데, z = 0)이 모델 원점이다. 메시는 그 원점 기준 좌표이고, 월드에서는
     <include> 의 <pose> 가 원점을 놓을 자리를 정한다. 회전(yaw)은 0 이다 (물체 방향은 아직 안 찾는다).
     38번 write_tile 을 그대로 써서 폴더 구성(visual.obj/mtl, 고유 이름의 텍스처, collision.obj, model.config, model.sdf)이
     타일과 같다. 모델은 static 이다 (움직이는 물체와 질량은 아직 없다).

시험한 결과 (합성 방 synth_clutter, --score-synth: 정답 표면 3 cm 이내 비율. 완성도는 점이 찍힌 곳만 센다)
  42번 덩어리 12개 중 점 300개 이상 11개가 모델이 된다. 평균 정확도 92.8%, 완성도 93.9% (5초).
    상자 더미·기계·선반·배관 95~99%, 의자 3개는 정확도 88~90% (가는 다리가 부풀어 정답 0.45 m 인 폭이 0.49~0.59 m 가 된다).
    책상은 42번이 둘로 쪼갠 조각마다 정확도 88~94%.
  월드 전체 (구조물 타일 + 물체 모델을 정답 메시와 비교): 정확도 99.4%, 완성도 99.0%. 물체가 타일에 들어 있는 기준(정확도 99.6%,
    완성도 98.8%)과 같은 수준이고 물체 부분 완성도는 93.9 → 94.5%. 공 600개 떨어뜨리기(PyBullet): 바닥 아래로 빠진 공 2개 (기준과 같음).
    단 38번 기본 밀도 trim(5%)으로는 구조물 완성도가 98.2 → 96.4% 로 떨어진다. 물체를 빼면 가짜 면이 줄어서 그 5% 가 진짜 벽·천장
    가장자리를 깎는다. 38번에 --density-quantile 0.01 을 주면 99.8%. 기준(물체 포함) 타일도 0.01 이면 98.2 → 99.7% 로 올라간다.
    물체 모델은 이 값에 둔감했다 (0.05 / 0.01 / 0 모두 평균 같음).
  Gazebo Fortress 가 월드를 불러와 타일 2개 + 물체 11개 모델이 모두 올라왔다 (ign model --list).

한계 (다음에 풀 것)
  - 위가 안 찍힌 물체(상자 더미, 기계)의 충돌 메시는 위가 뚫려 있다. 위에서 떨어진 공이 안으로 들어간다 (물체 위에 얹힌 공이
    기준 10개 → 6개). Open3D fill_holes 로는 안 닫힌다 (열린 가장자리가 남는다). 위쪽을 닫는 방법을 정해야 한다.
  - 의자 다리처럼 가는 것은 Poisson 이 부풀린다.
  - 회전(yaw)은 0, static 이다 (움직이는 물체, 질량·관성은 없다).

입력: data/<이름>/normals.ply, objects.npz, objects.json (42번), inspect.json
출력: data/<이름>/objects/models/<f1_obj_NNN>/ 와 data/<이름>/objects/objects.json
      (objects.json: 만든 모델 목록. 이름, 원점 위치(pose), 크기, 점 수, 삼각형 수. 38번이 읽는다)

실행:
  python 43_object_models.py synth_clutter --score-synth     # 합성 방은 정답 물체 메시와 비교
  python 38_tiles.py synth_clutter --grid 2x1 --with-objects # 물체를 뺀 타일 + 월드에 물체 include
"""
import argparse
import importlib
import json
import shutil
import time

import numpy as np
import open3d as o3d

from common import DATA_DIR

# 파일 이름이 숫자로 시작해서 import 문 대신 씀
m34 = importlib.import_module("34_export")
m38 = importlib.import_module("38_tiles")
m42 = importlib.import_module("42_objects")

# ---- 바꿔볼 만한 파라미터 ----
OBJ_PREFIX = "f1_obj"        # 모델 이름 앞부분 (<OBJ_PREFIX>_000). 타일은 f1_struct_*
MODEL_MIN_POINTS = 300       # 점이 이보다 적은 덩어리는 모델로 만들지 않고 구조물에 둔다
MIN_PIECE = 20               # 정리: 삼각형이 이보다 적은 떠 있는 조각은 지운다
FLOOR_BAND = m34.FLOOR_BAND  # 42번이 바닥으로 뺀 높이 [m]
FOOT_BLEND = 0.10            # 발 늘이기: 바닥 위 FLOOR_BAND 에서 여기까지(이만큼 위까지) 늘려 바닥에 닿게 한다 [m]
VISUAL_DENSITY = m38.VISUAL_DENSITY   # 보기용 삼각형 / 면적 [개/m²]
VISUAL_MIN = 300             # 보기용 삼각형 하한
COLLISION_MAX = 30_000       # 충돌용 삼각형 상한
COLLISION_LADDER = (10_000, 3_000, 1_000, 300, 100)   # 충돌용으로 줄여 볼 삼각형 수
COLLISION_TOL = 0.01         # 충돌용: 정리한 메시에서 벗어난 거리 95% 가 이 이하 [m]
N_SAMPLES = 20_000           # 벗어난 거리를 잴 때 메시 위에서 뽑을 점 수
TEXTURE_PX = 1024            # 물체 텍스처 한 변 목표 [px] (삼각형이 적으면 더 작아진다)
OK_DIST = 0.03               # 채점: 정답 표면에서 이 안이면 맞는 표면 [m] (33번과 같음)
CONFIG = m34.MODEL_CONFIG.replace("scene.sdf", "model.sdf").replace("34_export.py", "43_object_models.py")


def extend_feet(mesh):
    """바닥 위 FLOOR_BAND 이하는 FLOOR_BAND 만큼 내리고, FLOOR_BAND + FOOT_BLEND 이상은 그대로 두고, 그 사이는 선형으로 늘린다.
    (아래쪽 가장자리가 z = 0 에 닿고 위는 안 움직인다.) z 가 0 아래로 내려가면 0 으로 자른다."""
    V = np.asarray(mesh.vertices).copy()
    t = np.clip((V[:, 2] - FLOOR_BAND) / FOOT_BLEND, 0.0, 1.0)
    V[:, 2] = np.maximum(V[:, 2] - FLOOR_BAND * (1.0 - t), 0.0)
    out = o3d.geometry.TriangleMesh(mesh)
    out.vertices = o3d.utility.Vector3dVector(V)
    out.compute_vertex_normals()
    return out


def pick_collision(mesh):
    """충돌용: 정리한 메시를 COLLISION_MAX 로 맞추고, 래더를 따라 줄이다가 벗어난 거리 95% 가 COLLISION_TOL 을 넘기 직전의 것."""
    best = m34.decimate(mesh, COLLISION_MAX)
    samples = np.asarray(mesh.sample_points_uniformly(N_SAMPLES).points)
    for n in COLLISION_LADDER:
        if n >= len(best.triangles):
            continue
        cand = m34.decimate(best, n)
        if m34.deviation(samples, cand)[1] > COLLISION_TOL:
            break
        best = cand
    return best


def build_model(pcd, voxel, standing):
    """점 묶음 하나로 (보기용, 충돌용, 정리한 메시, 보고) 를 월드 좌표로 만든다. 메시가 비면 None."""
    raw, depth, _ = m38.poisson_trim(pcd, voxel)
    mesh, _ = m34.clean(raw, np.asarray(pcd.points), voxel, min_piece=MIN_PIECE)
    if len(mesh.triangles) == 0:
        return None
    if standing:
        mesh = extend_feet(mesh)
    n_visual = int(np.clip(mesh.get_surface_area() * VISUAL_DENSITY, VISUAL_MIN, len(mesh.triangles)))
    visual = m34.decimate(mesh, n_visual)
    collision = pick_collision(mesh)
    return visual, collision, mesh, dict(depth=depth, cleaned=len(mesh.triangles))


def to_local(mesh, origin):
    out = o3d.geometry.TriangleMesh(mesh)
    out.translate(-np.asarray(origin))
    return out


def score_object(model, points, gt_mesh):
    """정답 물체 메시와 비교: 정확도 = 모델 메시 위 점이 정답 표면 OK_DIST 이내인 비율, 완성도 = 정답 표면 중 점이 찍힌(스캔된) 곳이
    모델 메시 OK_DIST 이내인 비율 (찍히지 않은 밑면·윗면은 못 만드는 게 당연해서 뺀다)."""
    gt_scene = o3d.t.geometry.RaycastingScene()
    gt_scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(gt_mesh))
    model_scene = o3d.t.geometry.RaycastingScene()
    model_scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(model))
    dist = lambda scene, pts: scene.compute_distance(o3d.core.Tensor(pts.astype(np.float32))).numpy()
    mine = np.asarray(model.sample_points_uniformly(N_SAMPLES).points)
    accuracy = float((dist(gt_scene, mine) <= OK_DIST).mean())
    gt_pts = np.asarray(gt_mesh.sample_points_uniformly(4 * N_SAMPLES).points)
    scanned = np.asarray(o3d.geometry.PointCloud(o3d.utility.Vector3dVector(gt_pts)).compute_point_cloud_distance(
        o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points)))) <= OK_DIST
    completeness = float((dist(model_scene, gt_pts[scanned]) <= OK_DIST).mean()) if scanned.any() else float("nan")
    return accuracy, completeness


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/<이름>/ (42번까지 마친 것)")
    parser.add_argument("--min-points", type=int, default=MODEL_MIN_POINTS,
                        help=f"점이 이보다 적은 덩어리는 모델로 만들지 않는다 (기본 {MODEL_MIN_POINTS})")
    parser.add_argument("--score-synth", action="store_true", help="합성 방(synth_clutter)이면 정답 물체 메시와 비교")
    args = parser.parse_args()
    o3d.utility.random.seed(0)   # 면 위에서 점을 뽑는 계산이 무작위라서, 같은 입력이면 같은 결과가 나오게 고정
    m34.TEXTURE_MAX = TEXTURE_PX

    folder = DATA_DIR / args.name
    voxel = json.loads((folder / "inspect.json").read_text())["voxel"]
    pcd = o3d.io.read_point_cloud(str(folder / "normals.ply"))
    P = np.asarray(pcd.points)
    labels = np.load(folder / "objects.npz")["labels"]
    objects = json.loads((folder / "objects.json").read_text())["objects"]
    out = folder / "objects"
    shutil.rmtree(out, ignore_errors=True)   # 이전 모델이 남아 월드에 섞이지 않게
    (out / "models").mkdir(parents=True)
    if args.score_synth:
        gt_ids, gt_names = m42.gt_labels(P, args.name)
        gt_meshes = m42.gt_object_meshes(args.name)[1]

    todo = [o for o in objects if o["points"] >= args.min_points]
    print(f"[물체] 42번이 나눈 덩어리 {len(objects)}개 중 점 {args.min_points}개 이상 {len(todo)}개를 모델로 만든다 "
          f"(나머지 {len(objects) - len(todo)}개는 구조물에 남는다)")
    index, t_all = [], time.time()
    for o in todo:
        t0 = time.time()
        k = o["id"]
        name = f"{OBJ_PREFIX}_{k:03d}"
        sel = np.flatnonzero(labels == k)
        built = build_model(pcd.select_by_index(sel), voxel, o["kind"] == "standing")
        if built is None:
            print(f"  {name}: 메시가 비었다. 구조물에 남긴다")
            continue
        visual, collision, cleaned, rep = built
        lo, hi = np.asarray(cleaned.vertices).min(axis=0), np.asarray(cleaned.vertices).max(axis=0)
        origin = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, 0.0])
        tile = dict(name=name, bounds=[float(lo[0]), float(lo[1]), float(hi[0]), float(hi[1])])
        size_px, sizes = m38.write_tile(out / "models" / name, tile, to_local(visual, origin), to_local(collision, origin),
                                        config=CONFIG)
        row = dict(name=name, id=k, pose=[round(float(v), 4) for v in origin] + [0.0, 0.0, 0.0], kind=o["kind"],
                   size=[round(float(v), 3) for v in hi - lo], points=o["points"],
                   visual_triangles=len(visual.triangles), collision_triangles=len(collision.triangles), texture_px=size_px)
        line = (f"  {name}: 점 {o['points']:>6,}개 → 정리 {rep['cleaned']:>6,} / 보기용 {len(visual.triangles):>6,} / 충돌용 "
                f"{len(collision.triangles):>5,}개 | 크기 {np.round(hi - lo, 2)}")
        if args.score_synth:
            ids, cnt = np.unique(gt_ids[sel], return_counts=True)
            ids, cnt = ids[ids >= 0], cnt[ids >= 0]
            if len(ids):
                g = int(ids[np.argmax(cnt)])
                acc, comp = score_object(visual, P[sel], gt_meshes[g])
                row.update(gt=gt_names[g], accuracy=acc, completeness=comp)
                line += f" | {gt_names[g]}: 정확도 {acc:.1%}, 완성도 {comp:.1%}"
        print(line + f" ({time.time() - t0:.0f}초)")
        index.append(row)

    (out / "objects.json").write_text(json.dumps(dict(prefix=OBJ_PREFIX, objects=index), indent=1, ensure_ascii=False))
    total_v = sum(r["visual_triangles"] for r in index)
    total_c = sum(r["collision_triangles"] for r in index)
    print(f"\n[저장] 모델 {len(index)}개 (보기용 삼각형 합 {total_v:,}개, 충돌용 합 {total_c:,}개) → {out}/models/, {out}/objects.json "
          f"({time.time() - t_all:.0f}초)")
    if args.score_synth and index:
        accs = [r["accuracy"] for r in index if "accuracy" in r]
        comps = [r["completeness"] for r in index if r.get("completeness") == r.get("completeness")]
        print(f"  정답 비교 평균: 정확도 {np.mean(accs):.1%}, 완성도 {np.mean(comps):.1%}")


if __name__ == "__main__":
    main()
