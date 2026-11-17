"""
6단계-4: 나눈 결과를 AP25 / AP50 / PQ(Panoptic Quality)로 채점한다

왜?
  42번 채점(recall·purity 0.8 이상이면 ok)은 정답 물체를 하나씩 '맞았나 아닌가'로 센다. 3D 인스턴스 분할 논문과 같은 척도로
  재려고 ScanNet 식 AP25·AP50 (인스턴스 IoU 0.25·0.5)과 Panoptic Quality (PQ = SQ x RQ)를 계산한다.
  42번을 다시 돌려 나온 라벨(find_objects)을 정답과 비교하므로 42번 규칙을 바꾸면 이 점수도 바뀐다.

무엇을 센다
  예측: 42번 라벨. 구조물은 클래스 하나가 한 덩어리 (바닥 / 벽 / 지붕=천장), 물체는 덩어리마다 하나의 인스턴스.
  정답: S3DIS 는 천장·바닥·벽(구조물)과 table·chair·sofa·bookcase(물체 인스턴스). 합성 방은 방 껍데기(바닥·벽·천장)와 정답 물체 9개.
  * IoU 는 점 단위다: |예측 ∩ 정답| / |예측 ∪ 정답|.
  * 정답을 모르는 점(void)은 양쪽에서 뺀다. S3DIS 의 beam·column·window·door·board·clutter, 정답이 없는 점, 점이 MIN_UNIT_POINTS 보다
    적은 정답 단위가 void 다 (42번 채점과 같은 기준). void 가 절반 넘는 예측 덩어리는 틀린 예측(FP)으로 세지 않는다.
  * 정답 단위는 두 가지로 센다. 인스턴스 하나씩(붙어 있는 의자와 책상도 따로), 맞닿은 가구를 묶은 것('닿아 있으면 한 물체').
    두 점수의 차이가 거리만으로는 못 가르는 부분이다 (42번 docstring 참고).

AP (ScanNet 식, 클래스 구분 없음)
  예측이 클래스를 안 갖고 있어서(42번은 물체/구조물만 가른다) 물체 전체를 한 클래스로 센다. IoU > 0.25, > 0.5 인 짝이 맞는 예측이고,
  IoU 큰 짝부터 하나씩 맞춘다. AP = 점수 높은 예측부터 늘려 가며 그린 정밀도-재현율 곡선 아래 넓이다.
  42번 결과에는 예측마다 신뢰도가 없어서 모든 예측이 같은 점수다. 그러면 곡선이 한 점이라 AP = 정밀도 x 재현율 이다
  (같은 점수는 한꺼번에 자르는 방식. 점수 순서에 따라 값이 달라지지 않게 하려는 것). 점수를 주는 방법(예: 학습한 모델)이 생기면
  instance_ap(scores=...) 에 넘기면 곡선이 그려진다. AP 는 IoU 0.5~0.95 (0.05 간격) 평균도 함께 낸다.

PQ (Kirillov 2019)
  클래스마다 IoU > 0.5 인 짝을 맞는 것(TP), 못 맞춘 예측(FP)과 정답(FN)을 세어 SQ = TP 짝의 IoU 평균, RQ = TP / (TP + FP/2 + FN/2),
  PQ = SQ x RQ. 구조물은 바닥·벽·천장이 각각 한 클래스(한 덩어리 대 한 덩어리), 물체는 클래스 구분 없는 한 클래스다.
  전체 PQ 는 클래스 평균이다 (정답에 없는 클래스는 뺀다).

입력: data/<이름>/normals.ply (32번 결과). 합성 방은 21번 정답, S3DIS 는 44번이 만든 data/<이름>/raw/gt_points.npz
출력: data/<이름>/score_seg.json

실행:
  python 47_score_seg.py synth_clutter
  python 47_score_seg.py s3dis_conferenceRoom_1 s3dis_office_1 s3dis_office_31 s3dis_hallway_6 s3dis_pantry_1   # 방별 + 평균
"""
import argparse
import importlib
import json

import numpy as np
import open3d as o3d

from common import DATA_DIR

m42 = importlib.import_module("42_objects")

# ---- 바꿔볼 만한 파라미터 ----
AP_IOUS = (0.25, 0.5)                          # 점수로 따로 내는 IoU 기준
AP_RANGE = tuple(round(float(t), 2) for t in np.arange(0.5, 0.951, 0.05))  # AP 평균에 쓰는 IoU 기준
PQ_IOU = 0.5                                   # PQ 에서 맞는 짝의 IoU 기준 (이것보다 커야 한다)
VOID_FP = 0.5                                  # 예측 덩어리의 점이 이 비율을 넘게 void 면 틀린 예측으로 세지 않는다
STUFF = {"floor": m42.FLOOR, "wall": m42.WALL, "ceiling": m42.ROOF}   # 정답 구조물 클래스 → 42번 라벨


def _count(ids, size):
    """ids(-1 은 뺀다)가 가진 점 수. size 번호까지."""
    ids = ids[ids >= 0]
    return np.bincount(ids, minlength=size) if len(ids) else np.zeros(size, dtype=np.int64)


def match(pred, gt, valid, thr):
    """예측 덩어리(pred)와 정답 단위(gt)를 IoU > thr 인 짝으로 맞춘다. IoU 큰 짝부터 하나씩 고른다 (한 쪽을 두 번 못 쓴다).
    pred, gt: 점마다 번호 (-1 = 없음). valid: void 가 아닌 점. [(예측 번호, 정답 번호, IoU)]"""
    size_p, size_g = int(pred.max()) + 1, int(gt.max()) + 1
    if size_p <= 0 or size_g <= 0:
        return []
    both = valid & (pred >= 0) & (gt >= 0)
    if not both.any():
        return []
    code, inter = np.unique(pred[both].astype(np.int64) * size_g + gt[both], return_counts=True)
    p, g = code // size_g, code % size_g
    n_p, n_g = _count(pred[valid], size_p), _count(gt[valid], size_g)
    iou = inter / (n_p[p] + n_g[g] - inter)
    out, used_p, used_g = [], set(), set()
    for k in np.argsort(-iou, kind="stable"):
        if iou[k] <= thr:
            break
        if int(p[k]) in used_p or int(g[k]) in used_g:
            continue
        used_p.add(int(p[k]))
        used_g.add(int(g[k]))
        out.append((int(p[k]), int(g[k]), float(iou[k])))
    return out


def judged_predictions(pred, valid):
    """채점에 넣는 예측 덩어리 번호. void 가 VOID_FP 를 넘는 덩어리는 정답을 모르는 곳의 것이라 뺀다."""
    size = int(pred.max()) + 1
    if size <= 0:
        return np.array([], dtype=int)
    total, void = _count(pred, size), _count(pred[~valid], size)
    ids = np.flatnonzero(total > 0)
    return ids[void[ids] <= VOID_FP * total[ids]]


def instance_ap(pred, gt, valid, thr, scores=None):
    """클래스 구분 없는 인스턴스 AP 하나 (IoU 기준 thr). scores: 예측 번호 → 신뢰도 (없으면 모두 같은 점수 → AP = 정밀도 x 재현율).
    {ap, precision, recall, tp, fp, fn}"""
    gt_ids = np.unique(gt[valid & (gt >= 0)])
    ids = judged_predictions(pred, valid)
    matched = {p for p, _, _ in match(pred, gt, valid, thr)}
    tp_flag = np.array([int(i) in matched for i in ids], dtype=float)
    n_gt = len(gt_ids)
    tp, fp = int(tp_flag.sum()), int(len(ids) - tp_flag.sum())
    out = dict(tp=tp, fp=fp, fn=n_gt - tp,
               precision=tp / len(ids) if len(ids) else 0.0, recall=tp / n_gt if n_gt else float("nan"))
    if n_gt == 0:
        return dict(out, ap=float("nan"))
    if len(ids) == 0:
        return dict(out, ap=0.0)
    s = np.ones(len(ids)) if scores is None else np.array([scores[int(i)] for i in ids], dtype=float)
    order = np.argsort(-s, kind="stable")
    s, hit = s[order], tp_flag[order]
    cum_tp, count = np.cumsum(hit), np.arange(1, len(hit) + 1)
    last = np.r_[s[1:] != s[:-1], True]                    # 같은 점수는 한꺼번에 자른다
    prec, rec = (cum_tp / count)[last], (cum_tp / n_gt)[last]
    prec = np.maximum.accumulate(prec[::-1])[::-1]         # 정밀도를 오른쪽에서부터 단조롭게 (PR 곡선 포락선)
    return dict(out, ap=float(np.sum(np.diff(np.r_[0.0, rec]) * prec)))


def _pq(tp, fp, fn, iou_sum):
    sq = iou_sum / tp if tp else 0.0
    rq = tp / (tp + 0.5 * fp + 0.5 * fn) if (tp + fp + fn) else 0.0
    return dict(tp=tp, fp=fp, fn=fn, sq=sq, rq=rq, pq=sq * rq)


def panoptic_things(pred, gt, valid):
    """물체(클래스 구분 없는 한 클래스)의 PQ. {tp, fp, fn, sq, rq, pq}"""
    pairs = match(pred, gt, valid, PQ_IOU)
    matched = {p for p, _, _ in pairs}
    ids = judged_predictions(pred, valid)
    n_gt = len(np.unique(gt[valid & (gt >= 0)]))
    return _pq(len(pairs), int(sum(int(i) not in matched for i in ids)), n_gt - len(pairs), sum(i for _, _, i in pairs))


def panoptic_stuff(pred_mask, gt_mask, valid):
    """구조물 클래스 하나(예측 한 덩어리 대 정답 한 덩어리)의 PQ. 정답이 없으면 None."""
    g = gt_mask & valid
    if not g.any():
        return None
    p = pred_mask & valid
    iou = (p & g).sum() / max((p | g).sum(), 1)
    if iou > PQ_IOU:
        return _pq(1, 0, 0, float(iou))
    mostly_void = pred_mask.sum() > 0 and (pred_mask & ~valid).sum() > VOID_FP * pred_mask.sum()
    return _pq(0, int(pred_mask.sum() > 0 and not mostly_void), 1, 0.0)


def evaluate(labels, gt):
    """42번 라벨(labels)을 정답(gt: {stuff: {클래스: 점 마스크}, inst: 점별 정답 단위 번호, void: 점 마스크})으로 채점한다."""
    pred = np.where(labels >= 0, labels, -1)
    valid = ~gt["void"]
    ap = {thr: instance_ap(pred, gt["inst"], valid, thr) for thr in sorted(set(AP_IOUS) | set(AP_RANGE))}
    pq = {"object": panoptic_things(pred, gt["inst"], valid)}
    for name, lab in STUFF.items():
        r = panoptic_stuff(labels == lab, gt["stuff"][name], valid)
        if r is not None:
            pq[name] = r
    things = [pq["object"]["pq"]]
    stuff = [r["pq"] for k, r in pq.items() if k != "object"]
    ap_mean = float(np.nanmean([ap[t]["ap"] for t in AP_RANGE]))
    return dict(
        ap25=ap[0.25]["ap"], ap50=ap[0.5]["ap"], ap=ap_mean,
        ap_detail={f"{t:g}": {k: v for k, v in ap[t].items()} for t in AP_IOUS},
        pq=float(np.mean(things + stuff)), pq_things=float(np.mean(things)), pq_stuff=float(np.mean(stuff)) if stuff else float("nan"),
        classes=pq, n_gt=int(len(np.unique(gt["inst"][valid & (gt["inst"] >= 0)]))), n_pred=int(len(judged_predictions(pred, valid))))


def _drop_small(unit, void):
    """점이 MIN_UNIT_POINTS 보다 적은 정답 단위를 void 로 돌린다 (42번 채점이 세지 않는 단위)."""
    unit, void = unit.copy(), void.copy()
    for u in np.unique(unit[unit >= 0]):
        m = unit == u
        if m.sum() < m42.MIN_UNIT_POINTS:
            void[m] = True
            unit[m] = -1
    return unit, void


def synth_gt(P, name):
    """합성 방 정답: 방 껍데기(바닥·벽·천장)와 정답 물체. 정답 표면에서 TRUE_DIST 보다 먼 점은 void."""
    T = np.loadtxt(DATA_DIR / name / "prepared_transform.txt")
    shell = o3d.geometry.TriangleMesh.create_box(*m42.m21.CLUTTER_ROOM).transform(T)
    z0, z1 = np.asarray(shell.vertices)[:, 2].min(), np.asarray(shell.vertices)[:, 2].max()
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(shell))          # geometry id 0 = 방 껍데기
    names, meshes = m42.gt_object_meshes(name)
    for mesh in meshes:
        scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))
    ans = scene.compute_closest_points(o3d.core.Tensor(P.astype(np.float32)))
    gid = ans["geometry_ids"].numpy().astype(int) - 1                            # -1 = 껍데기
    q = ans["points"].numpy()
    near = np.linalg.norm(q - P, axis=1) <= m42.TRUE_DIST
    on_shell = near & (gid < 0)
    stuff = {"floor": on_shell & (q[:, 2] < z0 + 1e-3),
             "ceiling": on_shell & (q[:, 2] > z1 - 1e-3)}
    stuff["wall"] = on_shell & ~stuff["floor"] & ~stuff["ceiling"]
    inst = np.where(near & (gid >= 0), gid, -1)
    unit, void = _drop_small(inst, ~near)
    return dict(stuff=stuff, inst=unit, void=void, names=names)


def s3dis_gt(P, name, mode):
    """S3DIS 정답. mode: 'instance' 가구 하나씩, 'contact' 맞닿은 가구를 묶은 단위. 물체·구조물이 아닌 점(보·기둥·창·문·칠판·잡동사니, 정답 없음)은 void."""
    cls_names, inst_names, cls, inst = m42.s3dis_labels(P, name)
    cn = np.array(cls_names + ["(정답 없음)"])[cls]
    obj = np.isin(cn, m42.S3DIS_OBJECTS)
    stuff = {k: cn == k for k in STUFF}
    void = ~(obj | np.isin(cn, m42.S3DIS_BASE))
    unit = np.full(len(P), -1)
    if mode == "instance":
        unit[obj] = inst[obj]
    else:
        members = sorted({int(i) for i in np.unique(inst[obj])})
        if members:
            for a, u in m42.contact_components(P, inst, members).items():
                unit[inst == a] = u
    unit, void = _drop_small(unit, void)
    return dict(stuff=stuff, inst=unit, void=void, names=inst_names)


def print_report(title, r):
    d25, d50 = r["ap_detail"]["0.25"], r["ap_detail"]["0.5"]
    print(f"\n[{title}] 정답 단위 {r['n_gt']}개, 예측 물체 덩어리 {r['n_pred']}개")
    print(f"  AP25 {r['ap25']:.3f}  AP50 {r['ap50']:.3f}  AP(IoU 0.5~0.95) {r['ap']:.3f}")
    print(f"    IoU>0.25: 맞음 {d25['tp']}, 틀림 {d25['fp']}, 놓침 {d25['fn']} (정밀도 {d25['precision']:.2f}, 재현율 {d25['recall']:.2f})")
    print(f"    IoU>0.5 : 맞음 {d50['tp']}, 틀림 {d50['fp']}, 놓침 {d50['fn']} (정밀도 {d50['precision']:.2f}, 재현율 {d50['recall']:.2f})")
    print(f"  PQ {r['pq']:.3f}  (물체 {r['pq_things']:.3f}, 구조물 {r['pq_stuff']:.3f})")
    for k, c in r["classes"].items():
        print(f"    {k:8s} PQ {c['pq']:.3f}  SQ {c['sq']:.3f}  RQ {c['rq']:.3f}  (TP {c['tp']} FP {c['fp']} FN {c['fn']})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("names", nargs="+", help="data/<이름>/normals.ply 를 읽는다 (이름이 s3dis 로 시작하면 S3DIS, synth 로 시작하면 합성 방)")
    parser.add_argument("--gap", type=float, default=m42.OBJ_GAP, help=f"42번 --gap (기본 {m42.OBJ_GAP})")
    args = parser.parse_args()

    rows = {}                                  # 제목 → 결과
    for name in args.names:
        pcd = o3d.io.read_point_cloud(str(DATA_DIR / name / "normals.ply"))
        P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
        labels, objects, _ = m42.find_objects(P, N, args.gap)
        gts = {"합성 방 정답": synth_gt(P, name)} if name.startswith("synth") else {
            "인스턴스 하나씩": s3dis_gt(P, name, "instance"), "맞닿은 묶음": s3dis_gt(P, name, "contact")}
        out = {}
        for mode, gt in gts.items():
            out[mode] = evaluate(labels, gt)
            print_report(f"{name} | 정답 단위 = {mode}", out[mode])
            rows.setdefault(mode, []).append(out[mode])
        (DATA_DIR / name / "score_seg.json").write_text(json.dumps(dict(gap=args.gap, results=out), indent=1, default=float))
        print(f"저장: {DATA_DIR / name / 'score_seg.json'}")

    if len(args.names) > 1:
        for mode, rs in rows.items():
            f = lambda k: float(np.nanmean([r[k] for r in rs]))
            print(f"\n[평균 {len(rs)}개 방 | 정답 단위 = {mode}] AP25 {f('ap25'):.3f}  AP50 {f('ap50'):.3f}  AP {f('ap'):.3f}  "
                  f"PQ {f('pq'):.3f} (물체 {f('pq_things'):.3f}, 구조물 {f('pq_stuff'):.3f})")


if __name__ == "__main__":
    main()
