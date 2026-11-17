"""
6단계-3: S3DIS 방 하나를 점군 + 정답 라벨로 바꾸기 (물체 분리를 다른 데이터로 시험하려고)

S3DIS(Stanford Large-Scale Indoor Spaces)는 방마다 Annotations/<클래스>_<번호>.txt 로 물체(인스턴스)별 점군이 따로 들어 있다
(한 줄 = x y z r g b, 색은 0~255). 이 파일들을 합치면 방 전체 점군이 되므로, 점마다 클래스와 인스턴스 번호를 알 수 있다.
이 파일은 합친 점군(.ply, 색은 0~1)과 점별 정답 (gt_points.npz)을 만든다. 점군은 31번에 그대로 넣고,
정답은 42번 --score-s3dis 가 31번의 변환을 같이 적용해서 처리된 점에 옮겨 채점한다.

S3DIS는 비상업적 연구·학술 목적으로만 쓰고 재배포하지 않는다는 조건이 있다 (Terms of Data Sharing and Usage).
그래서 결과는 data/ 아래에만 만들고 (git에 안 올라간다) 월드·zip 은 만들지 않는다.

데이터는 믿을 수 없는 입력으로 보고, 이 스크립트는 저장소의 다른 모듈을 불러오지 않는다 (python -I 로 돌린다).

입력: S3DIS 방 폴더 (예: .../Area_1/office_1)
출력: data/<이름>/raw/<이름>.ply, data/<이름>/raw/gt_points.npz
      gt_points.npz: xyz (원본 좌표, float32), cls (클래스 번호), inst (인스턴스 번호, 방 안에서 파일마다 하나), cls_names, inst_names

실행:
  python -I 44_s3dis_room.py data/s3dis/raw/Area_1/office_1 --name s3dis_office_1
  python 31_prepare.py data/s3dis_office_1/raw/s3dis_office_1.ply --name s3dis_office_1 --no-view
"""
import argparse
import re
from pathlib import Path

import numpy as np
import open3d as o3d

DATA_DIR = Path(__file__).resolve().parent / "data"   # common.DATA_DIR 와 같다 (이 파일은 다른 모듈을 안 불러온다)


def read_xyzrgb(path):
    """한 줄에 숫자 6개인 텍스트를 (N, 6) 배열로 읽는다. 깨진 줄(설명 파일의 '잘못된 문자' 수정 목록 참고)은 건너뛴다."""
    try:
        return np.loadtxt(path, dtype=np.float64, ndmin=2, usecols=range(6))
    except ValueError:
        rows = []
        with open(path, "rb") as f:
            for line in f:
                t = line.split()
                if len(t) < 6:
                    continue
                try:
                    rows.append([float(v) for v in t[:6]])
                except ValueError:
                    continue
        return np.array(rows, dtype=np.float64).reshape(-1, 6)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("room", help="S3DIS 방 폴더 (Annotations 폴더가 들어 있는 곳)")
    parser.add_argument("--name", help="data/ 아래에 만들 폴더 이름 (기본: 방 폴더 이름)")
    args = parser.parse_args()

    room = Path(args.room)
    name = args.name or room.name
    files = sorted((room / "Annotations").glob("*.txt"))
    assert files, f"{room}/Annotations 에 .txt 가 없다"
    cls_names, inst_names, xyz, rgb, cls, inst = [], [], [], [], [], []
    for k, f in enumerate(files):
        c = re.sub(r"_?\d+$", "", f.stem)            # chair_3 -> chair, ceiling -> ceiling
        if c not in cls_names:
            cls_names.append(c)
        a = read_xyzrgb(f)
        xyz.append(a[:, :3]); rgb.append(a[:, 3:])
        cls.append(np.full(len(a), cls_names.index(c), np.int16)); inst.append(np.full(len(a), k, np.int32))
        inst_names.append(f.stem)
    xyz, rgb, cls, inst = (np.concatenate(v) for v in (xyz, rgb, cls, inst))

    whole = room / f"{room.name}.txt"
    if whole.exists():
        n_whole = sum(1 for _ in open(whole, "rb"))
        print(f"방 전체 파일 {n_whole:,}줄, 물체 파일을 합친 점 {len(xyz):,}개" +
              ("" if n_whole == len(xyz) else "  ← 다르다 (깨진 줄이 있었거나 일부 물체 파일이 빠졌다)"))
    print(f"범위: x {xyz[:, 0].min():.2f}~{xyz[:, 0].max():.2f}, y {xyz[:, 1].min():.2f}~{xyz[:, 1].max():.2f}, "
          f"z {xyz[:, 2].min():.2f}~{xyz[:, 2].max():.2f} (단위가 미터인지 확인)")
    print("클래스별 인스턴스 수 / 점 수:")
    for i, c in enumerate(cls_names):
        m = cls == i
        print(f"  {c:10s} {len(np.unique(inst[m])):3d}개 {m.sum():9,d}점")

    out = DATA_DIR / name / "raw"
    out.mkdir(parents=True, exist_ok=True)
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz))
    pcd.colors = o3d.utility.Vector3dVector(np.clip(rgb / 255.0, 0, 1))
    o3d.io.write_point_cloud(str(out / f"{name}.ply"), pcd)
    np.savez_compressed(out / "gt_points.npz", xyz=xyz.astype(np.float32), cls=cls, inst=inst,
                        cls_names=np.array(cls_names), inst_names=np.array(inst_names))
    print(f"\n저장: {out}/{name}.ply, gt_points.npz")


if __name__ == "__main__":
    main()
