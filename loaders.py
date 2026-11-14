"""
불러오기(adapter) 단계: 어떤 포인트클라우드든 아래 공통 형태로 바꾼다.
  - 단위 미터
  - z축이 위, 바닥이 z = 0, 방 가운데가 x = y = 0
  - 색이 있으면 유지, 법선은 버린다 (나중 단계에서 다시 구한다)

데이터셋마다 다른 점(파일 형식, 단위, 좌표축, 스캐너 좌표계 오프셋)은 최대한 여기서 흡수한다.
그래야 뒤 단계 코드가 특정 데이터셋에 묶이지 않는다.

지원 형식: .ply .pcd .xyz .xyzn .xyzrgb .pts (Open3D가 직접 읽음), .bin (KITTI: float32 x y z intensity),
          .e57 (측량 장비 표준 형식, pye57 필요)
"""
import numpy as np
import open3d as o3d

AXES = {"x": 0, "y": 1, "z": 2}
UP = np.array([0.0, 0.0, 1.0])


def load_cloud(path, scale=1.0):
    """파일을 읽어 PointCloud로 돌려준다. scale은 단위 변환용 (예: mm 데이터면 0.001)."""
    path = str(path)
    if path.endswith(".bin"):
        xyz = np.fromfile(path, dtype=np.float32).reshape(-1, 4)[:, :3]
        pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(xyz.astype(np.float64)))
    elif path.lower().endswith(".e57"):
        pcd = read_e57(path)
    else:
        pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        raise ValueError(f"점을 읽지 못했다: {path}")
    pcd.normals = o3d.utility.Vector3dVector(np.zeros((0, 3)))
    if scale != 1.0:
        pcd.scale(scale, center=(0, 0, 0))
    return pcd


def read_e57(path):
    """E57을 읽는다. 스캔이 여러 개면 각 스캔의 자세(pose)를 적용해 하나로 합친다."""
    import pye57  # E57을 읽을 때만 필요해서 여기서 불러온다

    e57 = pye57.E57(path)
    pts, cols = [], []
    for i in range(e57.scan_count):
        d = e57.read_scan(i, colors=True, ignore_missing_fields=True, transform=True)
        valid = np.ones(len(d["cartesianX"]), dtype=bool)
        if "cartesianInvalidState" in d:  # 0 = 정상, 1 = 방향만 있음, 2 = 측정 실패
            valid = d["cartesianInvalidState"] == 0
        pts.append(np.c_[d["cartesianX"], d["cartesianY"], d["cartesianZ"]][valid])
        if "colorRed" in d:
            rgb = np.c_[d["colorRed"], d["colorGreen"], d["colorBlue"]][valid].astype(np.float64)
            cols.append(rgb / (255.0 if rgb.max() <= 255 else 65535.0))
    pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack(pts)))
    if len(cols) == len(pts):
        pcd.colors = o3d.utility.Vector3dVector(np.vstack(cols))
    return pcd


def load_reference_normals(path):
    """채점용 정답 법선이 들어 있는 점 파일을 읽어 (점, 법선)을 돌려준다 (원본 좌표, 길이 0인 법선은 뺌).
      .e57: NavVis처럼 확장 필드 nor:normalX/Y/Z 에 법선을 넣어 둔 파일. pye57의 기본 읽기 함수는 확장 필드를
            거부해서 저수준 버퍼로 직접 읽는다. 스캔 자세(회전, 이동)를 점과 법선에 적용한다
      .ply/.pcd 등: 점마다 법선이 있는 파일 (예: 21번 합성 방)"""
    path = str(path)
    if path.lower().endswith(".e57"):
        import pye57
        from pye57 import libe57

        e57 = pye57.E57(path)
        names = ["cartesianX", "cartesianY", "cartesianZ", "nor:normalX", "nor:normalY", "nor:normalZ"]
        Ps, Ns = [], []
        for i in range(e57.scan_count):
            header = e57.get_header(i)
            if not all(k in header.point_fields for k in names):
                raise ValueError(f"{path} 스캔 {i}에 법선 필드(nor:normalX..)가 없다")
            node = e57.data3d[i]["points"]
            n = node.childCount()
            data = {k: np.empty(n, np.float64) for k in names}
            buffers = libe57.VectorSourceDestBuffer()
            for k in names:
                buffers.append(libe57.SourceDestBuffer(e57.image_file, k, data[k], n, True, True))
            node.reader(buffers).read()
            R, t = header.rotation_matrix, header.translation
            Ps.append(np.c_[data["cartesianX"], data["cartesianY"], data["cartesianZ"]] @ R.T + t)
            Ns.append(np.c_[data["nor:normalX"], data["nor:normalY"], data["nor:normalZ"]] @ R.T)
        P, N = np.vstack(Ps), np.vstack(Ns)
    else:
        pcd = o3d.io.read_point_cloud(path)
        if not pcd.has_normals():
            raise ValueError(f"{path}에 법선이 없다")
        P, N = np.asarray(pcd.points), np.asarray(pcd.normals)
    keep = np.linalg.norm(N, axis=1) > 0.5
    return P[keep], N[keep]


def rotation_between(a, b):
    """단위벡터 a를 단위벡터 b로 돌리는 회전 행렬 (로드리게스 공식)."""
    a, b = a / np.linalg.norm(a), b / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if c < -1 + 1e-9:  # 정반대: a에 수직인 아무 축으로 180도
        axis = np.cross(a, [1.0, 0, 0]) if abs(a[0]) < 0.9 else np.cross(a, [0, 1.0, 0])
        axis /= np.linalg.norm(axis)
        return 2 * np.outer(axis, axis) - np.eye(3)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx / (1 + c)


def ransac_plane(P, dist, iterations=2000, seed=0):
    """
    RANSAC으로 가장 많은 점이 지나는 평면을 찾는다. (a, b, c, d)(ax + by + cz + d = 0, 법선은 단위 길이)와 평면 가까이 있는 점 번호를 돌려준다.
    open3d의 segment_plane은 여러 스레드가 나눠 돌아서 시드를 줘도 같은 입력에서 결과가 조금씩 달랐다. 그래서 시드를 박은 numpy로 직접 한다.
    """
    rng = np.random.default_rng(seed)
    best_count, best = -1, None
    for tri in rng.integers(0, len(P), size=(iterations, 3)):
        a, b, c = P[tri]
        n = np.cross(b - a, c - a)
        length = np.linalg.norm(n)
        if length < 1e-12:  # 세 점이 한 줄 위에 있음
            continue
        n /= length
        count = np.count_nonzero(np.abs(P @ n - a @ n) < dist)
        if count > best_count:
            best_count, best = count, (n, a @ n)
    if best is None:
        raise RuntimeError("평면을 찾지 못했다")
    n, offset = best
    inl = np.flatnonzero(np.abs(P @ n - offset) < dist)
    center = P[inl].mean(axis=0)  # 평면 가까이 있는 점들로 평면을 다시 맞춘다 (최소제곱)
    n = np.linalg.svd(P[inl] - center, full_matrices=False)[2][2]
    return np.append(n, -n @ center), inl


def level_floor(pcd, up="z", band=0.3, angle_tol=15.0, dist=0.02, sample=300_000, seed=0):
    """
    바닥을 찾아 z = 0 수평면에 맞춘다. 결과 점군과 4x4 변환, 정보를 돌려준다. 같은 입력과 seed면 같은 결과가 나온다.
      1) up 축(예: 'z', '-y')이 +z가 되도록 대충 돌린다
      2) 가장 낮은 band [m] 높이 안의 점들에서 RANSAC으로 평면을 찾는다
      3) 그 평면의 법선이 +z에서 angle_tol 이내면 바닥으로 보고, 정확히 수평이 되게 돌린 뒤 z = 0으로 내린다
      4) x, y는 방 가운데가 원점이 되게 옮긴다
    """
    sign = -1.0 if up.startswith("-") else 1.0
    axis = np.zeros(3)
    axis[AXES[up[-1]]] = sign
    R0 = rotation_between(axis, UP)

    P = np.asarray(pcd.points) @ R0.T
    rng = np.random.default_rng(seed)
    S = P[rng.choice(len(P), min(sample, len(P)), replace=False)]
    low = S[S[:, 2] < np.percentile(S[:, 2], 1) + band]
    cand = low

    plane = None
    for attempt in range(3):  # 낮은 곳의 가장 큰 평면이 바닥이 아닐 수도 있으니 몇 번 시도
        model, inl = ransac_plane(cand, dist, seed=seed + attempt)
        n = model[:3]
        if abs(n[2]) > np.cos(np.deg2rad(angle_tol)):
            plane = (n * np.sign(n[2]), model[3] * np.sign(n[2]), len(inl))
            break
        cand = np.delete(cand, inl, axis=0)
    if plane is None:
        raise RuntimeError("바닥 평면을 찾지 못했다. up 축이 맞는지 확인하세요.")
    n, d, inliers = plane

    R = rotation_between(n, UP) @ R0
    Q = np.asarray(pcd.points) @ R.T
    cx, cy = (np.percentile(Q[:, :2], 1, axis=0) + np.percentile(Q[:, :2], 99, axis=0)) / 2
    t = np.array([-cx, -cy, d])  # 회전 뒤 바닥 평면은 z = -d 이므로 d만큼 올린다
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t

    out = o3d.geometry.PointCloud(pcd)
    out.transform(T)
    info = {"tilt_deg": float(np.degrees(np.arccos(min(1.0, n[2])))), "floor_inlier_ratio": inliers / len(low)}
    return out, T, info
