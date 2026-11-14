"""여러 스크립트에서 같이 쓰는 작은 함수와 경로."""
from pathlib import Path

import numpy as np
import open3d as o3d

DATA_DIR = Path(__file__).parent / "data"


def orient_toward_viewpoints(pcd, viewpoints, chunk=50_000):
    """sensor 방식: 점마다 가장 가까운 시점(센서) 쪽을 향하도록 법선을 뒤집는다."""
    p, n = np.asarray(pcd.points), np.asarray(pcd.normals).copy()
    viewpoints = np.asarray(viewpoints)
    for s in range(0, len(p), chunk):  # 점 x 시점 거리표가 커지지 않게 나눠서 계산
        q = p[s:s + chunk]
        near = viewpoints[np.argmin(((q[:, None] - viewpoints[None]) ** 2).sum(axis=2), axis=1)]
        block = n[s:s + chunk]
        block[np.einsum("ij,ij->i", block, near - q) < 0] *= -1
    pcd.normals = o3d.utility.Vector3dVector(n)


def orient_by_propagation(pcd, k=15, center=None):
    """tangent 방식: 이웃끼리 방향을 이어 맞춘 뒤, 다수가 center(기본: 점들의 평균)를 향하도록 전체 부호를 고른다."""
    pcd.orient_normals_consistent_tangent_plane(k)
    p, n = np.asarray(pcd.points), np.asarray(pcd.normals)
    c = p.mean(axis=0) if center is None else np.asarray(center)
    if np.mean(np.einsum("ij,ij->i", n, c - p) > 0) < 0.5:
        pcd.normals = o3d.utility.Vector3dVector(-n)


def height_colors(points):
    """보기 편하도록 높이(z)에 따라 파랑 -> 초록 -> 빨강으로 칠한다. 저장 파일에는 넣지 않는다."""
    z = points[:, 2]
    s = (z - z.min()) / max(np.ptp(z), 1e-9)
    return np.stack([np.clip(2 * s - 1, 0, 1), 1 - np.abs(2 * s - 1), np.clip(1 - 2 * s, 0, 1)], axis=1)
