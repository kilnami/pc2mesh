# pc2mesh

실내 LiDAR 점군을 시뮬레이터용 메시로 바꾸는 파이프라인이다. 특정 시뮬레이터에 묶이지 않게
공통 결과물(보기용 OBJ + 텍스처, 충돌용 OBJ)을 만들고, 그 위에 PyBullet·Isaac용 URDF와 Gazebo용 SDF를 얹는다.
넓은 현장은 타일로 나눠 월드 하나로 내보내고, 로봇 주행용 nav2 지도까지 만든다.

```
점군 → 불러오기·좌표 정리 → 진단·추천 → 방 밖 자르기·이상치 제거 → 법선(축 + 부호)
     → 재구성(Poisson / Ball Pivoting) → 정리 → 삼각형 줄이기(보기용 / 충돌용) → 텍스처 굽기
     → URDF / SDF 내보내기 → PyBullet·Gazebo 검증 → (넓은 현장) 타일 분할 → nav2 지도
```

스크립트는 단계 순서대로 번호가 붙어 있다. 각 파일 맨 위 설명에 왜 그렇게 했는지, 실행 방법, 시험한 결과가 적혀 있다.

## 스크립트

| 단계 | 파일 | 하는 일 |
|---|---|---|
| 2 합성 데이터 | `21_make_synth_room.py` | 정답을 아는 가상의 방(`--clutter`: 물체가 많은 방)을 가상 LiDAR로 스캔한다 |
| | `22_preprocess.py` | 복셀 다운샘플링과 통계적 이상치 제거 |
| | `23_normals.py` | 법선 추정: PCA로 축을 구하고 부호를 `none` / `sensor` / `tangent`로 정한다 |
| | `24_reconstruct.py` | Poisson(raw, trim)과 Ball Pivoting을 정답 메시와 비교한다 |
| 3 실제 데이터 | `31_prepare.py` | 불러오기, 다운샘플, 법선 반경·Poisson depth·부호 방식 추천(`inspect.json`) |
| | `32_clean_normals.py` | 방 밖 자르기, 이상치 제거, 가상 센서·혼합 법선 부호, 정답 없는 부호 검증 |
| | `33_reconstruct_eval.py` | 재구성 결과를 더 좋은 스캐너의 정답 점과 비교한다 (정확도, 완성도, F) |
| | `37_score_normals.py` | 법선 부호를 정답 법선으로 채점한다 |
| 4 내보내기 | `34_export.py` | 메시 정리, 삼각형 줄이기, 텍스처 굽기, OBJ / GLB / URDF / SDF 저장 |
| | `35_check_pybullet.py` | PyBullet에서 광선 검사와 공 떨어뜨리기 |
| | `36_check_gazebo.py` | Gazebo에서 같은 공 떨어뜨리기와 카메라 사진 |
| 5 타일·주행 | `38_tiles.py` | 바닥을 격자 타일로 나눠 타일별 메시·텍스처와 `world.sdf`를 만든다 |
| | `39_check_tiles.py` | 타일 월드의 기동 시간, 메모리, 경계 이음매를 Gazebo에서 잰다 |
| | `40_nav2_map.py` | 타일 메시를 높이 구간으로 잘라 nav2 지도(위치 추정용, 장애물용)를 만든다 |
| | `41_tiles_variant.py` | `38_tiles.py --floor-support`로 만든 패키지에서 받침 상자만 뺀 시험용 사본을 만든다 (받침이 없는 기본 패키지에는 쓸 수 없다) |
| 공통 | `loaders.py`, `common.py`, `view.py` | 파일 불러오기, 공통 함수, 3D 뷰어 |

## 준비

Python 3.10에서 만들고 시험했다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install open3d==0.20.0 numpy==2.2.6 matplotlib==3.10.9 pybullet==3.2.7 pye57==0.4.19
```

* `pye57`은 `.e57` 파일을 읽을 때만 쓴다.
* `36_check_gazebo.py`, `39_check_tiles.py`는 Gazebo Fortress(ROS 2 Humble과 짝인 Ignition Gazebo 6, `ign` 명령)가 있어야 한다.

## 데이터

`data/`는 용량이 커서(수 GB) git에 올리지 않는다. 스크립트가 필요한 폴더를 `data/` 아래에 만든다.
원본은 `data/<이름>/raw/`에 직접 받아 둔다.

| 이름 | 내용 | 출처 |
|---|---|---|
| `synth_room`, `synth_clutter` | 가상 방 | `21_make_synth_room.py`가 만든다 |
| `redwood_bedroom` | 침실 한 칸 (`bedroom.ply`) | Redwood Indoor LiDAR-RGBD, Bedroom (퍼블릭 도메인) |
| `hall101` | 책상 한 조각. 정답 스캐너(RTC360)와 입력 스캐너 4종 | Aalto, Hall_101_segment (Zenodo 3876364, CC BY 4.0) |
| `factory` | 공장 11 × 18 m (`Factory-small.e57`, NavVis VLX2) | NavVis 샘플. 라이선스가 적혀 있지 않아 재배포하지 않는다 |

## 실행 예

가상 방(정답을 알고 있어서 단계마다 숫자로 확인할 수 있다):

```bash
python 21_make_synth_room.py --no-view
python 22_preprocess.py --no-view
python 23_normals.py --no-view
python 24_reconstruct.py --no-view
```

공장(실제 점군에서 타일 월드와 nav2 지도까지):

```bash
python 31_prepare.py data/factory/raw/Factory-small.e57 --name factory --no-view
python 32_clean_normals.py factory --no-view
python 34_export.py factory --no-view                  # 메시 하나짜리 내보내기
python 38_tiles.py factory --grid 2x2 --visual-total 1000000 --collision-total 100000 --texture-px 2048
python 39_check_tiles.py factory                       # Gazebo 확인
python 40_nav2_map.py factory                          # nav2 지도
```

결과물은 `data/<이름>/export/`(메시 하나), `data/<이름>/tiles/`(타일 월드), `data/<이름>/nav2/`(지도)에 생긴다.
정답 점이 있으면 `--gt <파일>`로 정확도와 완성도도 같이 잰다.

## 알려 둘 것

* 바닥 받침 상자(`38_tiles.py --floor-support`)는 기본으로 넣지 않는다. 모델을 올려 읽는 플랫폼이 받침 아랫면을 건물 바닥으로 읽어 월드를 0.1 m 띄웠고, 그 월드에서 로봇이 걷지 못했다. 스캔이 바닥을 못 본 구멍은 올리는 쪽이 바닥 평면으로 메운다.
* 타일 월드는 아직 한 층짜리 현장만 된다.
* 지붕처럼 한쪽 면만 있는 메시는 바깥에서 보면 안 보인다. 양면 처리는 아직 적용하지 않았다.

아직 하지 않은 것: 층 나누기(1층·2층), 물체별 모델과 정답 상자, MuJoCo용 볼록 분해, 벽을 바닥까지 내리는 후처리.
