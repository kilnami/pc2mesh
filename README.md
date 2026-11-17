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
| 4 내보내기 | `34_export.py` | 메시 정리, 삼각형 줄이기, 충돌용 바닥 구멍 패치, 텍스처 굽기, OBJ / GLB / URDF / SDF 저장 |
| | `35_check_pybullet.py` | PyBullet에서 광선 검사와 공 떨어뜨리기 |
| | `36_check_gazebo.py` | Gazebo에서 같은 공 떨어뜨리기와 카메라 사진 |
| 5 타일·주행 | `38_tiles.py` | 바닥을 격자 타일로 나눠 타일별 메시·텍스처와 `world.sdf`를 만든다 |
| | `39_check_tiles.py` | 타일 월드의 기동 시간, 메모리, 경계 이음매를 Gazebo에서 잰다 |
| | `40_nav2_map.py` | 타일 메시를 높이 구간으로 잘라 nav2 지도(위치 추정용, 장애물용)를 만든다 |
| | `41_tiles_variant.py` | `38_tiles.py --floor-support`로 만든 패키지에서 받침 상자만 뺀 시험용 사본을 만든다 (받침이 없는 기본 패키지에는 쓸 수 없다) |
| | `45_tiles_flat_floor.py` | 타일 패키지에서 충돌 바닥의 높이만 z = 0 으로 평평하게 맞춘 시험용 사본을 만든다 (Go2 가 못 걷는 이유가 울퉁불퉁한 충돌 바닥인지 가리려고) |
| 6 물체 | `42_objects.py` | 점군에서 구조물(바닥·벽·지붕)을 빼고 남은 점을 물체별 덩어리로 나눈다 (합성 방은 정답 물체로 채점. 공장은 아직 안 됨) |
| | `43_object_models.py` | 물체 덩어리마다 모델 폴더(보기용·충돌용 메시, 텍스처, model.sdf)를 만든다. `38_tiles.py --with-objects` 가 이 모델들을 월드에 `<pose>` 와 함께 include 하고 그 물체 점은 타일에서 뺀다 |
| | `47_score_seg.py` | 42번이 나눈 결과를 AP25·AP50·PQ(Panoptic Quality)로 채점한다. 합성 방과 S3DIS 방의 정답을 쓰고, 인스턴스 하나씩과 맞닿은 묶음 두 가지 단위로 센다. 예측에 신뢰도가 없어서 AP 는 정밀도 x 재현율이다 |
| 공통 | `loaders.py`, `common.py`, `view.py` | 파일 불러오기, 공통 함수, 3D 뷰어 |

## 준비

Python 3.10에서 만들고 시험했다.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

* 코드가 직접 불러오는 패키지만 `requirements.txt`에 버전을 고정했다.
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

## 시험

합성 방 하나로 파이프라인을 처음부터 끝까지 돌려서 죽지 않고 대충 맞는 결과가 나오는지 보는 스모크 테스트가 있다.
스크립트를 임시 폴더에 복사해서 돌리므로 내 `data/`는 건드리지 않는다.

```bash
pip install -r requirements-dev.txt    # requirements.txt + pytest
pytest                    # 전부 (약 2~3분). Gazebo(ign)가 없으면 그 시험만 건너뛴다
pytest -m "not gazebo"    # Gazebo 없이
```

법선 부호, 재구성 정확도, PyBullet·Gazebo에서 공 떨어뜨리기, 타일 이음매와 텍스처 이름, nav2 지도 규칙, 같은 입력이면 같은 결과가 나오는지를 본다.
기준선은 합성 방에서 잰 값에 여유를 둔 것이라 정확한 숫자는 맞추지 않는다. 공장 같은 큰 현장의 문제는 잡지 못한다.

## 알려 둘 것

* 바닥 받침 상자(`38_tiles.py --floor-support`)는 기본으로 넣지 않는다. 모델을 올려 읽는 플랫폼이 받침 아랫면을 건물 바닥으로 읽어 월드를 0.1 m 띄웠고, 그 월드에서 로봇이 걷지 못했다. 스캔이 바닥을 못 본 구멍은 올리는 쪽이 바닥 평면으로 메운다.
* 충돌용 메시에는 바닥 패치를 얹는다 (`34_export.py`, `38_tiles.py`. `--no-floor-patch` 로 끈다). 두께 없는 면을 두 곳에 깐다: 바닥 점은 있는데 메시가 없는 구멍, 그리고 바닥 점이 아예 없지만 점으로 둘러싸인 범위 안인 자리(물체 밑 등. 바닥이 평평하다고 보고 바닥이 보이는 곳에서 2 m 이내만). 높이는 주변 메시 바닥을 따른다. 받침 상자와 달리 메시의 최저 높이를 바꾸지 않는다. 보기용에는 넣지 않는다.
  패치 삼각형은 충돌용 예산 밖에 더해진다.
* 벽 밑이 떠 있는 곳은 벽 치마로 바닥까지 내린다 (`34_export.py`, `38_tiles.py`. `--no-wall-skirt` 로 끈다). 벽은 바닥까지 이어져 있는 게 당연한데 스캔은 기계·팔레트 뒤의 벽 밑을 못 본다 (공장 동쪽 벽은 원본에서도 밑단 칸의 33%에 점이 없다). 점군에서 큰 수직 벽 평면을 찾아 열마다 벽이 처음 나오는 높이부터 바닥까지 얇은 면을 깐다. 1 m 넘게 떠 있는 열은 문 같은 열린 곳으로 보고 둔다. 라이다는 충돌용이 아니라 보기용 메시를 보므로 보기용에도 벽 색으로 넣는다.
  한계: 벽 평면을 직각 건물로 가정해 두 방향에서만 찾는다. 폭 기준은 min(3 m, 장면 크기의 40%)라서 작은 방의 짧은 벽도 찾지만, 점이 5000개 미만인 벽이나 채움 비율(벽 범위 중 점이 있는 칸)이 32.5% 미만인 벽은 놓친다. 공장에서는 큰 벽이 동쪽 하나뿐이라 나머지(기계 옆면 등)는 효과가 작다. 벽 가운데가 통째로 비어 있는 구멍(벽 높이 전체)은 메우지 않는다.
* 물체를 따로 모델로 빼는 흐름 (`42_objects.py` → `43_object_models.py` → `38_tiles.py --with-objects`)은 가구 많은 합성 방(`21_make_synth_room.py --clutter`)에서만 끝까지 돌려 봤다. 정답 비교는 월드 전체 정확도 99.4%, 완성도 99.0%로 물체를 타일에 넣은 기준과 같은 수준이다. 공장은 42번에서 막혀 있다 (지붕 판정, 이어진 거대 덩어리).
  물체를 빼면 구조물 타일의 밀도 trim(기본 5%)이 진짜 벽·천장 가장자리를 깎아서(합성 방 구조물 완성도 98.2 → 96.4%) `--density-quantile 0.01` 을 같이 준다.
  한계: 위가 안 찍힌 물체(상자 더미, 기계)의 충돌 메시는 위가 뚫려 있다. 회전(yaw)은 0이고 모델은 static 이다.
* 타일 월드는 아직 한 층짜리 현장만 된다.
* 지붕처럼 한쪽 면만 있는 메시는 바깥에서 보면 안 보인다. 양면 처리는 아직 적용하지 않았다.

아직 하지 않은 것: 층 나누기(1층·2층), 공장의 물체 분리, 위가 뚫린 물체 닫기, 움직이는 물체(질량)와 정답 상자, MuJoCo용 볼록 분해, 벽 가운데 구멍 메우기.
