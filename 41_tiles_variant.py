"""
5단계-2: 38번이 만든 타일 패키지에서 바닥 받침 상자(floor_support)만 뺀 시험용 사본을 만든다

(38번은 이제 받침을 기본으로 넣지 않는다. 이 스크립트는 --floor-support 로 만든 패키지나 예전 패키지에 쓴다)

왜: 플랫폼(메시 올리기)이 우리 월드의 바닥을 z = -0.109 m 로 읽고 월드를 +0.109 m 올렸다.
    이 숫자는 받침 상자 아랫면(윗면 -0.01 m - 두께 0.1 m)의 중앙값과 같다. 받침을 바닥으로 착각해서
    실제 바닥이 +0.1 m 로 올라갔고, 같은 Go2 가 주조룸 월드에서는 걷고 이 월드에서는 못 걸었다.
    받침을 뺀 사본으로 올렸더니 바닥 z = 0.000 으로 읽혔고 Go2 가 걸었다 (확인함).
    받침을 빼면 플랫폼이 충돌 메시의 바닥(-0.01 ~ -0.02 m)을 읽을 것이다. 구멍 메우기는 플랫폼의 바닥 평면에 맡긴다.
    충돌 메시, 보기용 메시, 텍스처, world.sdf 는 그대로여서 받침 하나만 바뀐 비교가 된다.

입력: data/<이름>/tiles/ (38번 결과)
출력: data/<이름>/tiles_nosupport/ 와 data/<이름>/tiles_nosupport.zip

실행:
  python 41_tiles_variant.py factory
"""
import argparse
import re
import shutil
import zipfile

from common import DATA_DIR

SUPPORT_RE = re.compile(r"[ ]*<!-- 바닥 받침.*?-->\s*<collision name=\"floor_support\">.*?</collision>\n", re.S)
README_NOTE = """  - 시험용 사본: model.sdf 에서 바닥 받침(floor_support)을 뺐다. 받침 아랫면을 바닥으로 읽는 도구가 월드를 올려서다.
    스캔이 바닥을 못 본 구멍에서는 로봇이 빠질 수 있다 (바닥 평면은 올리는 쪽이 넣는다).
"""


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("name", help="data/ 아래 폴더 이름 (38번까지 마친 것)")
    args = parser.parse_args()

    src = DATA_DIR / args.name / "tiles"
    dst = DATA_DIR / args.name / "tiles_nosupport"
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(src, dst)

    for sdf in sorted((dst / "models").glob("*/model.sdf")):
        text = sdf.read_text()
        new = SUPPORT_RE.sub("", text)
        assert new != text and "floor_support" not in new, f"{sdf}: 바닥 받침을 못 찾았다"
        sdf.write_text(new)
        print(f"  {sdf.parent.name}: 받침 뺌")

    # README: 받침 설명 두 줄을 시험용 안내로 바꾼다
    readme = dst / "README.txt"
    lines = readme.read_text().splitlines(keepends=True)
    i = next(k for k, line in enumerate(lines) if "floor_support" in line)
    readme.write_text("".join(lines[:i]) + README_NOTE + "".join(lines[i + 2:]))

    # zip: 압축 풀면 world.sdf, tiles.json, models/ 가 바로 나오게 (tiles.zip 과 같은 구조)
    out = DATA_DIR / args.name / "tiles_nosupport.zip"
    out.unlink(missing_ok=True)
    with zipfile.ZipFile(out, "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted(dst.rglob("*")):
            if f.is_file():
                z.write(f, f.relative_to(dst))
    print(f"→ {dst}/ , {out} ({out.stat().st_size / 2 ** 20:.1f} MB)")


if __name__ == "__main__":
    main()
