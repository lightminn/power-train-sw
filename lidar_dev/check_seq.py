# check_seq.py — record_seq.py로 만든 기록 폴더(l515-seq v1)를 점검한다. 구조 오류가 있으면 종료 코드 1.
# 사용법: python check_seq.py data/20261017_220000_wall_1m [다른 폴더 ...]
import json, os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seq_format import check_sequence


def main():
    if len(sys.argv) < 2:
        print(__doc__ or "사용법: python check_seq.py <기록 폴더> [...]")
        sys.exit(2)
    bad = 0
    for folder in sys.argv[1:]:
        E, W, S = check_sequence(folder)
        print(f"== {folder}: {'실패' if E else '통과'}")
        for e in E:
            print("  오류:", e)
        for w in W:
            print("  경고:", w)
        print("  요약:", json.dumps(S, ensure_ascii=False))
        bad += bool(E)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
