# tests/test_electromagnet.py
# 전자석 ON/OFF 동작 확인 스크립트
# 실행: python tests/test_electromagnet.py

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hardware import Electromagnet
import time

def test_electromagnet():
    em = Electromagnet()
    em.setup()

    print("전자석 ON (3초)")
    em.engage()
    time.sleep(3)

    print("전자석 OFF")
    em.release()
    time.sleep(1)

    em.cleanup()
    print("테스트 완료")

if __name__ == "__main__":
    test_electromagnet()
