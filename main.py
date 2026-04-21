# main.py
# CubeSat 도킹·충전 모듈 - 진입점
#
# 실행: python main.py
# 종료: Ctrl+C



from state_machine import DockingStateMachine
from utils import get_logger

log = get_logger("Main")


def main():
    sm = DockingStateMachine()
    try:
        sm.setup()
        sm.run(loop_hz=10.0)
    except KeyboardInterrupt:
        log.info("종료 요청")
    finally:
        sm.cleanup()


if __name__ == "__main__":
    main()
