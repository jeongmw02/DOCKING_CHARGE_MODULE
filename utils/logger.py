# utils/logger.py
# 공통 로거 설정. 모든 모듈이 get_logger()로 같은 핸들러를 공유한다.

import logging
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
import config


def get_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if logger.handlers:
        return logger

    level = getattr(logging, config.LOG_LEVEL.upper(), logging.DEBUG)
    logger.setLevel(level)

    fmt = logging.Formatter(
        fmt="%(asctime)s [%(levelname)-8s] %(name)-20s | %(message)s",
        datefmt="%H:%M:%S"
    )

    # 콘솔
    ch = logging.StreamHandler(sys.stdout)
    ch.setFormatter(fmt)
    logger.addHandler(ch)

    # 파일 (선택)
    if config.LOG_TO_FILE:
        fh = logging.FileHandler(config.LOG_FILE, encoding="utf-8")
        fh.setFormatter(fmt)
        logger.addHandler(fh)

    return logger
