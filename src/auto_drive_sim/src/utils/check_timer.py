#!/usr/bin/env python3
# -*- coding:utf-8 -*-
import time

class CheckTimer:
    def __init__(self):
        self._start = None
        self._end = None

    def start(self):
        self._start = time.time()
        self._end = None

    def end(self):
        if self._start is None:
            raise RuntimeError("Timer has not been started yet.")
        self._end = time.time()

    def check(self):
        if self._start is None:
            raise RuntimeError("Timer has not been started yet.")
        end_time = self._end if self._end is not None else time.time()
        elapsed = end_time - self._start
        print(f"[Timer] 경과 시간: {elapsed:.6f}초")
        return elapsed
