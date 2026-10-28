"""Minimal client for the worker's newline-delimited JSON-RPC protocol.

The worker is a long-lived Python process the plugin normally owns; scripts that
want to run experiments headlessly speak the same protocol through this client
instead of reimplementing the simulation setup.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORKER = os.path.join(HERE, "worker.py")


class Worker:
    """One worker process, one simulation session."""

    def __init__(self, python: str | None = None):
        python = python or sys.executable
        self.proc = subprocess.Popen(
            [python, WORKER],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            bufsize=1,
            cwd=HERE,
        )
        self.next_id = 1

    def call(self, method: str, params: dict | None = None, timeout: float = 600.0) -> dict:
        message = {"id": self.next_id, "method": method, "params": params or {}}
        self.next_id += 1
        assert self.proc.stdin and self.proc.stdout
        self.proc.stdin.write(json.dumps(message) + "\n")
        self.proc.stdin.flush()
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError(f"worker exited while waiting for {method}")
            reply = json.loads(line)
            if reply.get("id") != message["id"]:
                continue
            if not reply.get("ok"):
                raise RuntimeError(f"{method} failed: {reply.get('error')}")
            return reply.get("result") or {}
        raise TimeoutError(f"{method} did not answer within {timeout}s")

    def close(self) -> None:
        try:
            self.call("close", {}, timeout=30)
        except Exception:
            pass
        if self.proc.stdin:
            self.proc.stdin.close()
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
