#!/usr/bin/env python3
"""Tiny always-reloading window for the RoboCasa preview frame.

The simulation keeps rendering offscreen (so camera tools and the vision
detector keep working) and writes `preview.png` every couple of steps; this
window just reloads that file. Run it beside the agent, not inside it.
"""

from __future__ import annotations

import os
import sys
import time
import tkinter as tk
from pathlib import Path

from PIL import Image, ImageTk

HERE = Path(__file__).resolve().parent
PREVIEW = HERE / "preview.png"
REFRESH_MS = 150


def main() -> None:
    root = tk.Tk()
    root.title("RoboCasa 实时预览")
    root.configure(bg="#1f2a37")

    image_label = tk.Label(root, bg="#1f2a37")
    image_label.pack(padx=6, pady=6)
    status = tk.Label(root, text="等待画面…", fg="#9fb3c8", bg="#1f2a37", font=("TkDefaultFont", 10))
    status.pack(pady=(0, 6))

    state = {"photo": None, "mtime": None}

    def tick() -> None:
        try:
            stamp = PREVIEW.stat().st_mtime
            if stamp != state["mtime"]:
                image = Image.open(PREVIEW).convert("RGB")
                state["photo"] = ImageTk.PhotoImage(image)
                state["mtime"] = stamp
                image_label.configure(image=state["photo"])
            age = time.time() - (state["mtime"] or time.time())
            status.configure(text=f"画面年龄 {age:0.1f}s   |   文件 {PREVIEW.name}")
        except FileNotFoundError:
            status.configure(text=f"等待 {PREVIEW.name} …")
        except Exception as exc:  # noqa: BLE001 - keep the window alive
            status.configure(text=f"读取失败: {exc}")
        root.after(REFRESH_MS, tick)

    tick()
    root.mainloop()


if __name__ == "__main__":
    sys.exit(main())
