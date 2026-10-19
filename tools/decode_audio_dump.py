#!/usr/bin/env python3
# -*- coding: utf-8 -*-
r"""
decode_audio_dump.py — 把固件串口 dump 的 base64 音频还原成可播放的 .wav

配合 main/app_main.cpp 里的 DumpPcmBase64()：REAL 模式第 10 秒会把 1 秒
单声道 16kHz/16bit PCM 以 base64 打到串口，格式：

    I (12345) AppMain: ---AUDIO-DUMP-BEGIN--- REAL10s 16000 samples 16000Hz 16bit mono
    I (12345) AppMain: UklGRiQAAABXQVZFZm10IBAAAAABAAEAQB8AAEAfAAABAAgAZGF0YQAAAAA...
    I (12345) AppMain: ...（每行 76 个 base64 字符）
    I (12345) AppMain: ---AUDIO-DUMP-END---

用法（三种任选）：

  1) 把 monitor 的完整日志存成文件再跑：
       python tools\decode_audio_dump.py monitor.log

  2) 直接把日志粘贴到标准输入（Windows 下粘贴完按 Ctrl+Z 再回车）：
       python tools\decode_audio_dump.py

  3) 日志里有多段 dump 会自动切成多个文件：snore_dump_1.wav / _2.wav ...

输出：与日志同目录下的 snore_dump*.wav，可直接双击播放。
"""
from __future__ import annotations

import base64
import os
import re
import struct
import sys
import wave

BEGIN_RE = re.compile(r"---AUDIO-DUMP-BEGIN---\s*(\S+)\s+(\d+)\s+samples\s+(\d+)\s*Hz")
END_MARK = "---AUDIO-DUMP-END---"

# ESP-IDF 日志前缀： I (12345) AppMain: <msg>
LOG_PREFIX_RE = re.compile(r"^[IWEDV]\s*\(\s*\d+\s*\)\s*[^:]{0,40}:\s?")

B64_CHARS = set(
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/="
)


def strip_prefix(line: str) -> str:
    """去掉 ESP-IDF 日志前缀，返回真正的消息体。"""
    line = line.rstrip("\r\n")
    m = LOG_PREFIX_RE.match(line)
    if m:
        return line[m.end():]
    return line.strip()


def collect_dumps(text: str) -> list[dict]:
    """从日志文本里抽取所有 dump 段落。"""
    dumps: list[dict] = []
    cur: dict | None = None
    for raw in text.splitlines():
        msg = strip_prefix(raw)
        if not msg:
            continue

        if "---AUDIO-DUMP-BEGIN---" in msg:
            m = BEGIN_RE.search(msg)
            cur = {
                "tag": m.group(1) if m else "dump",
                "samples": int(m.group(2)) if m else 0,
                "rate": int(m.group(3)) if m else 16000,
                "b64": [],
            }
            continue

        if END_MARK in msg:
            if cur is not None:
                dumps.append(cur)
                cur = None
            continue

        if cur is not None:
            # 只保留纯 base64 字符（防止混进别的日志）
            chunk = "".join(ch for ch in msg if ch in B64_CHARS)
            if chunk:
                cur["b64"].append(chunk)
    return dumps


def write_wav(path: str, pcm: bytes, rate: int) -> None:
    with wave.open(path, "wb") as w:
        w.setnchannels(1)
        w.setsampwidth(2)          # 16-bit
        w.setframerate(rate)
        w.writeframes(pcm)


def pcm_stats(pcm: bytes) -> dict:
    n = len(pcm) // 2
    if n == 0:
        return {"n": 0}
    vals = struct.unpack("<%dh" % n, pcm[: n * 2])
    total = sum(vals)
    sq = sum(v * v for v in vals)
    peak = max(abs(v) for v in vals)
    return {
        "n": n,
        "dc": total / n,
        "rms": (sq / n) ** 0.5,
        "peak": peak,
    }


def main() -> int:
    # 中文 Windows 控制台默认 GBK：遇到无法编码的字符不要让脚本崩掉
    try:
        sys.stdout.reconfigure(errors="replace")   # type: ignore[attr-defined]
        sys.stderr.reconfigure(errors="replace")   # type: ignore[attr-defined]
    except Exception:  # noqa: BLE001
        pass

    if len(sys.argv) > 1:
        src_path = sys.argv[1]
        if not os.path.isfile(src_path):
            print(f"找不到文件: {src_path}", file=sys.stderr)
            return 1
        with open(src_path, "r", encoding="utf-8", errors="replace") as f:
            text = f.read()
        out_dir = os.path.dirname(os.path.abspath(src_path)) or "."
    else:
        print("从标准输入读取日志（粘贴完按 Ctrl+Z 再回车）...", file=sys.stderr)
        text = sys.stdin.read()
        out_dir = "."

    dumps = collect_dumps(text)
    if not dumps:
        print("没有找到 ---AUDIO-DUMP-BEGIN/END--- 段落。", file=sys.stderr)
        print("请确认：1) 固件已烧入带 DumpPcmBase64 的版本；"
              "2) 鼾声开关已切到 REAL；3) 已跑过 10 秒以上。", file=sys.stderr)
        return 2

    print(f"找到 {len(dumps)} 段音频 dump\n")
    for idx, d in enumerate(dumps, 1):
        b64 = "".join(d["b64"])
        # 补齐 padding
        pad = (-len(b64)) % 4
        try:
            pcm = base64.b64decode(b64 + "=" * pad, validate=False)
        except Exception as e:  # noqa: BLE001
            print(f"[{idx}] base64 解码失败: {e}", file=sys.stderr)
            continue

        rate = d["rate"] or 16000
        out = os.path.join(out_dir, f"snore_dump_{idx}.wav")
        write_wav(out, pcm, rate)

        st = pcm_stats(pcm)
        print(f"[{idx}] tag={d['tag']}  {st['n']} samples  "
              f"{st['n'] / rate:.2f}s @ {rate}Hz")
        print(f"     dc={st['dc']:.1f}  rms={st['rms']:.1f}  peak={st['peak']}  "
              f"-> {out}")
        # 简单判读（注：不用 emoji —— 中文 Windows 的控制台是 GBK，
        #         遇到 U+26A0 之类字符会直接 UnicodeEncodeError 崩掉）
        if st["rms"] < 5:
            print("     [!] rms 极小 -> 基本是静音，麦克风可能没在采集")
        elif abs(st["dc"]) > 200:
            print(f"     [!] dc 偏置偏大({st['dc']:.0f}) -> 建议在固件里做去直流")
        if st["peak"] >= 32700:
            print("     [!] peak 接近满量程 -> 可能削顶失真")
        print()

    print("完成。双击 .wav 即可试听；也可以直接拖进 Audacity 看频谱。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
