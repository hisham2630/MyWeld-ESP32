#!/usr/bin/env python3
"""Embed mono 16-bit PCM WAV as a C int16_t array."""
import struct
import sys
from pathlib import Path


def main() -> None:
    if len(sys.argv) != 4:
        print("usage: embed_wav.py input.wav output.c output.h")
        sys.exit(1)

    wav_path = Path(sys.argv[1])
    out_c = Path(sys.argv[2])
    out_h = Path(sys.argv[3])
    wav = wav_path.read_bytes()
    if wav[0:4] != b"RIFF":
        raise SystemExit(f"{wav_path} is not a RIFF WAV file")

    pos = 12
    fmt = None
    data_off = data_sz = None
    while pos < len(wav) - 8:
        cid = wav[pos : pos + 4]
        sz = struct.unpack_from("<I", wav, pos + 4)[0]
        pos += 8
        chunk = wav[pos : pos + sz]
        if cid == b"fmt ":
            fmt = struct.unpack("<HHIIHH", chunk[:16])
        elif cid == b"data":
            data_off, data_sz = pos, sz
        pos += sz + (sz & 1)

    if fmt is None or data_off is None:
        raise SystemExit("invalid WAV: missing fmt or data chunk")

    channels, rate, _, _, bits = fmt[1], fmt[2], fmt[3], fmt[4], fmt[5]
    if rate != 44100 or channels != 1 or bits != 16:
        raise SystemExit(f"expected 44100 Hz mono s16, got {rate} Hz ch={channels} bits={bits}")

    pcm = wav[data_off : data_off + data_sz]
    samples = len(pcm) // 2
    symbol = out_h.stem

    out_h.write_text(
        f"#ifndef {symbol.upper()}_H\n"
        f"#define {symbol.upper()}_H\n"
        "#include <stddef.h>\n"
        "#include <stdint.h>\n"
        f"extern const int16_t {symbol}_pcm[];\n"
        f"extern const size_t {symbol}_pcm_samples;\n"
        "#endif\n",
        encoding="utf-8",
    )

    lines = [
        f'#include "{out_h.name}"',
        "",
        f"const size_t {symbol}_pcm_samples = {samples};",
        f"const int16_t {symbol}_pcm[] = {{",
    ]
    for i in range(0, len(pcm), 2):
        val = struct.unpack_from("<h", pcm, i)[0]
        if (i // 2) % 12 == 0:
            lines.append("   ")
        lines[-1] += f" {val},"
    lines.append("};")
    out_c.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"embedded {samples} samples ({len(pcm)} bytes) -> {out_c.name}")


if __name__ == "__main__":
    main()
