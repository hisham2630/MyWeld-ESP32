#!/usr/bin/env python3
"""Generate boot/BLE voice prompt WAVs via edge-tts and embed as C PCM arrays."""
import asyncio
import subprocess
import sys
from pathlib import Path

import edge_tts

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"
SRC = ROOT / "src"
EMBED = ROOT / "scripts" / "embed_wav.py"

VOICE = "en-US-JennyNeural"
RATE = "+0%"
PROMPTS = {
    "voice_welcome": "Welcome",
    "voice_ready_to_pair": "Ready to pair",
    "voice_pairing": "Pairing",
    "voice_connected": "Device has been connected",
}


async def synthesize(text: str, mp3_path: Path) -> None:
    communicate = edge_tts.Communicate(text, VOICE, rate=RATE)
    await communicate.save(str(mp3_path))


def mp3_to_wav(mp3_path: Path, wav_path: Path) -> None:
    subprocess.run(
        [
            "ffmpeg",
            "-y",
            "-i",
            str(mp3_path),
            "-ar",
            "44100",
            "-ac",
            "1",
            "-sample_fmt",
            "s16",
            str(wav_path),
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def embed(wav_path: Path, symbol: str) -> None:
    out_c = SRC / f"{symbol}.c"
    out_h = SRC / f"{symbol}.h"
    subprocess.run(
        [sys.executable, str(EMBED), str(wav_path), str(out_c), str(out_h)],
        check=True,
    )


async def main() -> None:
    ASSETS.mkdir(parents=True, exist_ok=True)
    for symbol, text in PROMPTS.items():
        mp3 = ASSETS / f"{symbol}.mp3"
        wav = ASSETS / f"{symbol}.wav"
        print(f"synthesizing {symbol}: {text!r}")
        await synthesize(text, mp3)
        mp3_to_wav(mp3, wav)
        mp3.unlink(missing_ok=True)
        embed(wav, symbol)
    print("done")


if __name__ == "__main__":
    asyncio.run(main())
