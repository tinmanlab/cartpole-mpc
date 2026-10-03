from __future__ import annotations
import asyncio, os, shutil, subprocess, tempfile, time, urllib.request
from pathlib import Path
from playwright.async_api import async_playwright

ROOT = Path(__file__).resolve().parents[1]
MEDIA = ROOT / "media"
PORT = 8775
FFMPEG = Path(shutil.which("ffmpeg") or (Path.home() / ".cache/ms-playwright/ffmpeg-1011/ffmpeg-linux"))

def wait_server():
    url = f"http://127.0.0.1:{PORT}/index.html"
    for _ in range(60):
        try:
            with urllib.request.urlopen(url, timeout=.5) as r:
                if r.status == 200:
                    return url
        except Exception:
            time.sleep(.1)
    raise RuntimeError("local server did not start")

async def capture(url: str, tmp: Path) -> Path:
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        context = await browser.new_context(
            viewport={"width": 1280, "height": 800},
            record_video_dir=str(tmp),
            record_video_size={"width": 1280, "height": 800},
        )
        page = await context.new_page()
        await page.goto(url + "#full_nmpc", wait_until="load")
        await page.select_option("#controller", "full_nmpc")
        await page.select_option("#observer", "adaptive")
        await page.select_option("#scenario", "sensor")
        await page.wait_for_timeout(1200)
        await page.click("#pushR")
        await page.wait_for_timeout(1800)
        await page.click("#pushL")
        await page.wait_for_timeout(1800)
        await page.select_option("#goal", "0.5")
        await page.wait_for_timeout(2600)
        await page.select_option("#goal", "0")
        await page.wait_for_timeout(1500)
        video = page.video
        await context.close()
        src = Path(await video.path())
        await browser.close()
        return src

def convert(src: Path):
    from PIL import Image
    MEDIA.mkdir(parents=True, exist_ok=True)
    webm = MEDIA / "cartpole-mpc-demo.webm"
    gif = MEDIA / "cartpole-mpc-demo.gif"
    shutil.copy2(src, webm)
    with tempfile.TemporaryDirectory() as frames_dir:
        pattern = str(Path(frames_dir) / "frame-%04d.png")
        subprocess.run([
            str(FFMPEG), "-y", "-i", str(src),
            "-vf", "scale=960:-1", pattern
        ], check=True)
        all_paths = sorted(Path(frames_dir).glob("frame-*.png"))
        # Playwright records at 25 fps. Keep roughly 8.3 fps for a compact README loop.
        frames = [Image.open(p).convert("P", palette=Image.Palette.ADAPTIVE, colors=128)
                  for p in all_paths[::3]]
        if not frames:
            raise RuntimeError("no demo frames generated")
        frames[0].save(gif, save_all=True, append_images=frames[1:], duration=120,
                       loop=0, optimize=True, disposal=2)
        for frame in frames:
            frame.close()
    print(f"{gif}: {gif.stat().st_size:,} bytes")
    print(f"{webm}: {webm.stat().st_size:,} bytes")

async def main():
    server = subprocess.Popen(
        ["python3", "serve.py", "--port", str(PORT)],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        url = wait_server()
        with tempfile.TemporaryDirectory() as td:
            src = await capture(url, Path(td))
            convert(src)
    finally:
        server.terminate()
        try:
            server.wait(timeout=3)
        except subprocess.TimeoutExpired:
            server.kill()

if __name__ == "__main__":
    asyncio.run(main())
