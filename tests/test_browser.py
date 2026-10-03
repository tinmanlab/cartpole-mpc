from pathlib import Path
import asyncio, json
from playwright.async_api import async_playwright

ROOT = Path(__file__).resolve().parents[1]
HTML = (ROOT / "index.html").read_text(encoding="utf-8")

async def main():
    async with async_playwright() as p:
        browser = await p.chromium.launch(headless=True)
        page = await browser.new_page(viewport={"width": 1440, "height": 1000})
        mock = """<script>(()=>{const tools={};const mc={async registerTool(tool){tools[tool.name]=tool;}};Object.defineProperty(Document.prototype,'modelContext',{configurable:true,get(){return mc;}});window.__webmcpTools=tools;})();</script>"""
        await page.set_content(HTML.replace("<head>", "<head>"+mock, 1), wait_until="load")
        await page.wait_for_function("document.querySelector('#webmcpBadge').textContent.includes('registered')")
        assert await page.locator("#topicNav button").count() == 16
        await page.evaluate("window.__webmcpTools.cartpole_set_topic.execute({topic:'full_nmpc'})")
        await page.click("#useTopic")
        state = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert state["controller"] == "full_nmpc"
        assert state["solver"]["horizon"] == 30
        await page.evaluate("window.__webmcpTools.cartpole_set_topic.execute({topic:'centroidal_mpc'})")
        await page.click("#useTopic")
        state2 = json.loads(await page.evaluate("window.__webmcpTools.cartpole_run_steps.execute({steps:8})"))
        assert state2["controller"] == "centroidal_mpc"
        assert state2["solver"]["horizon"] == 32
        assert await page.evaluate("document.documentElement.scrollWidth-window.innerWidth") <= 1
        await browser.close()
        print("browser smoke: PASS")

asyncio.run(main())
