# /// script
# requires-python = ">=3.12"
# dependencies = ["playwright"]
# ///
"""End-to-end crawl of the running app.

Drives every stage, preset, side tab, controller page and role, shadow, experiment, scope operation, toggle
and builder edit; fails on any page or console error and on functional mismatches (dock, notices, plots,
comparison table, NaN on screen).

Run with `make e2e` (starts a dev server). `CHROMIUM` selects the browser executable.
"""

import asyncio
import json
import os
import sys

from playwright.async_api import Page, async_playwright

URL = os.environ.get("APP_URL", "http://localhost:5173/")


class Crawl:
    def __init__(self, page: Page) -> None:
        self.page = page
        self.where = "start"
        self.errors: list[str] = []
        page.on("pageerror", lambda e: self.fail(f"pageerror: {e}"))
        page.on("console", lambda m: self.fail(f"console.error: {m.text}") if m.type == "error" else None)

    def fail(self, message: str) -> None:
        self.errors.append(f"[{self.where}] {message}")

    async def js(self, code: str) -> object:
        return await self.page.evaluate(code)

    async def click(self, selector: str, text: str | None = None, *, last: bool = False) -> bool:
        """Click the first (or last) element matching `selector` (and exact `text`); False if none."""
        match = "" if text is None else f".filter(e => e.textContent === {json.dumps(text)})"
        pick = ".pop()" if last else "[0]"
        code = f"(() => {{ const e = [...document.querySelectorAll({json.dumps(selector)})]{match}{pick};"
        return bool(await self.js(code + " if (!e) return false; e.click(); return true; })()"))

    async def texts(self, selector: str) -> list[str]:
        return await self.js(
            f"[...document.querySelectorAll({json.dumps(selector)})].map(e => e.textContent)"
        )

    async def text(self, selector: str) -> str | None:
        return await self.js(f"document.querySelector({json.dumps(selector)})?.textContent ?? null")

    async def wait(self, ms: int) -> None:
        await self.page.wait_for_timeout(ms)

    # ------------------------------------------------------------------ areas

    async def build(self, preset: str) -> None:
        self.where = f"{preset}/build"
        await self.click(".preset-card", preset)
        await self.wait(400)
        for label in ["+ revolute", "+ prismatic", "+ fixed"]:
            self.where = f"{preset}/build {label}"
            await self.click("button", label)
            await self.wait(250)
        self.where = f"{preset}/build remove, undo, redo"
        await self.click(".tree-item button", "×", last=True)
        await self.wait(250)
        for label in ["↶", "↶", "↶", "↶", "↷", "↶"]:
            await self.click(".head-tools button", label)
            await self.wait(150)
        await self.wait(300)

    async def palette(self, preset: str) -> None:
        """Drag a spring from the palette onto a port, click the ground for its other end, step a value in
        the inspector, then drop a bar on the ground and close the inspector."""
        self.where = f"{preset}/palette"
        mouse = self.page.mouse
        box = await self.page.locator("#viewport").bounding_box()
        if not box:
            return
        for label, second in [("Spring", True), ("Bar", False)]:
            item = await self.page.locator(".palette-item", has_text=label).first.bounding_box()
            if not item:
                self.fail(f"no {label} in the palette")
                return
            await mouse.move(item["x"] + item["width"] / 2, item["y"] + item["height"] / 2)
            await mouse.down()
            await mouse.move(box["x"] + 500, box["y"] + 260, steps=4)
            ports = await self.js("[...document.querySelectorAll('.gizmos circle.port.target')].length")
            if not ports:
                self.fail("no ports light up while placing")
            await mouse.up()
            if second:
                await mouse.move(box["x"] + 560, box["y"] + 160, steps=3)
                await mouse.down()
                await mouse.up()
            await self.wait(400)
        if not await self.js("!!document.querySelector('.inspector:not([hidden])')"):
            self.fail("the inspector did not open on the placed part")
        await self.click(".inspector .stepper-arrow", last=True)
        await self.wait(200)
        await self.page.keyboard.press("Escape")
        await self.wait(200)
        if await self.js("!!document.querySelector('.inspector:not([hidden])')"):
            self.fail("Escape did not close the inspector")

    async def hierarchy(self, preset: str) -> None:
        self.where = f"{preset}/goals hierarchy"
        await self.click(".side-head .segmented button", "Goals")
        await self.wait(200)
        for _ in range(2):  # switch orientation, then elbow on (the tab re-renders after each)
            await self.click(".level .segmented button", "on")
            await self.wait(300)
        await self.click(".card-actions button", "here")
        await self.click(".param-card .segmented button", "joint 3")
        await self.wait(300)
        await self.page.keyboard.down("Alt")
        await self.page.mouse.move(420, 320)
        await self.page.mouse.down()
        await self.page.mouse.move(430, 330)
        await self.page.mouse.up()
        await self.page.keyboard.up("Alt")
        await self.wait(400)
        for _ in range(2):
            await self.click(".level .segmented button", "off")
            await self.wait(300)
        await self.click(".side-head .segmented button", "Control")
        await self.wait(300)

    async def tabs(self, preset: str) -> None:
        for tab in ["Goals", "Equations", "Model", "Control"]:
            self.where = f"{preset}/tab {tab}"
            await self.click(".side-head .segmented button", tab)
            await self.wait(300)

    async def open_page(self, name: str) -> None:
        await self.click(".page-picker")
        await self.js(
            "[...document.querySelectorAll('.ctrl-menu-item')]"
            f".find(b => b.querySelector('.name').textContent === {json.dumps(name)})?.click()"
        )
        await self.wait(150)

    async def controllers(self, preset: str) -> list[str]:
        await self.click(".page-picker")
        names = await self.texts(".ctrl-menu-item .name")
        await self.page.keyboard.press("Escape")
        for name in names:
            self.where = f"{preset}/page {name}"
            await self.open_page(name)
            if await self.click(".role-actions button", "Make primary"):
                await self.wait(250)
                shown = await self.text(".dock-primary span:nth-child(2)")
                if shown != name:
                    self.fail(f"dock shows {shown!r} after making {name!r} primary")
            await self.click(".card-actions button")
            await self.wait(100)
        return names

    async def shadows(self, preset: str) -> None:
        for k in range(4):
            self.where = f"{preset}/shadow {k}"
            await self.click(".dock-add")
            await self.js(
                "[...document.querySelectorAll('.ctrl-menu-item')].find(b => !b.disabled"
                " && !/primary|shadow|remove/.test(b.textContent))?.click()"
            )
            await self.wait(300)
        rows = len(await self.texts(".compare-row:not(.head)"))
        pills = len(await self.texts(".dock .pill"))
        if rows != pills + 1:
            self.fail(f"comparison table has {rows} rows for {pills} shadows + primary")
        await self.js(
            "[...document.querySelectorAll('.compare-actions button')]"
            ".find(b => b.textContent.startsWith('Restart'))?.click()"
        )
        await self.wait(300)

    async def experiments(self, preset: str, names: list[str]) -> None:
        for name in names:
            await self.open_page(name)
            for title in await self.texts(".lab-item b"):
                self.where = f"{preset}/{name}/experiment {title}"
                await self.js(
                    "[...document.querySelectorAll('.lab-item')]"
                    f".find(i => i.querySelector('b').textContent === {json.dumps(title)})"
                    "?.querySelector('button').click()"
                )
                await self.wait(700)
                opened = await self.text(".lab-item.open b")
                plots = await self.text(".scope-presets button.on")
                if opened != title or plots != title:
                    self.fail(f"notice {opened!r}, plots {plots!r}")
                values = await self.texts(".scope-chip .v, .compare-row .num")
                if any("NaN" in v or "undefined" in v for v in values):
                    self.fail("NaN values on screen")
            await self.click(".lab-row button", "Watch")

    async def scope(self, preset: str) -> None:
        for label in await self.texts(".scope-presets button"):
            self.where = f"{preset}/scope {label}"
            await self.click(".scope-presets button", label)
            await self.wait(200)
        self.where = f"{preset}/scope edit"
        await self.click(".scope-add")
        await self.click(".picker-item", last=True)
        await self.click(".scope-bar button", "+ plot")
        await self.wait(200)
        await self.click(".scope-drop", last=True)
        await self.click(".scope-chip .x", last=True)
        await self.wait(200)

    async def chrome(self, preset: str) -> None:
        for toggle in [
            "Frames",
            "Labels",
            "Dimensions",
            "Velocities",
            "Forces",
            "Free-body diagram",
            "Torque bars",
            "Energy gauge",
            "Trail",
            "Shadows",
            "Coriolis forces",
            "Keyboard legend",
        ]:
            self.where = f"{preset}/view {toggle}"
            await self.view_toggle(toggle)
            await self.view_toggle(toggle)
        self.where = f"{preset}/rewind"
        await self.page.keyboard.press("p")
        await self.wait(300)
        if not await self.js("!document.querySelector('.timeline').hidden"):
            self.fail("no timeline while paused")
        await self.js(
            "(() => { const s = document.querySelector('.timeline input');"
            " s.value = String(Math.floor(+s.max / 2)); s.dispatchEvent(new Event('input')); })()"
        )
        await self.click(".timeline button", "◀")
        await self.click(".timeline button", "▶")
        await self.click(".timeline button", "Resume here")
        await self.wait(300)
        self.where = f"{preset}/model friction"
        await self.click(".side-head .segmented button", "Model")
        await self.click(".segmented button", "none")
        await self.click(".segmented button", "as built")
        self.where = f"{preset}/keys"
        for key in ["Space", "r", "p", "p", "1", "2"]:
            await self.page.keyboard.press(key)
            await self.wait(80)
        self.where = f"{preset}/drag and push"
        mouse = self.page.mouse
        await mouse.move(400, 300)
        await mouse.down()
        await mouse.move(420, 320)
        await mouse.up()
        await self.page.keyboard.down("Shift")
        await mouse.move(400, 300)
        await mouse.down()
        await mouse.move(450, 330)
        await self.wait(300)
        await mouse.up()
        await self.page.keyboard.up("Shift")
        await self.wait(500)

    async def figure(self, preset: str) -> None:
        self.where = f"{preset}/figure"
        drawn = await self.js("document.querySelectorAll('.schematic canvas').length")
        if drawn != 2:
            self.fail(f"the figure has {drawn} canvases")
        for toggle in ["Frames", "Labels"]:
            await self.view_toggle(toggle)
            await self.wait(150)
            await self.view_toggle(toggle)
        for label in ["+", "−", "Fit"]:
            await self.click("#toggles button", label)
        await self.page.mouse.move(400, 350)
        await self.page.mouse.wheel(0, -300)
        await self.click("#toggles button", "Fit")
        await self.click("#toggles button", "Export ▾")
        async with self.page.expect_download():
            if not await self.js(
                "(() => { const b = [...document.querySelectorAll('.menu-item')]"
                ".find(e => e.textContent.startsWith('SVG')); b?.click(); return !!b; })()"
            ):
                self.fail("no SVG entry in the Export menu")

    async def view_toggle(self, label: str) -> None:
        """Flip one entry of the View menu (the menu closes again afterwards)."""
        await self.click("#toggles button", "View ▾")
        found = await self.js(
            "(() => { const b = [...document.querySelectorAll('.menu-check')]"
            f".find(e => e.textContent.startsWith({json.dumps(label)})); b?.click(); return !!b; }})()"
        )
        if not found:
            self.fail(f"no {label!r} in the View menu")
        await self.page.keyboard.press("Escape")

    async def preset(self, preset: str) -> None:
        await self.build(preset)
        await self.palette(preset)
        self.where = f"{preset}/derive"
        await self.click("[data-stage=derive]")
        await self.wait(600)
        await self.click(".eq-toc button", last=True)
        focus = self.page.locator("[data-focus]").first
        if await focus.count():
            await focus.scroll_into_view_if_needed()
            await focus.hover()
            await self.wait(150)
        self.where = f"{preset}/simulate"
        await self.click("[data-stage=simulate]")
        await self.wait(600)
        await self.tabs(preset)
        await self.hierarchy(preset)
        names = await self.controllers(preset)
        await self.shadows(preset)
        await self.experiments(preset, names)
        await self.scope(preset)
        await self.chrome(preset)
        await self.figure(preset)
        stats = (await self.text("#stats") or "").strip()
        print(f"{preset}: ok · {stats[:60]}")
        await self.click("[data-stage=build]")
        await self.wait(300)


async def main() -> int:
    async with async_playwright() as p:
        browser = await p.chromium.launch(executable_path=os.environ.get("CHROMIUM") or None)
        page = await browser.new_page(viewport={"width": 1440, "height": 900}, accept_downloads=True)
        crawl = Crawl(page)
        await page.goto(URL)
        await crawl.wait(2500)
        for preset in await crawl.texts(".preset-card span"):
            await crawl.preset(preset)
        await browser.close()
    unique = list(dict.fromkeys(crawl.errors))
    print(f"\n{len(unique)} errors")
    for e in unique:
        print(e)
    return 1 if unique else 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
