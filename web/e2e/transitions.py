# /// script
# requires-python = ">=3.12"
# dependencies = ["playwright"]
# ///
"""Focused browser checks of real worker transitions and experiment pause/cancellation.

Run `make e2e-smoke`; CHROMIUM optionally selects a local executable.
"""

import asyncio
import os

from playwright.async_api import Page, Route, async_playwright

URL = os.environ.get("APP_URL", "http://localhost:5173/")

RECORD = """() => {
  window.commands = []; window.responses = []; window.workers = []; window.mainGeneration = 0;
  const NativeWorker = window.Worker;
  window.Worker = class extends NativeWorker {
    constructor(...args) {
      super(...args);
      this.index = window.workers.length; window.workers.push(this);
      this.addEventListener('message', ({data}) => {
        if (this.index !== 0) return;
        if (data.type === 'built' && data.ok) window.mainGeneration = data.generation;
        if (data.type === 'observed') window.responses.push({id: data.id, t: data.data[0]});
      });
    }
    postMessage(message, ...args) {
      if (this.index === 0) window.commands.push(message);
      return super.postMessage(message, ...args);
    }
  };
}"""

CLIENT = """async () => {
  const {SimClient} = await import('/src/sim/client.ts');
  const {encode, PRESETS} = await import('/src/model/description.ts');
  const client = await SimClient.start();
  const assert = (ok, message) => { if (!ok) throw new Error(message); };
  const first = await client.build(encode(PRESETS['Two-link arm']()));
  assert(first.ok, 'initial build failed'); client.install(first);
  const generation = client.generation;
  const before = client.snap.m('M', 0, 0);
  client.send({type:'param', i:0.5, v:99});
  await client.refresh();
  assert(client.snap.m('M', 0, 0) === before, 'fractional parameter index was coerced');
  client.flag('MotorSpeed', 1);
  client.send({type:'state', i:0, q:0, qd:5});
  client.send({type:'input', i:0, v:100});
  client.send({type:'controller', slot:1, kind:0});
  client.send({type:'advance', steps:1});
  assert(await client.refresh(), 'observation acknowledgment failed');
  assert(client.snap.v('U') === 0 && client.snap.slots[1].v('U') === 0, 'shadow lost tuning');
  client.send({type:'target', i:0, v:0.7});
  client.send({type:'controller', slot:1, kind:0});
  await client.refresh();
  assert(Math.abs(client.snap.slots[0].v('ERR_Q') - client.snap.slots[1].v('ERR_Q')) < 1e-9,
    'shadow target cache differs');
  const discarded = await client.build(encode(PRESETS['Cart-pole']()), () => false);
  assert(!discarded.ok, 'obsolete candidate committed');
  const invalid = encode(PRESETS['Cart-pole']()); invalid[14] = -1;
  assert(!(await client.build(invalid)).ok, 'invalid build accepted');
  await client.refresh();
  assert(client.generation === generation && client.snap.nu === 2, 'failed build replaced old mechanism');
  const second = await client.build(encode(PRESETS['Cart-pole']()));
  assert(second.ok, 'replacement build failed'); client.install(second);
  assert(client.generation !== generation && client.snap.nu === 1, 'new metadata/snapshot not installed');
  window.flags = client.layout.flag;
  return client.stats.shared;
}"""


async def actual_time(page: Page) -> float:
    """Read an explicit worker observation after prior commands, including while paused."""
    await page.evaluate("""() => window.workers[0].postMessage({type:'observe', id:-99})""")
    await page.wait_for_function("window.responses.some(r => r.id === -99)")
    return await page.evaluate(
        "window.responses.splice(window.responses.findIndex(r => r.id === -99), 1)[0].t"
    )


async def experiment(page: Page) -> None:
    generation = await page.evaluate("window.mainGeneration")
    await page.locator(".preset-card").filter(has=page.get_by_text("Two-link arm", exact=True)).click()
    await page.wait_for_function("old => window.mainGeneration > old", arg=generation)
    await page.locator("[data-stage=simulate]").click()
    await page.locator(".page-picker").click()
    await page.locator(".ctrl-menu-item").filter(has=page.get_by_text("Impedance", exact=True)).click()
    await page.locator("#speed").select_option("2")
    item = page.locator(".lab-item").filter(has=page.get_by_text("The spring law", exact=True))
    await page.evaluate("window.commands.length = 0")
    await item.locator("button").click()
    await page.wait_for_function(
        "window.commands.some(m => m.type === 'flag' && m.id === window.flags.PushX)"
    )
    start = await actual_time(page)
    await page.locator("#run").click()  # pause during the push
    await page.evaluate("window.commands.length = 0")
    # Deliberately outwait the old wall-clock release while simulation time stays paused.
    await page.wait_for_timeout(4200)
    assert not await page.evaluate("""window.commands.some(m =>
        m.type === 'flag' && m.id === window.flags.PushOn && m.v === 0)"""), "push released while paused"
    await page.locator("#run").click()
    await page.wait_for_function(
        """window.commands.some(m =>
        m.type === 'flag' && m.id === window.flags.PushOn && m.v === 0)""",
        timeout=10_000,
    )
    end = await actual_time(page)
    assert 3.8 <= end - start <= 4.4, f"push duration used wall time: {end - start} simulated seconds"

    await page.evaluate("window.commands.length = 0")
    await item.locator("button").click()
    await page.wait_for_function(
        "window.commands.some(m => m.type === 'flag' && m.id === window.flags.PushX)"
    )
    await page.locator("[data-stage=build]").click()
    await page.evaluate("window.commands.length = 0")
    await page.wait_for_timeout(250)
    assert not await page.evaluate("window.commands.some(m => m.type === 'flag')"), (
        "obsolete experiment mutated Build stage"
    )


async def without_isolation(route: Route) -> None:
    response = await route.fetch()
    headers = {
        k: v
        for k, v in response.headers.items()
        if k.lower()
        not in {
            "cross-origin-opener-policy",
            "cross-origin-embedder-policy",
        }
    }
    await route.fulfill(response=response, headers=headers)


async def main() -> None:
    async with async_playwright() as p:
        browser = await p.chromium.launch(executable_path=os.environ.get("CHROMIUM") or None)
        for isolated in [True, False]:
            page = await browser.new_page(viewport={"width": 1440, "height": 900})
            errors: list[str] = []
            page.on("pageerror", lambda error, errors=errors: errors.append(str(error)))
            await page.add_init_script(f"({RECORD})()")
            if not isolated:
                await page.route(URL, without_isolation)
            await page.goto(URL)
            await page.wait_for_function(
                "window.mainGeneration > 0 && document.body.dataset.stage === 'build'"
            )
            assert await page.evaluate(CLIENT) == isolated, "wrong snapshot transport"
            await experiment(page)
            assert not errors, errors
            await page.close()
            transport = "shared memory" if isolated else "message fallback"
            print(f"{transport}: transitions, pause and cancellation passed", flush=True)
        await browser.close()


if __name__ == "__main__":
    asyncio.run(main())
