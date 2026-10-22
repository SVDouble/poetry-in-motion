# /// script
# requires-python = ">=3.12"
# dependencies = ["playwright"]
# ///
"""Measure actual WASM worker ticks in Chromium, with both snapshot transports.

Run `make e2e-profile`. JSON results go to stdout; no main UI worker runs in this fixture.
Timings include integration, publication, transport, checkpoint retention and periodic isolated timing.
They exclude browser rendering and main-thread DOM work.
"""

import asyncio
import json
import os

from playwright.async_api import async_playwright
from transitions import URL, without_isolation

PROFILE = """async () => {
  const {SimClient} = await import('/src/sim/client.ts');
  const {encode, PRESETS} = await import('/src/model/description.ts');
  const client = await SimClient.start();
  const cases = [];
  for (const name of ['Two-link arm', 'Seven-joint arm', 'Four-bar linkage']) {
    const mech = PRESETS[name]();
    let built = await client.build(encode(mech));
    if (!built.ok) throw new Error(built.error);
    const deriveMs = built.ms;
    client.install(built);
    mech.bodies[0].m *= 1.05;
    built = await client.build(encode(mech));
    if (!built.ok || !built.reused) throw new Error('Authored parameter reuse failed');
    client.install(built);
    const kind = built.controllers.specs.find(s => s.key === 'impedance').id;
    for (let slot = 0; slot < 5; slot++) client.send({type:'controller', slot, kind});
    let checkpoints = 0, retained = 0;
    const stop = client.onCheckpoint(f => { checkpoints++; retained = f.checkpoint - f.oldest + 1; });
    client.send({type:'run', running:true});
    await new Promise(r => setTimeout(r, 500));
    const report = await client.profile(750);
    client.send({type:'run', running:false});
    await client.refresh(); stop();
    if (!report || report.steps === 0 || checkpoints < 2) throw new Error('Worker profile incomplete');
    cases.push({name, deriveMs, parameterEditMs:built.ms, checkpoints, retained, ...report});
  }
  return cases;
}"""


async def main() -> None:
    async with async_playwright() as p:
        browser = await p.chromium.launch(executable_path=os.environ.get("CHROMIUM") or None)
        results = []
        for isolated in [True, False]:
            page = await browser.new_page()
            errors: list[str] = []
            page.on("pageerror", lambda error, errors=errors: errors.append(str(error)))
            fixture = URL + "e2e/blank.html"
            if not isolated:
                await page.route(fixture, without_isolation)
            await page.goto(fixture)
            cases = await page.evaluate(PROFILE)
            assert all(case["shared"] == isolated for case in cases), "wrong transport"
            assert not errors, errors
            results.extend(cases)
            await page.close()
        await browser.close()
        print(json.dumps(results, indent=2), flush=True)


if __name__ == "__main__":
    asyncio.run(main())
