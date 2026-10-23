"""Actual pointer interactions for independent bodies, joints, ground and draft persistence."""

from playwright.async_api import Page


async def doc(page: Page) -> dict:
    return await page.evaluate("JSON.parse(decodeURIComponent(atob(location.hash.slice(3))))")


async def settled(page: Page) -> None:
    await page.wait_for_timeout(300)


async def center(page: Page, selector: str, index: int = 0) -> tuple[float, float]:
    box = await page.locator(selector).nth(index).bounding_box()
    assert box, selector
    return box["x"] + box["width"] / 2, box["y"] + box["height"] / 2


async def carriage(page: Page) -> None:
    await page.locator('button[data-stage="build"]').click()
    await page.locator(".preset-card", has_text="Empty scene").click()
    await settled(page)
    await page.locator('[data-part="block"]').click()
    await page.mouse.click(400, 420)
    await settled(page)
    assert (await doc(page))["bodies"][0]["detached"]
    assert await page.locator('[data-stage="derive"]').is_disabled()
    await page.keyboard.press("Escape")
    await page.locator('[data-part="ground"]').click()
    await page.mouse.click(350, 580)
    await settled(page)
    body_point = await center(page, ".gizmos .port-group circle")
    ground_point = await center(page, ".gizmos .ground-point")
    await page.locator('[data-part="prismatic"]').click()
    await page.mouse.click(*ground_point)
    await page.mouse.click(*body_point)
    await page.wait_for_function("!document.querySelector('[data-stage=derive]').disabled")
    model = await doc(page)
    assert not model["bodies"][0].get("detached")
    assert model["bodies"][0]["joint"] == "prismatic"
    await page.locator('[data-joint="0"]').click()
    assert "horizontal ground guide cannot lift off" in await page.locator(".inspector").inner_text()
    await page.locator(".inspector button", has_text="passive").click()
    await settled(page)
    assert not (await doc(page))["bodies"][0]["actuated"]
    assert "1 DOF" in await page.locator('[data-joint="0"]').inner_text()
    await page.locator(".inspector button", has_text="motor").click()
    await settled(page)
    await page.keyboard.press("Escape")


async def shared_ground(page: Page) -> None:
    # The ground hatch and a coincident body origin remain independently selectable.
    ground = await center(page, ".gizmos .ground-anchor")
    await page.locator('[data-part="revolute"]').click()
    await page.mouse.click(*ground)
    await page.locator('[data-part="bar"]').click()
    await page.mouse.click(ground[0] + 90, ground[1] - 90)
    await settled(page)
    model = await doc(page)
    assert len(model["bodies"]) == 2
    assert model["bodies"][1]["parent"] == -1
    await page.keyboard.press("Control+z")
    await settled(page)
    assert len((await doc(page))["bodies"]) == 1
    await page.keyboard.press("Escape")


async def branch(page: Page) -> None:
    # A joint can be placed first, then completed by drawing a new body.
    body_point = await center(page, ".gizmos .port-group circle")
    await page.locator('[data-part="revolute"]').click()
    await page.mouse.click(*body_point)
    # Invalid destinations keep the chosen joint anchor for a retry.
    await page.mouse.click(*body_point)
    assert "placing" in (await page.locator("#viewport").get_attribute("class") or "")
    # Switching the new body's shape also preserves the pending anchor.
    await page.locator('[data-part="block"]').click()
    await page.locator('[data-part="bar"]').click()
    await page.mouse.click(body_point[0] + 100, body_point[1] - 120)
    await settled(page)
    model = await doc(page)
    assert len(model["bodies"]) == 2
    assert model["bodies"][1]["parent"] == 0
    assert model["bodies"][1]["joint"] == "revolute"
    assert not model["bodies"][1].get("detached")
    await page.keyboard.press("Escape")

    # Source points are continuous along a bar, not restricted to its three displayed ports.
    start = await center(page, ".gizmos .port-group circle", 0)
    tip = await center(page, ".gizmos .port-group circle", 2)
    fraction = 0.3
    source = tuple(a + fraction * (b - a) for a, b in zip(start, tip, strict=True))
    await page.locator('[data-part="fixed"]').click()
    await page.mouse.click(*source)
    await page.locator('[data-part="bar"]').click()
    await page.mouse.click(source[0] - 80, source[1] - 50)
    await settled(page)
    model = await doc(page)
    assert len(model["bodies"]) == 3
    assert model["bodies"][2]["parent"] == 1
    assert abs(model["bodies"][2]["mountS"] - fraction) < 0.01


async def persistence(page: Page) -> None:
    # Disconnecting changes draft readiness, and survives a full reload.
    await page.locator('[data-joint="2"]').click()
    await page.locator(".inspector button", has_text="disconnect joint").click()
    await settled(page)
    assert await page.locator('[data-stage="simulate"]').is_disabled()
    assert (await doc(page))["bodies"][2]["detached"]
    await page.reload()
    await page.wait_for_selector('[data-body="2"]')
    assert "Unconnected" in await page.locator('[data-body="2"]').inner_text()
    assert await page.locator('[data-stage="derive"]').is_disabled()

    # Restore a connected preset so subsequent simulation checks have a valid assembly.
    await page.locator(".preset-card", has_text="Cart-pole").click()
    await page.wait_for_function("!document.querySelector('[data-stage=derive]').disabled")
    await page.locator('[data-joint="0"]').click()
    await page.locator(".inspector button", has_text="disconnect joint").click()
    await settled(page)
    await page.keyboard.press("Control+z")
    await page.wait_for_function("!document.querySelector('[data-stage=derive]').disabled")
    assert not (await doc(page))["bodies"][0].get("detached")
    await page.keyboard.press("Escape")


async def ground_and_graph(page: Page) -> None:
    await page.locator(".graph-node.ground-node").click()
    assert "Ground point 1" in await page.locator(".inspector").inner_text()
    inputs = page.locator(".inspector input")
    await inputs.nth(0).fill("0.25")
    await inputs.nth(0).press("Tab")
    await settled(page)
    await inputs.nth(1).fill("-0.5")
    await inputs.nth(1).press("Tab")
    await settled(page)
    model = await doc(page)
    assert model["bodies"][0]["mountX"] == 0.25
    assert model["bodies"][0]["mountY"] == -0.5
    assert model["bodies"][1]["parent"] == 0
    await page.locator('[data-joint="0"]').click()
    assert await page.locator('[data-joint="0"]').get_attribute("aria-pressed") == "true"
    assert await page.locator('[data-body="0"]').get_attribute("aria-pressed") == "false"
    await page.keyboard.press("Escape")
    await page.locator(".preset-card", has_text="Mass–spring–damper").click()
    await settled(page)
    await page.locator(".graph-connections button").click()
    assert "Spring–damper" in await page.locator(".inspector-head").inner_text()
    assert await page.locator(".graph-connections button").get_attribute("aria-pressed") == "true"
    await page.keyboard.press("Escape")


async def long_graph(page: Page) -> None:
    await page.locator(".preset-card", has_text="Seven-joint arm").click()
    await settled(page)
    joint = page.locator('[data-joint="6"]')
    await joint.focus()
    await joint.press("Enter")
    assert await joint.evaluate("(e) => document.activeElement === e")
    await page.locator(".inspector button", has_text="passive").click()
    await settled(page)
    assert await page.locator(".connection-graph").evaluate("(e) => e.scrollTop") > 0
    assert await joint.get_attribute("aria-pressed") == "true"
    await page.keyboard.press("Escape")
    await page.keyboard.press("Delete")
    await settled(page)
    model = await doc(page)
    assert len(model["bodies"]) == 7
    assert model["bodies"][6]["detached"]  # Delete follows the highlighted joint, not its body.
    await page.locator(".preset-card", has_text="Cart-pole").click()
    await page.wait_for_function("!document.querySelector('[data-stage=derive]').disabled")


async def builder(page: Page) -> None:
    await carriage(page)
    await shared_ground(page)
    await branch(page)
    await persistence(page)
    await ground_and_graph(page)
    await long_graph(page)
