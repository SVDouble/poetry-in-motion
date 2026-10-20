# meowfer — one entry point for the three toolchains (uv / cargo / npm).
.DEFAULT_GOAL := help
SHELL := bash
.SHELLFLAGS := -euo pipefail -c

WASM_SRC := target/wasm32-unknown-unknown/release/meowfer_core.wasm
WASM_DST := web/src/wasm/meowfer_core.wasm

.PHONY: help setup hooks gen gen-check wasm dev build preview test lint fmt check bench clean

help: ## list targets
	@grep -E '^[a-z-]+:.*?## ' $(MAKEFILE_LIST) | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'

setup: hooks ## install all dependencies (uv, npm) and git hooks
	cd sym && uv sync
	cd web && npm ci

hooks: ## use the repo's git hooks (format + lint before commit)
	git config core.hooksPath .githooks

gen: ## derive the equations symbolically and regenerate Rust + LaTeX
	cd sym && uv run meowfer-derive

gen-check: ## fail if generated code is stale
	cd sym && uv run meowfer-derive --check

wasm: ## build the physics core for the browser
	cargo build --release --target wasm32-unknown-unknown -p meowfer-core
	@mkdir -p $(dir $(WASM_DST))
	@if command -v wasm-opt >/dev/null; then \
		wasm-opt -O3 --enable-simd --enable-bulk-memory --enable-nontrapping-float-to-int $(WASM_SRC) -o $(WASM_DST); \
	else cp $(WASM_SRC) $(WASM_DST); fi
	@ls -l $(WASM_DST) | awk '{print "wasm: " $$5 " bytes"}'

dev: wasm ## run the app with hot reload on http://localhost:5173
	cd web && npm run dev

build: gen-check wasm ## production bundle in web/dist
	cd web && npm run build

preview: build ## serve the production bundle
	cd web && npm run preview

test: wasm ## run every test suite
	cargo test --workspace --quiet
	cd sym && uv run pytest -q
	cd web && npm test --silent

lint: ## formatters in check mode + linters + type checkers
	cargo fmt --all --check
	cargo clippy --workspace --all-targets -- -D warnings
	cd sym && uv run ruff format --check . && uv run ruff check . && uv run mypy src
	cd web && npx biome ci . && npx tsc --noEmit

fmt: ## format and auto-fix everything
	cargo fmt --all
	cd sym && uv run ruff format . && uv run ruff check --fix .
	cd web && npx biome check --write .

check: lint gen-check test ## everything CI runs

bench: ## native throughput of the full control + RK4 step
	cargo run --release --example bench -p meowfer-core

clean: ## remove build outputs
	cargo clean
	rm -rf web/dist $(WASM_DST)
