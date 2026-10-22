/** Scalar WASM ABI shared by the worker and real-core test harnesses.
 * Read memory.buffer only after calls: allocation can grow and detach memory. */
export interface Core {
  memory: WebAssembly.Memory;
  build_reused(): number;
  integrate(steps: number): void;
  checkpoint_save(): number;
  checkpoint_oldest(): number;
  checkpoint_restore(id: number): number;
  flag_value(id: number): number;
  set_tasks(len: number): number;
  tasks(): number;
  tasks_ptr(): number;
  desc_buffer(len: number): number;
  prepare(len: number): number;
  commit_build(): number;
  discard_build(): void;
  error_ptr(): number;
  error_len(): number;
  derivation_ptr(): number;
  derivation_len(): number;
  reset(): void;
  step(n: number): void;
  bench_prepare(): void;
  bench(n: number): void;
  publish(): void;
  set_param(i: number, v: number): number;
  set_controller(slot: number, kind: number): void;
  set_ctrl_param(kind: number, i: number, v: number): number;
  param_value(i: number): number;
  target_value(i: number): number;
  sync_shadows(): void;
  controllers(): number;
  controllers_ptr(): number;
  set_target(i: number, v: number): number;
  set_input(i: number, v: number): number;
  set_flag(id: number, v: number): number;
  set_state(i: number, q: number, qd: number): number;
  kick(i: number, v: number): number;
  out_ptr(): number;
  out_len(): number;
  layout_ptr(): number;
  layout_len(): number;
  build(len: number): number;
  set_dt(dt: number): number;
  param_count(): number;
  controllers_len(): number;
}
