/**
 * The 3D viewport: WebGPU renderer (automatic WebGL2 fallback), an orthographic camera looking at the
 * plane of motion from slightly above and to the side (so the bodies read as solids), soft shadows on a
 * transparent backdrop, a metric grid and screen-space labels.
 */
import * as THREE from 'three/webgpu';
import { tex } from '../eq/tex';

interface Label {
  el: HTMLElement;
  obj: THREE.Object3D;
  dx: number;
  dy: number;
}

export class Stage {
  readonly scene = new THREE.Scene();
  readonly camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -50, 50);
  /** Per-system content; cleared when switching systems. */
  readonly root = new THREE.Group();
  readonly renderer: THREE.WebGPURenderer;
  private labels: Label[] = [];
  private readonly labelLayer: HTMLElement;
  private view = { cx: 0, cy: 0, width: 4, height: 4 };
  private size = { w: 1, h: 1 };
  private readonly tmp = new THREE.Vector3();
  private readonly ray = new THREE.Raycaster();
  private readonly plane = new THREE.Plane(new THREE.Vector3(0, 0, 1), 0);
  private readonly key: THREE.DirectionalLight;

  constructor(private readonly container: HTMLElement) {
    // `?webgl` forces the WebGL 2 backend (debugging / comparing backends)
    const forceWebGL = new URLSearchParams(location.search).has('webgl');
    this.renderer = new THREE.WebGPURenderer({ antialias: true, alpha: true, forceWebGL });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setClearColor(0x000000, 0);
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    container.append(this.renderer.domElement);

    this.labelLayer = document.createElement('div');
    this.labelLayer.className = 'labels';
    container.append(this.labelLayer);

    this.scene.add(this.root);

    const hemi = new THREE.HemisphereLight(0xdfe8ff, 0x1a1f2b, 1.5);
    this.key = new THREE.DirectionalLight(0xffffff, 2.6);
    this.key.position.set(2.5, 4, 6);
    this.key.castShadow = true;
    this.key.shadow.mapSize.set(2048, 2048);
    this.key.shadow.radius = 4;
    this.key.shadow.bias = -0.0005;
    Object.assign(this.key.shadow.camera, { left: -6, right: 6, top: 6, bottom: -6, near: 0.1, far: 30 });
    const rim = new THREE.DirectionalLight(0x8fb8ff, 1.1);
    rim.position.set(-6, -2, 4);

    // invisible wall behind the plane of motion that only shows shadows
    const backdrop = new THREE.Mesh(
      new THREE.PlaneGeometry(40, 40),
      new THREE.ShadowMaterial({ color: 0x000000, opacity: 0.32 }),
    );
    backdrop.position.z = -0.45;
    backdrop.receiveShadow = true;
    this.scene.add(hemi, this.key, this.key.target, rim, backdrop, this.grid());

    new ResizeObserver(() => this.resize()).observe(container);
  }

  async init(): Promise<void> {
    await this.renderer.init();
    this.resize();
  }

  get backend(): string {
    const b = (this.renderer as unknown as { backend?: { isWebGPUBackend?: boolean } }).backend;
    return b?.isWebGPUBackend ? 'WebGPU' : 'WebGL 2';
  }

  /** Frame (contain) the `width × height` region (m) centred at (cx, cy). */
  setView(cx: number, cy: number, width: number, height: number): void {
    this.view = { cx, cy, width, height };
    this.resize();
  }

  resize(): void {
    const w = Math.max(1, this.container.clientWidth);
    const h = Math.max(1, this.container.clientHeight);
    this.size = { w, h };
    this.renderer.setSize(w, h, false);
    const aspect = w / h;
    const halfH = Math.max(this.view.height / 2, this.view.width / 2 / aspect);
    const halfW = halfH * aspect;
    const { cx, cy } = this.view;
    Object.assign(this.camera, { left: -halfW, right: halfW, top: halfH, bottom: -halfH });
    this.camera.updateProjectionMatrix();
    // look at the centre of the view from slightly above and to the right
    const pitch = 0.16;
    const yaw = 0.2;
    this.camera.position.set(
      cx + 20 * Math.sin(yaw) * Math.cos(pitch),
      cy + 20 * Math.sin(pitch),
      20 * Math.cos(yaw) * Math.cos(pitch),
    );
    this.camera.near = 1;
    this.camera.far = 60;
    this.camera.lookAt(cx, cy, 0);
    this.camera.updateProjectionMatrix();
    this.key.position.set(cx + 2.5, cy + 4, 6);
    this.key.target.position.set(cx, cy, 0);
  }

  /** Pointer position → world coordinates on the z = 0 plane. */
  toWorld(clientX: number, clientY: number): [number, number] {
    const r = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((clientX - r.left) / r.width) * 2 - 1,
      -((clientY - r.top) / r.height) * 2 + 1,
    );
    this.ray.setFromCamera(ndc, this.camera);
    const hit = this.ray.ray.intersectPlane(this.plane, this.tmp);
    return hit ? [hit.x, hit.y] : [0, 0];
  }

  /** A KaTeX label that follows an object. */
  label(obj: THREE.Object3D, src: string, cls = '', dx = 8, dy = -8): HTMLElement {
    const el = document.createElement('div');
    el.className = `label ${cls}`;
    tex(el, src);
    this.labelLayer.append(el);
    this.labels.push({ el, obj, dx, dy });
    return el;
  }

  /** Let every solid in the system cast shadows (glowing overlays and lines do not). */
  castShadows(): void {
    this.root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh && (mesh.material as THREE.Material).type === 'MeshStandardMaterial')
        mesh.castShadow = true;
    });
  }

  clear(): void {
    this.root.traverse((o) => {
      const mesh = o as THREE.Mesh;
      mesh.geometry?.dispose();
      const mats = Array.isArray(mesh.material) ? mesh.material : mesh.material ? [mesh.material] : [];
      for (const m of mats) m.dispose();
    });
    this.root.clear();
    for (const l of this.labels) l.el.remove();
    this.labels = [];
  }

  render(): void {
    this.renderer.render(this.scene, this.camera);
    const { w, h } = this.size;
    for (const l of this.labels) {
      if (!l.obj.visible) {
        l.el.style.visibility = 'hidden';
        continue;
      }
      this.tmp.setFromMatrixPosition(l.obj.matrixWorld).project(this.camera);
      const x = ((this.tmp.x + 1) / 2) * w + l.dx;
      const y = ((1 - this.tmp.y) / 2) * h + l.dy;
      l.el.style.visibility = 'visible';
      l.el.style.transform = `translate3d(${x.toFixed(1)}px, ${y.toFixed(1)}px, 0)`;
    }
  }

  private grid(): THREE.Object3D {
    const minor: number[] = [];
    const major: number[] = [];
    const ext = 12;
    for (let k = -ext * 4; k <= ext * 4; k++) {
      const v = k / 4;
      const target = k % 4 === 0 ? major : minor;
      target.push(-ext, v, -1, ext, v, -1, v, -ext, -1, v, ext, -1);
    }
    const group = new THREE.Group();
    for (const [pts, opacity] of [
      [minor, 0.05],
      [major, 0.11],
    ] as const) {
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.Float32BufferAttribute(pts, 3));
      const m = new THREE.LineBasicMaterial({
        color: 0xaab6d3,
        transparent: true,
        opacity,
        depthWrite: false,
      });
      group.add(new THREE.LineSegments(g, m));
    }
    return group;
  }
}
