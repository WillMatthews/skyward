// A tile builder: this same wasm module in a Web Worker (src/tiles_web.rs).
// The page posts {type:"init", module} once, then {type:"build", z, x, y,
// template} (a tile) or {type:"job", kind, call, data} (a small job, e.g. the
// sky glow); the worker answers "ready", "db" (its airport database is in),
// "tile" (the finished tile, encoded), "job" (data undefined if it failed)
// or "error".
// (scripts/web-build.sh fills in the content-hashed module name.)
import init, { skyward_worker_init, skyward_worker_build, skyward_worker_job } from "./pkg/__GLUE__";

let ready = null;
self.onmessage = async (e) => {
  const m = e.data;
  if (m.type === "init") {
    ready = init({ module_or_path: m.module });
    await ready;
    self.postMessage({ type: "ready" });
    skyward_worker_init().then((ok) => self.postMessage({ type: "db", ok }));
    return;
  }
  await ready;
  if (m.type === "job") {
    let data;
    try {
      data = skyward_worker_job(m.kind, m.data);
    } catch (err) {
      data = undefined;
    }
    self.postMessage({ type: "job", call: m.call, data }, data ? [data.buffer] : []);
    return;
  }
  if (m.type === "build") {
    try {
      const data = await skyward_worker_build(m.z, m.x, m.y, m.template);
      self.postMessage({ type: "tile", data }, [data.buffer]);
    } catch (err) {
      self.postMessage({ type: "error", message: String(err) });
    }
  }
};
