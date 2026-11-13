// Build step for the browser version (scripts/web-build.sh): download
// OurAirports' CSVs and compact them into the gzipped lists the page loads
// (data/ourairports.tsv.gz, frequencies.tsv.gz, navaids.tsv.gz) with the
// same Rust code native runs use for cache/ — by running the freshly built
// wasm module in Node (no native build needed in CI).
//
//   node scripts/web-data.mjs <dist>        (needs <dist>/pkg from wasm-bindgen)
//
// Writes <dist>/data/*.tsv.gz and prints the names written, one per line.
// A list that fails to download is skipped (the page then falls back to the CSVs).
import { readFileSync, writeFileSync, mkdirSync, readdirSync } from "node:fs";
import { join, resolve } from "node:path";
import { pathToFileURL } from "node:url";

const dist = resolve(process.argv[2] || "dist");
const pkg = join(dist, "pkg");
const pick = (re) => readdirSync(pkg).find((f) => re.test(f));
const glue = pick(/^skyward(\.[0-9a-f]+)?\.js$/);
const wasm = pick(/^skyward_bg(\.[0-9a-f]+)?\.wasm$/);
if (!glue || !wasm) throw new Error(`no wasm-bindgen output in ${pkg}`);
const mod = await import(pathToFileURL(join(pkg, glue)).href);
await mod.default({ module_or_path: readFileSync(join(pkg, wasm)) });

const BASE = "https://davidmegginson.github.io/ourairports-data/";
async function get(name) {
  for (let attempt = 1; ; attempt++) {
    try {
      const r = await fetch(BASE + name, { signal: AbortSignal.timeout(120_000) });
      if (!r.ok) throw new Error(`HTTP ${r.status}`);
      return new Uint8Array(await r.arrayBuffer());
    } catch (e) {
      if (attempt >= 3) throw new Error(`${name}: ${e.message}`);
      await new Promise((ok) => setTimeout(ok, 3000 * attempt));
    }
  }
}

const lists = [
  ["ourairports.tsv.gz", "airports", ["airports.csv", "runways.csv"]],
  ["frequencies.tsv.gz", "frequencies", ["airport-frequencies.csv"]],
  ["navaids.tsv.gz", "navaids", ["navaids.csv"]],
];
mkdirSync(join(dist, "data"), { recursive: true });
for (const [out, kind, csvs] of lists) {
  try {
    const [a, b = new Uint8Array()] = await Promise.all(csvs.map(get));
    const gz = mod.skyward_compact_data(kind, a, b);
    if (!gz) throw new Error("list too short");
    writeFileSync(join(dist, "data", out), gz);
    console.error(`web-data: ${out}: ${((a.length + b.length) / 1e6).toFixed(1)} MB csv → ${(gz.length / 1e6).toFixed(2)} MB`);
    console.log("data/" + out);
  } catch (e) {
    console.error(`web-data: ${out} skipped (${e.message})`);
  }
}
