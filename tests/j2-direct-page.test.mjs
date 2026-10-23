import assert from "node:assert/strict";
import { readFileSync } from "node:fs";

const page = readFileSync(new URL("../dist/j2.html", import.meta.url), "utf8");

assert.match(page, /<title>J2 二维机构简图/);
assert.doesNotMatch(page, /<iframe|data-srcdoc=/, "J2 must render directly instead of decoding a nested srcdoc iframe");
assert.match(page, /id="leg-2d-mechanism"/);
assert.ok(Buffer.byteLength(page) < 80_000, "J2 direct page should stay below 80 KB");

console.log("J2 direct-page regression test passed");
