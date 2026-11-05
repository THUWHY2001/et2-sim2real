import { readFileSync, writeFileSync } from "node:fs";

const target = new URL("../dist/j2.html", import.meta.url);
const wrapped = readFileSync(target, "utf8");
const match = wrapped.match(/data-srcdoc="([\s\S]*?)"><\/iframe>/);

if (!match) throw new Error("Expected rendered J2 srcdoc wrapper");

const entities = {
  lt: "<",
  gt: ">",
  quot: '"',
  amp: "&",
};

const direct = match[1]
  .replace(/&#x([0-9a-f]+);/gi, (_, code) => String.fromCodePoint(parseInt(code, 16)))
  .replace(/&#([0-9]+);/g, (_, code) => String.fromCodePoint(Number(code)))
  .replace(/&(lt|gt|quot|amp);/g, (_, name) => entities[name])
  .replace("<title>Leg 2d Mechanism</title>", "<title>J2 二维机构简图</title>");

writeFileSync(target, direct, "utf8");
