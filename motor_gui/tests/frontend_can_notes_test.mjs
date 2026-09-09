import assert from "node:assert/strict";
import fs from "node:fs";
import vm from "node:vm";

const source = fs.readFileSync(new URL("../frontend/app.js", import.meta.url), "utf8")
  .replace(/\nmain\(\);\s*$/, "\n");
const context = { document: { createElement: () => ({textContent: ""}) }, Set };
vm.createContext(context);
vm.runInContext(source, context);
const panel = context.capabilityNotePanel({notes: ["fw-v0.5.6 CAN 온도 미지원 (USB로 확인)"]});
assert.match(panel.textContent, /온도 미지원/);
assert.equal(context.capabilityNotePanel({}).textContent, "");
console.log("CAN capability notes: 2 cases passed");
