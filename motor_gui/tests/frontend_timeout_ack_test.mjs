import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import vm from "node:vm";
import { fileURLToPath } from "node:url";


const HERE = path.dirname(fileURLToPath(import.meta.url));
const APP_JS = path.join(HERE, "..", "frontend", "app.js");


class Element {
  constructor(tag) {
    this.tag = tag;
    this.children = [];
    this.listeners = {};
    this.style = {};
    this.className = "";
    this.textContent = "";
    this.value = "";
    this.disabled = false;
  }

  appendChild(child) {
    this.children.push(child);
    return child;
  }

  addEventListener(name, callback) {
    this.listeners[name] = callback;
  }

  querySelectorAll(selector) {
    const found = [];
    const visit = (node) => {
      if (selector === ".logline" && node.className.split(" ").includes("logline")) {
        found.push(node);
      }
      node.children.forEach(visit);
    };
    this.children.forEach(visit);
    return found;
  }

  removeChild(child) {
    this.children.splice(this.children.indexOf(child), 1);
  }
}


function findTag(root, tag) {
  if (root.tag === tag) return root;
  for (const child of root.children) {
    const found = findTag(child, tag);
    if (found) return found;
  }
  return null;
}


function makeHarness(fetchImpl) {
  const log = new Element("div");
  let reloads = 0;
  const document = {
    createElement: (tag) => new Element(tag),
    getElementById: (id) => id === "log" ? log : null,
  };
  const source = fs.readFileSync(APP_JS, "utf8");
  const withoutMain = source.replace(/\nmain\(\);\s*$/, "\n");
  assert.notEqual(withoutMain, source, "app.js must end with main();");
  const context = {
    Date,
    JSON,
    Math,
    Set,
    console,
    document,
    fetch: fetchImpl,
    localStorage: { getItem: () => null, setItem: () => {} },
    location: { get host() { return "localhost"; }, reload: () => { reloads += 1; } },
    setTimeout,
  };
  vm.createContext(context);
  vm.runInContext(withoutMain, context, { filename: APP_JS });
  return { context, log, reloads: () => reloads };
}


function response(body) {
  return { json: async () => body };
}


async function commandCase(ack) {
  const harness = makeHarness(async (url) => {
    assert.equal(url, "/api/command");
    return response(ack);
  });
  const returned = await harness.context.postCommand({
    target: "odrive", op: "set_input", args: { vel: 8.0 },
  });
  return { ...harness, returned };
}


async function profileCase(ack, changeWhilePending = false) {
  let resolveApply;
  const harness = makeHarness(async (url, options) => {
    if (url === "/api/tunable_profiles") {
      return response({ bl70200: { label: "BL70200" } });
    }
    assert.equal(url, "/api/tunable_profiles/apply");
    assert.equal(JSON.parse(options.body).profile, "bl70200");
    if (changeWhilePending) {
      return new Promise((resolve) => { resolveApply = resolve; });
    }
    return response(ack);
  });
  const panel = await harness.context.tunableProfilePanel();
  const select = findTag(panel, "select");
  const apply = findTag(panel, "button");
  assert.ok(select && apply);
  select.value = "bl70200";
  select.listeners.change();
  const pending = apply.listeners.click();
  if (changeWhilePending) {
    select.value = "";
    select.listeners.change();
    resolveApply(response(ack));
  }
  await pending;
  return harness;
}


const unknownCommand = await commandCase({
  ok: false,
  status: "OUTCOME_UNKNOWN",
  detail: "command timeout after execution started; outcome unknown",
});
assert.equal(unknownCommand.returned.status, "OUTCOME_UNKNOWN");
assert.equal(unknownCommand.log.children.length, 1);
assert.match(unknownCommand.log.children[0].textContent, /명령 결과 미확정/);
assert.match(unknownCommand.log.children[0].textContent, /실제 상태.*재전송하지 마세요/);
assert.equal(unknownCommand.log.children[0].className, "logline warn");

const rejectedCommand = await commandCase({
  ok: false,
  status: "FINAL_REJECTED",
  detail: "cancelled before execution",
});
assert.match(rejectedCommand.log.children[0].textContent, /명령 거부/);
assert.equal(rejectedCommand.log.children[0].className, "logline err");

const successfulCommand = await commandCase({ ok: true, detail: "ok" });
assert.equal(successfulCommand.returned.ok, true);
assert.equal(successfulCommand.log.children.length, 0);

const unknownProfile = await profileCase({
  ok: false,
  status: "OUTCOME_UNKNOWN",
  detail: "profile apply timeout after execution started; outcome unknown",
});
assert.match(unknownProfile.log.children[0].textContent, /프로파일 적용 결과 미확정/);
assert.match(unknownProfile.log.children[0].textContent, /실제 상태.*재전송하지 마세요/);
assert.equal(unknownProfile.log.children[0].className, "logline warn");
assert.equal(unknownProfile.reloads(), 0);

const rejectedProfile = await profileCase({
  ok: false,
  status: "FINAL_REJECTED",
  detail: "cancelled before execution",
});
assert.match(rejectedProfile.log.children[0].textContent, /프로파일 적용 실패/);
assert.equal(rejectedProfile.log.children[0].className, "logline err");
assert.equal(rejectedProfile.reloads(), 0);

const successfulProfile = await profileCase({ ok: true, detail: "applied" });
assert.match(successfulProfile.log.children[0].textContent, /프로파일 적용: BL70200/);
assert.equal(successfulProfile.log.children[0].className, "logline");
assert.equal(successfulProfile.reloads(), 1);

for (const [ack, expectedText, expectedReloads] of [
  [{ ok: false, status: "OUTCOME_UNKNOWN" }, /프로파일 적용 결과 미확정: BL70200/, 0],
  [{ ok: false, status: "FINAL_REJECTED" }, /프로파일 적용 실패/, 0],
  [{ ok: true }, /프로파일 적용: BL70200/, 1],
]) {
  const changedSelection = await profileCase(ack, true);
  assert.match(changedSelection.log.children[0].textContent, expectedText);
  assert.equal(changedSelection.reloads(), expectedReloads);
}

console.log("frontend timeout ACK contract: 9 cases passed");
