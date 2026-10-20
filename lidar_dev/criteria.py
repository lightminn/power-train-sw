# 장애물·지형 기준값 JSON 읽기(10-20). v0.1은 최상위 rules 하나, v0.2부터는 프로필(profiles.<이름>.rules)과 기본 프로필(default_profile).
# v0.2의 최상위 rules는 기본 프로필과 같아서 예전처럼 crit["rules"]만 읽어도 된다. 다른 프로필은 profile 인자로 고른다.
import copy
import hashlib
import json


def load_criteria(path, profile=None):
    """기준값 JSON → dict. profile을 주면 그 프로필의 rules를 최상위 rules로 바꿔 돌려준다(criteria_profile에 이름)."""
    raw = open(path, "rb").read()
    crit = json.loads(raw.decode("utf-8"))
    profiles = crit.get("profiles") or {}
    name = profile or crit.get("default_profile")
    if profile and profile not in profiles:
        raise SystemExit(f"{path}: 프로필 {profile!r} 없음 — 있는 것 {sorted(profiles) or '(없음, v0.1 형식)'}")
    if profiles and name not in profiles:
        raise SystemExit(f"{path}: 기본 프로필 {name!r}이 profiles에 없음")
    if profiles and profile is None and crit.get("rules") not in (None, profiles[name]["rules"]):
        raise SystemExit(f"{path}: 최상위 rules가 기본 프로필 {name}과 다름 — 생성기로 다시 만들 것")
    if name in profiles:
        crit = copy.deepcopy(crit)
        crit["rules"] = crit["profiles"][name]["rules"]
    crit["criteria_profile"] = name if name in profiles else None
    crit["criteria_sha256"] = hashlib.sha256(raw).hexdigest()
    if "rules" not in crit:
        raise SystemExit(f"{path}: rules 없음")
    return crit
