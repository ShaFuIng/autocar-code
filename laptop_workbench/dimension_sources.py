"""Consolidate legacy dimensions without replacing existing measurements."""
from copy import deepcopy
import json
from urllib.request import Request, urlopen

LEGACY_FIELDS = {
    "track_width": "track_width",
    "line_width": "line_width",
    "track_outer_width": "track_outer_width",
    "envelope_width": "width",
    "envelope_length": "envelope_length",
    "deck_height": "deck_height",
    "body_height": "overall_height",
}


def consolidate(measurement, config):
    data = deepcopy(measurement["data"])
    items = data.setdefault("items", {})
    added = []
    for target, source in LEGACY_FIELDS.items():
        # A present unknown/empty measurement is intentional, never backfill it.
        if target in items:
            continue
        value = config["data"].get(source)
        if type(value) not in (float, int) or value <= 0:
            continue
        items[target] = {
            "value": value,
            "status": "review",
            "note": f"沿用早期config v{config['rev']}的{source}；非本次重新量測，來源備註保留於舊配置歷史。",
            "source": {"document": "config", "revision": config["rev"], "field": source},
        }
        added.append(target)
    if added:
        data.setdefault("source_migrations", []).append({
            "id": "six-pages-20261021", "from": "config",
            "revision": config["rev"], "fields": added,
        })
    return data, added


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    base = "http://127.0.0.1:8093"

    def read(key):
        with urlopen(base + "/api/document/" + key, timeout=10) as response:
            return json.load(response)

    current, legacy = read("vehicle-measurements"), read("config")
    data, added = consolidate(current, legacy)
    if args.apply and added:
        payload = json.dumps({"key": "vehicle-measurements", "rev": current["rev"], "data": data}).encode()
        request = Request(base + "/api/document", data=payload, headers={
            "Content-Type": "application/json", "X-Workbench": "1",
        })
        with urlopen(request, timeout=10) as response:
            result = json.load(response)
        print(json.dumps({"revision": result["rev"], "added": added}))
    else:
        print(json.dumps({"dry_run": not args.apply, "added": added}))
