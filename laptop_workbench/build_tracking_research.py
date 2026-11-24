"""Update only the research UI block; preserve other workbench integrations."""
from pathlib import Path

root = Path(__file__).resolve().parent
target = root / 'integration.js'
start = '\n// BEGIN TRACKING RESEARCH\n'
end = '\n// END TRACKING RESEARCH\n'
text = target.read_text(encoding='utf-8')
assert text.count(start) == text.count(end) == 1
prefix, rest = text.split(start, 1)
_, suffix = rest.split(end, 1)
target.write_text(prefix + start + (root / 'tracking-research-source.js').read_text(encoding='utf-8') + end + suffix, encoding='utf-8')
