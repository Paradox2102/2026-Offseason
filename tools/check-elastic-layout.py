"""Checks that every widget topic in the Elastic layout exists in a sim topic list.

Usage: python3 tools/check-elastic-layout.py src/main/deploy/elastic-layout.json build/sim-topics.txt
A widget that reads a table (field, swerve, alerts, chooser, command) passes when any topic is
under that table.
"""
import json
import sys

layout = json.load(open(sys.argv[1]))
topics = set(line.strip() for line in open(sys.argv[2]) if line.strip())
missing = []
for tab in layout["tabs"]:
    for widget in tab["grid_layout"]["containers"]:
        topic = widget["properties"]["topic"]
        if topic not in topics and not any(t.startswith(topic + "/") for t in topics):
            missing.append(f'{tab["name"]}: {widget["title"]} -> {topic}')
for line in missing:
    print("MISSING", line)
print(f"elastic layout: {len(missing)} missing topics")
sys.exit(1 if missing else 0)
