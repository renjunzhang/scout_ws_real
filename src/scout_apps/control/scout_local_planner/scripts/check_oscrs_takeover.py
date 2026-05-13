#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Summarise OSCRS takeover status from /anti_slosh_path/candidate_report."""

import argparse
import re
import sys

import rosbag


SUMMARY_RE = re.compile(r"summary:([^;]+)")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/anti_slosh_path/candidate_report")
    parser.add_argument("--require-takeover", action="store_true",
                        help="exit nonzero unless at least one report has takeover=1")
    return parser.parse_args()


def parse_summary(text):
    match = SUMMARY_RE.search(text)
    if not match:
        return {}
    out = {}
    for item in match.group(1).split(","):
        if "=" not in item:
            continue
        key, value = item.split("=", 1)
        out[key.strip()] = value.strip()
    return out


def is_takeover_report(row):
    selected = row.get("selected")
    if row.get("takeover") == "1":
        return True
    return (
        row.get("active") == "1"
        and row.get("fb") == "0"
        and selected not in ("", None, "original", "missing")
    )


def main():
    args = parse_args()
    count = 0
    active = 0
    takeover = 0
    fallback = 0
    fb_counts = {}
    selected_counts = {}
    last = {}
    with rosbag.Bag(args.bag) as bag:
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            row = parse_summary(str(msg.data))
            if not row:
                continue
            count += 1
            last = row
            if row.get("active") == "1":
                active += 1
            if is_takeover_report(row):
                takeover += 1
            if row.get("fallback") == "1":
                fallback += 1
            fb = row.get("fb", "missing")
            fb_counts[fb] = fb_counts.get(fb, 0) + 1
            selected = row.get("selected", "missing")
            selected_counts[selected] = selected_counts.get(selected, 0) + 1

    print(f"bag={args.bag}")
    print(f"reports={count} active_reports={active} takeover_reports={takeover} fallback_reports={fallback}")
    print(f"fb_counts={fb_counts}")
    print(f"selected_counts={selected_counts}")
    if last:
        print(
            "last="
            + ",".join(
                f"{key}={last.get(key, '')}"
                for key in ("selected", "geo", "oscrs", "active", "fallback", "fb", "takeover")
            )
        )
    if args.require_takeover and takeover <= 0:
        sys.stderr.write("error: no OSCRS takeover report found\n")
        sys.exit(1)


if __name__ == "__main__":
    main()
