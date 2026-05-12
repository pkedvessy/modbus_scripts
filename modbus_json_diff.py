#!/usr/bin/env python3

"""
Compare Modbus register JSON snapshots (modbus_read / modbus_scan --output shape).

Registers are keyed by Modbus semantics (holding|input × address); json slot keys
("5", "3_10") are preserved for traceability only.

Output JSON lists only differing registers: only_in_a, only_in_b, and
value_differences (raw changed or unreadable on one side).
"""

import argparse
import json
import re
import sys


_QUAL_KEY = re.compile(r"^(3|4)_(\d+)$")


def _fc_to_function(fc):

    return "input" if fc == 4 else "holding"


def resolve_register_fc_addr(slot_key, entry):

    """Same pairing rules as modbus_read.resolve_register_fc_addr."""

    fn = entry.get("function")

    if isinstance(fn, str):

        fns = fn.strip().lower()

        if fns == "input":

            fc_explicit = 4

        elif fns == "holding":

            fc_explicit = 3

        else:

            raise ValueError(f"bad function field {fn!r}")

    else:

        fc_explicit = None

    ea = entry.get("address")

    if ea is not None:

        addr = int(ea)

        fc = fc_explicit if fc_explicit is not None else 3

        return fc, addr

    sk = str(slot_key)

    mq = _QUAL_KEY.match(sk)

    if mq:

        return int(mq.group(1)), int(mq.group(2))

    try:

        addr = int(sk)

    except (TypeError, ValueError):

        return None, None

    fc = fc_explicit if fc_explicit is not None else 3

    return fc, addr


def register_id(slot_key, entry):

    fc, addr = resolve_register_fc_addr(slot_key, entry)

    if fc not in (3, 4) or addr is None:

        return None

    fn = _fc_to_function(fc)

    return fn, addr


def id_tuple_to_str(ident):

    return f"{ident[0]}:{ident[1]}"


def load_registers_json(path):

    with open(path, encoding="utf-8") as f:

        root = json.load(f)

    reg = root.get("registers")

    if not isinstance(reg, dict):

        raise SystemExit(f"{path}: missing or invalid 'registers' object")

    return root.get("meta"), reg


def index_registers(path_label, regs):

    by_id = {}
    skips = []

    for slot_key, entry in regs.items():

        if not isinstance(entry, dict):

            skips.append(("non_object_entry", slot_key))
            continue

        try:

            ident = register_id(slot_key, entry)

        except ValueError as e:

            skips.append(("bad_entry", slot_key, str(e)))
            continue

        if ident is None:

            skips.append(("unparsed", slot_key))
            continue

        if ident in by_id:

            raise SystemExit(
                f"{path_label}: duplicate register identity {id_tuple_to_str(ident)!r}"
            )

        by_id[ident] = {"json_key": str(slot_key), "entry": entry}

    return by_id, skips


def raw_optional(entry):

    if "raw" not in entry:

        return None

    raw = entry.get("raw")

    if raw is None:

        return None

    try:

        return int(raw)

    except (TypeError, ValueError):

        return None


def main():

    p = argparse.ArgumentParser(
        description="Diff two modbus JSON register snapshots (writes JSON report)",
    )
    p.add_argument("file_a", help="First JSON (e.g. earlier snapshot)")
    p.add_argument("file_b", help="Second JSON (e.g. later snapshot)")
    p.add_argument(
        "-o",
        "--output",
        default="register_diff.json",
        metavar="FILE",
        help="write diff JSON (default: register_diff.json)",
    )
    p.add_argument(
        "--include-meta",
        action="store_true",
        help="embed full meta from both files under output meta",
    )

    args = p.parse_args()

    meta_a, regs_a = load_registers_json(args.file_a)

    meta_b, regs_b = load_registers_json(args.file_b)

    idx_a, skip_a = index_registers(args.file_a, regs_a)

    idx_b, skip_b = index_registers(args.file_b, regs_b)

    ids_a = set(idx_a)

    ids_b = set(idx_b)

    only_a = sorted(ids_a - ids_b)

    only_b = sorted(ids_b - ids_a)

    common = sorted(ids_a & ids_b)

    value_diffs = []

    unchanged_count = 0

    for ident in common:

        pa = idx_a[ident]["entry"]

        pb = idx_b[ident]["entry"]

        ra = raw_optional(pa)

        rb = raw_optional(pb)

        if ra is None or rb is None:

            if ra != rb:

                value_diffs.append(
                    {
                        "id_string": id_tuple_to_str(ident),
                        "function": ident[0],
                        "address": ident[1],
                        "keys": {
                            "a": idx_a[ident]["json_key"],
                            "b": idx_b[ident]["json_key"],
                        },
                        "kind": "raw_availability_changed",
                        "a_raw": ra,
                        "b_raw": rb,
                        "entry_a": pa,
                        "entry_b": pb,
                    }
                )

            else:

                unchanged_count += 1

            continue

        if ra != rb:

            value_diffs.append(
                {
                    "id_string": id_tuple_to_str(ident),
                    "function": ident[0],
                    "address": ident[1],
                    "keys": {
                        "a": idx_a[ident]["json_key"],
                        "b": idx_b[ident]["json_key"],
                    },
                    "kind": "raw_changed",
                    "a_raw": ra,
                    "b_raw": rb,
                    "raw_delta_b_minus_a": rb - ra,
                    "entry_a": pa,
                    "entry_b": pb,
                }
            )

        else:

            unchanged_count += 1

    summary = {

        "registers_only_in_a": len(only_a),

        "registers_only_in_b": len(only_b),

        "value_difference_count": len(value_diffs),

        "skipped_parse_entries_first": len(skip_a),

        "skipped_parse_entries_second": len(skip_b),

    }

    out_root = {

        "meta": {

            "file_a": args.file_a,

            "file_b": args.file_b,

            "summary": summary,

            "skipped_entries": {

                "file_a": skip_a,

                "file_b": skip_b,

            },

        },

        "only_in_a": [

            {

                "id_string": id_tuple_to_str(ident),

                "function": ident[0],

                "address": ident[1],

                "json_key": idx_a[ident]["json_key"],

                "entry": idx_a[ident]["entry"],

            }

            for ident in only_a

        ],

        "only_in_b": [

            {

                "id_string": id_tuple_to_str(ident),

                "function": ident[0],

                "address": ident[1],

                "json_key": idx_b[ident]["json_key"],

                "entry": idx_b[ident]["entry"],

            }

            for ident in only_b

        ],

        "value_differences": value_diffs,

    }

    if args.include_meta:

        out_root["meta"]["meta_a"] = meta_a

        out_root["meta"]["meta_b"] = meta_b

    with open(args.output, "w", encoding="utf-8") as f:

        json.dump(out_root, f, indent=2, sort_keys=False)

    lines = []

    lines.append(f"Compared {args.file_a!r} -> {args.file_b!r}")

    lines.append(

        f"  only_in_a={summary['registers_only_in_a']}  "

        f"only_in_b={summary['registers_only_in_b']}  "

        f"value_diffs={summary['value_difference_count']}  "

        f"unchanged={unchanged_count}"

    )

    msg = "\n".join(lines) + "\n"

    sys.stdout.write(msg)

    sys.stdout.write(f"Wrote {args.output!r}\n")


if __name__ == "__main__":
    main()
