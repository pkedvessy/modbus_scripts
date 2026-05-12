#!/usr/bin/env python3

import argparse
import json
import re
import time

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException


class _IoFailedResponse:
    """Returned when pymodbus raises ModbusIOException (no reply / serial I/O)."""

    __slots__ = ()

    def isError(self):

        return True


_IO_FAILED = _IoFailedResponse()


def read_registers_group(client, mode, address, count, unit_id):

    """PyModbus 3.11+ uses device_id=; older releases used slave=."""

    if mode == "holding":
        reader = client.read_holding_registers
    elif mode == "input":
        reader = client.read_input_registers
    else:
        raise ValueError(mode)

    try:

        return reader(address=address, count=count, device_id=unit_id)

    except TypeError:

        try:

            return reader(address=address, count=count, slave=unit_id)

        except ModbusIOException:

            return _IO_FAILED

    except ModbusIOException:

        return _IO_FAILED


def read_block_or_split(
    client,
    mode,
    start,
    end,
    unit_id,
    split_delay,
    recover,
    depth=0,
):

    """
    Return list of (address, value) for inclusive range start..end.
    On failure, follow recover strategy (same as modbus_scan.py).
    """

    n = end - start + 1

    if depth > 24:
        return []

    rr = read_registers_group(client, mode, start, n, unit_id)

    if not rr.isError():

        regs = getattr(rr, "registers", None) or []

        if len(regs) != n:
            pass
        else:
            return [(start + i, regs[i]) for i in range(n)]

    if n == 1:
        return []

    if recover == "singles":

        out = []

        for a in range(start, end + 1):

            if a > start:
                time.sleep(split_delay)

            rr = read_registers_group(client, mode, a, 1, unit_id)

            if rr.isError():
                continue

            regs = getattr(rr, "registers", None) or []

            if len(regs) == 1:
                out.append((a, regs[0]))

        return out

    mid = start + (n // 2)
    left = read_block_or_split(
        client,
        mode,
        start,
        mid - 1,
        unit_id,
        split_delay,
        recover,
        depth + 1,
    )
    time.sleep(split_delay)
    right = read_block_or_split(
        client,
        mode,
        mid,
        end,
        unit_id,
        split_delay,
        recover,
        depth + 1,
    )

    return left + right


def fc_to_mode(fc):

    if fc == 4:
        return "input"
    return "holding"


_RE_FC = re.compile(r"FC\s*=\s*(\d+)", re.I)
_RE_REG = re.compile(r"REG\s*=\s*(\d+)", re.I)
DEFAULT_FC = 3


def parse_registers_file(path):

    pairs = []

    skip_markers = (
        "====",
        "FAST MODBUS",
        "FUNCTION CODE",
        "TOTAL FOUND",
        "EXECUTION TIME",
    )

    with open(path, encoding="utf-8", errors="replace") as f:

        for line in f:

            s = line.strip()

            if not s:
                continue

            if set(s) == {"="}:
                continue

            if any(m in s for m in skip_markers):
                continue

            mreg = _RE_REG.search(s)

            if not mreg:
                continue

            reg = int(mreg.group(1))
            mfc = _RE_FC.search(s)
            fc = int(mfc.group(1)) if mfc else DEFAULT_FC

            pairs.append((fc, reg))

    return sorted(set(pairs))


_QUAL_KEY = re.compile(r"^(3|4)_(\d+)$")


def resolve_register_fc_addr(slot_key, entry):

    """

    Entries from modbus_scan / modbus_read include address + function.

    Fallbacks: fc-qualified key "3_<addr>" / "4_<addr>", or numeric key addr.

    """

    fn = entry.get("function")

    if isinstance(fn, str):

        fns = fn.strip().lower()

        if fns == "input":

            fc_explicit = 4

        elif fns == "holding":

            fc_explicit = 3

        else:

            raise SystemExit(
                f"Register key {slot_key!r}: unknown function {fn!r}; "
                'expected "holding" or "input"'
            )

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


def register_collision_addresses(targets):

    addr_fcs = {}

    for fc, addr in targets:

        addr_fcs.setdefault(addr, set()).add(fc)

    return {
        addr
        for addr, fcs in addr_fcs.items()
        if len(fcs) > 1
    }


def register_out_key(fc, addr, collisions):

    if addr in collisions:

        return f"{fc}_{addr}"

    return str(addr)


def scan_style_register_entry(fc, addr, raw, slave_id):

    mode = fc_to_mode(fc)

    return {
        "address": addr,
        "device_id": slave_id,
        "function": mode,
        "raw": raw,
        "hex": f"0x{raw:04X}",
        "binary": f"{raw:016b}",
    }


def stub_register_entry(fc, addr, slave_id):

    """Shape without raw values when read failed — still valid --registers-json."""

    mode = fc_to_mode(fc)

    return {
        "address": addr,
        "device_id": slave_id,
        "function": mode,
    }


def parse_registers_json(path):

    """
    Load register list from modbus_scan / modbus_read JSON ({ "registers":
    … "address", "function": "holding"|"input" … }}}).
    Keys may be numeric strings or fc-qualified "3_<addr>" / "4_<addr>".
    """

    with open(path, encoding="utf-8") as f:

        data = json.load(f)

    registers = data.get("registers")

    if not isinstance(registers, dict):

        raise SystemExit(f"JSON in {path!r} must contain a 'registers' object")

    pairs = []

    for slot_key, entry in registers.items():

        if not isinstance(entry, dict):
            continue

        fc, addr = resolve_register_fc_addr(slot_key, entry)

        if fc not in (3, 4) or addr is None:

            continue

        pairs.append((fc, addr))

    return sorted(set(pairs))


def build_blocks(registers, max_size):

    regs = sorted(set(registers))

    if not regs:
        return []

    blocks = []

    start = regs[0]
    prev = regs[0]

    for reg in regs[1:]:

        contiguous = (reg == prev + 1)
        block_size = reg - start + 1

        if contiguous and block_size <= max_size:

            prev = reg
            continue

        blocks.append((start, prev))

        start = reg
        prev = reg

    blocks.append((start, prev))

    return blocks


def main():

    parser = argparse.ArgumentParser(
        description="Modbus RTU register reader (pymodbus, same read path as modbus_scan.py)",
    )

    parser.add_argument("--port", default="/dev/ttyUSB1")
    parser.add_argument("--slave", type=int, default=1)
    parser.add_argument("--baudrate", type=int, default=9600)
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="serial / response wait per attempt (seconds); match modbus_scan if needed",
    )
    parser.add_argument(
        "--retries",
        type=int,
        default=1,
        metavar="N",
        help="pymodbus retry count per request (same idea as modbus_scan)",
    )
    parser.add_argument("--output", default="snapshot.json")
    parser.add_argument(
        "--block",
        type=int,
        default=5,
        help="max contiguous registers per read attempt, 1–125",
    )
    parser.add_argument(
        "--split-delay",
        type=float,
        default=0.01,
        metavar="SEC",
        help="pause between split sub-reads on RS-485 (default 0.01)",
    )
    parser.add_argument(
        "--recover",
        choices=("bisect", "singles"),
        default="bisect",
        help="how to fill gaps after a failed multi-read (default bisect)",
    )
    parser.add_argument(
        "--no-split-fallback",
        action="store_true",
        help="on block error, skip the whole block (no bisect/singles recovery)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="suppress per-register progress lines",
    )

    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--registers-file",
        metavar="FILE",
        help="text file listing registers (scan output: FC=… REG=…) or REG=n lines",
    )
    src.add_argument(
        "--registers-json",
        metavar="FILE",
        help="modbus_scan / modbus_read --output JSON: read entries under "
        '"registers" (same format as this script writes)',
    )

    args = parser.parse_args()

    if args.registers_json:

        targets = parse_registers_json(args.registers_json)
        src_label = args.registers_json
    else:

        targets = parse_registers_file(args.registers_file)
        src_label = args.registers_file

    if not targets:

        raise SystemExit(f"No registers parsed from {src_label!r}")

    BLOCK_SIZE = max(1, min(args.block, 125))
    split_delay = max(0.0, args.split_delay)
    QUIET = args.quiet

    ser_kwargs = dict(
        port=args.port,
        baudrate=args.baudrate,
        bytesize=8,
        parity="N",
        stopbits=1,
        timeout=args.timeout,
    )

    try:

        client = ModbusSerialClient(retries=max(0, args.retries), **ser_kwargs)

    except TypeError:

        client = ModbusSerialClient(**ser_kwargs)

    if not client.connect():

        raise SystemExit("Serial connect failed")

    by_fc = {}

    for fc, reg in targets:

        by_fc.setdefault(fc, []).append(reg)

    collisions = register_collision_addresses(targets)

    registry = {}

    fc_set = {fc for fc, _ in targets}

    meta_fn = (
        fc_to_mode(next(iter(fc_set))) if len(fc_set) == 1 else "mixed"
    )

    print("")
    print("=" * 80)
    print("MODBUS RTU READ (pymodbus / scan-style blocks)")
    print("=" * 80)
    print(f"Source       : {src_label}")
    print(f"Unique targets: {len(targets)}")
    if args.no_split_fallback:
        print("Recovery     : disabled (--no-split-fallback)")
    else:
        print(f"Recovery     : {args.recover}")

    total_regs = 0
    missed = []
    t_start = time.perf_counter()

    try:

        for fc in sorted(by_fc.keys()):

            mode = fc_to_mode(fc)
            regs = by_fc[fc]
            blocks = build_blocks(regs, BLOCK_SIZE)

            print("")
            print(
                f"FUNCTION CODE {fc} ({mode})  "
                f"({len(regs)} registers, {len(blocks)} block(s))"
            )
            print("")

            for block_start, block_end in blocks:

                count = block_end - block_start + 1

                if not QUIET:
                    print(
                        f"READ BLOCK {block_start}-{block_end} "
                        f"({count} regs, FC={fc})"
                    )

                rr = read_registers_group(
                    client,
                    mode,
                    block_start,
                    count,
                    args.slave,
                )

                regs = getattr(rr, "registers", None) or []
                if not rr.isError() and len(regs) == count:

                    pairs = [
                        (block_start + i, regs[i])
                        for i in range(count)
                    ]

                elif args.no_split_fallback:

                    pairs = []

                else:

                    pairs = read_block_or_split(
                        client,
                        mode,
                        block_start,
                        block_end,
                        args.slave,
                        split_delay,
                        args.recover,
                    )

                    if len(pairs) < count and not QUIET:

                        missing = count - len(pairs)
                        print(
                            f"  … recovered {len(pairs)}/{count} registers "
                            f"({missing} no reply after split)"
                        )

                wanted = set(range(block_start, block_end + 1))
                got = {a for a, _ in pairs}
                for r in sorted(wanted - got):

                    rk = register_out_key(fc, r, collisions)

                    registry[rk] = stub_register_entry(fc, r, args.slave)

                    missed.append((fc, r))
                    if not QUIET:
                        print(f"FC={fc:<2} REG={r:<5} NO REPLY")

                for reg, raw in pairs:

                    rk = register_out_key(fc, reg, collisions)

                    registry[rk] = scan_style_register_entry(
                        fc,
                        reg,
                        raw,
                        args.slave,
                    )

                    total_regs += 1

                    if not QUIET:
                        print(
                            f"REG {reg:<5} "
                            f"DEC={raw:<6} "
                            f"HEX=0x{raw:04X} "
                            f"BIN={raw:016b}"
                        )

    finally:

        client.close()

    elapsed = time.perf_counter() - t_start

    recover_meta = (
        None
        if args.no_split_fallback
        else args.recover
    )

    payload = {
        "meta": {
            "port": args.port,
            "baudrate": args.baudrate,
            "device_id": args.slave,
            "function": meta_fn,
            "count": len(registry),
            "read_ok_count": total_regs,
            "miss_count": len(missed),
            "recover": recover_meta,
            "timeout": args.timeout,
            "retries": args.retries,
            "execution_time_s": round(elapsed, 3),
        },
        "registers": registry,
    }

    with open(args.output, "w", encoding="utf-8") as f:

        json.dump(
            payload,
            f,
            indent=2,
            sort_keys=False,
        )

    print("")
    print("=" * 80)
    print(f"TOTAL READ : {total_regs}")
    if missed:
        print(f"MISSED     : {len(missed)} register(s) (NO REPLY)")
    print(f"OUTPUT     : {args.output}")
    print(f"TIME       : {elapsed:.3f} s")
    print("=" * 80)


if __name__ == "__main__":
    main()
