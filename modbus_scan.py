#!/usr/bin/env python3

"""
Modbus RTU register scan (pymodbus).

If a *large* block read fails (timeout, gateway, odd slave behaviour) the old
behaviour was to skip the *whole* chunk — so shrinking --block seemed to "find
more registers" only because fewer addresses were thrown away per failure.

Recovery modes (--recover):

  bisect  — halve the failed window until each register works (default). Best
            when medium-sized reads often succeed after a big read fails.

  singles — after any failed multi-read, read that span one register at a time.
            Fewer wasted multi-read round-trips on slaves that reject or always
            time out long responses; usually *faster overall* for "read
            everything" on those devices. Tradeoff: no partial chunk wins from
            half-sized reads.

Use a large --block for the first attempt either way; it still wins when the
slave accepts full blocks.

Missing / non-responding registers are slow when pymodbus retries each
request: wall time is roughly on the order of --timeout × --retries (defaults
are tuned for fast scans). Use higher values only on noisy buses.
"""

import argparse
import json
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
    On failure, follow --recover strategy.
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


def main():

    p = argparse.ArgumentParser(
        description="Modbus RTU register scan (pymodbus)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--port", default="/dev/ttyUSB1")
    p.add_argument("--baudrate", type=int, default=9600)
    p.add_argument("--slave", type=int, default=1, help="Modbus unit / device id")
    p.add_argument("--start", type=int, required=True)
    p.add_argument("--end", type=int, required=True)
    p.add_argument("--block", type=int, default=5)
    p.add_argument(
        "--timeout",
        type=float,
        default=0.5,
        help="serial / response wait per attempt (seconds). Missing registers "
        "cost about this long per try; lower = faster scan, higher = more tolerant",
    )
    p.add_argument(
        "--retries",
        type=int,
        default=1,
        metavar="N",
        help="pymodbus retry count per Modbus request (default 1). The library "
        "default is often 3, which makes dead registers take several seconds each "
        "(timeout × multiple attempts). Increase on flaky links.",
    )
    p.add_argument(
        "--split-delay",
        type=float,
        default=0.01,
        metavar="SEC",
        help="pause between split sub-reads on RS-485 (default 0.01)",
    )
    p.add_argument(
        "--no-split-fallback",
        action="store_true",
        help="on block error, skip the whole block (old behaviour; fewer reads)",
    )
    p.add_argument(
        "--recover",
        choices=("bisect", "singles"),
        default="bisect",
        help="how to fill gaps after a failed multi-read: bisect (default), or "
        "single-register reads for that span (often faster if long reads always fail)",
    )
    p.add_argument(
        "--function",
        choices=("holding", "input"),
        default="holding",
        help="holding = FC03 read holding; input = FC04 read input",
    )
    p.add_argument(
        "--output",
        "-o",
        metavar="FILE",
        help="write all read registers as JSON to this file",
    )

    args = p.parse_args()

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

    print("=" * 70)
    print("FAST MODBUS RTU SCAN")
    print("=" * 70)
    if not args.no_split_fallback:
        extra = (
            f"(recover={args.recover} — "
            "use --recover singles if bisect feels slow on your slave)"
            if args.recover == "bisect"
            else "(recover=singles — one multi try per chunk, then singles on fail)"
        )
        print(extra)
    print("=" * 70)

    if not client.connect():
        raise SystemExit("Serial connect failed")

    t_start = time.perf_counter()

    found = 0
    block = max(1, min(args.block, 125))
    split_delay = max(0.0, args.split_delay)
    json_registers = {}

    try:
        addr = args.start

        while addr <= args.end:

            chunk_end = min(addr + block - 1, args.end)
            chunk_n = chunk_end - addr + 1

            print(f"READ BLOCK {addr}-{chunk_end} ({chunk_n} regs)")

            if args.no_split_fallback:

                rr = read_registers_group(
                    client,
                    args.function,
                    addr,
                    chunk_n,
                    args.slave,
                )

                if rr.isError():
                    print("FAILED (entire chunk skipped)")
                    addr = chunk_end + 1
                    continue

                pairs = [
                    (addr + i, rr.registers[i])
                    for i in range(len(rr.registers))
                ]

            else:
                pairs = read_block_or_split(
                    client,
                    args.function,
                    addr,
                    chunk_end,
                    args.slave,
                    split_delay,
                    args.recover,
                )

                if len(pairs) < chunk_n:
                    missing = chunk_n - len(pairs)
                    print(
                        f"  … recovered {len(pairs)}/{chunk_n} registers "
                        f"({missing} no reply after split)"
                    )

            for reg, value in pairs:

                json_registers[str(reg)] = {
                    "address": reg,
                    "device_id": args.slave,
                    "function": args.function,
                    "raw": value,
                    "hex": f"0x{value:04X}",
                    "binary": f"{value:016b}",
                }

                print(
                    f"REG {reg:<5} "
                    f"DEC={value:<6} "
                    f"HEX=0x{value:04X} "
                    f"BIN={value:016b}"
                )
                found += 1

            addr = chunk_end + 1

    finally:
        client.close()

    elapsed = time.perf_counter() - t_start

    payload = {
        "meta": {
            "port": args.port,
            "baudrate": args.baudrate,
            "device_id": args.slave,
            "function": args.function,
            "start": args.start,
            "end": args.end,
            "count": len(json_registers),
            "recover": args.recover,
            "timeout": args.timeout,
            "retries": args.retries,
            "execution_time_s": round(elapsed, 3),
        },
        "registers": json_registers,
    }

    if args.output:

        with open(args.output, "w", encoding="utf-8") as f:

            json.dump(payload, f, indent=2, sort_keys=False)

    print("=" * 70)
    print(f"FOUND {found} REGISTERS")
    print(f"TIME       : {elapsed:.3f} s")
    if args.output:
        print(f"JSON       : {args.output}")
    print("=" * 70)


if __name__ == "__main__":
    main()
