#!/usr/bin/env python3

from pymodbus.client import ModbusSerialClient
import time

PORT = "/dev/ttyUSB1"
SLAVE = 1

client = ModbusSerialClient(
    port=PORT,
    baudrate=9600,
    parity='N',
    stopbits=1,
    bytesize=8,
    timeout=1.0
)

if not client.connect():
    print("FAILED TO CONNECT")
    exit(1)

START = 0
END = 2500
BLOCK = 1

print("=== SCANNING ===")

found = 0

for addr in range(START, END):

    try:
        rr = client.read_holding_registers(
            address=addr,
            count=BLOCK,
            slave=SLAVE
        )

        if rr.isError():
            print(f"{addr:<5} ERROR")
        else:
            value = rr.registers[0]

            print(f"{addr:<5} = {value}")
            found += 1

    except Exception as e:
        print(f"{addr:<5} EXCEPTION {e}")

    # XR controllers need pacing
    time.sleep(0.05)

print()
print(f"FOUND {found} REGISTERS")

client.close()