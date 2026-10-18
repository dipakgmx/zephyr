#!/usr/bin/env python3
# Copyright (c) 2025 Dipak Shetty
# SPDX-License-Identifier: Apache-2.0
"""Minimal AC Client for the peripheral_acs sample.

Runs ECDH key exchange (confirmation method None), the KDF key exchange,
then writes CTS Current Time, reads it back 2 s later and reads HRS Body
Sensor Location over ACS Data In, with the answers arriving on ACS Data Out
Notify.

Pick the switch that matches the sample build:
  --gcm   default build
  --ccm   -DEXTRA_CONF_FILE=overlay-cipher-ccm.conf
  --gmac  -DEXTRA_CONF_FILE=overlay-cipher-gmac.conf
  --cmac  -DEXTRA_CONF_FILE=overlay-cipher-cmac.conf

  pip install bleak cryptography
  python3 acs_client.py [--gcm | --ccm | --gmac | --cmac] [--address <address>] [--polls N]
"""

import argparse
import asyncio
import hashlib
import hmac
import os
import struct
from datetime import datetime

from bleak import BleakClient, BleakScanner
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import ec
from cryptography.hazmat.primitives.ciphers import algorithms
from cryptography.hazmat.primitives.ciphers.aead import AESCCM, AESGCM
from cryptography.hazmat.primitives.cmac import CMAC
from cryptography.hazmat.primitives.kdf.hkdf import HKDF

DEVICE_NAME = "ACS CTS HRS Demo"


def uuid16(v):
    return f"0000{v:04x}-0000-1000-8000-00805f9b34fb"


UUID_STATUS = uuid16(0x2B2F)
UUID_DATA_IN = uuid16(0x2B30)
UUID_DATA_OUT_NTF = uuid16(0x2B31)
UUID_DATA_OUT_IND = uuid16(0x2B32)
UUID_CP = uuid16(0x2B33)

UUID_CURRENT_TIME = 0x2A2B
UUID_BODY_SENSOR_LOCATION = 0x2A38

KEY_ID_ECDH = 0x0001
KEY_ID_KDF = 0x0002

# Hard-coded for the sample builds: ISC_ID, Key_ID, nonce fixed part size, MAC size.
ALGOS = {
    "gcm": (0x0001, 0x0003, 4, 16),
    "ccm": (0x0006, 0x0004, 5, 8),
    "gmac": (0x0004, 0x0006, 4, 16),
    "cmac": (0x0005, 0x0005, 0, 16),
}

# Every ACS field is sent least significant octet first; the crypto takes
# values most significant octet first, so most values are reversed once.


def rev(b):
    return bytes(reversed(b))


def le16(v):
    return struct.pack("<H", v)


def hkdf16(ikm, salt_wire, info_wire):
    return HKDF(hashes.SHA256(), 16, rev(salt_wire), rev(info_wire)).derive(ikm)


def hmac256(key, msg):
    return hmac.new(key, msg, hashlib.sha256).digest()


def aes_cmac(key, msg):
    c = CMAC(algorithms.AES(key))
    c.update(msg)
    return c.finalize()


def cts_time(t):
    # Year, month, day, hours, minutes, seconds, day of week, fractions256,
    # adjust reason (manual time update).
    return struct.pack(
        "<HBBBBBBBB", t.year, t.month, t.day, t.hour, t.minute, t.second, t.isoweekday(), 0, 0x01
    )


def fmt_time(v):
    year, month, day, hh, mm, ss = struct.unpack_from("<HBBBBB", v)
    return f"{year:04}-{month:02}-{day:02} {hh:02}:{mm:02}:{ss:02}"


class AcsClient:
    def __init__(self, client, algo):
        self.client = client
        self.algo = algo
        self.isc_id, self.key_id, self.fixed_size, self.mac_size = ALGOS[algo]
        self.queues = {u: asyncio.Queue() for u in (UUID_CP, UUID_DATA_OUT_NTF, UUID_DATA_OUT_IND)}
        self.rx = {}
        self.alg_key = None
        self.client_fixed = os.urandom(self.fixed_size)  # nonce fixed part, MSO first
        self.server_fixed = None
        self.tx_seq = 0

    async def start(self):
        for uuid in self.queues:
            await self.client.start_notify(uuid, self._on_segment(uuid))
        # Status is not segmented: Status_Flags, Restriction_Map_ID.
        self.status = asyncio.Queue()
        await self.client.start_notify(UUID_STATUS, lambda _, d: self.status.put_nowait(d[0]))

    def _on_segment(self, uuid):
        # Segmentation header: bit 0 first, bit 1 last, bits 2-7 rolling counter.
        def handler(_, data):
            hdr, payload = data[0], bytes(data[1:])
            self.rx[uuid] = payload if hdr & 0x01 else self.rx.get(uuid, b"") + payload
            if hdr & 0x02:
                self.queues[uuid].put_nowait(self.rx.pop(uuid))

        return handler

    async def write(self, uuid, msg):
        chunk = max(self.client.mtu_size - 3 - 1, 1)
        parts = [msg[i : i + chunk] for i in range(0, len(msg), chunk)]
        for i, part in enumerate(parts):
            hdr = (i == 0) | ((i == len(parts) - 1) << 1) | ((i % 64) << 2)
            await self.client.write_gatt_char(uuid, bytes([hdr]) + part, response=True)

    async def recv(self, uuid=UUID_CP, timeout=10):
        return await asyncio.wait_for(self.queues[uuid].get(), timeout)

    async def cp(self, opcode, operand=b"", expect=None):
        """Send a CP request; return the response message (opcode stripped)."""
        await self.write(UUID_CP, bytes([opcode]) + operand)
        msg = await self.recv()
        if msg[0] == 0x00:  # Response Code: req opcode, result
            if msg[2] != 0x01:
                raise RuntimeError(f"CP opcode 0x{msg[1]:02x} failed: code 0x{msg[2]:02x}")
            return b""
        if expect is not None and msg[0] != expect:
            raise RuntimeError(f"unexpected CP response {msg.hex()}")
        return msg[1:]

    async def start_key_exchange(self, key_id):
        try:
            # Confirmation method None (0x00), action Not Applicable (0xFF).
            await self.cp(0x11, le16(key_id) + b"\x00\xff")
        except RuntimeError:
            # A key kept from an earlier run blocks a new exchange; drop it.
            print("  Start Key Exchange refused, invalidating the stored key and retrying")
            await self.cp(0x14, le16(KEY_ID_ECDH))
            await self.cp(0x11, le16(key_id) + b"\x00\xff")

    async def ecdh(self):
        print("ECDH key exchange")
        await self.start_key_exchange(KEY_ID_ECDH)

        priv = ec.generate_private_key(ec.SECP256R1())
        nums = priv.public_key().public_numbers()
        cx, cy = nums.x.to_bytes(32, "big"), nums.y.to_bytes(32, "big")
        rsp = await self.cp(0x1B, le16(KEY_ID_ECDH) + b"\x20" + rev(cx) + b"\x20" + rev(cy), 0x1C)
        sx, sy = rev(rsp[3:35]), rev(rsp[36:68])
        server_pub = ec.EllipticCurvePublicNumbers(
            int.from_bytes(sx, "big"), int.from_bytes(sy, "big"), ec.SECP256R1()
        ).public_key()
        shared = priv.exchange(ec.ECDH(), server_pub)

        rsp = await self.cp(0x21, le16(KEY_ID_ECDH), 0x22)
        salt_len = rsp[2]
        salt, info = rsp[3 : 3 + salt_len], rsp[4 + salt_len :]
        ecdh_key = hkdf16(shared, salt, info)

        # Confirmation (AuthValue all zero for method None).
        conf_salt = hmac256(bytes(32), sx + sy + cx + cy)
        conf_key = hmac256(conf_salt, ecdh_key + bytes(32))
        rand = os.urandom(32)
        rsp = await self.cp(0x1D, le16(KEY_ID_ECDH) + rev(hmac256(conf_key, rand)), 0x1E)
        server_code = rev(rsp[2:34])
        rsp = await self.cp(0x1F, le16(KEY_ID_ECDH) + rev(rand), 0x20)
        if hmac256(conf_key, rev(rsp[2:34])) != server_code:
            raise RuntimeError("server confirmation code does not match")
        await self.expect_kex_done()
        print("  ECDH key established")
        return ecdh_key

    async def expect_kex_done(self):
        msg = await self.recv()
        if msg[0] != 0x12 or msg[3] != 0x00:
            raise RuntimeError(f"key exchange failed: {msg.hex()}")
        # The server ends the procedure (storing the ECDH key first), then
        # indicates Status with Security Established set; wait for that.
        while not (await asyncio.wait_for(self.status.get(), 10)) & 0x02:
            pass

    async def kdf(self, ecdh_key):
        print("KDF key exchange")
        # The client nonce fixed part goes in before the algorithm key exists.
        # CMAC has no nonce.
        if self.fixed_size:
            await self.cp(0x23, le16(self.key_id) + rev(self.client_fixed))
        await self.start_key_exchange(KEY_ID_KDF)
        rsp = await self.cp(0x21, le16(KEY_ID_KDF), 0x22)
        salt_len = rsp[2]
        self.alg_key = hkdf16(ecdh_key, rsp[3 : 3 + salt_len], rsp[4 + salt_len :])
        await self.expect_kex_done()

        # The algorithm record ends with the server's nonce fixed part.
        if self.fixed_size:
            rsp = await self.cp(0x0D, le16(self.key_id), 0x0E)
            self.server_fixed = rev(rsp[-self.fixed_size :])
        print(f"  {self.algo.upper()} key derived")

    async def resource_handles(self):
        """Map characteristic UUID16 -> Resource Handle from the UUID map."""
        rsp = await self.cp(0x07, expect=0x08)
        handles, i = {}, 0
        print("Resource handles:")
        while i < len(rsp):
            attr_type, handle, size = rsp[i], struct.unpack_from("<H", rsp, i + 1)[0], rsp[i + 3]
            uuid = rsp[i + 4 : i + 4 + size]
            print(f"  0x{handle:04x}  type 0x{attr_type:02x}  UUID {rev(uuid).hex()}")
            i += 4 + size + (1 if attr_type != 0x02 else 0)  # services carry a count
            if attr_type == 0x02 and size == 2:
                handles[struct.unpack("<H", uuid)[0]] = handle
        return handles

    def seal(self, nonce, body):
        """Protect body (wire order) and return it as sent."""
        # GCM and CCM: rev(ciphertext || MAC). GMAC and CMAC: rev(MAC) || body.
        if self.algo == "gcm":
            return rev(AESGCM(self.alg_key).encrypt(nonce, rev(body), None))
        if self.algo == "ccm":
            return rev(AESCCM(self.alg_key, self.mac_size).encrypt(nonce, rev(body), None))
        if self.algo == "gmac":
            mac = AESGCM(self.alg_key).encrypt(nonce, b"", rev(body))
        else:
            mac = aes_cmac(self.alg_key, rev(body))
        return rev(mac) + body

    def unseal(self, nonce, data):
        """Check and remove the protection of data; return the body (wire order)."""
        if self.algo == "gcm":
            return rev(AESGCM(self.alg_key).decrypt(nonce, rev(data), None))
        if self.algo == "ccm":
            return rev(AESCCM(self.alg_key, self.mac_size).decrypt(nonce, rev(data), None))
        mac, body = rev(data[: self.mac_size]), data[self.mac_size :]
        if self.algo == "gmac":
            AESGCM(self.alg_key).decrypt(nonce, mac, rev(body))  # raises InvalidTag
        elif not hmac.compare_digest(aes_cmac(self.alg_key, rev(body)), mac):
            raise RuntimeError("CMAC verify failed")
        return body

    async def protected_access(self, resource_handle, value=b""):
        """Read (empty value) or write a protected resource; return the value read back."""
        # Drop leftovers, e.g. the CTS notification a Current Time write triggers.
        while not self.queues[UUID_DATA_OUT_NTF].empty():
            self.queues[UUID_DATA_OUT_NTF].get_nowait()

        # Data In: ISC_ID | Nonce_Var (not for CMAC) | protected(Resource_Handle | value).
        nonce, nonce_var = None, b""
        if self.fixed_size:
            seq = self.tx_seq
            self.tx_seq += 1
            nonce = self.client_fixed + seq.to_bytes(8, "big")
            nonce_var = seq.to_bytes(8, "little")
        body = le16(resource_handle) + value
        await self.write(UUID_DATA_IN, le16(self.isc_id) + nonce_var + self.seal(nonce, body))

        # Protected notifications (e.g. Heart Rate Measurement) share Data Out
        # Notify; skip them until the reply for this resource arrives.
        while True:
            msg = await self.recv(UUID_DATA_OUT_NTF)
            nonce, data = None, msg[2:]
            if self.fixed_size:
                seq = int.from_bytes(msg[2:10], "little")
                nonce, data = self.server_fixed + seq.to_bytes(8, "big"), msg[10:]
            plain = self.unseal(nonce, data)
            rh = struct.unpack("<H", plain[:2])[0]
            if rh == resource_handle:
                return plain[2:]
            print(f"  (notification from resource 0x{rh:04x}: {plain[2:].hex()})")


async def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0], allow_abbrev=False)
    algo = parser.add_mutually_exclusive_group()
    for name in ALGOS:
        algo.add_argument(
            f"--{name}", dest="algo", action="store_const", const=name, help=f"AES-{name.upper()}"
        )
    parser.set_defaults(algo="gcm")
    parser.add_argument("--address", help="device address (macOS: CoreBluetooth UUID)")
    parser.add_argument(
        "--polls",
        type=int,
        default=3,
        metavar="N",
        help="Current Time reads, 2 s apart (default 3)",
    )
    args = parser.parse_args()

    if args.address:
        target = args.address
    else:
        print(f"Scanning for '{DEVICE_NAME}'...")
        target = await BleakScanner.find_device_by_name(DEVICE_NAME, timeout=15)
        if target is None:
            raise SystemExit("device not found")

    async with BleakClient(target) as client:
        print(f"Connected, MTU {client.mtu_size}, AES-{args.algo.upper()}")
        acs = AcsClient(client, args.algo)
        await acs.start()

        handles = await acs.resource_handles()
        ecdh_key = await acs.ecdh()
        await acs.kdf(ecdh_key)

        ct = handles[UUID_CURRENT_TIME]
        now = datetime.now().replace(microsecond=0)
        value = await acs.protected_access(ct, cts_time(now))
        print(f"Wrote Current Time {now}, read back {fmt_time(value)}  [{value.hex()}]")

        for _ in range(args.polls):
            await asyncio.sleep(2)
            value = await acs.protected_access(ct)
            print(f"Current Time 2 s later: {fmt_time(value)}  [{value.hex()}]")

        value = await acs.protected_access(handles[UUID_BODY_SENSOR_LOCATION])
        print(f"Body Sensor Location: {value.hex()}")

        # Plain ACS CP while the locked map (ID 1) is active.
        await acs.cp(0x13)
        print("Invalidated all established security")

        # The server drops the keys after the Success response; check before leaving.
        keys = await acs.cp(0x0F, expect=0x10)
        print(f"Current Key List: {keys[0]} key(s) {keys[1:].hex()}")


if __name__ == "__main__":
    asyncio.run(main())
