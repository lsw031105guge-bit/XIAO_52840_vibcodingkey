import asyncio
import struct
import wave
from bleak import BleakClient, BleakScanner

NUS_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
NUS_RX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # write to peripheral
NUS_TX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # notify from peripheral

DEVICE_NAME = "XIAO MacroPad"

class Session:
    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.session_id = None
        self.rate = 16000
        self.samples = 0
        self.bps = 2
        self.ch = 1
        self.buf = bytearray()
        self.expected_bytes = 0
        self.next_seq = 0

sess = Session()
client_ref = {"client": None}

def save_wav(path: str, pcm_bytes: bytes, rate: int, channels: int, sampwidth: int):
    with wave.open(path, "wb") as wf:
        wf.setnchannels(channels)
        wf.setsampwidth(sampwidth)
        wf.setframerate(rate)
        wf.writeframes(pcm_bytes)

async def handle_notify(_: int, data: bytearray):
    c: BleakClient = client_ref["client"]
    if c is None:
        return

    if len(data) >= 4 and data[0:4] == b"AUD1":
        if len(data) < 16:
            return
        session_id, = struct.unpack_from("<I", data, 4)
        rate, = struct.unpack_from("<H", data, 8)
        samples, = struct.unpack_from("<I", data, 10)
        bps = data[14]
        ch = data[15]

        sess.reset()
        sess.active = True
        sess.session_id = session_id
        sess.rate = rate
        sess.samples = samples
        sess.bps = bps
        sess.ch = ch
        sess.expected_bytes = samples * bps
        sess.buf = bytearray()
        sess.next_seq = 0

        print(f"[HDR] session={session_id} rate={rate} samples={samples} bytes={sess.expected_bytes} bps={bps} ch={ch}")
        return

    if len(data) >= 1 and data[0:1] == b"D":
        if len(data) < 5 or not sess.active:
            return
        seq, payload_len = struct.unpack_from("<HH", data, 1)
        payload = data[5:5 + payload_len]

        if len(payload) != payload_len:
            return

        if seq != sess.next_seq:
            print(f"[WARN] out of order seq={seq} expected={sess.next_seq} (still ack)")
        else:
            sess.buf.extend(payload)
            sess.next_seq += 1

        ack = struct.pack("<cH", b"A", seq)
        await c.write_gatt_char(NUS_RX_UUID, ack, response=False)
        return

    if len(data) == 5 and data[0:1] == b"E":
        if not sess.active:
            return
        session_id, = struct.unpack_from("<I", data, 1)
        print(f"[EOT] session={session_id} got_bytes={len(sess.buf)} expected={sess.expected_bytes}")

        out_raw = f"session_{sess.session_id}.raw"
        out_wav = f"session_{sess.session_id}.wav"

        with open(out_raw, "wb") as f:
            f.write(sess.buf)

        save_wav(out_wav, bytes(sess.buf), sess.rate, sess.ch, sess.bps)
        print(f"[SAVED] {out_raw} {out_wav}")

        sess.reset()
        return

async def main():
    dev = await BleakScanner.find_device_by_filter(lambda d, ad: (d.name or "") == DEVICE_NAME, timeout=10.0)
    if dev is None:
        raise RuntimeError(f"Device '{DEVICE_NAME}' not found. Pair/connect it in Bluetooth settings first.")

    async with BleakClient(dev) as c:
        client_ref["client"] = c
        print("[BLE] connected")
        await c.start_notify(NUS_TX_UUID, handle_notify)
        print("[BLE] notify started; press and release key3 to send audio")
        while True:
            await asyncio.sleep(1)

if __name__ == "__main__":
    asyncio.run(main())