import sys
import time
import struct
import zlib
import serial

SYNC = bytes([0x55, 0xAA])
CMD_PING = 0x01
CMD_START = 0x02
CMD_DATA = 0x03
CMD_FINISH = 0x04

RESP_ACK = 0x06
RESP_NACK = 0x15

def CRC16(data: bytes) -> int:
	crc = 0xFFFF
	for b in data:
		crc ^= (b << 8)
		for _ in range(8):
			if crc & 0x8000:
				crc = ((crc << 1) ^ 0x1021) & 0xFFFF
			else:
				crc = (crc << 1) & 0xFFFF
	return crc

def FlasherRead(ser: serial.Serial, nextSequence: int, timeout: float = 2.0) -> int:
	timestamp = time.time()
	syncState = 0
	while (time.time() - timestamp) < timeout:
		byte = ser.read(1)
		if not byte:
			continue

		val = byte[0]

		if syncState == 0:
			if val == 0x55:
				syncState = 1
		elif syncState == 1:
			if val == 0xAA:
				syncState = 2
				break
			elif val == 0x55:
				syncState = 1
			else:
				syncState = 0

	if syncState != 2:
		raise TimeoutError("Timeout waiting for SYNC (0x55 0xAA)")

	# Read remaining 5 bytes with accumulated timeout loop
	payload = bytearray()
	timestamp = time.time()
	while len(payload) < 5 and (time.time() - timestamp) < 2.0:
		chunk = ser.read(5 - len(payload))
		if chunk:
			payload.extend(chunk)

	if len(payload) < 5:
		raise TimeoutError(f"Incomplete response packet received (got {len(payload)}/5 bytes)")

	respType = payload[0]
	seq = payload[1]
	errorCode = payload[2]
	rxCRC = struct.unpack("<H", payload[3:5])[0]

	calcCRC = CRC16(payload[0:3])
	if calcCRC != rxCRC:
		raise RuntimeError(f"CRC Mismatch (Calc: 0x{calcCRC:04X}, Rx: 0x{rxCRC:04X})")

	if respType != RESP_ACK:
		raise RuntimeError(f"NACK received (Sequence: {seq}, Error code: {errorCode})")

	return errorCode

def FlasherWrite(ser: serial.Serial, cmd: int, seq: int, payload: bytes):
	length = len(payload)
	header = struct.pack("<BBH", cmd, seq, length)
	crc = CRC16(header + payload)
	packet = SYNC + header + payload + struct.pack("<H", crc)
	ser.write(packet)
	ser.flush()
	FlasherRead(ser, seq)

def FlasherWriteFW(port: str, baud: int, path: str):
	with open(path, "rb") as f:
		data = f.read()

	fwSize = len(data)
	fwCRC32 = zlib.crc32(data) & 0xFFFFFFFF
	print(f"Loaded: {path} ({fwSize} bytes, CRC32: 0x{fwCRC32:08X})")

	ser = serial.Serial(port=port, baudrate=baud, timeout=0.2)
	time.sleep(0.1)
	ser.reset_input_buffer()
	ser.reset_output_buffer()

	print(f"Connecting to FSBL on {port}...")
	connected = False
	for attempt in range(1, 10):
		try:
			print(f"  Handshake attempt {attempt}/9... ", end="", flush=True)
			FlasherWrite(ser, CMD_PING, 0, b"")
			connected = True
			print("Connected!")
			break
		except Exception as e:
			print(f"Failed ({e})")
			time.sleep(0.3)

	if not connected:
		print("\nCould not connect to FSBL.")
		ser.close()
		return

	print("Starting download session...")
	payloadStart = struct.pack("<II", fwSize, fwCRC32)
	FlasherWrite(ser, CMD_START, 0, payloadStart)

	chunkSize = 512
	seq = 1
	offset = 0
	timestamp = time.time()
	while offset < fwSize:
		chunk = data[offset:offset + chunkSize]
		FlasherWrite(ser, CMD_DATA, seq & 0xFF, chunk)
		offset += len(chunk)
		seq += 1
		pct = (offset / fwSize) * 100
		speed = (offset / (time.time() - timestamp)) / 1024
		print(f"\rStreaming to PSRAM: {pct:5.1f}% [{offset}/{fwSize} bytes] @ {speed:5.1f} KB/s", end="", flush=True)

	print("\nVerifying whole-image CRC32 on target...")
	FlasherWrite(ser, CMD_FINISH, seq & 0xFF, b"")
	print(f"Staged in PSRAM in {time.time() - timestamp:.2f}s")

	print("\nTarget is flashing HyperFlash... Live logs:")
	ser.timeout = 1.0
	timestamp = time.time()
	while (time.time() - timestamp) < 25.0:
		line = ser.readline()
		if line:
			print(line.decode("utf-8", errors="replace"), end="", flush=True)

	ser.close()

if __name__ == "__main__":
	if len(sys.argv) < 3:
		# PowerShell: & py "./Boot/Source/flashUART.py" COM7 ./build/Debug-XIP/Instinct/Instinct_Signed.bin 
		print("Usage: python flashUART.py <COM_PORT> <BINARY_PATH>")
		sys.exit(1)
	FlasherWriteFW(sys.argv[1], 115200, sys.argv[2])