import sys
import time
import struct
import zlib
import serial

SYNC = bytes([0x55, 0xAA])

CMD_PING = 0x01
CMD_SET_BAUD = 0x02
CMD_START = 0x03
CMD_DATA = 0x04
CMD_FINISH = 0x05

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

def FlasherRead(ser: serial.Serial, expectedSequence: int, timeout: float = 2.0) -> int:
	timestamp = time.perf_counter()
	syncState = 0
	while(time.perf_counter() - timestamp) < timeout:
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
	timestamp = time.perf_counter()
	while len(payload) < 5 and (time.perf_counter() - timestamp) < timeout:
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
		raise RuntimeError(f"CRC Mismatch (Calc: 0x{calcCRC:04X}, RX: 0x{rxCRC:04X})")

	if seq != expectedSequence:
		raise RuntimeError(f"Sequence Mismatch (Expected: {expectedSequence}, RX: {seq})")

	if respType != RESP_ACK:
		raise RuntimeError(f"NACK received (Sequence: {seq}, Error code: {errorCode})")

	return errorCode

def FlasherWrite(ser: serial.Serial, cmd: int, seq: int, payload: bytes, timeout: float = 2.0, maxRetries: int = 3):
	length = len(payload)
	header = struct.pack("<BBH", cmd, seq, length)
	crc = CRC16(header + payload)
	packet = SYNC + header + payload + struct.pack("<H", crc)

	lastError = None
	for attempt in range(1, maxRetries + 1):
		try:
			if attempt > 1:
				ser.reset_input_buffer()
			ser.write(packet)
			ser.flush()
			FlasherRead(ser, seq, timeout=timeout)
			return
		except Exception as e:
			lastError = e
			if attempt < maxRetries:
				time.sleep(0.05)

	raise RuntimeError(f"Command 0x{cmd:02X} (seq {seq}) failed after {maxRetries} attempts: {lastError}")

def FlasherSetBaud(ser: serial.Serial, newBaud: int, seq: int = 0):
	payloadBaud = struct.pack("<I", newBaud)
	# Send SetBaud and receive ACK at current baudrate
	FlasherWrite(ser, CMD_SET_BAUD, seq, payloadBaud, timeout=1.0, maxRetries=3)

	# Switch host serial baudrate and allow USB-UART bridge + MCU USART to settle
	ser.baudrate = newBaud
	time.sleep(0.03)
	ser.reset_input_buffer()

	# Verify communication at the new baudrate
	FlasherWrite(ser, CMD_PING, (seq + 1) & 0xFF, b"", timeout=0.5, maxRetries=3)

def FlasherWriteFW(port: str, defaultBaud: int, transferBaud: int, path: str):
	with open(path, "rb") as f:
		data = f.read()

	fwSize = len(data)
	fwCRC32 = zlib.crc32(data) & 0xFFFFFFFF
	print(f"Loaded: {path} ({fwSize} bytes, CRC32: 0x{fwCRC32:08X})")

	ser = serial.Serial(port=port, baudrate=defaultBaud, timeout=0.2)
	time.sleep(0.1)
	ser.reset_input_buffer()
	ser.reset_output_buffer()

	print(f"Connecting to FSBL on {port} @ {defaultBaud} bps...")
	connected = False
	for attempt in range(1, 10):
		try:
			print(f"  Handshake attempt {attempt}/9... ", end="", flush=True)
			FlasherWrite(ser, CMD_PING, 0, b"", timeout=0.5, maxRetries=1)
			connected = True
			print("Connected!")
			break
		except Exception as e:
			print(f"Failed ({e})")
			ser.reset_input_buffer()
			time.sleep(0.25)

	if not connected:
		print("\nCould not connect to FSBL.")
		ser.close()
		return

	try:
		if transferBaud != defaultBaud:
			print(f"Switching baudrate to {transferBaud} bps... ", end="", flush=True)
			FlasherSetBaud(ser, transferBaud, seq=0)
			print("OK!")

		print("Starting download session...")
		payloadStart = struct.pack("<II", fwSize, fwCRC32)
		FlasherWrite(ser, CMD_START, 0, payloadStart, timeout=1.0)

		chunkSize = 512
		seq = 1
		offset = 0
		timestamp = time.perf_counter()

		while offset < fwSize:
			chunk = data[offset:offset + chunkSize]
			FlasherWrite(ser, CMD_DATA, seq & 0xFF, chunk, timeout=1.0, maxRetries=4)
			offset += len(chunk)
			seq += 1
			elapsed = max(time.perf_counter() - timestamp, 1e-6)
			pct = (offset / fwSize) * 100
			speed = (offset / elapsed) / 1024
			print(f"\rStreaming to PSRAM: {pct:5.1f}% [{offset}/{fwSize} bytes] @ {speed:5.1f} KB/s", end="", flush=True)

		print("\nVerifying whole-image CRC32 on target...")
		# Whole-image CRC32 in software over several MBs can take a few hundred ms, allow a 5s timeout
		FlasherWrite(ser, CMD_FINISH, seq & 0xFF, b"", timeout=5.0, maxRetries=1)
		print(f"Staged in PSRAM in {time.perf_counter() - timestamp:.2f}s")

	finally:
		# Target restores defaultBaud after Finish, Error, or Timeout
		if ser.baudrate != defaultBaud:
			time.sleep(0.02)
			ser.baudrate = defaultBaud
			ser.reset_input_buffer()

	print(f"\nTarget is flashing HyperFlash (console @ {defaultBaud} bps)... Live logs:")
	ser.timeout = 0.5
	timestamp = time.perf_counter()
	try:
		while (time.perf_counter() - timestamp) < 25.0:
			line = ser.readline()
			if line:
				decodedLine = line.decode("utf-8", errors="replace")
				print(decodedLine, end="", flush=True)
	except KeyboardInterrupt:
		print("\nLog monitoring stopped by user.")
	finally:
		ser.close()

if __name__ == "__main__":
	if len(sys.argv) < 3:
		# PowerShell: & py "./Boot/Source/flashUART.py" COM7 ./build/Debug-XIP/Instinct/Instinct_Signed.bin [BAUD]
		print("Usage: python flashUART.py <COM_PORT> <BINARY_PATH> [TRANSFER_BAUD]")
		sys.exit(1)

	portArg = sys.argv[1]
	pathArg = sys.argv[2]
	transferBaudArg = int(sys.argv[3]) if len(sys.argv) >= 4 else 921600

	FlasherWriteFW(portArg, 115200, transferBaudArg, pathArg)