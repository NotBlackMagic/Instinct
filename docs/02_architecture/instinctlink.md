# InstinctLink Binary Protocol
The Instinct firmware uses **InstinctLink** (`Instinct/Modules/Messaging/instinctLink.hpp`) as its native, low-overhead binary bridge over UART. It handles live telemetry streaming, runtime parameter tuning, and Hardware-in-the-Loop (HITL)/Software-in-the-Loop (SITL) topic injection with **Vantage** (the PC ground station and simulator).

InstinctLink runs in an independent ThreadX thread at **500 Hz** (2 ms period) with zero dynamic allocation and up to **16 concurrent topic stream slots**. Rather than maintaining per-message serialization code, it reads and injects raw packed bytes directly through `TopicBase` (`CopyPayload()` and `PublishRaw()`), sharing the 16-bit `(instance << 8) | topicID` addressing scheme used by [PubSub](./pubsub.md), using the upper address space, addresses above `0xFF`, for protocol RPC commands. It also integrates directly with [ParamService](./parameters.md) for parameter table dumps and real-time parameter changes.

## Packet Format
All multi-byte fields are **little-endian** and **1-byte packed** (using `#pragma pack(push, 1)` in C++ and `[StructLayout(LayoutKind.Sequential, Pack = 1)]` in C#). Total frame overhead is 6 bytes, with a maximum frame size of 261 bytes.

| Byte Offset | Field | Type | Size | Description |
| :--- | :--- | :--- | :--- | :--- |
| `0` | `sync0` | `uint8` | 1 | Preamble low byte: `0xAA` |
| `1` | `sync1` | `uint8` | 1 | Preamble high byte: `0x55` (`syncWord = 0x55AA`) |
| `2..3` | `packetID` | `uint16` | 2 | Packed `(instance << 8) \| id` (low byte = ID, high byte = instance) |
| `4` | `len` | `uint8` | 1 | Payload length in bytes (`0` to `255`) |
| `5 .. 4+len` | `payload` | `uint8[len]` | `len` | Raw packed message or RPC struct |
| `5+len` | `crc` | `uint8` | 1 | 8-bit XOR checksum over `packetID`, `len`, and `payload` |

Checksum calculation:

```text
crc = frame[2] ^ frame[3] ^ frame[4] ^ payload[0] ^ ... ^ payload[len - 1]
```

## Packet Routing (`packetID`)
The high byte (`instance = packetID >> 8`) separates PubSub topic traffic from protocol control commands.

### Topic Plane (Addresses from `0x0000` to `0xFDFF`)

**Outbound (MCU to PC):** The periodic stream scheduler checks each active slot and copies data via `TopicBase::CopyPayload()`. A packet is only transmitted if the topic's `version` changed since the last transmission (message was updated). As a special case, `TopicID::ParamUpdate` (`0x00C8`) is subscribed directly and sent immediately on any parameter change serving as a parameter changed ACK.

**Inbound (PC to MCU):** Received payloads are injected straight into the matching `Broker` topic via `TopicBase::PublishRaw()` for HITL/SITL sensor injection. The received `len` must match `sizeof(T)` exactly.

### Protocol Plane (Addresses from `0xFF00` to `0xFFFF`)
Used for stream configuration, topic discovery, and parameter RPCs:

| `packetID` | Command | Direction | Request Payload | Response |
| :--- | :--- | :--- | :--- | :--- |
| `0xFF00` | `StreamConfig` | PC to MCU | `StreamConfigPayload` (4 Bytes) | None (starts, updates, or disables stream) |
| `0xFF01` | `ParamRequestList` | Bidirectional | Empty (0 Bytes) | Sequence of `0xFF01` frames with `ParamInfoPayload` (40 Bytes each) |
| `0xFF02` | `ParamRequestRead` | PC to MCU | `ParamReadPayload` (2 Bytes) | Single `0xFF01` frame with `ParamInfoPayload` (40 Bytes) |
| `0xFF03` | `ParamSet` | PC to MCU | `ParamSetPayload` (7 Bytes) | `ParamUpdateMsg` on `0x00C8` (if accepted) |
| `0xFF04` | `ParamReset` | PC to MCU | Empty (0 Bytes) | `ParamUpdateMsg` on `0x00C8` (`paramIndex = 0xFFFF`, `name = "RESET_ALL"`) |
| `0xFF05` | `TopicRequestList` | Bidirectional | Empty (0 Bytes) | Sequence of `0xFF05` frames with `TopicInfoPayload` (19 Bytes each) |

## RPC Payload Layouts
All strings are fixed-size null-terminated ASCII (`char[16]`, max 15 characters plus `\0`).

### `StreamConfigPayload` (`0xFF00`, 4 bytes)

| Offset | Field | Type | Notes |
| :--- | :--- | :--- | :--- |
| `0` | `topicID` | `uint8` | Target `TopicID` |
| `1` | `instance` | `uint8` | Target `TopicInstance` |
| `2..3` | `rate` | `uint16` | Target stream rate in Hz (`0` disables and frees slot) |

### `ParamReadPayload` (`0xFF02`, 2 bytes)

| Offset | Field | Type | Notes |
| :--- | :--- | :--- | :--- |
| `0..1` | `index` | `uint16` | Sequential parameter index (`0` to `totalParams - 1`) |

### `ParamSetPayload` (`0xFF03`, 7 bytes)

| Offset | Field | Type | Notes |
| :--- | :--- | :--- | :--- |
| `0..1` | `index` | `uint16` | Target parameter index |
| `2` | `type` | `uint8` | `0` = `Float`, `1` = `Int32` |
| `3..6` | `value` | `float` / `int32` | 4-byte `ParamValue` union |

### `ParamInfoPayload` (`0xFF01`, 40 bytes)

| Offset | Field | Type | Notes |
| :--- | :--- | :--- | :--- |
| `0..1` | `index` | `uint16` | Sequential parameter index |
| `2..5` | `hash` | `uint32` | Compile-time FNV-1a hash of `name` |
| `6` | `type` | `uint8` | `0` = `Float`, `1` = `Int32` |
| `7` | `flags` | `uint8` | Bitmask: `0x01` ReadOnly, `0x02` Protected, `0x04` Volatile |
| `8..11` | `currentVal` | `ParamValue` | Active runtime value (4 B) |
| `12..15` | `defaultVal` | `ParamValue` | Factory default value (4 B) |
| `16..19` | `minVal` | `ParamValue` | Minimum allowed value (4 B) |
| `20..23` | `maxVal` | `ParamValue` | Maximum allowed value (4 B) |
| `24..39` | `name` | `char[16]` | Null-terminated parameter name |

### `TopicInfoPayload` (`0xFF05`, 19 bytes)

| Offset | Field | Type | Notes |
| :--- | :--- | :--- | :--- |
| `0` | `topicID` | `uint8` | Registered `TopicID` |
| `1` | `instance` | `uint8` | Registered `TopicInstance` |
| `2` | `payloadSize` | `uint8` | Exact `sizeof(T)` in bytes |
| `3..18` | `name` | `char[16]` | Null-terminated topic name |