# STM32N6570-DK ↔ Cube Orange — MAVLink UART Interoperability Report

> **From:** ST Microelectronics / STM32N6 Developer  
> **To:** ArduPilot / Cube Orange Team  
> **Status:** Unresolved — Seeking guidance on MAVLink stream compatibility

---

## Table of Contents

1. [Context & Objective](#context--objective)
2. [Hardware Setup](#hardware-setup)
3. [Problem Statement](#problem-statement)
4. [What Was Observed](#what-was-observed)
5. [What Was Tried](#what-was-tried)
6. [MAVLink-Specific Issues](#mavlink-specific-issues)
7. [Open Questions for the ArduPilot Team](#open-questions-for-the-ardupilot-team)
8. [Expected Outcome](#expected-outcome)
9. [Environment](#environment)

---

## Context & Objective

This project attempts to establish a **reliable MAVLink communication link** between an **STM32N6570-DK** discovery board (running bare-metal HAL firmware) and a **Cube Orange** autopilot running ArduPilot.

The STM32N6 is connected to the Cube Orange via UART (USART2, 57600 baud — default Cube Orange TELEM rate). The board is expected to:

1. Receive a continuous MAVLink telemetry stream from the Cube Orange
2. Parse incoming frames using the standard MAVLink C library (common dialect)
3. Extract GPS data (`GPS_RAW_INT`) and heartbeat (`HEARTBEAT`) messages

This use case is representative of a **companion computer or smart peripheral** scenario, where an STM32 acts as a MAVLink-aware node alongside an ArduPilot autopilot.

---

## Hardware Setup

| Element | Detail |
|---|---|
| MCU Board | STM32N6570-DK (Cortex-M55, STM32N657X0HXQ) |
| Autopilot | Cube Orange running ArduPilot |
| Connection | UART — USART2 on STM32N6 → TELEM port on Cube Orange |
| Baud rate | 57600 (default Cube Orange) — 115200 also tested |
| MAVLink dialect | Common (v2) |
| Debug output | USART1 (VCP) |

### Wiring Diagram

```
  STM32N6570-DK                          Cube Orange
  ┌───────────────────────┐              ┌──────────────────────┐
  │                       │              │                      │
  │  USART2               │              │  TELEM2              │
  │                       │     ╔════╗   │  (6-pin JST-GH)      │
  │  PD5  (TX) ───────────┼─────╢ X  ╟──┼─ Pin 2 (TX out)      │
  │                       │     ║    ║   │                      │
  │  PF6  (RX) ───────────┼─────╢ X  ╟──┼─ Pin 3 (RX in)       │
  │                       │     ╚════╝   │                      │
  │  GND ─────────────────┼──────────────┼─ Pin 6 (GND)         │
  │                       │   crossed    │                      │
  │  USART1 (VCP)         │              │  USB                 │
  │  → USB debug console  │              │  → Mission Planner   │
  └───────────────────────┘              └──────────────────────┘

  Cube Orange TELEM1/TELEM2 pinout (JST-GH 6-pin):
  ┌─────┬──────────┬──────────────────────────────┐
  │ Pin │ Signal   │ Notes                        │
  ├─────┼──────────┼──────────────────────────────┤
  │  1  │ VCC 5V   │ ⚠️  Do NOT connect to STM32   │
  │  2  │ TX (OUT) │ → connects to STM32 PF6 (RX) │
  │  3  │ RX (IN)  │ → connects to STM32 PD5 (TX) │
  │  4  │ CTS      │ Not used                     │
  │  5  │ RTS      │ Not used                     │
  │  6  │ GND      │ → connects to STM32 GND      │
  └─────┴──────────┴──────────────────────────────┘
```

> ⚠️ **TX → RX / RX → TX** — wires must be crossed. Connecting TX→TX causes the STM32 to read back its own transmitted frames (`sysid=255` loopback) — this exact mistake was encountered during development.
>
> ⚠️ **No level shifter required** — both devices operate at 3.3V logic on UART.
>
> ⚠️ **Do NOT connect the Cube Orange 5V pin** to the STM32 board.

---

## Problem Statement

Despite correct physical wiring and baud rate configuration, **the STM32N6 receives bytes but cannot parse any valid MAVLink message** from the Cube Orange.

Diagnostic counters collected over 5-second windows show:

| Metric | Value |
|---|---|
| Bytes received / 5s | ~8000 ✅ |
| MAVLink messages parsed | 0 ❌ |
| UART errors / 5s | ~5000 ❌ |

Bytes are clearly arriving — the physical link is functional. However, the MAVLink parser (`mavlink_parse_char`) never produces a valid message, and the UART peripheral is overwhelmed with errors.

---

## What Was Observed

### Observation 1 — Byte rate is consistent with a MAVLink stream

At 57600 baud, ~8000 bytes/5s corresponds to approximately **1600 bytes/s**, which is consistent with a Cube Orange TELEM port streaming GPS + heartbeat + attitude messages at standard rates. The data is arriving.

### Observation 2 — Zero valid MAVLink frames parsed

Despite feeding every received byte into `mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)`, no message is ever successfully decoded. This suggests either:

- **Byte loss** between reception and parsing (buffer overruns dropping bytes mid-frame)
- **Frame synchronization loss** — the parser never finds a valid `0xFD` / `0xFE` start-of-frame marker
- **MAVLink version or dialect mismatch** — the Cube Orange may be sending frames in a format not matched by the C library version in use

### Observation 3 — UART overrun cascade

The STM32N6 HAL reports thousands of `HAL_UART_ERROR_ORE` (overrun) errors per second. This means the hardware receive register is being overwritten before the CPU can read it — a classic symptom of interrupt-based reception being too slow for the incoming data rate.

A contributing factor was identified: a blocking debug `HAL_UART_Transmit` call inside `HAL_UART_ErrorCallback`, which stalled the CPU for ~1ms per error, creating a feedback loop. Removing it reduced errors significantly but did not eliminate them, nor did it restore MAVLink parsing.

---

## What Was Tried

### Attempt 1 — Byte-by-byte interrupt reception

`HAL_UART_Receive_IT(&huart2, &rx_byte, 1)` re-armed in each RX callback. This approach cannot sustain 57600 baud with back-to-back MAVLink frames. Overruns occur when the Cube Orange sends bursts of multiple messages in rapid succession.

**Result:** Bytes received, 0 messages parsed, >5000 UART errors/5s.

### Attempt 2 — Remove blocking calls from error callback

Identified that `DBG()` inside `HAL_UART_ErrorCallback` was calling `HAL_UART_Transmit` in blocking/polling mode, causing ~1ms CPU stall per error and multiplying overruns. Removed all blocking calls from the error path.

**Result:** Error count reduced but not eliminated. MAVLink parsing still 0.

### Attempt 3 — DMA circular reception with `HAL_UARTEx_ReceiveToIdle_DMA`

Switched to DMA-based reception using GPDMA1 (the only DMA controller available on STM32N6, replacing the legacy DMA1/DMA2 of previous STM32 families). Reception is triggered once with a 512-byte circular buffer and re-armed in the `HAL_UARTEx_RxEventCallback`.

```c
HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, RX_DMA_BUF_SIZE);

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint16_t i = 0; i < Size; i++) {
        if (mavlink_parse_char(MAVLINK_COMM_0, rx_dma_buf[i], &msg, &status)) {
            // process message
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, rx_dma_buf, RX_DMA_BUF_SIZE);
}
```

**Result:** DMA reception reduces overrun errors dramatically. However, **MAVLink parsing still returns 0 valid messages**, which shifts the focus from the reception layer to the **MAVLink frame content and structure** itself.

---

## MAVLink-Specific Issues

With DMA overruns largely eliminated, the parser receives a continuous, uninterrupted byte stream — yet still produces no valid frames. The following MAVLink-level issues are suspected:

### Issue 1 — MAVLink version detection

The Cube Orange TELEM port may output **MAVLink v2 frames** (`0xFD` start byte) while the STM32 firmware was initially tested with a MAVLink library expecting v1 frames (`0xFE`). The `mavlink_parse_char` function handles both, but only if the library is compiled without `MAVLINK_COMM_NUM_BUFFERS` restrictions that could disable v2 support.

**Question for ArduPilot team:** What exact MAVLink version and start byte does the Cube Orange output on TELEM at default configuration?

### Issue 2 — Stream activation (`REQUEST_DATA_STREAM` / `SET_MESSAGE_INTERVAL`)

ArduPilot autopilots may not stream messages by default to an unknown companion device. Some TELEM ports require the connected device to send a `REQUEST_DATA_STREAM` or `SET_MESSAGE_INTERVAL` command before the autopilot begins transmitting GPS and telemetry data at the expected rate.

If the STM32N6 never sends such a request, the Cube Orange may be sending only **heartbeats** (1 Hz, ~17 bytes/frame) — far fewer bytes than the ~1600 bytes/s observed, suggesting something else is also being transmitted without being requested.

**Question for ArduPilot team:** What is transmitted by default on TELEM ports without a prior handshake? Is a `REQUEST_DATA_STREAM` required to receive `GPS_RAW_INT`? What is the default stream set and rate?

### Issue 3 — System ID / Component ID and message targeting

The MAVLink C library can silently discard messages if system ID or component ID filtering is active. The Cube Orange typically uses `sysid=1`, `compid=1`. The STM32 parser uses `MAVLINK_COMM_0` which by default accepts all system IDs — but this should be verified against the actual ArduPilot output.

**Question for ArduPilot team:** Does the Cube Orange emit messages with a specific `target_system` / `target_component` that could cause them to be discarded by a generic MAVLink parser?

### Issue 4 — Frame alignment loss after overrun

Even a single dropped byte during an overrun event causes the MAVLink parser to lose frame synchronization. The parser must then wait for the next valid start byte (`0xFD`) to re-sync. If the Cube Orange sends back-to-back frames with no idle gap, the parser may never re-sync once alignment is lost — especially at startup when the first bytes are often missed.

**Question for ArduPilot team:** Is there a minimum inter-frame gap in the Cube Orange UART output? Can the output rate be configured to allow a slower embedded device to recover frame synchronization?

### Issue 5 — MAVLink C library version compatibility

The common MAVLink C headers have evolved significantly. Some versions include breaking changes in `mavlink_status_t` or `MAVLINK_COMM_0` buffer management that can silently prevent successful parsing.

**Question for ArduPilot team:** What version of the MAVLink C library (commit or tag) is confirmed compatible with the frames emitted by the current Cube Orange ArduPilot firmware?

---

## Open Questions for the ArduPilot Team

| # | Question | Priority |
|---|---|---|
| 1 | What MAVLink version (v1/v2) and start byte does Cube Orange TELEM output by default? | 🔴 Critical |
| 2 | Is `REQUEST_DATA_STREAM` or `SET_MESSAGE_INTERVAL` required before `GPS_RAW_INT` is emitted? | 🔴 Critical |
| 3 | What is the default message set and emission rate on a Cube Orange TELEM port at 57600? | 🔴 Critical |
| 4 | Is there a minimum inter-frame idle gap, or are frames emitted continuously back-to-back? | 🟡 Important |
| 5 | Does ArduPilot set `target_system`/`target_component` in streamed messages that could cause filtering? | 🟡 Important |
| 6 | What MAVLink C library version is confirmed compatible with current Cube Orange firmware? | 🟡 Important |
| 7 | Is there an existing reference implementation for a bare-metal STM32 node receiving MAVLink from ArduPilot? | 🟢 Nice to have |

---

## Expected Outcome

The expected working scenario is:

1. Cube Orange emits a MAVLink stream on TELEM port (GPS + heartbeat at minimum)
2. STM32N6 receives bytes continuously via GPDMA1 DMA circular buffer
3. `mavlink_parse_char` successfully decodes frames and returns `GPS_RAW_INT` messages
4. GPS coordinates are extracted and available to the STM32 application layer

This is a standard companion-computer integration pattern. Any guidance from the ArduPilot team on the exact frame format, streaming configuration, required handshake procedure, and confirmed MAVLink library version would allow this integration to be completed and serve as a reference for future STM32 ↔ ArduPilot projects.

---

## Environment

| Tool / Component | Version / Detail |
|---|---|
| STM32CubeIDE | 2.1.1 |
| STM32CubeN6 firmware package | April 2026 |
| GNU Arm Toolchain | 14.3.rel1 |
| Target MCU | STM32N657X0HXQ (Cortex-M55) |
| MAVLink C library | common dialect, MAVLINK_COMM_NUM_BUFFERS=1 |
| Cube Orange firmware | ArduPilot (version to be confirmed) |
| TELEM port used | TELEM2 (assumed) |
| Baud rate | 57600 (default Cube Orange) — 115200 also tested |
| OS | Windows 11 |
