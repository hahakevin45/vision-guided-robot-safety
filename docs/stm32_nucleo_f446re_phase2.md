# STM32 Nucleo-F446RE Phase 2 Bring-Up

The current host diagnostic proves Linux can open `/dev/ttyACM0` and transmit
the first binary heartbeat packet:

```text
a5 01 00 04 00 aa
```

The failure is that STM32 returns no bytes. The board firmware must read this
6-byte command packet and transmit the current 10-byte state packet.

## CubeIDE Setup

Target board assumption: STM32 Nucleo-F446RE.

Use the ST-LINK virtual COM port UART:

- USART2
- PA2: `USART2_TX`
- PA3: `USART2_RX`
- Baudrate: `115200`
- Word length: 8 bits
- Parity: none
- Stop bits: 1
- Hardware flow control: none

Copy these files into the CubeIDE project:

```text
firmware/common/protocol.h
firmware/common/protocol.c
firmware/common/state_machine.h
firmware/common/state_machine.c
firmware/common/motor_output.h
firmware/common/motor_output.c
firmware/stm32_hal/stm32_phase2_app.h
firmware/stm32_hal/stm32_phase2_app.c
firmware/stm32_hal/stm32_motor_driver.h
firmware/stm32_hal/stm32_motor_driver.c
firmware/stm32_hal/stm32_encoder.h
firmware/stm32_hal/stm32_encoder.c
```

If CubeIDE keeps source files under `Core/Src` and headers under `Core/Inc`,
either preserve the folder structure or adjust the include paths.

## main.c Integration

In `main.c`, include the app header:

```c
#include "stm32_phase2_app.h"
```

After `MX_USART2_UART_Init();`, initialize the app:

```c
stm32_phase2_app_init(&huart2);
```

Inside the main loop:

```c
while (1)
{
    stm32_phase2_app_poll();
}
```

## Expected Host Result

After flashing the board, run:

```bash
python3 -m vgr_driver.cli.diagnose_serial \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --command HEARTBEAT \
  --report outputs/stm32_serial_diagnostic.json
```

Expected result:

```text
SERIAL DIAGNOSTIC: PASS
```

Then run the full certification:

```bash
python3 -m vgr_driver.cli.certify_serial_bridge \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --settle-s 2.0 \
  --report outputs/real_mcu_serial_certification.json
```

Expected command/state sequence:

| Command | MCU State | Error |
| --- | --- | --- |
| `HEARTBEAT` | `ARMED` | `OK` |
| `FORWARD` | `TRACKING` | `OK` |
| `TURN_LEFT` | `TRACKING` | `OK` |
| `TURN_RIGHT` | `TRACKING` | `OK` |
| `STOP` | `SAFE_STOP` | `OK` |

## If It Still Times Out

- Confirm `/dev/ttyACM0` disappears and reappears when the Nucleo USB cable is unplugged.
- Confirm CubeIDE generated `MX_USART2_UART_Init()` for `115200 8N1`.
- Confirm `stm32_phase2_app_poll()` is actually called in the `while (1)` loop.
- Confirm the firmware was flashed after adding the app files.
- If another serial monitor is open, close it before running the Python tool.
