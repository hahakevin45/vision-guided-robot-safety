# STM32 Phase 2 Without CubeIDE GUI

This project can flash the STM32 board from the terminal. This is the preferred
path when operating the computer remotely from a phone.

## Existing Local Project

The machine already has a CubeMX/HAL project:

```text
$STM32_PROJECT_DIR
```

It has:

- STM32F446RE target
- USART2 / `huart2`
- baudrate `115200`
- Debug makefile
- ST-LINK virtual COM path expected as `/dev/ttyACM0`

## One-Command Build And Flash

From this repo:

```bash
export STM32_PROJECT_DIR=/path/to/generated/cubemx/project
python3 tools/stm32_phase2_cli.py
```

What it does:

1. Copies Phase 2 firmware files into the CubeMX project.
2. Patches `main.c` to initialize and poll the Phase 2 UART app.
3. Patches the generated Debug makefile inputs for the new C files.
4. Builds with CubeIDE's bundled `arm-none-eabi-gcc`.
5. Converts the `.elf` to `.bin`.
6. Flashes with `st-flash`.

Build only, without flashing:

```bash
python3 tools/stm32_phase2_cli.py --project /path/to/generated/cubemx/project --skip-flash
```

## Verify After Flash

```bash
python3 -m vgr_driver.cli.diagnose_serial \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --command HEARTBEAT
```

Then:

```bash
python3 -m vgr_driver.cli.certify_serial_bridge \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --settle-s 2.0 \
  --report outputs/real_mcu_serial_certification.json
```

Expected final line:

```text
SERIAL BRIDGE CERTIFICATION: PASS
```
