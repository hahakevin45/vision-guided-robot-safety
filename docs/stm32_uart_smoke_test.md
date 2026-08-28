# STM32 UART Smoke Test

Use this when Phase 2 certification receives zero bytes from `/dev/ttyACM0`.

Flash a minimal firmware that only prints `VGR_READY` every 500 ms on USART2 and
toggles LD2:

```bash
python3 tools/stm32_phase2_cli.py \
  --project /path/to/generated/cubemx/project --smoke
```

If the flash step already completed but no serial data appears, reset the MCU:

```bash
st-flash reset
```

Then read text from the serial port:

```bash
python3 -m vgr_driver.cli.read_serial_text \
  --device /dev/ttyACM0 \
  --baudrate 115200 \
  --duration-s 5 \
  --report outputs/stm32_uart_smoke_read.json
```

Expected:

```text
SERIAL TEXT READ: PASS
```

and `rx_text` should contain repeated:

```text
VGR_READY
```

If this still receives no bytes, the issue is below the protocol layer:

- wrong serial device
- ST-LINK VCP not connected to USART2
- firmware not running
- board not the expected Nucleo-F446RE wiring
- another process is holding or consuming the serial port
