# Firmware Integration Notes

`firmware/common/` contains portable C code for the controller side of Phase 2.
It is intentionally HAL-free so it can be copied into either STM32CubeIDE or an
ESP-IDF/Arduino project.

Expected MCU loop:

```c
uint8_t rx[VGR_COMMAND_PACKET_LEN];
uint8_t tx[VGR_STATE_PACKET_LEN];
vgr_command_packet_t command;
vgr_state_packet_t state;
vgr_error_t error;

if (uart_read_exact(rx, VGR_COMMAND_PACKET_LEN)) {
    if (vgr_decode_command(rx, &command, &error)) {
        error = vgr_controller_apply(&controller, &command, millis());
    }
    state.sequence = command.sequence;
    state.state = controller.state;
    state.error = error;
    state.uptime_ms = (uint16_t)millis();
    vgr_encode_state(&state, tx);
    uart_write(tx, VGR_STATE_PACKET_LEN);
}

vgr_controller_tick(&controller, millis());
```

For the first real-board check, connect USB serial and run:

```bash
python3 -m vgr_driver.cli.certify_serial_bridge --device /dev/ttyACM0 --baudrate 115200
```

The first host command is binary, not text:

```text
a5 01 00 04 00 aa
```

That is `HEARTBEAT` with sequence `0`.
