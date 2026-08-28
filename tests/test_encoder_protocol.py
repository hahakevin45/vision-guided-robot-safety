from vgr_core.protocol import (
    ENCODER_PACKET_LEN,
    EncoderPacket,
    decode_encoder,
    encode_encoder,
)


def test_encoder_packet_round_trips_signed_counts():
    packet = EncoderPacket(
        sequence=9,
        left_count=-12345,
        right_count=67890,
        flags=0x03,
    )

    raw = encode_encoder(packet)
    decoded = decode_encoder(raw)

    assert len(raw) == ENCODER_PACKET_LEN
    assert decoded == packet


def test_encoder_packet_bad_checksum_rejected():
    raw = bytearray(
        encode_encoder(EncoderPacket(sequence=1, left_count=10, right_count=-10))
    )
    raw[-1] ^= 0xFF

    try:
        decode_encoder(bytes(raw))
    except ValueError as exc:
        assert "checksum" in str(exc)
    else:
        raise AssertionError("bad checksum should be rejected")
