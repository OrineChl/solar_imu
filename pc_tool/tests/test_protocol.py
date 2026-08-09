import struct
import unittest

from app.protocol.codec import FrameParser, encode_frame
from app.protocol.commands import Command, config_payload
from app.protocol.models import DeviceIdentity, DeviceStatus, EventRecord, ImuLive
from app.transport.simulator import Simulator


class ProtocolTests(unittest.TestCase):
    def test_known_get_status_frame(self):
        self.assertEqual(encode_frame(Command.GET_STATUS, 1).hex(" ").upper(),
                         "55 AA 01 01 01 00 00 D9 FA 0D 0A")

    def test_parser_handles_noise_and_split_frames(self):
        packet = encode_frame(Command.GET_IMU_LIVE | 0x80, 9, b"\x00" + bytes(16))
        parser = FrameParser()
        self.assertEqual(parser.feed(b"noise" + packet[:7]), [])
        frames = parser.feed(packet[7:] + b"tail")
        self.assertEqual(len(frames), 1)
        self.assertEqual(frames[0].sequence, 9)

    def test_config_is_exact_firmware_layout(self):
        self.assertEqual(config_payload(750, 30, 3600, 3550),
                         struct.pack("<HHIH", 750, 30, 3600, 3550))

    def test_simulator_models(self):
        sim = Simulator()
        status = DeviceStatus.parse(sim.transact(Command.GET_STATUS, b"")[1])
        self.assertEqual(status.sleep_sec, 3600)
        identity = DeviceIdentity.parse(sim.transact(Command.GET_DEVICE_ID, b"")[1])
        self.assertTrue(identity.imei_valid)
        live = ImuLive.parse(sim.transact(Command.GET_IMU_LIVE, b"")[1])
        self.assertGreater(live.acc_z, 900)
        queue = sim.transact(Command.READ_QUEUE, b"")[1]
        self.assertEqual(EventRecord.parse(queue[2:]).event_id, 17)


if __name__ == "__main__":
    unittest.main()
