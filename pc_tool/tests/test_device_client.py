import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from PySide6.QtCore import QEventLoop, QTimer
from PySide6.QtWidgets import QApplication

from app.device_client import DeviceClient
from app.protocol.commands import Command
from app.protocol.models import DeviceStatus


class DeviceClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def wait_until(self, predicate, timeout_ms=1500):
        loop = QEventLoop()
        poll = QTimer()
        poll.setInterval(10)
        poll.timeout.connect(lambda: loop.quit() if predicate() else None)
        timeout = QTimer()
        timeout.setSingleShot(True)
        timeout.timeout.connect(loop.quit)
        poll.start()
        timeout.start(timeout_ms)
        loop.exec()
        poll.stop()
        return predicate()

    def test_simulator_request_crosses_worker_thread(self):
        client = DeviceClient()
        responses = []
        client.response.connect(lambda *args: responses.append(args))
        client.connect_device("SIMULATOR", simulator=True)
        self.assertTrue(self.wait_until(lambda: client.is_connected))
        client.request(Command.GET_STATUS)
        self.assertTrue(self.wait_until(lambda: bool(responses)))
        command, status, data = responses[0]
        self.assertEqual((command, status), (Command.GET_STATUS, 0))
        self.assertEqual(DeviceStatus.parse(data).sleep_sec, 3600)
        client.disconnect_device()


if __name__ == "__main__":
    unittest.main()
