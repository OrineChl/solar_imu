import sys
from pathlib import Path

from PySide6.QtWidgets import QApplication

from app.ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Solar IMU Tool")
    theme = Path(__file__).resolve().parent / "ui" / "theme.qss"
    if theme.exists():
        app.setStyleSheet(theme.read_text(encoding="utf-8"))
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
