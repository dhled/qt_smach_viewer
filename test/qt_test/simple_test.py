#!/usr/bin/env python
from pytestqt import qt_compat
from pytestqt.qt_compat import qt_api

from qt_smach_viewer.viewer import SmachViewerWidget

def test_basics(qtbot, qapp):
    """
    Basic test that works more like a sanity check to ensure we are setting up a QApplication
    properly and are able to display a simple event_recorder.
    """
    widget = SmachViewerWidget(None)
    qtbot.addWidget(widget)
    widget.setWindowTitle("W1")
    assert widget.windowTitle() == "W1"
    widget.OnQuit(None)
