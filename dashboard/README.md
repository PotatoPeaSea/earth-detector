# Earth Detector dashboard

Live view of the ESP32 detector over USB (Web Serial), plus a built-in simulator.

Serve this folder over localhost (Web Serial needs a secure context; `localhost` counts):

    python -m http.server 8080

Then open <http://localhost:8080/> in Chrome or Edge, click **Connect USB** and pick the CH340 port
(921600 baud). Only one program can hold the port, so close any serial monitor first.
**Simulator** runs a fake two-sensor device through the same decoder, with no hardware.

- Connecting sets DTR and RTS low straight after opening so the board is not reset. If your adapter
  resets it anyway, the dashboard re-requests the settings when the board announces itself.
- **Record** keeps the decimated samples in memory and downloads a CSV when you press Stop
  (same columns as `tools/monitor.py --csv`).
- Time view plots the filtered axis with the most energy per sensor (shown in the legend, locked while
  an event is running); the spectrum uses the x+y+z vector.

Protocol tests (Node, no dependencies):

    node test/protocol.test.js                 # round-trip tests only
    node test/protocol.test.js capture.bin     # plus a raw byte capture from the board
