# Hexapod PC Bluetooth GUI

## Setup

1. Flash the ESP32 firmware.
2. In Windows Bluetooth settings, pair with `Hexapod-Control`.
3. Windows should create a Bluetooth COM port.
4. Create and use the local GUI Python environment:

```powershell
cd C:\Users\USER\Documents\PlatformIO\Projects\Initial\pc_gui
py -3.12 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
```

5. Run the GUI:

```powershell
.\.venv\Scripts\python.exe robot_gui.py
```

## Finding the Bluetooth COM Port

Do not choose the USB cable port. In your screenshot, `COM12 - Silicon Labs CP210x...` is the USB serial chip, not Bluetooth.

To find the Bluetooth COM port:

1. Open Windows Settings.
2. Go to `Bluetooth & devices`.
3. Pair with `Hexapod-Control`.
4. Open `More Bluetooth settings`.
5. Open the `COM Ports` tab.
6. Look for `Standard Serial over Bluetooth link`.
7. Use the `Outgoing` COM port in the GUI.

If there is no Bluetooth COM port:

- Remove/unpair `Hexapod-Control`, then pair again.
- Make sure the ESP32 firmware with Bluetooth Serial is flashed and powered.
- Restart Bluetooth on the laptop.
- Some laptops show the COM port only after pairing is fully complete.

## Commands Sent

- Forward: `f` or `fN`
- Rotate right: `r` or `rN`
- Rotate left: `l` or `lN`
- Home: `h`
- Lift leg: `1` to `6`
- Help: `?`
