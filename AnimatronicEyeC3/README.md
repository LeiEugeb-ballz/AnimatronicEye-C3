# Animatronic Eye — ESP32-C3 Final Controller

Self-contained firmware for the single-eye ESP32-C3 SuperMini build. Web control
is primary, the original motion set has been rebuilt as safe nonblocking
animations, and dual analogue joysticks remain optional.

## Confirmed wiring

| Function | ESP32-C3 pin |
| --- | ---: |
| Top eyelid servo signal | GPIO7 |
| Bottom eyelid servo signal | GPIO5 |
| Horizontal eye servo signal | GPIO6 |
| Vertical eye servo signal | GPIO10 |
| Joystick 1 X | GPIO0 (ADC) |
| Joystick 1 Y | GPIO1 (ADC) |
| Joystick 2 X | GPIO3 (ADC) |
| Joystick 2 Y | GPIO4 (ADC) |
| Optional joystick 1 switch | GPIO20 |
| Optional joystick 2 switch | GPIO21 |
| Joystick supply | 3V3 only |
| Joystick ground | GND |

The four servo assignments above were physically identified and tested. GPIO5
was repaired and confirmed working. GPIO2, GPIO8 and GPIO9 remain unused because
they are ESP32-C3 boot-strapping pins.

If the salvaged joystick board has only eight wires total and no button outputs,
leave GPIO20 and GPIO21 unconnected. Verify its traces or measure the pins before
connecting it. Never feed a C3 ADC input from a 5 V joystick rail.

The external servo supply ground and ESP32 ground **must be common**. Do not
power the servos from the ESP32 board. STOP detaches PWM control signals; it does
not physically remove external servo power.

## Network access

Connect directly to the board over its Wi-Fi access point.

- SSID: `AnimatronicEye`
- Password: `eyecontrol32`
- Address: `http://192.168.4.1`
- Recovery interface: `http://192.168.4.1/basic`

The board runs in AP-only mode.

## Web UI

- Current firmware: `2.2.0-ap`.
- **Control:** touch XY gaze pad, independent eyelids, Web/Classic/Dual modes.
- **Perform:** Blink, Scan, Diagonal, Roll, Centre continuous Random Show, Desk mode, and Nap now.
- **Calibrate:** guarded per-servo endpoints plus four-axis joystick capture.
- **Setup:** AP status, firmware and device telemetry.

The four section tabs work independently of the controller script. The full page includes the script and streams directly from flash. The radar pad sends live gaze updates during a drag and holds the last position on release. The recovery UI at `/basic` works without JavaScript.

Servo signals always boot detached. Animations never arm outputs automatically.
Press **Enable outputs** explicitly, and use the persistent red STOP button to
cancel motion and detach every channel.

## Desk mode

Enable all outputs, then select **Perform → Desk mode**. The eye holds a series
of small, varied glances and sometimes blinks. After roughly three to six minutes
awake it droops shut for one minute, wakes, and goes back to looking around.
The sequence runs on the ESP32-C3 while the browser is closed.

**Nap now** starts the nap immediately. If Desk mode is active, it resumes after
waking. A radar-pad command, **End show + centre**, or **STOP / DETACH** ends
Desk mode. Servo motion follows the saved calibration for all four channels.

## Joystick modes

- **Joystick 1 / Classic:** J1 controls horizontal and vertical gaze; the lids
  follow vertical motion.
- **Dual stick:** J1 controls gaze; J2 controls the two eyelids.
- Either optional switch triggers a blink.

Web mode remains the startup default because disconnected analogue inputs float.
Before selecting a joystick mode, capture centres with both sticks released,
then capture Min and Max for each axis and set inversion where required.

## Build and upload

```powershell
pio run
pio run --target upload --upload-port COM21
```

Use the COM port assigned to your board. The downloadable application image is intended for an existing installation with the same partition table. The factory image includes the bootloader and partition table. The final source
is the single file `src/AnimatronicEyeC3/AnimatronicEyeC3.ino`.
