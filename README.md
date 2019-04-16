# K.Iot.ClimateStation

Simple, small and distributed climate station.

## Working with project

Just clone repository and open project in [PlatformIO IDE](https://platformio.org/platformio-ide).

## Troubleshooting

### Get right COM-port

PlatformIO can enumerate all connected devices within their COM-port, ID and description.
Just enter:

```bash
pio device list
```

### pio shows no device :c

Try to install latest
[CP210x USB to UART drivers](https://www.silabs.com/community/interface/forum.topic.html/cp210x_usb_to_uartd-cuKd)
or [CH340 USB to UART drivers](https://sparks.gogo.co.nz/ch340.html).
