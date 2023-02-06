# RPi Mission

This branch is for deployment purposes on the Mission Controller board like RPi.


## Running

Determine the serial device ArduPilot autopilot
```bash
ls /dev/serial/by-id
```

My output:
```bash
pi@raspberrypi:~/fixedwing-mission $ ls /dev/serial/by-id/
usb-ArduPilot_fmuv2_2A004C001951303330343831-if00
```

```bash
python main.py --port <dev-id>
```

My \<dev-id\> is "/dev/serial/by-id/usb-ArduPilot_fmuv2_2A004C001951303330343831-if00"

## Cron Job

```bash
sudo crontab -e
```

Add:

```bash
@reboot sh /home/pi/fixedwing-mission/launch.sh >/tmp/fixedwing-mission/cronlog 2>&1
```
    