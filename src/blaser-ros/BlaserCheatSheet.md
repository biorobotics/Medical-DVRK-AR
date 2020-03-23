# BlaserMini 2.0 HW Setup

## Setup Blaser USB-Ethernet connection

If you are using ubuntu, connect USB port to your computer and in
your network dropdown menu, click `Edit Connections` and click `Add`.

Select type `Ethernet` and click `Create`.

Name the connection "`Share Ethernet`", go to tab `Ethernet`, select
blaser (D6:E9:01:60:2A:A2 for A1001) as Device, then go to `IPv4` tab
and choose `Link-Local Only` as Method.

Click `Save`, in a couple seconds you should see connection established.
Then you can try pinging the blaser device:

Note that we use BlaserMiniA1001 here as our ID but that will
change across different blaser instances. Check the label on blaser
hardware for actual device ID.

```bash
ping BlaserMiniA1001.local
```

If ping goes through, you are set! Proceed to log in section.

(Follow BlaserMiniSWSetupGuide.pdf for more details)

## Log into Blaser onboard Linux

### Establish ssh connection
```bash
ssh pi@BlaserMiniA1001.local
```
(Password: biotrobotics)

### open up camera and laser process

```bash
screen
```

```bash
cd blaserctl
```

```bash
$ sudo ./stream_laser_1280x960.sh
```

(# turn off streaming process just hit `CTRL+C`)

(# shut down blaser Linux: `sudo shutdown -h now` )

Once streaming is running, you will be able to view the streamed video
from blaser camera at this url:
`http://blaserminia1001.local:8080/?action=stream`

If you can see the streamed video then you have completed the hardware setup
step!

## Next steps
Go back to README.md for software setup and demo instructions.

## Appendix

### List blaser network status
```bash
avahi-browse -art
```

### Manually turn Laser/LED on or off on blaser Linux
```bash
cd ~/blaserctl
# [Laser_ON, LED_ON]
sudo python blaserctl_pwm.py 100 100
# [Laser_ON, LED_OFF]
sudo python blaserctl_pwm.py 100 0
# [Laser_OFF, LED_OFF]
sudo python blaserctl_pwm.py 0 0
# Values can be adjusted between [0, 100] to achieve continuous intensity control (coming soon tm)
```
