# Fixed-Wing Project RPi Config

Install Ubuntu Server or Raspbian Lite for RPi, firstly.

>**📝 Note:**
For Raspbian you can follow the instructions:
https://medium.com/@jrcharney/connect-your-raspberry-pi-to-your-computer-via-ethernet-4564e1e68922

## Connection

Connect your RPi to network and find its IP and username. Then:

```bash
ssh username@remote_host
```

For example in my case: username@remote_host = ubuntu@192.168.43.114 or pi@raspberrypi.local

## Environmental Setup for Ubuntu Server
Environmental setup for Ubuntu Server on RPi. 

Development packs for Ubuntu Server:
```bash
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget 
sudo apt install libgl1 libxml2 libxslt1.1 libxslt1-dev
```

for Python:
```bash
sudo apt install python2 python2-dev
sudo apt install python-pip
```

python2 packages:
```pip2
opencv-python<=4.2.0.32
```

python3 packages:
```pip
opencv-python
dronekit
```

```bash
sudo -H pip2 install wheel
sudo -H pip2 install dronekit==2.9.1
sudo -H pip2 install dronekit-sitl==3.2.0
sudo -H pip2 install opencv-python-headless==4.2.0.32
```

## Environmental Setup for Rasbpian Lite

Development packs:
```bash
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget 
sudo apt install libgl1 libxml2 libxslt1.1 libxslt1-dev
sudo apt install python3 python3-dev python3-pip
```

Pip modules:
```bash
sudo -H pip install opencv-python-headless imutils dronekit pyserial
```

MavProxy installation
```bash
sudo apt install python3-wxgtk4.0 python3-matplotlib python3-lxml python3-pygame
sudo -H pip install PyYAML mavproxy
```


>**📝 Note:**
For OpenCV installlation on GUI'ness systems remove the "-headless" flag at the end to get <imshow()> like GUI functions.
This flag is for servers with no GUI.

>**📝 Note:**
"sudo -H" is for adding scripts to "PATH".

## Sync Mission Files at RPi

SSH key-based authentication:
```bash
ssh-keygen
ssh-copy-id username@remote_host
```

Sync:
```bash
rsync -a mission/rpi_mission username@remote_host:/home/<user>/mission/
```

\<user\> is the username of the remote

## References

*   SSH Key Based Auth, https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server
 
*   RSync to remote file manuplations, https://www.digitalocean.com/community/tutorials/how-to-use-rsync-to-sync-local-and-remote-directories

* OpenCV readme, https://github.com/opencv/opencv-python
