# Fixed-Wing Project RPi Config

## Environmental Setup
Environmental setup for Ubuntu Server on RPi

```bash
sudo apt install python2
sudo apt install python-pip
sudo apt install build-essential zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev libreadline-dev libffi-dev wget
```

## Sync Mission Files at RPi
refs:  
1 https://www.digitalocean.com/community/tutorials/how-to-configure-ssh-key-based-authentication-on-a-linux-server
 
2 https://www.digitalocean.com/community/tutorials/how-to-use-rsync-to-sync-local-and-remote-directories

For example in my case: username@remote_host = ubuntu@192.168.43.114

SSH key-based authentication:
```bash
ssh-keygen
ssh-copy-id username@remote_host
```

Sync:
```bash
rsync -a mission/rpi_mission username@remote_host:/home/ubuntu/mission/
```