#!/bin/sh

# Load startup dependencies here, to ensure they're copied into RAM
# tce-load on startup does not respect copy2fs.lst, only copy2fs.flg
TCEDIR=$(/usr/bin/readlink /etc/sysconfig/tcedir)
sudo -u tc /usr/bin/tce-load -cli $(cat $TCEDIR/onstartup.lst)

# Set hostname
sethostname trackcam

/opt/bootlocal.sh &