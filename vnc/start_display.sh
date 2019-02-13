#!/bin/bash

# To be run on remote computer

export DISPLAY=:1
Xvfb :1 -screen 0 1024x768x16 &
sleep 1

#/etc/X11/xinit/xinitrc &
#/etc/X11/Xsession &
#exec gnome-session
exec lightdm-session &
#sudo service lightdm restart
