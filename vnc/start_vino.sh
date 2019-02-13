#!/bin/bash

# To be run on remote computer

export DISPLAY=:1
gsettings set org.gnome.Vino enabled true
gsettings set org.gnome.Vino prompt-enabled false
gsettings set org.gnome.Vino require-encryption false
sleep 1
/usr/lib/vino/vino-server
