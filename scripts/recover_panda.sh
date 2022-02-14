#!/bin/bash
echo "Recovering Panda; Sit tight and wait for reboot..."
pkill -f openpilot
cd /data/openpilot/panda/board
./recover.sh
echo "Recovering Panda; If no reboot in 5 min try again.."
sudo reboot
reboot