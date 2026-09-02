#!/bin/bash
gnome-terminal -- bash -c '
cd /home/qiqi/sp_vision_hero_18
while true; do
    ./build/standard_mpc
    sleep 2
done
exec bash
'
