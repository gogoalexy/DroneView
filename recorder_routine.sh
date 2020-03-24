#!/bin/bash

DE=$XDG_CURRENT_DESKTOP

# No device error will be directed to stderr not stdout
if [[ $DE == "" ]]; then
  echo "Blind mode."
  python3 ~/repo/DroneView/press2record.py
fi
