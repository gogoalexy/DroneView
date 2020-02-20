#!/bin/bash

monitor=$(tvservice -n)

# No device error will be directed to stderr not stdout
if [[ $monitor == "" ]]; then
  echo "Blind mode."
  python3 press2record.py
fi
