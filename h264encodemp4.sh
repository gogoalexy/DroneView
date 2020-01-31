#!/bin/bash

rm droneview.mp4
MP4Box -add $1:fps=60 droneview.mp4
