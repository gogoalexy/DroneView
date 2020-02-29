#!/bin/bash

infilename=$1
fps=30
if [ $2 ]; then
	fps=$2
fi
outfilename=${infilename/%.h264/.mp4}
MP4Box -add ${infilename}:fps=${fps} ${outfilename}
