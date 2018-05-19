#!/bin/bash

ffmpeg -i input.ogv \
       -c:v libx264 -preset veryslow -crf 22 \
       -c:a libmp3lame -qscale:a 2 -ac 2 -ar 0 \
       output.mp4