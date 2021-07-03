#!/usr/bin/bash

arecord -D hw:Prod -f S16_LE -r 192000 -d 1 record.wav

