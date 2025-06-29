#!/bin/bash
cd /home/sandberrypi/sandscape/src/
python3 -m venv .venv
source .venv/bin/activate
echo "in the virtual env"
ls -lah
python main.py