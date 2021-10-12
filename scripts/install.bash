#!/bin/bash
set -e

apt-get install python3-venv
python3 -m venv .venv
pip install -r requirements.txt