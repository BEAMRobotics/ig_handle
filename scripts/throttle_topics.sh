#!/bin/bash

set -e # exit on first error

rosrun topic_tools throttle messages /thermal/image_raw 2.0 /thermal/image_raw_throttled
