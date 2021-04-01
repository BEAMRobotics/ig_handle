#!/bin/bash

set -e # exit on first error

echo "Starting ig_handle"
now=$(date +"%Y-%m-%d-%H-%M-%S")

main()
{
  echo "Starting launch files..."
  roslaunch ig_handle includes.launch
}

main