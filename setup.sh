#!/bin/bash
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y