apt update
rosdep fix-permissions && rosdep update 
cd robot_ws 
rosdep install --from-paths src --ignore-src -r -y
colcon build --build-base arm64_build --install-base arm64_install
colcon bundle --build-base arm64_build --install-base arm64_install --bundle-base arm64_bundle --apt-sources-list /opt/cross/apt-sources.yaml
