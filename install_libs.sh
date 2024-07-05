#!/bin/bash

libs=("libViconDataStreamSDK_CPP.so" "libboost_system-mt-x64.so.1.75.0" "libboost_thread-mt-x64.so.1.75.0" "libboost_timer-mt-x64.so.1.75.0" "libboost_chrono-mt-x64.so.1.75.0")

script_path=$(dirname $(realpath $0))
cd "$script_path"

echo "Installing shared libraries, please wait"
for lib in "${libs[@]}"; do
    # echo "Copying $lib"
    sudo cp DataStreamSDK_1.12.0/$lib /usr/lib
    echo -n "."
done

sudo chmod 0755 "${libs[@]/#/\/usr\/lib\/}"
echo -n "."
sudo ldconfig
echo -n "."
echo
echo "Installation finished!!"
