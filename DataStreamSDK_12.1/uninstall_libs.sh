#!/bin/sh
echo "Uninstalling shared libraries, please wait"
sudo rm /usr/lib/libViconDataStreamSDK_CPP.so
echo "."
sudo rm /usr/lib/libboost_system-mt-x64.so.1.75.0
echo "."
sudo rm /usr/lib/libboost_thread-mt-x64.so.1.75.0
echo "."
sudo rm /usr/lib/libboost_timer-mt-x64.so.1.75.0
echo "."
sudo rm /usr/lib/libboost_chrono-mt-x64.so.1.75.0
echo "."
sudo ldconfig
echo "."
echo "Uninstalllation finished"

