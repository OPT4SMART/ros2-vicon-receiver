#!/bin/sh
echo "Uninstalling shared libraries, please wait"
sudo rm /usr/lib/libViconDataStreamSDK_CPP.so
echo "."
sudo rm /usr/lib/libboost_system-mt.so.1.58.0
echo "."
sudo rm /usr/lib/libboost_thread-mt.so.1.58.0
echo "."
sudo rm /usr/lib/libboost_timer-mt.so.1.58.0
echo "."
sudo rm /usr/lib/libboost_chrono-mt.so.1.58.0
echo "."
sudo ldconfig
echo "."
echo "Uninstalllation finished"

