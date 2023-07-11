#!/bin/sh
echo "Installing shared libraries, please wait"
sudo cp libViconDataStreamSDK_CPP.so /usr/lib
echo "."
sudo cp libboost_system-mt.so.1.58.0 /usr/lib
echo "."
sudo cp libboost_thread-mt.so.1.58.0 /usr/lib
echo "."
sudo cp libboost_timer-mt.so.1.58.0 /usr/lib
echo "."
sudo cp libboost_chrono-mt.so.1.58.0 /usr/lib
echo "."
sudo chmod 0755 /usr/lib/libViconDataStreamSDK_CPP.so /usr/lib/libboost_system-mt.so.1.58.0 /usr/lib/libboost_thread-mt.so.1.58.0 /usr/lib/libboost_timer-mt.so.1.58.0 /usr/lib/libboost_chrono-mt.so.1.58.0
echo "." 
sudo ldconfig
echo "."
echo "Installlation finished"

