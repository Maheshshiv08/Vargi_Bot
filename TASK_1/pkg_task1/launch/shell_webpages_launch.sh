#!/bin/bash

# Store URL in a variable
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1nIMUv1CZJfc4_oQ9J8y3AkKVSKILN69ectViZNo70pw/edit#gid=0"
URL3="https://docs.google.com/spreadsheets/d/1qnOXov_XgPT1Yxera6XSlFQuTd1HLHsRX06L9waSojM/edit#gid=0"

# Print some message
echo "** Opening $URL1 in Chrome **"
echo "** Opening $URL2 in Chrome **"
echo "** Opening $URL3 in Chrome **"


# Use firefox to open the URL in a new window
firefox -new-window $URL1 $URL2 $URL3

