#!/bin/bash

while true
do
  bash timeout.sh -t 1200 -i 50 -d 20 ./runQual1.sh
  sleep 10
done
