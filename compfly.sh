#!/bin/bash

cd flight_controller
gcc -o week6 week6.cpp -lwiringPi -lm
./week6
