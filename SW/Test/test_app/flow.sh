#!/bin/bash

exit 0


./waf configure

# Test robot.
./waf build && ./build/test_servos w 0 100
./waf build && ./build/test_servos w 0 500
./waf build && ./build/test_servos w 0 900

# Test chassis.
# Speed.
./waf build && ./build/test_bldc +20
./waf build && ./build/test_bldc -20
# Steer.
./waf build && ./build/test_servos w 1 500
./waf build && ./build/test_servos w 1 650
./waf build && ./build/test_servos w 1 350
