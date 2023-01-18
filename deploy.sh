#!/bin/bash

scp src/main/deploy/* lvuser@10.15.38.2:/home/lvuser/deploy/
ssh lvuser@10.15.38.2 killall frcUserProgram
scp build/exe/frcUserProgram/release/frcUserProgram lvuser@10.15.38.2:~/
