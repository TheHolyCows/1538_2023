#!/bin/bash

ssh lvuser@10.15.38.2 killall frcUserProgram
scp build/exe/frcUserProgram/linuxathena/release/frcUserProgram lvuser@10.15.38.2:~/
