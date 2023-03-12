#!/bin/bash

ssh -o StrictHostKeyChecking=no lvuser@10.15.38.2 killall frcUserProgram
scp -o StrictHostKeyChecking=no build/exe/frcUserProgram/linuxathena/release/frcUserProgram lvuser@10.15.38.2:~/
