#!/bin/bash
make bin
sudo dfu-util -a 0 -d 0483:df11 -s 0x08002000:leave -D mainbms.bin 
