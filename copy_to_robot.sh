#!/bin/bash

rsync -azvh --exclude build --exclude devel --exclude docs * grupo7@192.168.1.129:~/g7_project

[ $? -eq 0 ] && echo "Copied to Robot!"

# if [ $USER =  rafael ]; then
#     USER=luke
#     IP=192.168.1.129
# else
#     USER=rafael
#     IP=192.168.1.113
# fi

# rsync -azvh --exclude build --exclude devel --exclude docs * $USER@$IP:~/MobileRoboticsLab

# [ $? -eq 0 ] && echo "Copied to Teammate!"
