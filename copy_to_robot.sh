#!/bin/bash

rsync -azvh --exclude build --exclude devel --exclude docs * grupo7@192.168.1.129:~/g7_project