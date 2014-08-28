#!/bin/bash
grep -r -n --color=always --with-filename $1 src/se306_p1_pkg/src/*
exit 0
