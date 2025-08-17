#!/bin/sh
readelf -d $1 | grep SONAME | awk '{print $5}' | awk -Fso. '{print $2}' | awk -F] '{print $1}'
