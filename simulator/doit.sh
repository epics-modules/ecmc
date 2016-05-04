#!/bin/sh
uname_S=$(uname -s 2>/dev/null || echo unknown)
uname_M=$(uname -m 2>/dev/null || echo unknown)
uname_R=$(uname -r 2>/dev/null | sed -e "s/[()/]/-/g"|| echo unknown)

binary=simMotor

if which valgrind >/dev/null 2>/dev/null; then
  valg='valgrind  --leak-check=full   --show-reachable=yes'
fi 
( cd .. &&
	./checkws.sh
) &&
make && $valg ${uname_S}_${uname_M}_${uname_R}/$binary "$@"

