#!/bin/sh

if test -z "$1"; then
  echo >&2 "today.sh <filename>"
  exit 1
fi
>"$1" &&
printf 'const char *today_str = "' >>"$1" &&
today=$(date +%y-%m-%d-%H.%M.%S) &&
printf $today  >>"$1"
printf '";'    >>"$1"


