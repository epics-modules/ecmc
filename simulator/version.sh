#!/bin/sh

if test -z "$1"; then
  echo >&2 "gitversion.sh <filename>"
  exit 1
fi
>"$1" &&
printf 'const char *version_str = "' >>"$1" &&
shatoday=$(git rev-parse HEAD  | cut -b 1-9) &&

diffes=$(git diff)
if test -n "$diffes"; then
  dirty=y
else
  diffes=$(git diff --staged)
  if test -n "$diffes"; then
    dirty=y
  fi
fi
if test -n "$dirty"; then
  printf dirty-  >>"$1"
fi
printf $shatoday  >>"$1"
printf '";'    >>"$1"


