#!/usr/bin/env bash

if [[ "$1" == "--help" ]]; then
  echo "Usage: $0 [--sort-by-name]"
  echo "Prints a table of all ports defined in config.hpp"
  echo "  --sort-by-name: Sort by name instead of port"
  exit 0
fi

# The regex
REGEX="#define ([A-Z1-9_]+) -?([0-9]+|'[A-Za-z]').*?\$"

# Things that match the regex, but we don't want to print
IGNORE_NAMES=(
  "ODOMETRY_TICKS_PER_INCH"
  "ODOMETRY_WHEEL_DIAMETER"
  "DRIVE_TRACK_WIDTH"
)

# if --sort-by-name is passed, sort by name instead of port
SORT_COLUMN=2
if [[ "$1" == "--sort-by-name" ]]; then
  SORT_COLUMN=1
fi

# Read ./src/config.hpp or ../src/config.hpp (whichever exists)
FILE_CONTENTS=$(cat ./src/config.hpp 2>/dev/null || cat ../src/config.hpp 2>/dev/null)

# Find all lines that match the regex:
# "#define (?<name>[A-Z1-9_]+) (-)?(?<port>[0-9]+)"
# and print as a table using column
echo "$FILE_CONTENTS" \
  | grep -P "$REGEX" \
  | sed -E "s/$REGEX/\1 \2/" \
  | grep -vE "$(IFS='|'; echo "${IGNORE_NAMES[*]}")" \
  | sort -k$SORT_COLUMN -n \
  | sed -E "s/\s/,/g" \
  | sed -E "s/_/ /g" \
  | sed -E "s/([A-Z])([A-Z]?+)(\s)?/\1\L\2\3/g" \
  | column -t -N "Name,Port" -s "," -o "    "
