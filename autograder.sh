#!/usr/bin/env bash
cd "$(dirname "$0")"

CORRECT_DIR="correct"
OUT_DIR="output"

# Colors
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
CYAN="\033[36m"
BOLD="\033[1m"
RESET="\033[0m"

pass=0
fail=0

# Default comparison: .out
COMPARE_EXT="out"      # extension we compare (out or wb)
COMPARE_DESC=".out"    # human-readable label

# Optional flag: -w -> compare .wb instead of .out
if [[ "$1" == "-w" ]]; then
  COMPARE_EXT="wb"
  COMPARE_DESC=".wb"
  shift
fi

# Store results for table
RESULT_PROGS=()
RESULT_STATUS=()

# Auto-detect programs based on presence of correct/<prog>/<prog>.<ext>.correct
PROGS=()
for d in "$CORRECT_DIR"/*; do
  [ -d "$d" ] || continue
  prog="$(basename "$d")"
  if [ -f "$d/$prog.$COMPARE_EXT.correct" ]; then
    PROGS+=("$prog")
  fi
done

if [ "${#PROGS[@]}" -eq 0 ]; then
  echo -e "${RED}No <prog>${COMPARE_DESC}.correct files detected under 'correct/<prog>/'.${RESET}"
  exit 1
fi

echo -e "${CYAN}${BOLD}Running tests (comparing ${COMPARE_DESC} files)...${RESET}"
echo

for prog in "${PROGS[@]}"; do
  printf "Running %-20s ... " "$prog"

  # Always build <prog>.out (this run should generate both .out and .wb in output/)
  if ! make -s "${prog}.out" > /dev/null 2>&1; then
    echo -e "${YELLOW}ERROR${RESET}"
    RESULT_PROGS+=("$prog")
    RESULT_STATUS+=("ERROR")
    fail=$((fail+1))
    continue
  fi

  golden="correct/$prog/$prog.$COMPARE_EXT.correct"
  actual="$OUT_DIR/$prog.$COMPARE_EXT"

  # Compare the chosen files (.out or .wb), suppress diff output
  if diff -u "$golden" "$actual" > /dev/null 2>&1; then
    echo -e "${GREEN}PASS${RESET}"
    RESULT_PROGS+=("$prog")
    RESULT_STATUS+=("PASS")
    pass=$((pass+1))
  else
    echo -e "${RED}FAIL${RESET}"
    RESULT_PROGS+=("$prog")
    RESULT_STATUS+=("FAIL")
    fail=$((fail+1))
  fi
done

echo
echo -e "${CYAN}${BOLD}Results:${RESET}"
printf '%-20s | %-10s\n' "Program" "Result"
printf '%-20s-+-%-10s\n' "--------------------" "----------"

for i in "${!RESULT_PROGS[@]}"; do
  prog="${RESULT_PROGS[$i]}"
  status="${RESULT_STATUS[$i]}"

  case "$status" in
    PASS)
      color="$GREEN"
      ;;
    FAIL)
      color="$RED"
      ;;
    ERROR)
      color="$YELLOW"
      ;;
    *)
      color="$RESET"
      ;;
  esac

  printf '%-20s | %b%-10s%b\n' "$prog" "$color" "$status" "$RESET"
done

echo
if [ "$fail" -eq 0 ]; then
  echo -e "${GREEN}${BOLD}Summary:${RESET} ${GREEN}$pass passed, $fail failed${RESET}"
else
  echo -e "${RED}${BOLD}Summary:${RESET} ${GREEN}$pass passed${RESET}, ${RED}$fail failed${RESET}"
fi
