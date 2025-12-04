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

# Store results for table
RESULT_PROGS=()
RESULT_STATUS=()

# Auto-detect programs
PROGS=()
for d in "$CORRECT_DIR"/*; do
  [ -d "$d" ] || continue
  prog="$(basename "$d")"
  if [ -f "$d/$prog.out.correct" ]; then
    PROGS+=("$prog")
  fi
done

if [ "${#PROGS[@]}" -eq 0 ]; then
  echo -e "${RED}No <prog>.out.correct files detected under 'correct/<prog>/'.${RESET}"
  exit 1
fi

echo -e "${CYAN}${BOLD}Running tests...${RESET}"
echo

for prog in "${PROGS[@]}"; do
  printf "Running %-20s ... " "$prog"

  # Run ONLY make <prog>.out, suppress make's output
  if ! make -s "${prog}.out" > /dev/null 2>&1; then
    echo -e "${YELLOW}ERROR${RESET}"
    RESULT_PROGS+=("$prog")
    RESULT_STATUS+=("ERROR")
    fail=$((fail+1))
    continue
  fi

  # Compare ONLY the .out files, suppress diff output
  if diff -u "correct/$prog/$prog.out.correct" "$OUT_DIR/$prog.out" > /dev/null 2>&1; then
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
