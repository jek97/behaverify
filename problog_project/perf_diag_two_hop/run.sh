#!/usr/bin/env bash
# perf_diag_two_hop/run.sh
#
# Reproduces the problem3 Bug0 performance investigation's key result:
# chaining TWO full evaluations of the tree (see two_hop_diag.pl)
# times out ProbLog's exact inference at the problem's default 5x5x5
# noise table (125 discrete noise combinations per hop), but succeeds
# at reduced tables with only ONE active noise axis -- 3x1x1 (3
# combinations/hop) and 5x1x1 (5/hop) both succeed -- while ANY config
# with TWO OR MORE active axes times out, regardless of how few values
# each one takes: 3x3x1 (9/hop, two axes), 3x3x3 (27/hop, three axes),
# and the 5x5x5 default (125/hop, three axes) all time out alike. So
# it's specifically the NUMBER OF ACTIVE NOISE AXES that drives the
# blowup, not the per-axis value count or the raw combination count --
# 5x1x1 (5 combos, one axis) is tractable, 3x3x1 (9 combos, two axes)
# is not, despite 9 being a smaller raw world count than 5.
#
# What this does, for EACH of the noise configs in this directory:
#   1. Copies that config over problems/problem3/config.yaml.
#   2. Regenerates problems/problem3/config_generated.pl from it.
#   3. Strips module/theory/basic_action_theory.pl's own default
#      QUERIES section (its ~48 report queries pull in a different,
#      unrelated cost -- see two_hop_diag.pl's own header) and appends
#      two_hop_diag.pl in its place.
#   4. Runs `problog` against it with a 5-minute timeout.
#   5. Reverts all three files via `git checkout`, whatever happened.
#
# Usage (from anywhere):
#   bash perf_diag_two_hop/run.sh
#
# Refuses to run if basic_action_theory.pl / problem3's config.yaml /
# config_generated.pl already have uncommitted changes, so it can
# never clobber real work -- commit or stash first if it complains.

set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."   # repo root

THEORY=module/theory/basic_action_theory.pl
CONFIG=problems/problem3/config.yaml
CONFIG_GEN=problems/problem3/config_generated.pl
DIAG_DIR=perf_diag_two_hop

for f in "$THEORY" "$CONFIG" "$CONFIG_GEN"; do
    if ! git diff --quiet -- "$f"; then
        echo "Refusing to run: $f has uncommitted changes -- commit or stash first." >&2
        exit 1
    fi
done

cleanup() {
    git checkout -- "$THEORY" "$CONFIG" "$CONFIG_GEN"
}
trap cleanup EXIT

run_one() {
    local label="$1" config_src="$2"

    echo "=================================================================="
    echo "  $label"
    echo "=================================================================="

    cp "$config_src" "$CONFIG"
    python3 -c "
import sys
sys.path.insert(0, 'module/translators')
from config_to_prolog import generate
generate('$CONFIG', '$CONFIG_GEN')
"

    # Replace the default QUERIES section (from its own "10. QUERIES"
    # header to end of file) with this diagnostic's single query.
    sed -i '/^% 10\. QUERIES$/,$d' "$THEORY"
    cat "$DIAG_DIR/two_hop_diag.pl" >> "$THEORY"

    set +e
    time BT_PROBLEM_DIR="$(pwd)/problems/problem3" timeout 300 problog "$THEORY"
    local code=$?
    set -e
    echo "---- exit code: $code (124 = hit the 5-minute timeout) ----"

    git checkout -- "$THEORY" "$CONFIG" "$CONFIG_GEN"
    echo
}

run_one "3x1x1 noise -- 3 combinations/hop, ONE active axis (position only)" "$DIAG_DIR/config_noise_3.yaml"
run_one "5x1x1 noise -- 5 combinations/hop, ONE active axis (position only)" "$DIAG_DIR/config_noise_511.yaml"
run_one "3x3x1 noise -- 9 combinations/hop, TWO active axes (position+tangential)" "$DIAG_DIR/config_noise_331.yaml"
run_one "3x3x3 noise -- 27 combinations/hop, THREE active axes (3 values each)" "$DIAG_DIR/config_noise_333.yaml"
run_one "5x5x5 noise -- 125 combinations/hop, THREE active axes (this problem's default)" "$DIAG_DIR/config_noise_555.yaml"
