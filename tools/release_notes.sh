#!/usr/bin/env bash
# Prints the release notes of a version: its section of CHANGELOG.md, from the heading "## <version>" to the next
# "## " heading, without the heading and the blank lines around it. Links into the repository (docs/..., README.md,
# CHANGELOG.md) become absolute, so that they work on GitHub's release page.
#   tools/release_notes.sh 0.5.8 > notes.md      # then: the text of the GitHub release v0.5.8
set -euo pipefail
version=${1:?usage: tools/release_notes.sh <version>, for example 0.5.8}
version=${version#v}
root=$(cd "$(dirname "$0")/.." && pwd)
base=https://github.com/xblick/xblick/blob/main
awk -v heading="## $version" '
    $0 == heading { found = 1; inside = 1; next }
    inside && /^## / { exit }
    inside { lines[++n] = $0 }
    END {
        if (!found) { print "no heading \"" heading "\" in CHANGELOG.md" > "/dev/stderr"; exit 1 }
        first = 1; while (first <= n && lines[first] ~ /^[ \t]*$/) first++
        last = n; while (last >= first && lines[last] ~ /^[ \t]*$/) last--
        if (first > last) { print "the section \"" heading "\" is empty" > "/dev/stderr"; exit 1 }
        for (i = first; i <= last; i++) print lines[i]
    }' "$root/CHANGELOG.md" \
    | sed -e "s#](docs/#](${base}/docs/#g" -e "s#](README.md#](${base}/README.md#g" \
          -e "s#](CHANGELOG.md#](${base}/CHANGELOG.md#g"
