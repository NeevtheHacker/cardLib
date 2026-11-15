#!/usr/bin/env bash
# refresh_starter.sh
# Why: starter/ carries its own applied copy of cardLib, which goes stale after any library
#      change. This packages the library and re-applies it, the way a team updates.
# Owns: the refresh steps. Run from anywhere inside the repo, after changing cardLib.
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
VERSION="$(sed -nE 's/^VERSION:=[[:space:]]*([^[:space:]]+).*/\1/p' "$ROOT/cardLib/Makefile")"
if [ -z "$VERSION" ]; then
  echo "refresh_starter: no VERSION:= line in cardLib/Makefile" >&2
  exit 1
fi

echo "== packaging cardLib@$VERSION"
cd "$ROOT/cardLib"
pros make template
# Drop any cached cardLib@VERSION first: with an old copy still in the PROS cache, the fetch
# can leave the stale one in place and apply it.
pros c purge-template "cardLib@$VERSION" --force ||
  echo "refresh_starter: no cached cardLib@$VERSION to purge"
pros c fetch "cardLib@$VERSION.zip"

echo "== applying cardLib@$VERSION to starter/"
cd "$ROOT/starter"
pros c apply cardLib --force-apply

echo "== done: starter/ now has cardLib@$VERSION"
