#!/usr/bin/env bash
# Build a phase image. Usage: containers/build.sh [humble-classic]
set -euo pipefail
cd "$(dirname "$0")/.."

flavor="${1:-humble-classic}"
git submodule update --init --recursive
podman build -f "containers/${flavor}.Containerfile" -t "jezero-lab:${flavor}" .
