#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$repo_dir"
exec python3 digital_twin_viewer/sim_server.py \
  --host 0.0.0.0 \
  --port "${PORT:-8080}"
