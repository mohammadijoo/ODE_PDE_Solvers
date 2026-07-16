#!/usr/bin/env bash
set -euo pipefail
# Build and run the 2D heat equation example
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.heat2d.Heat2DApp exec:java
