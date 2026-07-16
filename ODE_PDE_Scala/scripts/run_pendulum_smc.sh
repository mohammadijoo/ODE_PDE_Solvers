#!/usr/bin/env bash
set -euo pipefail
# Build and run the cart-pole sliding mode control example
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."
mvn -q -DskipTests package
mvn -q -Dexec.mainClass=com.mohammadijoo.odepde.pendulum_sliding_mode.CartPoleSmcApp exec:java
