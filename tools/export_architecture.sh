#!/usr/bin/env bash
# Export the architecture diagram (the only ```mermaid block in docs/how_it_works.md) to
# docs/media/architecture.svg. Needs node (npx fetches @mermaid-js/mermaid-cli) and a Chrome for puppeteer.
#   tools/export_architecture.sh [chrome-executable]   (default /usr/bin/google-chrome)
set -euo pipefail
cd "$(dirname "$0")/.."
chrome=${1:-/usr/bin/google-chrome}
tmp=$(mktemp -d log/arch.XXXXXX)
trap 'rm -rf "$tmp"' EXIT
awk '/^```mermaid$/{f=1; next} /^```$/{if (f) exit} f' docs/how_it_works.md > "$tmp/architecture.mmd"
[ -s "$tmp/architecture.mmd" ] || { echo "no mermaid block in docs/how_it_works.md" >&2; exit 1; }
printf '{"executablePath": "%s", "args": ["--no-sandbox"]}\n' "$chrome" > "$tmp/puppeteer.json"
npx -y @mermaid-js/mermaid-cli@11.17.0 -i "$tmp/architecture.mmd" -o docs/media/architecture.svg -b white \
    -p "$tmp/puppeteer.json"
echo "wrote docs/media/architecture.svg"
