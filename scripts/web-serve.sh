#!/usr/bin/env bash
# Build the browser version (scripts/web-build.sh) and serve dist/ on
# http://127.0.0.1:${PORT:-8098}/ (local only). `--no-build` serves as is.
#   scripts/web-serve.sh            build + serve
#   PROFILE=dev scripts/web-serve.sh   quicker build
set -euo pipefail
cd "$(dirname "$0")/.."
[[ ${1:-} == --no-build ]] || scripts/web-build.sh
OUT=${OUT:-dist}
PORT=${PORT:-8098}
echo "Skyward: http://127.0.0.1:$PORT/  (Ctrl-C stops)"
exec python3 - "$OUT" "$PORT" <<'PY'
import functools, http.server, sys
root, port = sys.argv[1], int(sys.argv[2])
class H(http.server.SimpleHTTPRequestHandler):
    extensions_map = {**http.server.SimpleHTTPRequestHandler.extensions_map, ".wasm": "application/wasm", ".js": "text/javascript", ".md": "text/plain; charset=utf-8", "": "text/plain; charset=utf-8"}
    def end_headers(self):
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()
    def log_message(self, *a):
        pass
http.server.ThreadingHTTPServer(("127.0.0.1", port), functools.partial(H, directory=root)).serve_forever()
PY
