#!/bin/sh
# 起一个本地静态服务器并打开沙盒（visitor.json 需要 http:// 才能被读取）
cd "$(dirname "$0")"
PORT=${1:-8000}
python3 -m http.server "$PORT" >/dev/null 2>&1 &
sleep 1
open "http://localhost:$PORT/visitor.html" 2>/dev/null || echo "打开 http://localhost:$PORT/visitor.html"
wait
