#!/bin/sh
# 起一个本地静态服务器并打开沙盒（config.json 需要 http:// 才能被读取）
PORT=${1:-8000}
python3 -m http.server "$PORT" >/dev/null 2>&1 &
sleep 1
open "http://localhost:$PORT/index.html" 2>/dev/null || echo "打开 http://localhost:$PORT/index.html"
wait
