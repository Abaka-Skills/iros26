"""One topic's process (started by the backend): python -m myaloha.topics NAME --port P --backend URL"""
import argparse

from myaloha.topics import TOPICS
from myaloha.topics.base import configure_logging, serve

ap = argparse.ArgumentParser(prog="python -m myaloha.topics")
ap.add_argument("name", choices=sorted(TOPICS))
ap.add_argument("--port", type=int, required=True)
ap.add_argument("--backend", default="http://127.0.0.1:8765")
args = ap.parse_args()
configure_logging()
serve(TOPICS[args.name](args.backend), args.port)
