#!/usr/bin/env python3
"""Execution-only entry; the preview CLI intentionally has no execution flag."""
from test_live_pipeline import main

if __name__ == '__main__':
    raise SystemExit(main(execute=True))
