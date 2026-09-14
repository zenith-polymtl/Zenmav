import os
import sys

# ZENMAV_SRC lets the same tests run against another copy of the sources (e.g. a baseline)
SRC = os.environ.get("ZENMAV_SRC", os.path.join(os.path.dirname(__file__), "..", "src"))
sys.path.insert(0, os.path.abspath(SRC))
