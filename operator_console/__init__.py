"""RX-only observation console; commands use only the token-gated ops channel."""

# This package is entered before app/__main__/runtime_smoke import NumPy.
# Assign rather than setdefault: inherited laptop shell values may be >1.
import os

os.environ["OPENBLAS_NUM_THREADS"] = "1"
