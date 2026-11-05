"""Start one normal console with optional FPS/draw/map timing evidence.

Uses the existing manual or integrated entrypoint, receiver and decoder instances.
No synthetic data is generated. Hardware identity and optical latency are not inferred.
"""
import argparse
import os
from pathlib import Path
import sys


def observed_console(original, output):
    from .performance_probe import ConsolePerformanceProbe
    class ObservedConsole(original):
        def __init__(self,*positional,**kwargs):
            super().__init__(*positional,**kwargs)
            self._performance_probe = ConsolePerformanceProbe(self,output)
        def _on_destroy(self,*values):
            probe = getattr(self,'_performance_probe',None)
            if probe is not None:
                probe.close()
            super()._on_destroy(*values)
    return ObservedConsole


def main():
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--metrics-output', type=Path, required=True)
    parser.add_argument('--integrated', action='store_true', help='use paired operator_console entrypoint')
    args, remaining = parser.parse_known_args()
    if args.metrics_output.exists() or not args.metrics_output.parent.is_dir():
        parser.error('--metrics-output must be a new file in an existing directory')
    from . import app
    ObservedConsole = observed_console(app.OperatorConsole,args.metrics_output)
    app.OperatorConsole = ObservedConsole
    if args.integrated:
        from .__main__ import main as integrated_main
        return integrated_main(remaining)
    sys.argv = [sys.argv[0],*remaining]
    app.main()


if __name__ == '__main__':
    result = main()
    # Match the integrated entrypoint: runtime/GTK cleanup precedes the native
    # exit guard against the installed libsrt global destructor race.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if result is None else result)
