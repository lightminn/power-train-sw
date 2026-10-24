"""Timing assertions shared by synthetic execution/performance harnesses."""


def callback_gap(start, end, beats):
    """Include both boundaries: absent callbacks cannot look like zero delay."""
    return max([end-start] if not beats else
               [beats[0]-start, end-beats[-1],
                *(b-a for a, b in zip(beats, beats[1:]))])
