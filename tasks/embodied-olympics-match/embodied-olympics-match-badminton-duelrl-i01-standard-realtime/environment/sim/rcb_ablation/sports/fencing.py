"""Fencing (format bout, phrase by phrase against the scripted fencer): the no-success-signal edits only.

The bout has no physical state between phrases (every phrase starts on guard; the observation already holds both
fencers' intentions, delays and the phrase's length), so there is no state table: the state query is not offered.
Left out under no success signal: the score (also after each phrase), each phrase's result and the black card (it
ends the bout with a loss)."""
from __future__ import annotations

HIDE_KEYS = ("score_after", "how")


def hide(obj):
    def walk(x):
        if isinstance(x, dict):
            return {k: walk(v) for k, v in x.items() if not (k == "card" and str(v).startswith("P-black"))}
        if isinstance(x, list):
            return [walk(v) for v in x]
        return x
    return walk(obj)
