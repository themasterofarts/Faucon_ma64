"""Configuration pytest du depot.

Rend le module faucon_common importable depuis les sources, pour lancer les
tests sans avoir installe le workspace (colcon build). Si le workspace est
source, la version installee (identique) est utilisee de la meme facon.
"""
import os
import sys

_COMMON = os.path.join(os.path.dirname(os.path.abspath(__file__)), "faucon_common")
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
