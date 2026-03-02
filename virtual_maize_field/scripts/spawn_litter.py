#!/usr/bin/env python3
"""Light wrapper script: invokes the real spawn_litter module.

The heavy logic lives in ``virtual_maize_field.spawn_litter`` so that the code can
be installed as a Python package and executed via ``ros2 run``. This stub
maintains compatibility for ad-hoc `python3 scripts/spawn_litter.py` launches.
"""

from virtual_maize_field.spawn_litter import main

if __name__ == '__main__':
    main()
