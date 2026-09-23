#!/usr/bin/env python3
"""PyInstaller entry for Zerith H1 Pro Dora node."""

from __future__ import annotations

import multiprocessing

multiprocessing.freeze_support()

from zerith_h1_pro.main import main

if __name__ == "__main__":
    main()
