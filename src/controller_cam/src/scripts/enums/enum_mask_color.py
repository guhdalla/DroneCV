#!/usr/bin/env python3

from enum import Enum
import numpy as np

class EnumColorMask(Enum):

    GREEN = {"L": np.array([40, 70, 70]), "U": np.array([90, 210, 200])},
    GRAY = {"L": np.array([0, 0, 0]), "U": np.array([126, 25, 35])}

