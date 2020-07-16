#!/usr/bin/env python
import numpy as np
import math
import matplotlib.pyplot as plt
from squaternion import quat2euler


a = quat2euler(0, 0, 0, 1, degrees=True)
print a
print a[0]
print a[2]