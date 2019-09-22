import math


def approx(a, b, negl=1e-6):
    return math.fabs(a - b) <= negl
