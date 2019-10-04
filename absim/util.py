import math


def approx(a, b, negl=1e-6):
    """Gets if two integral types are approximately equivalent.

    Parameters
    ----------
    a : int, float
        lhs
    b : int, float
        rhs
    negl : int, float
        maximum allowed difference

    Return
    ------
    bool
        if a ~= b
    """
    return math.fabs(a - b) <= negl
