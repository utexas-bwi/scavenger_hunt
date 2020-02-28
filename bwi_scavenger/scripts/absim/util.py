import math


def approx(a, b, negl=1e-6):
    """Returns if two integral types are approximately equal.

    Parameters
    ----------
    a
        rhs
    b
        lhs
    negl
        maximum allowed difference to be considered approximately equal

    Returns
    -------
    bool
        |a-b| <= negl
    """
    return math.fabs(a - b) <= negl
