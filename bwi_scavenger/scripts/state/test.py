from state import *


s = SystemStateVector()
s["t_now"] = 4
s["t_abort"] = 6

t = StateTransition("t_now", LT, "t_abort")
print(t.eval(s))
