import math


n = 5
m = 4

o = 0
for i in range(0, n):
    o += math.factorial(i) * (n ** m)

print(o)
