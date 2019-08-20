i = 5
b = i.to_bytes(4, byteorder="little", signed=True)
j = 3
b0 = j.to_bytes(4, byteorder="little", signed=True)

b += b0

print(b)
print(type(b))
