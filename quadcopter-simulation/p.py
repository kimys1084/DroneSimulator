import numpy as np

z = np.array([[1,2,3]])
a = np.array([[1],[2],[3]])

print z
print z.shape
w = np.transpose(z)
print a.shape

q = z+ np.transpose(z)
