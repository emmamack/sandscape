import bezier
import numpy as np
import matplotlib.pyplot as plt
import math

# nodes = np.asfortranarray([
#     [0.0, 0.625, 1.0],
#     [0.0, 0.5  , 0.5],
# ])
nodes = np.asfortranarray([
    [0.0, 0, 1.0],
    [0.0, 1  , 1],
])
curve = bezier.Curve(nodes, degree=2)

s_vals = np.linspace(0.0, 1.0, 20)
pts = curve.evaluate_multi(s_vals)

print(pts)


for ind in [1, 2, 3, 4, 5, 6, 7, 8, 9]:
    dist = math.sqrt((pts[0][ind] - pts[0][ind-1])**2 + (pts[1][ind] - pts[1][ind-1])**2)
    print (dist)

plt.figure(figsize=(8, 6))
plt.scatter(pts[0], pts[1], color='blue', label='Descretization')
plt.scatter(nodes[0], nodes[1], color='red', label='Bezier definition')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()


# TODO: distances not equal between points
# ...this might be surprisingly difficult to fix, and might not be worth it: 
# https://math.stackexchange.com/questions/321293/find-coordinates-of-equidistant-points-in-bezier-curve