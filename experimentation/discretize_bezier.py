import bezier
import numpy as np
import matplotlib.pyplot as plt

nodes = np.asfortranarray([
    [0.0, 0.625, 1.0],
    [0.0, 0.5  , 0.5],
])
curve = bezier.Curve(nodes, degree=2)

s_vals = np.linspace(0.0, 1.0, 5)
pts = curve.evaluate_multi(s_vals)

print(pts)

plt.figure(figsize=(8, 6))
plt.scatter(pts[0], pts[1], color='blue', label='Descretization')
plt.scatter(nodes[0], nodes[1], color='red', label='Bezier definition')
plt.grid(True)
plt.legend()
plt.axis('equal')
plt.show()

#TODO: distances not equal between points