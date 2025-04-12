import numpy as np
import matplotlib.pyplot as plt

# On our frame, we have T-motor MN4014 with 15x55 CF props, running 6S battery.
# From the T-motor website, the data is
throttle = np.array([0.5, 0.65, 0.75, 0.85, 1.0])
thrust_kg = np.array([1.250, 1.630, 1.950, 2.370, 2.620])
thrust_N = 9.81 * thrust_kg

p = np.polynomial.Polynomial.fit(throttle, thrust_N, deg=2)
print(f"Coefficients: {p.coef}")

# Plot for verification
x = np.arange(0, 1.0, 0.01)
y = p.coef[0] * x**2 + p.coef[1]*x + p.coef[2]
plt.plot(x,y, c='r', label='fit')
plt.scatter(throttle, thrust_N, c='b', label="data")
plt.legend()
plt.show()
