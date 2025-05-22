import json
import matplotlib.pyplot as plt
import numpy as np

# Load the JSON file
with open("ransac_output.json", "r") as f:
    data = json.load(f)

# Extract points and lines
points = data["points"]
lines = data["lines"]

# Plot points
x_points = [p["x"] for p in points]
y_points = [p["y"] for p in points]

print(len(x_points), len(y_points))
plt.scatter(x_points, y_points, color="blue", label="Points")

# Plot lines
# x_range = np.linspace(0, 15, 100)  # Adjust range as needed
# for i, line in enumerate(lines):
#     m = line["m"]
#     c = line["c"]
#     y_line = m * x_range + c
#     plt.plot(x_range, y_line, label=f"Line {i + 1}: y = {m:.2f}x + {c:.2f}")

# Configure plot
plt.title("RANSAC Detected Lines")
plt.xlabel("X")
plt.ylabel("Y")
plt.xlim(0, 15)
plt.ylim(-14, 14)
plt.legend()
plt.grid()
plt.show()