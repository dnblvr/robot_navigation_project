import json
import matplotlib.pyplot as plt
import numpy as np
import os
from PIL import Image

# Load debug data from JSON file
with open("ransac_debug.json", "r") as f:
    debug_data = json.load(f)

# Example datapoints (replace with actual points used in C code)
points = np.array([
    [1.0, 2.1], [2.0, 4.0], [3.0, 6.1], [4.0, 8.0],  # Inliers
    [5.0, 10.0], [6.0, 12.0], [7.0, 14.0], [8.0, 16.0]
])

# Create output directory for frames
os.makedirs("frames", exist_ok=True)

# Plot each iteration
frames = []
for i, entry in enumerate(debug_data):
    print(f"Processing iteration {i+1}/{len(debug_data)}...")
    
    # Extract model parameters
    m, c = entry["m"], entry["c"]

    # Generate line points
    x = np.linspace(0, 10, 100)
    y = m*x + c

    # Plot points and line
    plt.figure(figsize=(8, 6))
    plt.scatter(points[:, 0], points[:, 1], color="blue", label="Data Points")
    plt.plot(x, y, color="red", label=f"Iteration {i}: y = {m:.2f}x + {c:.2f}")
    plt.xlim(0, 10)
    plt.ylim(0, 20)
    plt.legend()
    plt.title(f"RANSAC Iteration {i}")
    plt.xlabel("X")
    plt.ylabel("Y")

    # Save frame
    frame_path = f"frames/frame_{i:03d}.png"
    plt.savefig(frame_path)
    frames.append(frame_path)
    plt.close()

# Create GIF
images = [Image.open(frame) for frame in frames]
images[0].save("ransac_iterations.gif", save_all=True, append_images=images[1:], duration=100, loop=0)

print("GIF saved as ransac_iterations.gif")