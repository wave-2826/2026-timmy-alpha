import os
import json
import matplotlib.pyplot as plt
import numpy as np

SHOT_TYPE_MAP_PATH = os.path.join(os.path.dirname(__file__), "../../../logs/shot_type_map.csv")

with open(SHOT_TYPE_MAP_PATH, "r") as f:
    lines = f.readlines() # X, Y, ShotType
    lines = [tuple(line.strip().split(",")) for line in lines[1:]] # [(X, Y, ShotType), ...]

FIELD_CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.json")

with open(FIELD_CONFIG_PATH, "r") as f:
    config = json.load(f)

def inches_to_meters(inches: float) -> float:
    return inches * 0.0254

top_left: tuple[int, int] = tuple(config["topLeft"])
bottom_right: tuple[int, int] = tuple(config["bottomRight"])
width_meters: float = inches_to_meters(config["widthInches"])
height_meters: float = inches_to_meters(config["heightInches"])

def field_to_image(field_x: float, field_y: float) -> tuple[float, float]:
    return (
        field_x / width_meters * (bottom_right[0] - top_left[0]) + top_left[0],
        field_y / height_meters * (bottom_right[1] - top_left[1]) + top_left[1],
    )

# Draw `image.png` as the background
img = plt.imread(os.path.join(os.path.dirname(__file__), "image.png"))

plt.rcParams['figure.subplot.left'] = 0.05
plt.rcParams['figure.subplot.right'] = 0.95
plt.rcParams['figure.subplot.top'] = 0.95
plt.rcParams['figure.subplot.bottom'] = 0.05

plt.imshow(img)

# Overlay an alpha layer with the nearest datapoint from lines colored by shot type
shot_types = set(line[2] for line in lines)

x = np.array([float(line[0]) / width_meters * (bottom_right[0] - top_left[0]) + top_left[0] for line in lines])
y = np.array([float(line[1]) / height_meters * (bottom_right[1] - top_left[1]) + top_left[1] for line in lines])
shot_type_to_int = {shot_type: i for i, shot_type in enumerate(shot_types)}
shot_type_ints = np.array([plt.cm.Paired(shot_type_to_int[line[2]]) for line in lines])
plt.scatter(x, y, c=shot_type_ints, s=10, alpha=1)

# Add to legend
handles = [plt.Line2D([0], [0], marker='o', color='w', label=shot_type, markersize=10, markerfacecolor=plt.cm.Paired(shot_type_to_int[shot_type])) for shot_type in shot_types]
plt.legend(handles=handles, loc='upper right')

plt.show()