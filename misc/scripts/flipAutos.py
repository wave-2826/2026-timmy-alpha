# Enumerate .traj files in src/main/deploy/choreo
import os
import json

FIELD_WIDTH_Y = 8.043

def flip_y(y):
    return FIELD_WIDTH_Y - point[1]
def flip_heading(heading):
    return -heading
def flip_expression_value_y(val):
    # e.g. {"exp":"7.464865684509277 m", "val":7.464865684509277}
    if isinstance(val, (int, float)):
        return FIELD_WIDTH_Y - val
    elif isinstance(val, dict) and "val" in val:
        flipped_val = FIELD_WIDTH_Y - val["val"]
        return {"exp": f"{flipped_val} m", "val": flipped_val}
    else:
        raise ValueError(f"Unsupported value type for flipping: {val}")
def flip_expression_value_heading(val):
    if isinstance(val, (int, float)):
        return -val
    elif isinstance(val, dict) and "val" in val:
        flipped_val = -val["val"]
        return {"exp": f"{flipped_val} rad", "val": flipped_val}
    else:
        raise ValueError(f"Unsupported value type for flipping: {val}")

choreoDir = "src/main/deploy/choreo"
for filename in os.listdir(choreoDir):
    if filename.endswith(".traj") and filename.startswith("Left"):
        newfilename = "Right" + filename[4:-5] + "Generated.traj"
        print(f"Flipping {filename} into {newfilename}")
        
        with open(os.path.join(choreoDir, filename), "r") as f:
            data = json.load(f)

        data["name"] = newfilename[:-5]

        # TODO: Flip constraints
        for wp in data["params"]["waypoints"]:
            # just use val and create expression from it
            wp["y"] = flip_expression_value_y(wp["y"])
            wp["heading"] = flip_expression_value_heading(wp["heading"])
        
        with open(os.path.join(choreoDir, newfilename), "w") as f:
            json.dump(data, f, indent=4)
