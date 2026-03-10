from dataclasses import dataclass

# -----------------------------
# RPI Config
# -----------------------------
@dataclass
class Config:
    ### Paths
    # Adjust the dateset to be used here
    path: str = "datasets/2026-01-27_08-20-41"
    csv_path: str = path + "/labels/labels.csv"
    images_dir: str = path + "/images"


    ### Motor thresholds
    max_throttle = 40
    max_angle = 90 