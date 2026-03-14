from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory
import os


# -----------------------------
# RPI Config
# -----------------------------
@dataclass
class Config:
    ### Model path
    # Where your model weights are saved
    pkg_share = get_package_share_directory('self_driving_pkg')
    model_path = os.path.join(pkg_share, 'models', 'model.pt')
    
    ### Motor thresholds
    max_throttle = 40
    max_angle = 90 

    