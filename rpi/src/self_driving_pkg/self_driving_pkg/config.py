from dataclasses import dataclass

# -----------------------------
# RPI Config
# -----------------------------
@dataclass
class Config:
    ### Model path
    # Where your model weights are saved
    model_path = './models/model.pt' 

    ### Motor thresholds
    max_throttle = 40
    max_angle = 90 

    