from dataclasses import dataclass


# -----------------------------
# RPI Config
# -----------------------------
@dataclass
class Config:
    ### Model path
    model_path = "/home/aqil/fully-self-driving-Pi/rpi/src/self_driving_pkg/self_driving_pkg/models/model.pt"
    
    ### Motor thresholds
    max_throttle = 40
    max_angle = 90 

    