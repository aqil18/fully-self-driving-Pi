from dataclasses import dataclass

# -----------------------------
# RPI Config
# -----------------------------
@dataclass
class Config:
    ### Model path
    # Where your model weights are saved
    model_path = 'rpi/src/self_driving_pkg/self_driving_pkg/models/model.pt' 
    
    ### Motor thresholds
    max_throttle = 40
    max_angle = 90 

    