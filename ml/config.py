from dataclasses import dataclass

# -----------------------------
# ML Config
# -----------------------------
@dataclass
class Config:
    ### Paths
    # Adjust the dateset to be used here
    path: str = "datasets/2026-01-27_08-20-41"
    csv_path: str = path + "/labels/labels.csv"
    images_dir: str = path + "/images"

    ### Training hyperparameters
    # Allows for reproducible random nubmers
    seed: int = 42
    # Number of examples before updating model weights
    batch_size: int = 64 
    # Controls how big the weight updates are
    learning_rate: float = 1e-3 
    # Number of times the model has seen the dataset
    epochs: int = 1
    val_frac: float = 0.15  # last 15% used as val
    # Number of CPU proccesses preparing the data 
    num_workers: int = 2

    # saving
    save_path: str = "models/model.pt"
    
    ### Image preprocessing
    out_h: int = 66
    out_w: int = 200