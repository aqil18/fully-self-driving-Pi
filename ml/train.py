import os
import cv2
import math
import pandas as pd
import numpy as np
from preprocessor import PreProcessor 
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
from config import Config
from pipilotnet import PiPilotNet


cfg = Config()


# -----------------------------
# Reproducibility
# -----------------------------
def set_seed(seed: int):
    import random
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.cuda.manual_seed_all(seed)


# -----------------------------
# Dataset
# -----------------------------
class DrivingDataset(Dataset):
    def __init__(self, df: pd.DataFrame, images_dir: str, preprossor: PreProcessor, augment: bool):
        self.df = df.reset_index(drop=True)
        self.images_dir = images_dir
        self.augment = augment
        self.preprocessor = preprossor

    def __len__(self):
        return len(self.df)

    ### Adjusts some examples to counteract some results
    ### ! This may do more harm than good
    def _augment(self, rgb: np.ndarray, steering: float):
        # brightness jitter
        if np.random.rand() < 0.5:
            factor = 0.6 + 0.8 * np.random.rand()
            rgb = np.clip(rgb * factor, 0.0, 1.0)

        # horizontal flip (and negate steering)
        if np.random.rand() < 0.5:
            rgb = np.ascontiguousarray(rgb[:, ::-1, :])
            steering = -steering

        return rgb, steering

    def __getitem__(self, idx):
        # Gets the path of image, steering angle and throttle
        row = self.df.iloc[idx]
        fname = row["filename"]
        steering = float(row["steering"])
        throttle = float(row["throttle"])

        # Reads in image
        path = os.path.join(self.images_dir, fname)
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(f"Could not read image: {path}")
        
        # Preprocesses and augments the image
        rgb = self.preprocessor.preprocess(bgr)
        if self.augment:
            rgb, steering = self._augment(rgb, steering)


        # DO WE NEED TO DO THIS TOO?
        # HWC -> CHW
        # reorders the axes of the image array 
        # makes it into channel, heigh, width shape
        chw = np.transpose(rgb, (2, 0, 1))
        # 3D tensor for image [channel][vert pixel][horiz pixel]
        x = torch.from_numpy(chw).float()
        # 1D tensor for the steering and throttle
        y = torch.tensor([steering/cfg.max_angle], dtype=torch.float32)
        z = torch.tensor([throttle/cfg.max_throttle], dtype=torch.float32)
        return x, y, z


# -----------------------------
# Train / Eval
# -----------------------------
def run_epoch(model, loader, optimizer, device, train: bool):
    if train:
        # .train is a Pytorch method 
        model.train()
    else:
        model.eval()

    total_loss = 0.0
    total_n = 0
    mse = nn.MSELoss()

    for x, y, z in loader:

        x = x.to(device)
        y = y.to(device)
        z = z.to(device)
        

        if train:
            optimizer.zero_grad()

        pred_steering, pred_throttle = model(x)   # Returns 2 tensors
        

        loss_steering = mse(pred_steering, y)
        loss_throttle = mse(pred_throttle, z)
        loss = 0.7 * loss_steering + 0.3 * loss_throttle

        if train:
            loss.backward()
            optimizer.step()

        total_loss += loss.item() * x.size(0)
        total_n += x.size(0)

    return total_loss / max(1, total_n)

def main():
    set_seed(cfg.seed)
    preprocessor = PreProcessor()
    if not os.path.exists(cfg.csv_path):
        raise FileNotFoundError(f"Missing {cfg.csv_path}.")
    
    ### Read in csv of image, steering angle, throttle
    # Copies csv into panda data frame
    df = pd.read_csv(cfg.csv_path)
    required_cols = {"filename", "steering", "throttle"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {required_cols}. Found: {list(df.columns)}")

    ### Copies and splits into training and validation data frame
    # Chronological split: first part train, last part val
    n = len(df)
    n_val = max(1, int(math.floor(n * cfg.val_frac)))
    train_df = df.iloc[: n - n_val].copy()
    val_df = df.iloc[n - n_val :].copy()

    ### Create a torch legal dataset object
    # NOTE: Torch requires object to have getitem and len methods
    train_ds = DrivingDataset(train_df, cfg.images_dir, preprocessor, augment=True)
    val_ds = DrivingDataset(val_df, cfg.images_dir, preprocessor, augment=False)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")
    print(f"Train samples: {len(train_ds)} | Val samples: {len(val_ds)}")
    
    # Data loaders allow us to iterate through the dataset by batch size
    train_loader = DataLoader(train_ds, batch_size=cfg.batch_size, shuffle=True,
                              num_workers=cfg.num_workers, pin_memory=(device == "cuda"))
    val_loader = DataLoader(val_ds, batch_size=cfg.batch_size, shuffle=False,
                            num_workers=cfg.num_workers, pin_memory=(device == "cuda"))

    model = PiPilotNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.learning_rate)

    best_val = float("inf")

    for epoch in range(1, cfg.epochs + 1):
        train_loss = run_epoch(model, train_loader, optimizer, device, train=True)
        val_loss = run_epoch(model, val_loader, optimizer, device, train=False)

        print(f"Epoch {epoch:02d}/{cfg.epochs} | train MSE: {train_loss:.5f} | val MSE: {val_loss:.5f}")

        if val_loss < best_val:
            best_val = val_loss
            torch.save({
                "model_state": model.state_dict(),
                "config": cfg.__dict__,
            }, cfg.save_path)
            print(f"  saved best -> {cfg.save_path}")

    # quick test inference on the last val image
    test_row = val_df.iloc[-1]
    test_path = os.path.join(cfg.images_dir, test_row["filename"])
    bgr = cv2.imread(test_path, cv2.IMREAD_COLOR)
    ds_tmp = DrivingDataset(val_df.iloc[-1:].copy(), cfg.images_dir, preprocessor, augment=False)
    x, y, z = ds_tmp[0]
    x = x.unsqueeze(0).to(device)

    model.eval()
    with torch.no_grad():
        pred_steering, pred_throttle = model(x)

    print("\nQuick test:")
    print(f"Image: {test_row['filename']}")
    print(f"Actual steering:  {float(test_row['steering']): .3f}")
    print(f"Actual throttle:  {float(test_row['throttle']): .3f}")
    print(f"Pred steering:    {pred_steering.item() * 90}") 
    print(f"Pred throttle:    {pred_throttle.item() * 40}")


if __name__ == "__main__":
    main()