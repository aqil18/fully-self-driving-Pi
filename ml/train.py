import os
import cv2
import math
import pandas as pd
import numpy as np
from dataclasses import dataclass

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader


# -----------------------------
# Config
# -----------------------------
@dataclass
class Config:
    csv_path: str = "datasets/2026-01-13_08-27-03/labels/labels.csv"
    images_dir: str = "datasets/2026-01-13_08-27-03/images"

    # image preprocessing
    out_h: int = 66
    out_w: int = 200

    # training
    seed: int = 42
    batch_size: int = 64
    lr: float = 1e-3
    epochs: int = 8
    val_frac: float = 0.15  # last 15% used as val
    num_workers: int = 2

    # saving
    save_path: str = "models/model.pt"


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
# Model (small PilotNet-style)
# -----------------------------
class PiPilotNet(nn.Module):
    def __init__(self):
        super().__init__()
        # input: (B, 3, 66, 200)
        self.conv1 = nn.Conv2d(3, 24, kernel_size=5, stride=2)
        self.conv2 = nn.Conv2d(24, 36, kernel_size=5, stride=2)
        self.conv3 = nn.Conv2d(36, 48, kernel_size=5, stride=2)
        self.conv4 = nn.Conv2d(48, 64, kernel_size=3, stride=1)
        self.conv5 = nn.Conv2d(64, 64, kernel_size=3, stride=1)

        # We'll infer flatten dim on first forward if needed
        self._flatten_dim = None

        self.fc1 = nn.Linear(1, 100)  # placeholder; we replace after infer
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 1)

    def _ensure_fc1(self, x):
        if self._flatten_dim is None:
            self._flatten_dim = x.shape[1]
            self.fc1 = nn.Linear(self._flatten_dim, 100).to(x.device)

    def forward(self, x):
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = F.relu(self.conv5(x))

        x = torch.flatten(x, 1)
        self._ensure_fc1(x)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))
        steering = torch.tanh(self.fc4(x))  # [-1, 1]
        return steering


# -----------------------------
# Dataset
# -----------------------------
class DrivingDataset(Dataset):
    def __init__(self, df: pd.DataFrame, images_dir: str, out_h: int, out_w: int, augment: bool):
        self.df = df.reset_index(drop=True)
        self.images_dir = images_dir
        self.out_h = out_h
        self.out_w = out_w
        self.augment = augment

    def __len__(self):
        return len(self.df)

    def _preprocess(self, bgr: np.ndarray) -> np.ndarray:
        """
        Simple & safe preprocess:
        - crop lower 60% (road-ish)
        - resize to (out_w, out_h)
        - convert BGR->RGB
        - normalize to [0,1]
        """
        h, w = bgr.shape[:2]

        # crop: keep bottom 60%
        top = int(h * 0.40)
        cropped = bgr[top:, :]

        resized = cv2.resize(cropped, (self.out_w, self.out_h), interpolation=cv2.INTER_AREA)
        rgb = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        return rgb

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
        row = self.df.iloc[idx]
        fname = row["filename"]
        steering = float(row["steering"])

        path = os.path.join(self.images_dir, fname)
        bgr = cv2.imread(path, cv2.IMREAD_COLOR)
        if bgr is None:
            raise FileNotFoundError(f"Could not read image: {path}")

        rgb = self._preprocess(bgr)
        if self.augment:
            rgb, steering = self._augment(rgb, steering)

        # HWC -> CHW
        chw = np.transpose(rgb, (2, 0, 1))
        x = torch.from_numpy(chw).float()
        y = torch.tensor([steering], dtype=torch.float32)
        return x, y


# -----------------------------
# Train / Eval
# -----------------------------
def run_epoch(model, loader, optimizer, device, train: bool):
    if train:
        model.train()
    else:
        model.eval()

    total_loss = 0.0
    total_n = 0
    mse = nn.MSELoss()

    for x, y in loader:
        x = x.to(device)
        y = y.to(device)

        if train:
            optimizer.zero_grad()

        pred = model(x)
        loss = mse(pred, y)

        if train:
            loss.backward()
            optimizer.step()

        total_loss += loss.item() * x.size(0)
        total_n += x.size(0)

    return total_loss / max(1, total_n)


def main():
    set_seed(cfg.seed)

    if not os.path.exists(cfg.csv_path):
        raise FileNotFoundError(f"Missing {cfg.csv_path}. Create it as filename,steering.")

    df = pd.read_csv(cfg.csv_path)
    required_cols = {"filename", "steering"}
    if not required_cols.issubset(df.columns):
        raise ValueError(f"CSV must contain columns: {required_cols}. Found: {list(df.columns)}")

    # chronological split: first part train, last part val
    n = len(df)
    n_val = max(1, int(math.floor(n * cfg.val_frac)))
    train_df = df.iloc[: n - n_val].copy()
    val_df = df.iloc[n - n_val :].copy()

    train_ds = DrivingDataset(train_df, cfg.images_dir, cfg.out_h, cfg.out_w, augment=True)
    val_ds = DrivingDataset(val_df, cfg.images_dir, cfg.out_h, cfg.out_w, augment=False)

    device = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"Using device: {device}")
    print(f"Train samples: {len(train_ds)} | Val samples: {len(val_ds)}")

    train_loader = DataLoader(train_ds, batch_size=cfg.batch_size, shuffle=True,
                              num_workers=cfg.num_workers, pin_memory=(device == "cuda"))
    val_loader = DataLoader(val_ds, batch_size=cfg.batch_size, shuffle=False,
                            num_workers=cfg.num_workers, pin_memory=(device == "cuda"))

    model = PiPilotNet().to(device)
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.lr)

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
    ds_tmp = DrivingDataset(val_df.iloc[-1:].copy(), cfg.images_dir, cfg.out_h, cfg.out_w, augment=False)
    x, y = ds_tmp[0]
    x = x.unsqueeze(0).to(device)

    model.eval()
    with torch.no_grad():
        pred = model(x).item()

    print("\nQuick test:")
    print(f"Image: {test_row['filename']}")
    print(f"GT steering:  {float(test_row['steering']): .3f}")
    print(f"Pred steering:{pred: .3f}")


if __name__ == "__main__":
    main()
