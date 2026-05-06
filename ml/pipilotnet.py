import torch
import torch.nn as nn
import torch.nn.functional as F

# CNN flatten dim for input (3, 66, 200) through 5 conv layers = 1152
_CNN_FLAT_DIM = 1152

# -----------------------------
# CNN Model
# -----------------------------
class PiPilotNet(nn.Module):
    def __init__(self):
        super().__init__()
        # input: (B, 3, 66, 200)
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2)
        self.conv2 = nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2)
        self.conv3 = nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2)
        self.conv4 = nn.Conv2d(in_channels=48, out_channels=64, kernel_size=3, stride=1)
        self.conv5 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1)

        # fc1 takes flattened CNN features (1152) + lane offset scalar (1)
        self.fc1 = nn.Linear(_CNN_FLAT_DIM + 1, 100)
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 2)

    def forward(self, x, offset):
        """
        x      : (B, 3, 66, 200) image tensor
        offset : (B, 1) lane offset normalised to [-1, 1]
        """
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))
        x = F.relu(self.conv4(x))
        x = F.relu(self.conv5(x))

        x = torch.flatten(x, 1)             # (B, 1152)
        x = torch.cat([x, offset], dim=1)   # (B, 1153)

        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        x = F.relu(self.fc3(x))

        out = self.fc4(x)
        steering = torch.tanh(out[:, 0:1])   # [-1, 1]
        throttle = torch.sigmoid(out[:, 1:2]) # [0, 1]

        return steering, throttle

