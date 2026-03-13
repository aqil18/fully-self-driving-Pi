import torch
import torch.nn as nn
import torch.nn.functional as F

# -----------------------------
# CNN Model
# -----------------------------
class PiPilotNet(nn.Module):
    def __init__(self):
        super().__init__()
        ### Convolutional layers
        # input: (B, 3, 66, 200)
        # Input channels is 3 - because we have R G B
        self.conv1 = nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2)
        self.conv2 = nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2)
        self.conv3 = nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2)
        self.conv4 = nn.Conv2d(in_channels=48, out_channels=64, kernel_size=3, stride=1)
        self.conv5 = nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1)

        # We'll infer flatten dim on first forward if needed
        self._flatten_dim = None

        ### Fully connected layers
        self.fc1 = nn.Linear(1, 100)  # placeholder; we replace after infer
        self.fc2 = nn.Linear(100, 50)
        self.fc3 = nn.Linear(50, 10)
        self.fc4 = nn.Linear(10, 2) # 2 Outputs - steering and throttle

    def _ensure_fc1(self, x):
        if self._flatten_dim is None:
            # CNN output must be flattened as it has a 3d array 
            # -> convert into 1d array
            self._flatten_dim = x.shape[1]
            self.fc1 = nn.Linear(self._flatten_dim, 100).to(x.device)
    

    ### forward defines which layesrs are data passes through and in what order
    def forward(self, x):
        ### ReLu is the activation function used
        # Rectified Linear unit -> ReLU(x) = max(0, x)
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

        out = self.fc4(x)   # shape: (B, 2)
        
        steering = torch.tanh(out[:, 0:1])   # keep in [-1, 1]
        throttle = torch.sigmoid(out[:, 1:2])  # keep in [0, 1]

        return steering, throttle

