
import torch
import torch.nn as nn

from typing import Iterable

Tensors = Iterable[torch.Tensor]

class Conv(nn.Sequential):
    def __init__(self, in_channels, out_channels, kernel_size=1, stride=1, padding=0):
        super().__init__(
            nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True)
        )

class DoubleConv(nn.Sequential):
    def __init__(self, in_features, out_features):
        super().__init__(
            nn.Linear(in_features, out_features),
            nn.ReLU(inplace=True)
        )

class Dense(nn.Sequential):
    def __init__(self, in_features, out_features):
        super().__init__(
            nn.Linear(in_features, out_features),
            nn.ReLU(inplace=True)
        )

class DenseNet(nn.Module):
    def __init__(
        self,
        in_channels,
        out_channels,
        densenet_id: str = '121',
        freeze: bool = False
    ):
        super().__init__()

        self.register_buffer('mean', torch.tensor([0.485, 0.456, 0.406]).view(-1, 1, 1))
        self.register_buffer('std', torch.tensor([0.295]))

        self.densenet = torch.hub.load(
            'pytorch/vision:v0.8.1',
            f'densenet{densenet_id}',
            pretrained=True
        )

        # Remove last layer
        self.densenet = nn.Sequential(*list(self.densenet.features))

        if freeze:
            for param in self.densenet.parameters():
                param.requires_grad = False

        # new layers
        self.first = Conv(in_channels, 3)

        self.convs = nn.ModuleList([
            Conv(1024, 128, kernel_size=5),
            Conv(128, 16)
        ])

        if densenet_id == '161':
            self.convs.insert(0, Conv(2208, 1024))

        self.last = nn.Sequential(
            nn.Flatten(),
            nn.Linear(16 * 1 * 6, out_channels),
            nn.Softmax(dim=1)
        )

    def forward(self, x:Tensors) -> Tensors:
        x = self.first(x)

        x = (x - self.mean) / self.std
        x = self.densenet(x)

        for conv in self.convs:
            x = conv(x)

        x = self.last(x)

        return x
    

class UNet(nn.Module):
    def __init__(self, in_channels, out_channels):
        super().__init__()
        depth = 4

        self.downs = nn.ModuleList(
            [DoubleConv(in_channels, 64)] + [
                DoubleConv(64 * (2 ** i), 128 * (2 ** i))
                for i in range(depth)
            ]
        )
        self.maxpool = nn.MaxPool2d(2, ceil_mode=True)
        self.upsample = nn.Upsample(
            scale_factor=2,
            mode='bilinear',
            align_corners=False
        )

        self.ups = nn.ModuleList([
            DoubleConv((64 + 128) * (2 ** i), 64 * (2 ** i))
            for i in reversed(range(depth))
        ])

        self.lasts = nn.Sequential(
            nn.Conv2d(64, out_channels, 1),
            nn.LogSoftmax(dim=1)
        )

    def forward(self, x: Tensors):
        features, shapes = [], []

        # Downhill gradient
        for down in self.downs[:-1]:
            x = down(x)
            features.append(x)
            shapes.append(x.shape[-2:])
            x = self.maxpool(x)

        x = self.downs[-1](x)

        # uphill
        for up in self.ups:
            pass