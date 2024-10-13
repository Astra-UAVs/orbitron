import io
import json
import torch
import numpy as np
import cv2
import torchvision.transforms as transforms

from tqdm import tqdm
from PIL import Image, ImageFilter
from torch.utils.data import Dataset

def to_edges(img: Image.Image) -> Image.Image:
    """
    Extract the edges from an image.
    """

    # Convert to numpy array
    img = np.array(img)

    # Extract edges
    lo_thresh = 50
    hi_thresh = 250
    filter_size = 3

    img = cv2.Canny(
        img,
        lo_thresh,
        hi_thresh,
        apertureSize=filter_size,
        L2gradient=True
    )

    # Convert to PIL Image
    img = Image.fromarray(img)

    return img

class ImageDataset(Dataset):
    def __init__(self, json_path, augment=False, dtype=torch.long, edges=False) -> None:
        super().__init__()
        
        # data
        self.data = []
        # processing
        self.process = transforms.ToTensor()

        if augment:
            self.augment = transforms.RandomChoice([
                lambda x:x,
                lambda x: x.filter(ImageFilter.BLUR),
                lambda x: x.filter(ImageFilter.EDGE_ENHANCE),
                lambda x: x.filter(ImageFilter.SMOOTH),
                transforms.ColorJitter(
                    brightness=0.25,
                    contrast=(0.2, 0.6),
                    saturation=(0.2, 0.6)
                )
            ])
        else:
            self.augment = None

        self.dtype = dtype
        self.edges = edges

        with open(json_path, 'r') as json_file:
            pairs = json.load(json_file)
            for pair in tqdm(pairs):
                data = []

                # open the file
                with open(pair['image'], 'r') as img_file:
                    img_bytes = io.BytesIO(img_file.read())
                    data.append(img_bytes)

                # target
                with open(pair['target'], 'r') as img_file:
                    img_bytes = io.BytesIO(img_file.read())
                    data.append(img_bytes)

                self.data.append(tuple(data))

    def __getitem__(self, index):
        inp, tgrt = self.data[index]
        inp = Image.open(inp)

        if self.edges:
            inp = to_edges(inp)
        elif self.augment is not None:
            inp = self.augment(inp)

        inp = self.process(inp)

        # target
        trgt = Image.open(trgt)
        trgt = np.array(trgt, dtype='uint8', copy=True)
        trgt = torch.from_numpy(trgt)
        trgt = trgt.to(dtype=self.dtype)
        trgt = trgt.squeeze(0)

        return inp, trgt
    
    def __len__(self):
        return len(self.data)