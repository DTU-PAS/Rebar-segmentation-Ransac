import numpy as np
import torch
import torchvision.transforms as T
import torchvision.transforms.functional as F


class Compose(T.Compose):
    def __call__(self, img, target=None):
        for t in self.transforms:
            img, target = t(img, target)
        return img, target


class RandomHorizontalFlip(T.RandomHorizontalFlip):
    def forward(self, img, target=None):
        if np.random.random() < self.p:
            img = F.hflip(img)
            if target is None:
                return img, target
            target = torch.flip(target, dims=(2,))

        return img, target


class RandomVerticalFlip(T.RandomVerticalFlip):
    def forward(self, img, target=None):
        if np.random.random() < self.p:
            img = F.vflip(img)
            if target is None:
                return img, target
            target = torch.flip(target, dims=(1,))

        return img, target


def get_transform(train, transforms=None):
    transforms = [] if transforms is None else transforms

    if train:
        # transforms require the image to be a tensor
        transforms.append(RandomVerticalFlip(p=.5))
        transforms.append(RandomHorizontalFlip(p=.5))

    return Compose(transforms)