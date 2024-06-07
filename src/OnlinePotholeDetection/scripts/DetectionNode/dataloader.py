import numpy as np
import torch
from torch.utils.data import Dataset
import collections


class OnlineDataset(Dataset):
    def __init__(self, transforms=None, dataset_length: int = 3000, nb_prev_images: int = 0, sampling_strategy: str = "loss_weighted"):
        self.transforms = transforms
        self.dataset_length = dataset_length
        self.previous_images = collections.deque(maxlen=nb_prev_images+1)
        self.imgs = collections.deque(maxlen=dataset_length)
        self.losses = collections.deque(maxlen=dataset_length)
        self.sampling_strategy = sampling_strategy

    def __getitem__(self, idx):
        # load images and masks and apply transformations
        img, target = self.get(idx)

        if self.transforms is not None:
            img, target = self.transforms(img, target)
        return img, target

    def get(self, idx):
        # loads the images and masks, but does not apply any transformations
        img, mask = self.imgs[idx]
        img = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0
        target = torch.from_numpy(mask).unsqueeze(0).float() / 255.0
        return img, target

    def add_image_and_label(self, image, label):
        # put in the current image in the history, so we can just concatenate one deque instead of an array + a deque
        # NOTE: this means we have to add one everytime we want to check dimensions
        self.previous_images.append(image)
        if len(self.previous_images) < self.previous_images.maxlen:
            # if we don't have necessary previous images, we tile the image along the channel dimension
            image = np.tile(image, (1, 1, self.previous_images.maxlen))
        else:
            # concatenate the image sequence along the channel dimension. (we wrap in deque to make a copy)
            image = np.concatenate(list(collections.deque(self.previous_images)), axis=-1)

        if self.sampling_strategy.lower() == "uniform":
            self.imgs.append((image, label))
            self.losses.append(None)
        elif self.sampling_strategy.lower() == "loss_weighted":
            if len(self) < self.dataset_length:
                # the memory is not full yet, so add entry to end of memory
                self.imgs.append((image, label))
                self.losses.append(None)
            else:
                # memory is full - replace the entry with the lowest loss (least likely to be sampled again)
                idx = np.array(self.losses, dtype=np.float).argmin()
                self.imgs[idx] = (image, label)
                self.losses[idx] = None
        else:
            raise ValueError('sampling strategy not supported')

    def sample_indicies(self, batch_size: int):
        if self.sampling_strategy.lower() == "uniform":
            return np.random.choice(len(self), batch_size)
        elif self.sampling_strategy.lower() == "loss_weighted":
            # these are the new entries we guarantee will be seen
            sample_indicies_guarantee = np.where(np.array(self.losses) == None)[0][:batch_size]
            if len(sample_indicies_guarantee) == batch_size:
                sample_indicies_fill = np.array([], dtype=np.int)
            else:
                # after getting all the new entries, we sample the rest of the batch with old datapoints weighted by their loss
                rest = np.where(np.array(self.losses) != None)[0]  # Get index all the old datapoints
                np_loss = np.array(self.losses, dtype=np.float)[rest]  # Convert to loss ndarray
                probabilities = np_loss / np.sum(np_loss)  # Normalize to probability (#NOTE: maybe this could be a softmax?)
                sample_indicies_fill = np.random.choice(rest, (batch_size - len(sample_indicies_guarantee)), p=probabilities)
            sample_indicies = np.hstack((sample_indicies_guarantee, sample_indicies_fill))
            assert len(sample_indicies) == batch_size
            return sample_indicies
        else:
            raise ValueError('sampling strategy not supported')

    def __len__(self):
        return len(self.imgs)
