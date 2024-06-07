#!/home/samwise/pythonvenvs/pothole/bin/python
import collections

import rospy
from sensor_msgs.msg import Image
import message_filters

from datetime import datetime
import os
import cv2
import numpy as np
import torch
from torch.utils.tensorboard import SummaryWriter
from cv_bridge import CvBridge

from unet import UNet
from transforms import get_transform
from dataloader import OnlineDataset


def collate_fn(batch):
    return tuple(zip(*batch))


class DeepNode:
    def __init__(self, image_topic, label_topic, batch_size=16, dataset_length: int = 1000, nb_prev_images: int = 2):
        self.inference_pub = None
        self.writer = None
        self.logs = None
        self.dataset = None
        self.total_images_received = 0
        self.batch_size = batch_size
        self.dataset_length = dataset_length
        self.nb_prev_images = nb_prev_images
        self.device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.bridge = CvBridge()
        self.image_topic = image_topic
        self.label_topic = label_topic

        self.model_is_setup = False
        self.model = UNet(input_channels=3 + 3*self.nb_prev_images, num_classes=1, num_filters=[32, 64, 128, 192], initializers={'w': 'orthogonal', 'b': 'normal'}).to(self.device)
        self.criterion = torch.nn.BCEWithLogitsLoss(size_average=False, reduce=False, reduction='none').to(self.device)
        self.optimizer = torch.optim.Adam(list(self.model.parameters()), lr=1e-5, weight_decay=0)
        self.model_is_setup = True

    def setup_logs(self, prefix: str = ""):
        TIMESTAMP = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.logs = os.path.join('sessions/', TIMESTAMP) + f"{prefix}"
        for dir_ in ['weights', 'logs', 'figures', 'tensorboard']:
            os.makedirs(os.path.join(self.logs, dir_), exist_ok=True)
        self.writer = SummaryWriter(log_dir=self.logs)

    def setup_subscribers_and_publisher(self):
        self.inference_pub = rospy.Publisher('inference', Image, queue_size=10)
        image_sub = message_filters.Subscriber(self.image_topic, Image)
        label_sub = message_filters.Subscriber(self.label_topic, Image)

        ts = message_filters.ApproximateTimeSynchronizer([image_sub, label_sub], 10, 0.1, allow_headerless=True)
        ts.registerCallback(self.image_capture_callback)

    def train(self):
        rospy.loginfo('training..')
        self.setup_logs('-train')
        self.dataset = OnlineDataset(transforms=get_transform(train=True), dataset_length=self.dataset_length, nb_prev_images=self.nb_prev_images, sampling_strategy='loss_weighted')
        self.setup_subscribers_and_publisher()

        checkpoint_freq = 100

        iteration = 0
        while not rospy.is_shutdown():
            while len(self.dataset) > self.batch_size:
                sample_indicies = self.dataset.sample_indicies(self.batch_size)

                images, targets = list(zip(*[self.dataset[idx] for idx in sample_indicies]))
                images = torch.stack(list(image.to(self.device) for image in images))
                targets = torch.stack(list(target.to(self.device) for target in targets))

                outputs = self.model(images)

                with torch.no_grad():
                    pred = (torch.sigmoid(outputs) > 0.5).float()
                    miou = torch.mean(self.iou_score(pred, targets))

                loss = self.criterion(input=outputs, target=targets)
                losses = torch.mean(loss)
                for j, i in enumerate(sample_indicies):
                    # update the loss, so we can sample images with bad loss more often
                    self.dataset.losses[i] = torch.mean(loss[j]).item()
                rospy.loginfo(f'Iteration {iteration} loss: {losses:.4f}, batch mIOU: {miou:.4f}, [images seen: {iteration * self.batch_size}, dataset size: {len(self.dataset)}, total images received: {self.total_images_received}]')
                self.writer.add_scalar(f"loss", losses.item(), iteration)
                self.writer.add_scalar(f"miou", miou.item(), iteration)
                self.writer.add_scalar(f"image_seen", iteration * self.batch_size, iteration)
                self.writer.add_scalar(f"dataset_size", len(self.dataset), iteration)
                self.writer.add_scalar(f"total_images_recv", self.total_images_received, iteration)

                self.optimizer.zero_grad()
                losses.backward()
                self.optimizer.step()

                iteration += 1
                if iteration % checkpoint_freq == 0 and iteration > 0:
                    self.model.save(os.path.join(self.logs, 'weights/', f'model_step_{iteration}.pth'))

                np_image, target = self.dataset.get(-1)
                self.inference(np_image)

                if iteration % 10000 == 0 and iteration > 0:
                    return

    def validate(self, model_path=None):
        rospy.loginfo(f'validating..')

        if model_path is None:
            model_path = '/home/samwise/catkin_ws/src/OnlinePotholeDetection/scripts/DetectionNode/sessions/2023-07-31_11-23-05-train/weights/model_step_700.pth'
            self.model.load_state_dict(torch.load(model_path, map_location=self.device), strict=False)
            rospy.loginfo('loaded model')

        self.setup_logs('-val')
        self.dataset = OnlineDataset(transforms=get_transform(train=False), nb_prev_images=self.nb_prev_images)
        self.setup_subscribers_and_publisher()

        while not rospy.is_shutdown():
            if len(self.dataset) <= 0:
                continue
            try:
                rospy.wait_for_message(self.image_topic, Image, timeout=10)
            except rospy.exceptions.ROSException:
                break
        rospy.loginfo(f'collected {len(self.dataset)} datapoints - ready to validate')

        from torch.utils.data import DataLoader
        from torch.utils.data.sampler import SequentialSampler
        test_sampler = SequentialSampler(range(len(self.dataset)))
        test_loader = DataLoader(self.dataset, batch_size=1, sampler=test_sampler, num_workers=0, collate_fn=collate_fn)
        miou = []
        for iteration, (images, targets) in enumerate(test_loader):
            image = torch.stack(list(i.to(self.device) for i in images))
            target = torch.stack(list(t.to(self.device) for t in targets))
            outputs = (torch.sigmoid(self.model(image)) > 0.5).bool()
            iou = self.iou_score(outputs, target)
            self.writer.add_scalar(f"miou", iou.item(), iteration)
            miou.append(iou)
        miou = torch.vstack(miou).mean().item()
        rospy.loginfo(f'mIOU = {miou}')

    def iou_score(self, outputs: torch.Tensor, labels: torch.Tensor, reduction: str = 'mean'):
        intersection = torch.logical_and(outputs, labels)
        intersection = torch.count_nonzero(intersection, dim=(2, 3))

        union = torch.logical_or(outputs, labels)
        union = torch.count_nonzero(union, dim=(2, 3))
        iou = intersection / union
        if reduction.lower() == 'mean':
            # if there's neither a prediction nor gt available (i.e. union = 0), we make
            # a correct prediction and the score should be 1, to reflect this
            iou[union == 0] = 1.
        elif reduction.lower() == 'none':
            pass
        return torch.nan_to_num(iou)

    def inference(self, th_image):
        """ publish the results of inference on the incoming image topic """
        if not self.model_is_setup:
            return
        image = th_image.unsqueeze(0).to(self.device).float()
        output = (torch.sigmoid(self.model(image)) > 0.5).long() * 255
        output = output.cpu().permute(0, 2, 3, 1).squeeze().numpy().astype(np.uint8)
        msg = self.cv_to_ros_image(output)
        self.inference_pub.publish(msg)

    @staticmethod
    def ros_to_cv_image(msg):
        return np.frombuffer(msg.data, np.uint8).reshape((msg.height, msg.width, 3)).copy()

    @staticmethod
    def cv_to_ros_image(img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()  # TODO: keep timestamp with inferred image
        msg.width = img.shape[1]
        msg.height = img.shape[0]
        msg.step = img.shape[1] * 3
        msg.encoding = 'rgb8'
        msg.data = img.ravel().repeat(3).tolist()
        return msg

    def image_capture_callback(self, image, label):
        """ save an image + label pair to the (volatile) dataset """
        self.total_images_received += 1
        cv_image = self.ros_to_cv_image(image)
        cv_label = self.ros_to_cv_image(label)

        cv_label = cv2.cvtColor(cv_label, cv2.COLOR_RGB2GRAY)
        self.dataset.add_image_and_label(cv_image, cv_label)

    def image_generator(self, timer):
        """ temporary function instead of subscribing to an actual ROS topic """
        # TODO: remove function
        raise NotImplementedError('this function should not be used anymore.')
        image = np.random.randint(0, 255, (3, 640, 480)).astype(dtype=np.float32)
        label = np.random.randint(0, 1, (1, 640, 480)).astype(dtype=np.float32)
        self.image_capture_callback(image, label)


if __name__ == "__main__":
    rospy.init_node('DeepNode', anonymous=True)
    dn = DeepNode("/camera/color/image_raw", "/label")
    # dn.train()
    dn.validate()
