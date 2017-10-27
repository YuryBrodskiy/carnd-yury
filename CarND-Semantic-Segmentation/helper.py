import re
import random
import numpy as np
import os.path
import scipy.misc
import shutil
import zipfile
import time
import tensorflow as tf
from urllib.request import urlretrieve
from tqdm import tqdm


from glob import glob
import re
import cv2
import scipy.misc
from sklearn.utils import shuffle


class DLProgress(tqdm):
    last_block = 0

    def hook(self, block_num=1, block_size=1, total_size=None):
        self.total = total_size
        self.update((block_num - self.last_block) * block_size)
        self.last_block = block_num


def maybe_download_pretrained_vgg(data_dir):
    """
    Download and extract pretrained vgg model if it doesn't exist
    :param data_dir: Directory to download the model to
    """
    vgg_filename = 'vgg.zip'
    vgg_path = os.path.join(data_dir, 'vgg')
    vgg_files = [
        os.path.join(vgg_path, 'variables/variables.data-00000-of-00001'),
        os.path.join(vgg_path, 'variables/variables.index'),
        os.path.join(vgg_path, 'saved_model.pb')]

    missing_vgg_files = [vgg_file for vgg_file in vgg_files if not os.path.exists(vgg_file)]
    if missing_vgg_files:
        # Clean vgg dir
        if os.path.exists(vgg_path):
            shutil.rmtree(vgg_path)
        os.makedirs(vgg_path)

        # Download vgg
        print('Downloading pre-trained vgg model...')
        with DLProgress(unit='B', unit_scale=True, miniters=1) as pbar:
            urlretrieve(
                'https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/vgg.zip',
                os.path.join(vgg_path, vgg_filename),
                pbar.hook)

        # Extract vgg
        print('Extracting model...')
        zip_ref = zipfile.ZipFile(os.path.join(vgg_path, vgg_filename), 'r')
        zip_ref.extractall(data_dir)
        zip_ref.close()

        # Remove zip file to save space
        os.remove(os.path.join(vgg_path, vgg_filename))


def gen_batch_function(data_folder, image_shape):
    """
    Generate function to create batches of training data
    :param data_folder: Path to folder that contains all the datasets
    :param image_shape: Tuple - Shape of image
    :return:
    """
    def get_batches_fn(batch_size):
        """
        Create batches of training data
        :param batch_size: Batch Size
        :return: Batches of training data
        """
        image_paths = glob(os.path.join(data_folder, 'image_2', '*.png'))
        label_paths = {
            re.sub(r'_(lane|road)_', '_', os.path.basename(path)): path
            for path in glob(os.path.join(data_folder, 'gt_image_2', '*_road_*.png'))}
        background_color = np.array([255, 0, 0])

        random.shuffle(image_paths)
        for batch_i in range(0, len(image_paths), batch_size):
            images = []
            gt_images = []
            for image_file in image_paths[batch_i:batch_i+batch_size]:
                gt_image_file = label_paths[os.path.basename(image_file)]

                image = scipy.misc.imresize(scipy.misc.imread(image_file), image_shape)
                gt_image = scipy.misc.imresize(scipy.misc.imread(gt_image_file), image_shape)

                gt_bg = np.all(gt_image == background_color, axis=2)
                gt_bg = gt_bg.reshape(*gt_bg.shape, 1)
                gt_image = np.concatenate((gt_bg, np.invert(gt_bg)), axis=2)

                images.append(image)
                gt_images.append(gt_image)

            yield np.array(images), np.array(gt_images)
    return get_batches_fn


def getData(data_folder):
    image_paths = glob(os.path.join(data_folder, 'image_2', '*.png'))
    label_paths_dic = {
        re.sub(r'_(lane|road)_', '_', os.path.basename(path)): path
        for path in glob(os.path.join(data_folder, 'gt_image_2', '*_road_*.png'))}

    label_paths = [label_paths_dic[os.path.basename(img_path)] for img_path in image_paths]
    return image_paths, label_paths


def resize11(image, image_shape):
    new_shape = (int(image_shape[1]*1.1), int(image_shape[0]*1.1))
    return cv2.resize(image,new_shape, interpolation = cv2.INTER_LINEAR)


def crop(image,x,y, image_shape):
    return image[y:y+image_shape[0],x:x+image_shape[1]]


def rot(image, angle, image_shape):
    rows, cols = image_shape
    M = cv2.getRotationMatrix2D((cols/2, rows/2), angle, 1.3)
    return cv2.warpAffine(image, M, (cols, rows))


def shiftUp(image, image_shape):
    return crop(resize11(image,image_shape),5,5, image_shape)


def shiftDown(image, image_shape):
    return crop(resize11(image,image_shape),0,0, image_shape)


def rotCCW(image, image_shape):
    return rot(image,-5, image_shape)


def rotCW(image, image_shape):
    return rot(image,5, image_shape)


def load_img(path, image_shape,  aug = lambda x, shape: x):
    return aug(scipy.misc.imresize(scipy.misc.imread(path), image_shape), image_shape)


def load_gt(path, image_shape,  aug = lambda x, shape: x):
    background_color = np.array([255, 0, 0])
    gt_image = aug(scipy.misc.imresize(scipy.misc.imread(path), image_shape), image_shape)
    gt_bg = np.all(gt_image == background_color, axis=2)
    gt_bg = gt_bg.reshape(*gt_bg.shape, 1)
    gt_image = np.concatenate((gt_bg, np.invert(gt_bg)), axis=2)
    return gt_image


augment_list = \
        [
            lambda x, shape: x,
            lambda x, shape: shiftUp(x, shape),
            lambda x, shape: shiftDown(x, shape),
            lambda x, shape: rotCCW(x, shape),
            lambda x, shape: rotCW(x, shape),
            lambda x, shape: np.fliplr(x),
            lambda x, shape: np.flipud(x),
          ]


def augmentData(img_paths, label_paths, image_shape):
    img_load  = []
    gt_load = []
    for img_path, label_path in zip(img_paths, label_paths):
        for aug in augment_list:
            img_load.append((img_path, image_shape, aug))
            gt_load.append((label_path, image_shape, aug))
    return img_load, gt_load


def gen_batch_function_load(img_loads, gt_loads):
    def get_batches_fn(batch_size):
        """
        Create batches of training data
        :param batch_size: Batch Size
        :return: Batches of training data
        """
        img_p, gt_p = shuffle(img_loads, gt_loads)
        images = []
        gt_images = []

        for batch_i in range(0, len(img_p), batch_size):
            end = batch_i + batch_size
            images = []
            gt_images = []
            for img_load,  gt_load in zip(img_p[batch_i:end], gt_p[batch_i:end]):
                images.append(load_img(*img_load))
                gt_images.append(load_gt(*gt_load))
            yield np.array(images), np.array(gt_images)
    return get_batches_fn


def gen_test_output(sess, logits, keep_prob, image_pl, data_folder, image_shape):
    """
    Generate test output using the test images
    :param sess: TF session
    :param logits: TF Tensor for the logits
    :param keep_prob: TF Placeholder for the dropout keep robability
    :param image_pl: TF Placeholder for the image placeholder
    :param data_folder: Path to the folder that contains the datasets
    :param image_shape: Tuple - Shape of image
    :return: Output for for each test image
    """
    for image_file in glob(os.path.join(data_folder, 'image_2', '*.png')):
        image = scipy.misc.imresize(scipy.misc.imread(image_file), image_shape)

        im_softmax = sess.run(
            [tf.nn.softmax(logits)],
            {keep_prob: 1.0, image_pl: [image]})
        im_softmax = im_softmax[0][:, 1].reshape(image_shape[0], image_shape[1])
        segmentation = (im_softmax > 0.5).reshape(image_shape[0], image_shape[1], 1)
        mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
        mask = scipy.misc.toimage(mask, mode="RGBA")
        street_im = scipy.misc.toimage(image)
        street_im.paste(mask, box=None, mask=mask)

        yield os.path.basename(image_file), np.array(street_im)


def save_inference_samples(output_dir, data_dir, sess, image_shape, logits, keep_prob, input_image):
    # Make folder for current run
    os.makedirs(output_dir)

    # Run NN on test images and save them to HD
    print('Training Finished. Saving test images to: {}'.format(output_dir))
    image_outputs = gen_test_output(
        sess, logits, keep_prob, input_image, os.path.join(data_dir, 'data_road/testing'), image_shape)
    for name, image in image_outputs:
        scipy.misc.imsave(os.path.join(output_dir, name), image)
