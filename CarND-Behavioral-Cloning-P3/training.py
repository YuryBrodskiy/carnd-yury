import csv
import cv2
import numpy as np
import argparse

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Activation, Dropout
from keras.layers import Cropping2D
from keras.layers.normalization import BatchNormalization

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
from keras.callbacks import ModelCheckpoint
from keras.callbacks import EarlyStopping
import random
import pickle

# how long the network will be trained, adjustable via command line
s_epoch = 10

# tune-able parameters of data augmentation, has significant effect on drive quality
s_steer_offset = 0.05
s_steer_gain = 5.00

# list of used data augmentation
augment_list = \
    [
        lambda x: get_img(x, 1),
        lambda x: get_img(x, 0),
        lambda x: get_img(x, 2),
        lambda x: image_flip(*get_img(x, 1)),
        lambda x: image_flip(*get_img(x, 0)),
        lambda x: image_flip(*get_img(x, 2)),
    ]


def image_flip(image, value):
    """
    Flipping the image  and the measured steering angle
    This is needed to avoid bias of steering to the left due to circular nature of the track

    :param image: np.array pixel values
    :param value: float steering angle
    :return: tuple image and steering angle
    """

    image_flipped = np.fliplr(image)
    measurement_flipped = -value
    return image_flipped, measurement_flipped


def get_data_list(s_dir):
    """
    Read csv file and prepare the list of accessible files with augmentation indexes
    This list can be directly used by generator to supply the images for learning
    If steering angle is smaller the 0.05 deg it is a straight part of the tack
    else it is a turn or recovery value
    To avoid bias to straight driving the for values of below 0.05 we randomly take
    one of the available images(original or augmented) and for bigger values all images are taken

    :param s_dir: address to train data
    :return: list of rows from csv file with corrected address and augment index
    """
    lines = []
    with open(s_dir + 'driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        it_reader = iter(reader)
        next(it_reader)
        for line in it_reader:
            for i in range(3):
                filename = line[i].split('/')[-1]
                line[i] = s_dir + 'IMG/' + filename
                # for index in range(len(augment_list)):
                #     lines.append((line, index))

            for index in range(len(augment_list)):
                lines.append((line, index))

    return lines


def get_img(line, index):
    """
    For left and right images the steering angle is devised using
    Ackerman steering geometry to land on the trajectory generated by the central image
    with a constant correction factor.

    :param line: addresses to images
    :param index: 0- center, 1 -left, 2-right
    :return: tuple image and steering angle
    """
    correction_offset = np.array([0.0, s_steer_offset, -s_steer_offset])
    measurement = float(line[3])
    if measurement == 0.0:
        correction_gain = [1.0, 1.0, 1.0]
        correction_offset *= 2.0
    elif measurement > 0.0:
        correction_gain = [1.0, s_steer_gain, 1.0/s_steer_gain]
    elif measurement < 0.0:
        correction_gain = [1.0, 1.0/s_steer_gain, s_steer_gain]
    result_steering = measurement*correction_gain[index] + correction_offset[index]
    img = cv2.imread(line[index])
    return img, min(max(result_steering, -1), 1)


def generator(samples, batch_size=32):
    """

    :param samples: list of rows from csv file with corrected address and augment index
    :param batch_size: number of images to return each iteration
    :return: tuple of 4D image array (batch, height, width, channels) and steering angels (batch, 1)
    """
    num_samples = len(samples)

    while 1:  # Loop forever so the generator never terminates
        samples = shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset + batch_size]
            images = []
            measurements = []
            for line, aug_index in batch_samples:
                img, m = augment_list[aug_index](line)
                images.append(img)
                measurements.append(m)
            x_data = np.array(images)
            y_data = np.array(measurements)
            yield shuffle(x_data, y_data)


def model_nvidia():
    """
    :return: keras model of Nvidia network
    """
    model = Sequential()
    model.add(Cropping2D(cropping=((50, 20), (0, 0)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: x / 255.0 - 0.5))
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), activation='relu', init='he_normal'))
    model.add(Dropout(0.4))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), activation='relu', init='he_normal'))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), init='he_normal'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(Convolution2D(64, 3, 3, activation='relu', init='he_normal'))
    model.add(Convolution2D(64, 3, 3, activation='relu', init='he_normal'))
    model.add(Dropout(0.4))
    model.add(Flatten())
    model.add(Dense(1164, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dropout(0.6))
    model.add(Dense(100, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(50, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dropout(0.2))
    model.add(Dense(10, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(1))
    return model


def model_nvidia_6():
    """
    :return: keras model of Nvidia network with a single dropout layer
    """
    model = Sequential()
    model.add(Cropping2D(cropping=((50, 20), (5, 5)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: x / 255.0 - 0.5))
    model.add(Convolution2D(24, 5, 5, subsample=(2, 2), activation='relu', init='he_normal'))
    model.add(Convolution2D(36, 5, 5, subsample=(2, 2), activation='relu', init='he_normal'))
    model.add(Convolution2D(48, 5, 5, subsample=(2, 2), init='he_normal'))
    model.add(Activation('relu'))
    model.add(Convolution2D(64, 3, 3, activation='relu', init='he_normal'))
    model.add(Convolution2D(64, 3, 3, activation='relu', init='he_normal'))
    model.add(Flatten())
    model.add(Dense(1164, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dropout(0.6))
    model.add(Dense(100, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(50, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(10, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(1))
    return model


def menu():
    """
    command line menu, need to simplify parameter tuning via AWS
    :return:
    """
    global s_steer_offset
    global s_steer_gain
    global s_epoch
    parser = argparse.ArgumentParser(description='Setup training of the model')
    parser.add_argument(
        'model_name',
        type=str,
        default='model_basic',
        help='which model is trained')

    parser.add_argument(
        'dir',
        type=str,
        default='',
        help='location of data')

    parser.add_argument(
        '--offset',
        type=float,
        default=s_steer_offset,
        help='The bias correction value for left and right images'
    )

    parser.add_argument(
        '--gain',
        type=float,
        default=s_steer_gain,
        help='The affine correction value for left and right images, based on distance between cameras'
    )

    parser.add_argument(
        '--num',
        type=int,
        help='number of images to use, skip to use all')

    parser.add_argument(
        '--epoch',
        type=int,
        help='epoch to train')

    args = parser.parse_args()
    samples = get_data_list(args.dir)
    if args.num:
        samples = samples[0:args.num]
    models = \
        {
            'model_nvidia': model_nvidia,
            'model_nvidia_6': model_nvidia_6,
        }
    if args.offset:
        s_steer_offset = args.offset
    if args.gain:
        s_steer_gain = args.gain
    if args.epoch:
        s_epoch = args.epoch

    # output all to verify correct parameter setup

    print('Training on num samples ', len(samples))
    print('Epoch ', s_epoch)
    print('Steering offset is ', s_steer_offset)
    print('Steering gain is ', s_steer_gain)
    print('Data from directory ' + args.dir)
    print('Model name ' + args.model_name)

    return samples, models[args.model_name](), args.model_name


def main():
    """
    The training pipe line

    :return: train history data, model with minimal validation loss, and final model
    """
    samples, model, result_name = menu()
    file_name = result_name + str(s_steer_offset) + '_' + str(s_steer_gain)

    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=32)
    validation_generator = generator(validation_samples, batch_size=32)

    model.compile(loss='mse', optimizer='adam')
    # save best model based on the minimal validation loss
    saver = ModelCheckpoint(
        filepath='result/' + file_name + '_best{val_loss:.2f}.h5',
        monitor='val_loss',
        verbose=0,
        save_best_only=True,
        save_weights_only=False,
        mode='auto',
        period=1)
    # if validation loss does not improve of next 5 epoch stop
    killer = EarlyStopping(
        monitor='val_loss',
        min_delta=0.00001,
        patience=5,
        verbose=0,
        mode='auto')
    # train and store history
    train_history = model.fit_generator(
        train_generator,
        samples_per_epoch=len(train_samples),
        validation_data=validation_generator,
        nb_val_samples=len(validation_samples),
        nb_epoch=s_epoch,
        callbacks=[saver, killer])
    # store the model
    model.save('result/' + file_name + '.h5')
    # store history
    with open('result/p_' + file_name + '.p', 'wb') as f:
        pickle.dump(train_history.history, f)


if __name__ == '__main__':
    main()
