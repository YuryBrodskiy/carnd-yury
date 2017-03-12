import csv
import cv2
import numpy as np
import argparse

from keras.models import Sequential
from keras.models import Model
from keras.layers import merge
from keras.layers import Flatten, Dense, Lambda, Input
from keras.layers import Convolution2D, MaxPooling2D
from keras.layers import Activation, Dropout
from keras.layers import Cropping2D
from keras.layers.normalization import BatchNormalization

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import matplotlib.pyplot as plt
from keras.callbacks import ModelCheckpoint
from keras.callbacks import EarlyStopping
import pickle

s_corr = 0.1 # for simlicity static set by menu used by image read
s_dir = '../data_org/'
s_epoch = 5

def get_image(source_path):
    filename = source_path.split('/')[-1]
    current_path = s_dir + 'IMG/' + filename
    return cv2.imread(current_path)


def image_flip(image, value):
    image_flipped = np.fliplr(image)
    measurement_flipped = -value
    return image_flipped, measurement_flipped


def get_data_list():
    lines = []
    with open(s_dir + 'driving_log.csv') as csvfile:
        reader = csv.reader(csvfile)
        it_reader = iter(reader)
        next(it_reader)
        for line in it_reader:
            for indx in range(len(augment_list)):
                lines.append((line, indx))
    return lines


def get_img(line, index):
    correction = [0.0, s_corr, -s_corr]
    return get_image(line[index]), float(line[3])+correction[index]

 
def twist_l(image, value):
    rows, cols, ch = image.shape
    diff = 20
    pts1 = np.float32([[0,0],[cols-diff,diff],[0,rows],[cols-diff,rows-diff]])
    pts2 = np.float32([[0,0],[cols,0],[0,rows],[cols,rows]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(image,M,(cols,rows))
    return dst, value - 0.05


def twist_r(image, value):
    rows, cols, ch = image.shape
    diff = 20
    pts1 = np.float32([[diff,diff],[cols,0],[diff,rows-diff],[cols,rows]])
    pts2 = np.float32([[0,0],[cols,0],[0,rows],[cols,rows]])
    M = cv2.getPerspectiveTransform(pts1,pts2)
    dst = cv2.warpPerspective(image,M,(cols,rows))
    return dst, value + 0.05

augment_list = \
    [
     lambda x: get_img(x, 0),
     lambda x: get_img(x, 1),
     lambda x: get_img(x, 2),
     lambda x: image_flip(*get_img(x, 0)),
     lambda x: image_flip(*get_img(x, 1)),
     lambda x: image_flip(*get_img(x, 2)),
    ]


def generator(samples, batch_size=32):
    num_samples = len(samples)

    while 1:  # Loop forever so the generator never terminates
        samples = shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            measurements = []
            for line, aug_index in batch_samples:
                img, m = augment_list[aug_index](line)
                images.append(img)
                measurements.append(m)
            x_data = np.array(images)
            y_data = np.array(measurements)
            yield shuffle(x_data, y_data)


def model_seq():
    model = Sequential()
    model.add(Cropping2D(cropping=((50, 20), (0, 0)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: x / 255.0 - 0.5))
    model.add(Convolution2D(10, 3, 3, activation='relu', init='he_normal'))
    model.add(Convolution2D(10, 3, 3, activation='relu', init='he_normal'))
    model.add(Convolution2D(10, 3, 3, init='he_normal'))
    model.add(BatchNormalization())
    model.add(Activation('relu'))
    model.add(MaxPooling2D())
    model.add(Convolution2D(11, 3, 3, activation='relu', init='he_normal'))
    model.add(MaxPooling2D())
    model.add(Convolution2D(13, 3, 3, activation='relu', init='he_normal'))
    model.add(MaxPooling2D())
    model.add(Convolution2D(15, 3, 3, activation='relu', init='he_normal'))
    model.add(MaxPooling2D())
    model.add(Flatten())
    model.add(Dense(200, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dropout(0.6))
    model.add(Dense(120, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(1))
    return model


def model_ybr():
    input_img = Input(shape=(160, 320, 3))
    root = Cropping2D(cropping=((50, 20), (0, 0)))(input_img)
    root = Lambda(lambda x: x / 255.0 - 0.5)(root)
    root = Convolution2D(20, 3, 3, activation='relu')(root)
    root = Convolution2D(20, 3, 3, activation='relu')(root)
    root = Convolution2D(20, 3, 3, activation='relu')(root)
    root = BatchNormalization()(root)

    branch_a = Convolution2D(20, 3, 3, activation='relu')(root)
    branch_a = MaxPooling2D(pool_size=(2, 4))(branch_a)
    branch_a = Convolution2D(13, 3, 3, activation='relu')(branch_a)
    branch_a = MaxPooling2D(pool_size=(2, 4))(branch_a)
    branch_a = Convolution2D(15, 3, 3, activation='relu')(branch_a)
    branch_a = BatchNormalization()(branch_a)
    branch_a = Convolution2D(20, 3, 3, activation='relu')(branch_a)
    branch_a = Convolution2D(20, 3, 3, activation='relu')(branch_a)
    branch_a = Convolution2D(20, 3, 3, activation='relu')(branch_a)
    branch_a = MaxPooling2D()(branch_a)
    branch_a = BatchNormalization()(branch_a)
    branch_a = Flatten()(branch_a) # 500
    branch_a = Dropout(0.5)(branch_a)

    branch_b = Convolution2D(20, 5, 5, activation='relu')(root)
    branch_b = MaxPooling2D(pool_size=(4, 8))(branch_b)
    branch_b = Convolution2D(13, 3, 3, activation='relu')(branch_b)
    branch_b = MaxPooling2D(pool_size=(2, 4))(branch_b)
    branch_b = Convolution2D(20, 5, 5, activation='relu')(branch_b)
    branch_b = BatchNormalization()(branch_b)
    branch_b = Flatten()(branch_b)  # 500
    branch_b = Dropout(0.5)(branch_b)

    base = merge([branch_a, branch_b], mode='concat')
    base = Dense(100)(base)
    base = Activation('relu')(base)
    #base = Dropout(1.0)(base)
    base = Dense(120)(base)
    base = Activation('relu')(base)
    base = Dense(1)(base)
    model = Model(input=input_img, output=base)
    return model


def model_basic():
    model = Sequential()
    model.add(Cropping2D(cropping=((50, 20), (0, 0)), input_shape=(160, 320, 3)))
    model.add(Convolution2D(10, 3, 3, activation='relu', init='he_normal'))
    model.add(MaxPooling2D(pool_size=(2, 4)))
    model.add(Convolution2D(10, 3, 3, activation='relu', init='he_normal'))
    model.add(MaxPooling2D(pool_size=(2, 4)))
    model.add(Convolution2D(10, 5, 5, activation='relu', init='he_normal'))
    model.add(MaxPooling2D())
    model.add(Flatten())
    model.add(Dense(50, init='he_normal'))
    model.add(Activation('relu'))
    model.add(Dense(1, init='he_normal'))
    return model


def menu():
    parser = argparse.ArgumentParser(description='Setup training of the model')
    parser.add_argument(
        'model_name',
        type=str,
        default='model_basic',
        help='which model is trained')

    parser.add_argument(
        '--corr',
        type=float,
        default=0.2,
        help='The  correction value for left and right images'
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
    samples = get_data_list()
    if args.num:
        samples = samples[0:args.num]
    models = \
        {
            'model_seq': model_seq,
            'model_ybr': model_ybr,
            'model_basic': model_basic
        }
    global s_corr
    s_corr = args.corr

    global s_epoch
    s_epoch = args.epoch
    return samples, models[args.model_name](), args.model_name


def main():
    samples, model, result_name = menu()

    print('Training on num samples ', len(samples))
    print('Epoch ', s_epoch)
    print('Steering correction is ', s_corr)
    print('Data from directory ' + s_dir)
    print('Model name ' + result_name)
    train_samples, validation_samples = train_test_split(samples, test_size=0.2)
    # compile and train the model using the generator function
    train_generator = generator(train_samples, batch_size=32)
    validation_generator = generator(validation_samples, batch_size=32)

    model.compile(loss='mse', optimizer='adam')

    saver = ModelCheckpoint(
        filepath='result/'+result_name + str(s_corr)+'best.h5',
        monitor='val_loss',
        verbose=0,
        save_best_only=True,
        save_weights_only=False,
        mode='auto',
        period=1)
    killer = EarlyStopping(
        monitor='val_loss',
        min_delta=0.00001,
        patience=5,
        verbose=0,
        mode='auto')
    train_history = model.fit_generator(
        train_generator,
        samples_per_epoch=len(train_samples),
        validation_data=validation_generator,
        nb_val_samples=len(validation_samples),
        nb_epoch=s_epoch,
        callbacks=[saver, killer])
    model.save('result/'+result_name + str(s_corr)+'.h5')
    with open('result/p_' + result_name + str(s_corr) + '.p', 'wb') as f:
        pickle.dump(train_history.history, f)

if __name__ == '__main__':
    main()
