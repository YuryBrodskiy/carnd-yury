import argparse
import os.path

import shutil
import tensorflow as tf
import time
import scipy.misc
import helper
import warnings
from distutils.version import LooseVersion
import project_tests as tests
import numpy as np
from tqdm import tqdm

from sklearn.model_selection import train_test_split

# Check TensorFlow Version
assert LooseVersion(tf.__version__) >= LooseVersion('1.0'), \
    'Please use TensorFlow version 1.0 or newer.  You are using {}'.format(tf.__version__)
print('TensorFlow Version: {}'.format(tf.__version__))

# Check for a GPU
if not tf.test.gpu_device_name():
    warnings.warn('No GPU found. Please use a GPU to train your neural network.')
else:
    print('Default GPU Device: {}'.format(tf.test.gpu_device_name()))


def load_vgg(sess, vgg_path):
    """
    Load Pre-trained VGG Model into TensorFlow.
    :param sess: TensorFlow Session
    :param vgg_path: Path to vgg folder, containing "variables/" and "saved_model.pb"
    :return: Tuple of Tensors from VGG model (image_input, keep_prob, layer3_out, layer4_out, layer7_out)
    """
    # TODO: Implement function
    #   Use tf.saved_model.loader.load to load the model and weights
    vgg_tag = 'vgg16'
    vgg_input_tensor_name = 'image_input:0'
    vgg_keep_prob_tensor_name = 'keep_prob:0'
    vgg_layer3_out_tensor_name = 'layer3_out:0'
    vgg_layer4_out_tensor_name = 'layer4_out:0'
    vgg_layer7_out_tensor_name = 'layer7_out:0'

    tf.saved_model.loader.load(sess, [vgg_tag], vgg_path)
    vgg_graph = tf.get_default_graph()
    vgg_input = vgg_graph.get_tensor_by_name(vgg_input_tensor_name)
    vgg_keep_prob = vgg_graph.get_tensor_by_name(vgg_keep_prob_tensor_name)
    vgg_layer3_out = vgg_graph.get_tensor_by_name(vgg_layer3_out_tensor_name)
    vgg_layer4_out = vgg_graph.get_tensor_by_name(vgg_layer4_out_tensor_name)
    vgg_layer7_out = vgg_graph.get_tensor_by_name(vgg_layer7_out_tensor_name)

    return vgg_input, vgg_keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out

if __name__ == '__main__':
    tests.test_load_vgg(load_vgg, tf)


def layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes):
    """
    Create the layers for a fully convolutional network.  Build skip-layers using the vgg layers.
    :param vgg_layer7_out: TF Tensor for VGG Layer 3 output
    :param vgg_layer4_out: TF Tensor for VGG Layer 4 output
    :param vgg_layer3_out: TF Tensor for VGG Layer 7 output
    :param num_classes: Number of classes to classify
    :return: The Tensor for the last layer of output
    """
    # TODO better use batch norm and dropout
    with tf.name_scope("semantic_layer"):
        conv_class = tf.layers.conv2d(vgg_layer7_out, num_classes, 1, padding='same',
                                      kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                      kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())

        conv_class = tf.nn.relu(conv_class)
        conv_4 = tf.layers.conv2d(vgg_layer4_out, num_classes, 1, padding='same',
                                  kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                  kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())
        conv_4 = tf.nn.relu(conv_4)
        conv_3 = tf.layers.conv2d(vgg_layer3_out, num_classes, 1, padding='same',
                                  kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                  kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())

        conv_3 = tf.nn.relu(conv_3)

        output = tf.layers.conv2d_transpose(conv_class, num_classes, 4, 2, padding='same',
                                            kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                            kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())
        output = tf.nn.relu(output)
        output = tf.add(output, conv_4)
        output = tf.layers.conv2d_transpose(output, num_classes, 4, strides=(2, 2), padding='same',
                                            kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                            kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())
        output = tf.nn.relu(output)
        output = tf.add(output, conv_3)

    image_ss = tf.layers.conv2d_transpose(output, num_classes, 16, strides=(8, 8), padding='same', name="image_ss",
                                          kernel_regularizer=tf.contrib.layers.l2_regularizer(1e-3),
                                          kernel_initializer=tf.contrib.layers.xavier_initializer_conv2d())
    tf.add_to_collection("image_ss", image_ss)
    return image_ss

if __name__ == '__main__':
    tests.test_layers(layers)


def initialize_uninitialized(sess):
    '''
    snippet from https://stackoverflow.com/questions/35164529/in-tensorflow-is-there-any-way-to-just-initialize-uninitialised-variables
    '''
    global_vars          = tf.global_variables()
    is_not_initialized   = sess.run([tf.is_variable_initialized(var) for var in global_vars])
    not_initialized_vars = [v for (v, f) in zip(global_vars, is_not_initialized) if not f]

    #print ([str(i.name) for i in not_initialized_vars]) # only for testing
    if len(not_initialized_vars):
        sess.run(tf.variables_initializer(not_initialized_vars))
    sess.run(tf.local_variables_initializer())
    assert len(sess.run(tf.report_uninitialized_variables())) == 0, "Everything is initialized"
    return not_initialized_vars


def optimize(nn_last_layer, correct_label, learning_rate, num_classes, sess = None):
    """
    Build the TensorFLow loss and optimizer operations.
    :param nn_last_layer: TF Tensor of the last layer in the neural network
    :param correct_label: TF Placeholder for the correct label image
    :param learning_rate: TF Placeholder for the learning rate
    :param num_classes: Number of classes to classify
    :param sess: if None all network is trained, else only uninitialized parameters
    :return: Tuple of (logits, train_op, cross_entropy_loss)
    """
    # TODO: Implement function
    logits = tf.reshape(nn_last_layer, (-1, num_classes), name="logits")
    cross_entropy_loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=correct_label))

    class_gt = tf.argmax(correct_label, axis=-1)
    class_predicted = tf.argmax(nn_last_layer, axis=-1)

    iou, iou_op = tf.metrics.mean_iou(class_gt, class_predicted, num_classes, name="mean_iou")

    optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate)
    if sess is not None:
        train_vars = initialize_uninitialized(sess)
        training_op = optimizer.minimize(cross_entropy_loss, var_list=train_vars)
        sess.run(tf.local_variables_initializer())
    else:
        training_op = optimizer.minimize(cross_entropy_loss)

    tf.add_to_collection("iou_value", iou)
    tf.add_to_collection("iou_op", iou_op)

    tf.add_to_collection("logits", logits)
    tf.add_to_collection("loss_operation", cross_entropy_loss)
    tf.add_to_collection("training_operation", training_op)
    tf.add_to_collection("class_predicted", class_predicted)
    return logits, training_op, cross_entropy_loss

if __name__ == '__main__':
    tests.test_optimize(optimize)


def train_nn(sess, epochs, batch_size, get_train_batches, get_valid_batches, train_op, cross_entropy_loss, input_image,
             correct_label, keep_prob, learning_rate, output_dir=None):
    """
    Train neural network and print out the loss during training.
    :param sess: TF Session
    :param epochs: Number of epochs
    :param batch_size: Batch size
    :param get_train_batches: Function to get batches of training data.  Call using get_batches_fn(batch_size)
    :param get_valid_batches: Function to get batches of validation data.  Call using get_batches_fn(batch_size)
    :param train_op: TF Operation to train the neural network
    :param cross_entropy_loss: TF Tensor for the amount of loss
    :param input_image: TF Placeholder for input images
    :param correct_label: TF Placeholder for label images
    :param keep_prob: TF Placeholder for dropout keep probability
    :param learning_rate: TF Placeholder for learning rate
    :param output_dir: Log directory
    """
    # TODO: Implement function
    initialize_uninitialized(sess)
    sess.run(tf.local_variables_initializer())
    if output_dir is not None:
        images_for_summary, _ = next(get_valid_batches(1))
        writer = tf.summary.FileWriter(os.path.join(output_dir, 'log'), graph=sess.graph)
        tf.summary.scalar('valid loss', tf.placeholder(tf.float32, name="valid_loss_summary"))
        tf.summary.scalar('valid IoU', tf.placeholder(tf.float32,  name="valid_IoU_summary"))
        tf.summary.scalar('train loss', tf.placeholder(tf.float32, name="train_loss_summary"))
        tf.summary.scalar('train IoU', tf.placeholder(tf.float32,  name="train_IoU_summary"))
        image_output = tf.nn.softmax(tf.get_collection("logits")[0])
        tf.summary.image('ss_image_summary', tf.placeholder(tf.float32, [None, None, None, 3],  name="ss_image_summary"))
        summary = tf.summary.merge_all()
        saver = tf.train.Saver()

    if len(tf.get_collection("iou_value")) != 0:
        iou = tf.get_collection("iou_value")[0]
        iou_op = tf.get_collection("iou_op")[0]
    else:
        iou = tf.constant(0.0)
        iou_op = tf.constant(0.0)

    print("Training...")
    for i in range(epochs):
        print("EPOCH {} ...".format(i + 1))
        total_loss_train = 0
        num_examples_train = 0
        sess.run(tf.local_variables_initializer())
        for batch_x, batch_y in tqdm(get_train_batches(batch_size)):
            loss, _, _  = sess.run([cross_entropy_loss, train_op, iou_op],
                                   feed_dict={
                                     input_image: batch_x,
                                     correct_label: batch_y,
                                     keep_prob: 0.7,
                                     learning_rate: 1e-3}
                     )
            num_examples_train += len(batch_x)
            total_loss_train += (loss * len(batch_x))
        total_iou_train = sess.run(iou)
        total_loss_train = total_loss_train / num_examples_train
        print("Train  Loss = {:.3f}".format(total_loss_train), "IoU  = {:.3f}".format(total_iou_train))

        total_loss_valid = 0
        num_examples_valid = 0
        sess.run(tf.local_variables_initializer())

        for batch_x, batch_y in tqdm(get_valid_batches(batch_size)):
            loss, _ = sess.run([cross_entropy_loss, iou_op],
                                            feed_dict={
                                             input_image: batch_x,
                                             correct_label: batch_y,
                                             keep_prob: 1.0})
            total_loss_valid += (loss * len(batch_x))
            num_examples_valid += len(batch_x)

        total_iou_valid = sess.run(iou)
        total_loss_valid = total_loss_valid / num_examples_valid
        if output_dir is not None:
            image_shape = (160, 576)
            im_softmax = sess.run(image_output, feed_dict={keep_prob: 1.0, input_image: images_for_summary})
            im_softmax = im_softmax[:, 1].reshape(image_shape[0], image_shape[1])
            segmentation = (im_softmax > 0.5).reshape(image_shape[0], image_shape[1], 1)
            mask = np.dot(segmentation, np.array([[0, 255, 0, 127]]))
            mask = scipy.misc.toimage(mask, mode="RGBA")
            street_im = scipy.misc.toimage(images_for_summary.reshape(image_shape[0], image_shape[1], 3))
            street_im.paste(mask, box=None, mask=mask)

            writer.add_summary(sess.run(summary,
                                        feed_dict={
                                            "valid_loss_summary:0": total_loss_valid,
                                            "valid_IoU_summary:0": total_iou_valid,
                                            "train_loss_summary:0": total_loss_train,
                                            "train_IoU_summary:0": total_iou_train,
                                            "ss_image_summary:0": [np.asarray(street_im)],
                                        }), i+1)

        print("Valid  Loss = {:.3f}".format(total_loss_valid), "IoU  = {:.3f}".format(total_iou_valid))
    if output_dir is not None:
        saver.save(sess, os.path.join(output_dir, 'ybr_VGG_SS_vA'))
        print("Model saved")


if __name__ == '__main__':
    tests.test_train_nn(train_nn)



def run(transfer):
    num_classes = 2
    image_shape = (160, 576)
    data_dir = './data'
    runs_dir = './runs'
    output_dir = os.path.join(runs_dir, str(time.time()))
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    tests.test_for_kitti_dataset(data_dir)

    # Download pretrained vgg model
    helper.maybe_download_pretrained_vgg(data_dir)

    # OPTIONAL: Train and Inference on the cityscapes dataset instead of the Kitti dataset.
    # You'll need a GPU with at least 10 teraFLOPS to train on.
    #  https://www.cityscapes-dataset.com/

    with tf.Session() as sess:
        # Path to vgg model
        vgg_path = os.path.join(data_dir, 'vgg')


        # OPTIONAL: Augment Images for better results
        #  https://datascience.stackexchange.com/questions/5224/how-to-prepare-augment-images-for-neural-network

        # Done: Build NN using load_vgg, layers, and optimize function
        vgg_input, vgg_keep_prob, vgg_layer3_out, vgg_layer4_out, vgg_layer7_out = load_vgg(sess, vgg_path)
        image_ss = layers(vgg_layer3_out, vgg_layer4_out, vgg_layer7_out, num_classes)
        correct_label = tf.placeholder(tf.float32, [None, None, None, num_classes], name="correct_label")
        learning_rate = tf.placeholder(tf.float32, name="learning_rate")
        if transfer:
            logits, train_op, cross_entropy_loss = optimize(image_ss, correct_label, learning_rate, num_classes, sess)
        else:
            logits, train_op, cross_entropy_loss = optimize(image_ss, correct_label, learning_rate, num_classes)

        X, y = helper.augmentData(*helper.getData(os.path.join(data_dir, 'data_road/training')), image_shape)
        X_train, X_valid, y_train, y_valid = train_test_split(X, y, test_size=0.1)

        get_train_batches = helper.gen_batch_function_load(X_train, y_train)
        get_valid_batches = helper.gen_batch_function_load(X_valid, y_valid)



        # TODO: Train NN using the train_nn function

        train_nn(sess, 10, 8, get_train_batches, get_valid_batches, train_op, cross_entropy_loss, vgg_input,
                 correct_label, vgg_keep_prob, learning_rate, output_dir)

        # TODO: Save inference data using helper.save_inference_samples

        helper.save_inference_samples(os.path.join(output_dir, 'images'), data_dir, sess, image_shape, logits, vgg_keep_prob, vgg_input)

        # OPTIONAL: Apply the trained model to a video


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Setup training of the model')
    parser.add_argument(
        'type',
        type=str,
        default='transfer',
        help='which model is trained')
    args = parser.parse_args()
    types = {'transfer': True, 'scratch': False}
    print(args.type)
    run(types[args.type])

