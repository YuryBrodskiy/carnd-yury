[//]: # (Image References)

[original_train_set]: ./report_img/original_train_set.png "Visualization"
[hist_dataset]: ./report_img/hist_dataset.png "Distribution"
[enh_image]: ./report_img/enh_img.png "Image enhance"
[aug_image]: ./report_img/aug_img.png "Image modified"
[my_softmax]: ./report_img/softmax_my.png "Softmax visualization"
[test1]: ./test_img/test1.png "Curve to the right"
[test2]: ./test_img/test2.png "Speed limit 20"
[test3]: ./test_img/test3.png "Stop"
[test4]: ./test_img/test4.png "Speed limit 20"
[test5]: ./test_img/test5.png "Dangerous curve"

 
 # Traffic Sign Recognition Project 
 
 ## Abstract
 
The goals of this project are the following:
* Explore, summarize and visualize given data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images 
 
A new Neural Network arhcitecture was build for Traffic Sign Recognition based on literature review. This architecture is capable of achiving validation accuracy 96.5% and test accuracy   94.3% on given sets. It also capable of generalizing the photo images to conceptual images of a sign, but it only shows 50% accuracy on complex images chosen from Internet.

The code is accesible on [github link](https://github.com/...)
 
 ## Data Set Summary & Exploration

The provided data bundle consist of 34799 images available for training, 4410 images for validation and 12630 images for the final test. Each image is a 32 by 32 pixels with three 8bit-channels of RGB colors. There are 43 categories of traffic signs. (Summary is attained in 2nd cell of notebook)

The images for training data set are visualized below by displaying one sample from first 26 categories sorted by class id. It is interesting to note that without image processing recognizing the traffic signs in this data set can be challenging even of an experienced driver. The examples of such complicated images are in column 2 of rows 3 and 4. (showUnique  function in 3 cell i used to create the figure)

![alt text][original_train_set]

The distribution of images over the categories of traffic signs in training and validation set are visualized below using a histogram. It is, arguably, the most important information about the data set. The histogram of the training set(green) shows that there are almost 10 times more images of class 1 then there are of class 0. Such strong disbalance in available images will create a prediction bias in the neural network after training. It is because neural networks capture statistical information of the data set as well as relations between image content and class.  Furthermore, the validation set contains similar disbalance. Thus validation accuracy will be high due to the statical expectation of specific classes, while this assumption is not valid for an unknown "test" data set.

![alt text][hist_dataset]

In summary, the data set contains low-quality images with low resolution, color depth, and contrast. The data set has an uneven distribution over image classes, that may hamper the prediction model.

 ## Image preprocessing
 
It is clearly preferable that any the neural network learn all necessary processing of the data, as it will result in simpler and more uniform implemenation. However, [P. Sermanet and Y. LeCun](http://yann.lecun.com/exdb/publis/pdf/sermanet-ijcnn-11.pdf)  clearly state that switching away from RGB data repsentation and contrast normalization provide significant improvement during classification task.

During this project, three color representations RGB, YUV, HSV were tested. Using YUV or HSV results in ~2-3 % improvement on the accuracy of classification on the validation set compare to RGB. Therefore HSV representation is used for final report. 

To improve image contrast, a histogram equalization is used on "value" channel  (brightness of a color) of the HSV image representation. This operation calculates a distribution of the "value" over the image and stretches the distribution to a complete available range.  This results in an increase of global and local contrast with minimal change in image content. It does, however, result in discolorization, i.e. color shift artifacts on the image. This technique also improves the accuracy of classification.

The image is then cast to float representation; this effectively means that all values in the image are range from 0.0 to 1.0. It is done to facilitate the training process as smaller numbers should increase the initial convergence. The data is not zero centered, as several experiments did not show any improvement if it is done.

The code for image preprocessing is defined in function enhance_img(cell 5 of the notebook). The original and preprocessed images are shown below.

![alt text][enh_image]

 ## Data set augmentation
 
Unfortunately, as can be seen for the histogram above there only 180 images are available for "Speed limit (20km/h)" sign and several other signs also have a small number of images.  This can result that network is not able to learn those classes very well. Therefore the original data set is augmented with modified images. All images are subjected to deformations and added into the data set. 
Following deformation are applied to the images (augmentation functions are defined in cell 6):
 + zoom 110% and crop to size in center
 + zoom 110% and crop to size up left corner 
 + zoom 110% and crop to size down right corner
 + rotate clockwise and counterclockwise 10 deg
 + Gaussian filter with kernel 3
 + 4 perspective deformations are applied to imitate photos taken from various angles
The figure below illustrate the augmented set for one image:

![alt text][aug_image]
 
Using these augmentations expand the train data set up to 347990 images. This new dataset still has a significant disbalance in the number of images per sign class. That will be addressed at training time.

## Design and Test a The Neural Network 

<img style="float: right;height:800px;" src="./report_img/ybrnet.png"/>

### Network architecture

The architecture of the final neural network is depicted in the figure on the right (defined in cell 10). The network can be divided into three conceptual stages. 

The first stage receives the preprocessed image. The stage consists of two convolution layers with ReLu activations and separated by dropout layer. The intention of the stage is to extract low-level features from pixel values.  

The second stage receives the low-level features extracted by the first. The second stage consists of two branches. The right branch does a fine grain processing. The convolutional filters and max pool operation are chosen to smallest size with the intention to capture the local information about the image. The left branch is on the other hand, quickly reduce the dimensionality of the problem using MaxPool operation with big filter size and big stride. This branch is intended to grasp global information about the image.

The third stage consists of two fully connected layers. It is the primal task to use features extracted by the second stage to perform the actual classification task. Two layers are used to allow the network to learn the ["kernel trick"](https://people.eecs.berkeley.edu/~jordan/courses/281B-spring04/lectures/lec3.pdf) for the separation of classes. 

### Network design process

For the purpose of gaining more experience with TensorFlow, I have chosen to design my network. The design is heavily relying on literature suggestions, with minor adjustments made as results of experimentation.

The work of [P.Sermanet and Y.LeCun](http://yann.lecun.com/exdb/publis/pdf/sermanet-ijcnn-11.pdf) and LeNet-5 were used a inspiration for the design. The LeNet-5 was extended with a skip connection as suggested by LeCun. This boosts the accuracy of classification ~ 2-5%.

Sequential 3x3 convolutions replaced 5x5 convolutions, as recommended in [Stanford CS231n](https://www.youtube.com/watch?v=pA4BsUK3oP4&index=11&list=PLwQyV9I_3POsyBPRNUU_ryNfXzgfkiw2p). It reduces the number of tunable parameters from 25 per filter to total of 18 per filter (9x2 layers).  It also increases model capacity due to additional nonlinearity. 

Furthermore, one fully connected layer was removed. An experiment showed that it has little effect on quality of classification in the validation set. Moreover, it significantly reduces the number of parameters to train. Instead, all convolution layer were deepened to have more filters per layer thus capturing more features.

Dropout layers are used for regularization. Dropout layers are inserted after the first convolution layer to increase robustness to image changes and after each branch and after first fully connected layer to force redundant classification paths in the network. The outputs are  logits for Softmax classification.

### Network Training

For training the network and Adam optimization algorithm is used, its adaptive nature minimizes the effect of learning rate choice. It also means that there is no need to update the learning rate during the training.

Therefore main tuning hyperparameter (apart from network architecture) is dropout rate.

An overfit test is being used to determine suitable values of dropout rate.  During the overfit test, the network is deliberately forced to overfit the training set. This is achieved by training on very small of images (2 images per class) until 100% accuracy. This test shows that reducing dropout rate below 75% will reduce network capacity to the level where even such small set of images cannot be classified with sufficient accuracy.  87% dropout rate provide a good trade-off between learning rate, network capacity, and generalization

The several experiments showed that L2 regularization does not give the positive effect on the training process or classification accuracy on the validation set. It is not used during final training

The training is done over 100 epochs with a batch size of 128 images. The significant change to training process is adjusting the probability of appearance of each class in the batch. The goal is to have a flat distribution of each class in training set during one epoch. An even distribution prevents the network to expect some classes appear more often, thus the classification is only done based on image content. 

Thus training iteration consists of 5 steps:

 1. Shuffle the total training set
 2. Select equal number of images from each class ( using based on minimal number of images available for one class)
 3. Shuffled smaller resulting set again to prevent dependency on the order
 4.  Use SGD with batches of images from evenly distributed image set to minimize cross entropy loss with logits
 5. Evaluate validation accuracy and estimate training set accuracy using subset of training data

Final results for this architecture are:
 
 | Set name       | Num Images  | accuracy         |
 |:---------------|------------:|:----------------:|
 | training       |347990       |        0.994     | 
 | validation     |4410         |        0.965     | 
 | test           |12630        |        0.943     | 

 ## Test a Model on New Images

From the web search, 6 traffic signs were selected. The selection criteria  described in the table:

 | Sign image                                                       | Class| Name          | Selection Criteria         |
 |:-----------------------------------------------------------------|:----:|:--------------|:-----------------|
 | <img style="height:64px; width:64px" src="./test_img/test1.png"/>|  23  |Slippery road  | Slightly abstructed by snow  | 
 | <img style="height:64px; width:64px" src="./test_img/test2.png"/>|  14  |Stop           | Heavily abstructed by snow.  | 
 | <img style="height:64px; width:64px" src="./test_img/test3.png"/>|  0   |Speed limit 20 | A conceptual image.\* Few availble photos in training data set   | 
 | <img style="height:64px; width:64px" src="./test_img/test4.png"/>|  20  |Curve to the right| A conceptual image.\*     | 
 | <img style="height:64px; width:64px" src="./test_img/test5.png"/>|  0   |Speed limit 20    | Few availble photos in training data set    |  
 | <img style="height:64px; width:64px" src="./test_img/test6.png"/>|  0   |Speed limit 20    | Same  image but scaled to 32x32px   | 

 \* A conceptual image is not a photo, and therefore significantly differs from the training, validation or test data sets. It should demonstrate how good the network can generalize from photos. 
 
 Images with different levels of obstruction a significantly more challenging to recognize. For example, to recognize the chosen stop sign a driver mostly relies on shape rather than on content. content.

### Discussion of the results

<img style="width:300px; float: right" src="./report_img/softmax_my.png"/>

All images are scaled to fit 32x32 px using linear interpolation. Each image is then processed using histogram equalization as described above. The network was used to predict the classes. The figure on the right displays the results. The number on the right of each image is the actual class of the image. The bar chart next to each image displays top 3 softmax probabilities produced by the network.
 
The first image is a "Slippery road" sign. The scaling down of the image significantly reduced its quality and introduced all kind of artifacts. The result is nearly unrecognizable to a human. However, the network provides a very confident estimate of the class, and it is correct.
 
 The second image is a "Stop" sign. The content of the sign is partially obscured and scaling the image also add some strange aura around the sign. It is how quite recognizable. The estimate for the network is also very confident ~100% but it wrong. Class 34 is a "Turn left ahead" sign.  
 
 Images 3 and 4 are correctly classified. This a good indication that network can generalize from the data. Since it capable of capturing and identifying the concepts just by training on photos of the signs. However, on sign 4 "Curve to the right," the softmax probabilities suggest that other signs also have similar features "Bicycles crossing" and "General caution."
 
 Finally, a big surprise that images 5 and 6 the photo of "Speed limit 20" sign was not recognized correctly.
It seems that scaling down artifacts and a limited number of examples of that type of sign in the training data set resulted in incorrect classification. The most likely suggestions are also "speed limit" signs but for different speed values; in order of decreasing likelihood "Speed limit (50km/h)", "Speed limit (70km/h)" and "Roundabout mandatory."

The summary of the predictions and highest probability is presented in table below:

| Image			     | Probability |     Prediction	    | 
|:------------------:|:-----------:|:------------------:| 
| Slippery road      |   0.993     | Slippery road      | 
| Stop               |   0.999     | Turn left ahead    |
| Speed limit 20     |   0.999     | Speed limit 20     |
| Curve to the right |   0.840     | Curve to the right	|
| Speed limit 20	 |   0.618     | Speed limit 50     |
| Speed limit 20     |   0.626     | Speed limit 70     |

 ### Summary and Conclusion
In this project, the provided data set was analyzed. Both contenet of the images was examined and the statistical properites of the data set. The nesseccary measures were taken to maximze the quality of the generatlization process during traing. 

The traning data set was augmented with distorted images as recommended in the literature. This operation has increased the size of the training set by a factor of 10.

A new network architecture for neural network was proposed. It is a convolutional neural network with depth of 5 non-leanar level and ReLu type of non-learity. The network features a skip connection and dropout layer for redundant classification paths. 

The network was trained and tested on provided data set yielding training accuracy 99.4% validation accuracy of 96.5% and test accuracy of 94.3%. This indicates that network has achieved a relatively good level of generalization for given example. It is, however,  a low degree of accuracy compare to state of the art solutions as published by P.Sermanet and Y.LeCun. Future imporvment can be achieved by adding Batch Norm layers into the network.

6 images were retrived from the web for additional testing. The images are of different qualtiy and resolution. These images were down sampled  to 32x32 px and preprocessed in identical manner as traning set. The network was used to classify  these images yeilding 50% accuracy. This is significatly less then can be expeced based on the results of the test set. It is, however, explainable as the images were deliberatly chose to be complex and a process of down sampling images to lower size was different then for training set. 