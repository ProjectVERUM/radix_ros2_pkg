# Behaviors

Behaviors are the main way to configure the behavior of the map in more detail.
This is done by using a strategy pattern with templates.

You can find the current different behaviors in the `radix_map/include/radix_map/behavior` folder.
You can switch between them by changing the `behavior` parameter in the `radix_ros/params/main_params.yaml`.

The name of the behavior is defined in the `radix_map/include/radix_map/map_factory.hpp` file, which maps the name to the class.

Currently the following behaviors are available:

## Basic

The Basic behavior is the simplest behavior. It takes new label points and stores them in a limited history queue within each voxel, specifically keeping only the last five seen labels. The voxel maintains a label parameter which is always synchronized to reflect the most recently seen label from the history queue.

## Semantic

The Semantic behavior builds on the basic behavior by maintaining a histogram of label probabilities for each voxel, storing the likelihood of all observed labels over time. As new points are added, the voxel updates these probabilities using an exponential moving average, effectively capturing the weighted distribution of labels within the voxel. For publishing, it selects the label with the highest probability along with its associated confidence.


## Gaussian

The Gaussian behavior extends the Semantic behavior by enhancing the geometric representation of the voxel's content. It maintains the existing histogram of label probabilities while simultaneously tracking Gaussian statistics for the points contained within the voxel.This involves modeling the 3D spatial distribution of the observed points by incrementally calculating and storing the mean and the covariance matrix.
For publishing, the behavior outputs a combined semantic and geometric representation: the label with the maximum probability along with its associated confidence, and the calculated Gaussian statistics.
