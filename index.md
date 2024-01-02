---
title: Relative Geometry Constraints in Ceres
---

<p align="center">
  <img src="diagrams/intro_car_side.png" alt="intro car side image"/>
</p>


# Introduction

Structure-from-motion is the most commonly used pipeline for creating 3D models from 2D images. Image feature-matches are fed into an optimization algorithm known as Bundle Adjustment, which simultaneously estimates the positions of the cameras, as well as a 3D point cloud of all the matched pixels. The diagram at the top of this page depicts the entire output of Bundle Adjustment, with the estimated position of 6 cameras, as well as the reconstructed object (a red car). Each camera in this simulation has a focal length of 525, and an image dimension of 600x600. Below are six images, rendered from each camera's orientation. The green lines indicate the camera's viewing direction.

<p align="center">
  <img src="diagrams/cars.png" alt="cars image"/>
</p>




