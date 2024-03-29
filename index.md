---
title: Geometric Priors in Bundle Adjustment
---

### [Author: John (Jack) Messerly](https://www.linkedin.com/in/jack-messerly-567b9b96/)
### [main page](https://jp-mess.github.io/blog-main/)
### [code](https://github.com/jp-mess/ceres-geometric-constraints)

<br>
<br>  

<p align="center">
  <img src="diagrams/intro_car.png" alt="Intro Car"/>
</p>

<br>


# What is a geometric prior?

**First, a (very) brief review of 3D computer vision:** Structure-from-motion is the most commonly used pipeline for creating 3D models from 2D images. In structure-from-motion, images (and their feature matches) are fed into an optimization algorithm known as Bundle Adjustment, which simultaneously estimates the positions of the cameras, as well as a 3D point cloud of all the matched pixels. The diagram at the top of this page depicts the entire output of a simulated Bundle Adjustment, with the estimated position of 6 cameras, as well as the reconstructed object (a red car). Each camera in this simulation has a focal length of 525, and an image dimension of 600x600. The green lines indicate the camera's viewing direction. Below are the 6 images that were taken by these 6 cameras, and then used to reconstruct the car in the above diagram:

<br>

<p align="center">
  <img src="diagrams/cars.png" alt="cars image" style="width: 75%; height: auto;"/>
</p>

<br>


**Geometric Priors:** What if we know (beforehand) that all the cameras in this system lie on a ring, like in the diagram above? Could we use that information be helpful in estimating the positions of the 6 cameras? This is what I would call a "geometric prior", and in real-world applications this is a common scenario because the cameras will be on some sort of rig (or robot). It could also be called a "geometric constraint", but that would imply it makes the optimization more difficult, when in reality it makes it easier, and the output global error will be lower, since vanilla bundle adjustment works on reprojection error alone, and has no global reference of where to put its output. 

Any relative geometric constraint imposed on the cameras can be implemented as a reparameterization, and it generally makes the optimization more efficient if you have a reasonable prior estimate of that geometry (i.e. you know the ring should be roughly horizontal and above the car, not below it). Instead of learning all of the 3D positions of the cameras independently, ceres can learn the 6 parameters for the 3D ring, and then for each camera, simply learn the angle of the ring that each camera falls on. This trick can also work for:

1. Planes, lines, or other geometric shapes with a small number of parameters
2. Relative distances (enforce all cameras to be the same distance away)

Below is a visualization of what this means; in addition to the rough positions of the cameras, we have a rough idea of what kind of ring the cameras will fall on. Bundle Adjustment can solve for both simultaneously.

<p align="center">
  <img src="diagrams/optimization_with_prior.png" alt="Optimization with Geometric Constraints"/>
</p>

<br>
<br>

# Setting up a basic bundle adjustment problem

I assume the reader is familiar with textbook Bundle Adjustment and nonlinear least-squares. I've added a section here which goes over how to build a cost function and set parameter blocks in ceres, since these are the things that will be modified to implement geometry constraints. Ceres will store the parameters you want to estimate in "parameter blocks", which are pointers to contiguous places in memory where those parameters are. I've illustrated the parameter block for the extrinsic parameters in the system. In thoe code, I've added these as a `camera_manifold`, so that the quaternions can be estimated more efficiently. The world points are in a secondary block, and the intrinsic parameters are in a third. You can see in the code that I've frozen the intrinsic parameters.

<br>

<p align="center">
  <img src="diagrams/camera_params.jpg" alt="Camera Parameters"/>
</p>

<br>

## Some basic parameter blocks


```cpp
for (int i = 0; i < quat_problem.num_observations(); ++i) {
  ceres::CostFunction* cost_function = QuatCost::Create(observations[2 * i + 0], observations[2 * i + 1]);
  
  // Get the entire camera parameter block (which includes quaternion, translation, and intrinsics)
  double* extrinsics = quat_problem.mutable_extrinsic_for_observation(i);
  double* intrinsics = quat_problem.mutable_intrinsic_for_observation(i);
  double* point = quat_problem.mutable_point_for_observation(i);

  ceres::Manifold* camera_manifold = new ceres::ProductManifold<ceres::QuaternionManifold, ceres::EuclideanManifold<3>>{};

  problem.AddParameterBlock(intrinsics, 3);
  problem.SetParameterBlockConstant(intrinsics);

  problem.AddParameterBlock(extrinsics, 7, camera_manifold);

  // Add the residual block to the problem.
  problem.AddResidualBlock(cost_function, nullptr /* squared loss */, extrinsics, intrinsics, point);
}
```

<br>

## A basic cost function (reprojection error)

A reminder that the pose matrix of a camera rotates a point from the camera's optical frame, to the world frame, so an inverse is needed for reprojection error.

```cpp
template <typename T>
bool operator()(const T* const extrinsic_params, // Camera parameters
                const T* const intrinsic_params, // frozen intrinsics
                const T* const point,  // 3D point
                T* residuals) const {

  // Camera parameters: quaternion (4), translation (3), intrinsics (3)
  const T* quaternion = extrinsic_params;
  const T* translation = extrinsic_params + 4;
  const T* intrinsics = intrinsic_params;

  // Conjugate of the quaternion for inverse rotation.
  // Eigen uses x,y,z,w format when loading in quaternions this way
  T conjugate_quaternion[4] = {quaternion[0], 
                               -quaternion[1], 
                               -quaternion[2], 
                               -quaternion[3]};

  // Apply inverse translation: point - translation.
  T translated_point[3] = {point[0] - translation[0],
                           point[1] - translation[1],
                           point[2] - translation[2]};


  // Rotate the translated point using the conjugate of the camera quaternion.
  T rotated_translated_point[3];
  ceres::QuaternionRotatePoint(conjugate_quaternion, translated_point, rotated_translated_point);

  // Project the 3D point onto the 2D camera plane.
  const T& focal = intrinsics[0];
  const T& cx = intrinsics[1];
  const T& cy = intrinsics[2];

  const T xp = rotated_translated_point[0] / rotated_translated_point[2];
  const T yp = rotated_translated_point[1] / rotated_translated_point[2];

  const T predicted_x = focal * xp + cx;
  const T predicted_y = focal * yp + cy;

  // The error is the difference between the predicted and observed positions.
  residuals[0] = predicted_x - observed_x;
  residuals[1] = predicted_y - observed_y;
}
```

<br>
<br>


# Adding a "ring" geometric prior

To implement ring constraints, we don't need to adjust the cost function at all. Instead, we update what's stored in the parameter blocks, and how we take the reprojection error. The full 3D [x, y, z] point in each camera is replaced by a single `theta` parameter, because each camera's 3D position can be reduced to the angle at which it lives on the ring. This means we have to update our code to include functions that project 3D points to the ring, and vice versa. We also add some parameter blocks for the 8 total ring parameters (the ring's center point, its quaternion orientation, and its radius), and estimate those simultaneously with the camera positions (represented by a `theta` for the translation, and a quaternion for the orientation), as well as the 3D world points. In theory, we've reduced our workload. In the above basic example, we had 6 x 10 = 60 total camera parameters, whereas now we have (6 x 5) + 8 = 38 total camera parameters.

<br>

<p align="center">
  <img src="diagrams/ring_drawing.jpg" alt="Ring Drawing" style="width: 85%; height: auto;"/>
</p>

<br>

<p align="center">
  <img src="diagrams/ring_camera_params.jpg" alt="Ring + Camera Parameters" style="width: 85%; height: auto;"/>
</p>

<br>

<p align="center">
  <img src="diagrams/just_ring_params.jpg" alt="Ring Parameters" style="width: 85%; height: auto;"/>
</p>

<br>
<br>

## Projecting a 3D point to an angle on the ring

This will be used to initialize the problem (we want to project our initial 3D points onto a ring, so that we can change the parameter blocks).

```cpp
double ProjectPointOntoRing(const Eigen::Vector3d& point, 
                            const Eigen::Vector3d& center, 
                            const Eigen::Quaterniond& ring_orientation) {
    // Step 1: Translate the point to the ring's coordinate system
    Eigen::Vector3d translated_point = point - center;

    // Step 2: Rotate the translated point to align with the ring's local coordinate system
    // Inverse rotation is used to bring the point into the ring's coordinate frame
    Eigen::Quaterniond inverse_orientation = ring_orientation.conjugate();
    Eigen::Vector3d aligned_point = inverse_orientation * translated_point;

    // Step 3: Since the ring lies in the XY plane of its local coordinate system,
    // the Z-component of aligned_point can be ignored for theta calculation.
    // Compute the angle between the X-axis and the projected point in the XY plane
    double theta = atan2(aligned_point.y(), aligned_point.x());

    return theta;
}
```

<br>

## Converting an angle back to a 3D point

I've made this one templated because it actually has to be used in the ceres cost function (ceres will guess a `theta`, then to take a reprojection error, we need to extract the pose matrix).

```cpp
template <typename T>
Eigen::Matrix<T, 3, 1> ThetaTo3DPoint(const T& theta, 
                                      const Eigen::Matrix<T, 3, 1>& center, 
                                      const Eigen::Quaternion<T>& ring_orientation, 
                                      const T& radius) {
    // Step 1: Calculate the point's position in the ring's local XY plane
    Eigen::Matrix<T, 3, 1> point_in_plane(radius * cos(theta), radius * sin(theta), T(0));

    // Step 2: Rotate the point from the ring's local coordinate system to the global coordinate system
    Eigen::Matrix<T, 3, 1> point_in_global_space = ring_orientation * point_in_plane;

    // Step 3: Translate the point by the ring's center to get its position in the original coordinate system
    Eigen::Matrix<T, 3, 1> point_in_original_space = point_in_global_space + center;

    return point_in_original_space;
}
```

<br>

## Updated parameter blocks for the ring

```cpp
for (int i = 0; i < ring_problem.num_observations(); ++i) {
    ceres::CostFunction* cost_function = RingCost::Create(
        observations[2 * i + 0], observations[2 * i + 1]);

    // Get camera parameters and point for this observation
    double* extrinsics = ring_problem.mutable_extrinsic_for_observation(i);
    double* intrinsics = ring_problem.mutable_intrinsic_for_observation(i);
    double* point = ring_problem.mutable_point_for_observation(i);
    double* geometry = ring_problem.mutable_geometry_params();

    // Use a product manifold of AngleManifold (for theta) and QuaternionManifold (for the quaternion)
    ceres::Manifold* camera_manifold = new ceres::ProductManifold<ceres::AutoDiffManifold<AngleManifold, 1, 1>, ceres::QuaternionManifold>{};

    problem.AddParameterBlock(intrinsics, 3);
    problem.SetParameterBlockConstant(intrinsics);

    ceres::Manifold* ring_manifold = new ceres::ProductManifold<ceres::EuclideanManifold<3>, ceres::QuaternionManifold, ceres::EuclideanManifold<1>>{};

    problem.AddParameterBlock(geometry, 8, ring_manifold);

    // Assuming camera has 5 parameters (1 for theta, 4 for quaternion)
    // problem.AddParameterBlock(extrinsics, 5, camera_manifold);
    problem.AddParameterBlock(extrinsics, 5);

    // Add the residual block to the problem
    problem.AddResidualBlock(cost_function, nullptr /* squared loss */, extrinsics, intrinsics, point, geometry);
}
```

<br>

## A ring coordinate reprojection error function

```cpp
template <typename T>
bool operator()(const T* const extrinsic_params, // Camera parameters
                const T* const intrinsic_params, // frozen intrinsics
                const T* const point,  // 3D point
                const T* const ring_params, // Ring parameters
                T* residuals) const {

    // Camera parameters: quaternion (4), theta (1), intrinsics (3)
    const T* camera_quaternion = extrinsic_params;
    const T& theta = extrinsic_params[4];  // Theta is the fifth parameter
    const T* intrinsics = intrinsic_params;

    // Conjugate of the camera quaternion for inverse rotation.
    T conjugate_camera_quaternion[4] = {camera_quaternion[0], 
                                        -camera_quaternion[1], 
                                        -camera_quaternion[2], 
                                        -camera_quaternion[3]};

    // Extract the ring parameters: center (3), orientation quaternion (4), radius (1)
    Eigen::Matrix<T, 3, 1> center;
    Eigen::Quaternion<T> ring_orientation;
    center << ring_params[0], ring_params[1], ring_params[2];
    // Eigen uses x,y,z,w format when loading in quaternions this way (it's the API that is wrong)
    ring_orientation.coeffs() << ring_params[3], ring_params[4], ring_params[5], ring_params[6];
    const T& radius = ring_params[7];

    // Convert theta back to translation
    Eigen::Matrix<T, 3, 1> translation = ThetaTo3DPoint(theta, center, ring_orientation, radius);

    // Apply inverse translation: point - translation.
    T translated_point[3] = {point[0] - translation[0],
                             point[1] - translation[1],
                             point[2] - translation[2]};

    // Rotate the translated point using the conjugate of the camera quaternion.
    T rotated_translated_point[3];
    ceres::QuaternionRotatePoint(conjugate_camera_quaternion, translated_point, rotated_translated_point);

    // Project the 3D point onto the 2D camera plane.
    const T& focal = intrinsics[0];
    const T& cx = intrinsics[1];
    const T& cy = intrinsics[2];

    const T xp = rotated_translated_point[0] / rotated_translated_point[2];
    const T yp = rotated_translated_point[1] / rotated_translated_point[2];

    const T predicted_x = focal * xp + cx;
    const T predicted_y = focal * yp + cy;

    // The error is the difference between the predicted and observed positions.
    residuals[0] = predicted_x - observed_x;
    residuals[1] = predicted_y - observed_y;

    return true;
}
```

<br>
<br>

# Analysis

Aside from having less overall camera extrinsic parameters (38 instead of 60), is our estimation any better with the ring constraints? This depends on two unknowns:

1. How good our initial ring estimate is
2. How good our initial camera estimates are

"Ring" bundle adjustment does better than "basic" bundle adjustment when the initial ring estimate is good compared to the initial camera estimates. If the initial camera estimates are great, then the ring doesn't have much opportunity to do much. If the initial ring estimate is bad, then it hurts us more than helps us. I'll attempt to quantify the benefit of using a geometric prior by setting up some simulations that illustrate this point. When I add noise, I refer to adding zero-mean Gaussian noise to all the parameters of either the camera extrinsic parameters, or the ring parametres (center, normal, radius). So, adding `std = 1` noise will add white noise with a standard deviation of 1, independently distributed, to each parameter in the positioning.

<br>

## Very high camera noise 

<p align="center">
  <img src="diagrams/high_noise.png" alt="High Noise" style="width: 50%; height: auto;"/>
</p>

In the image above, the camera positional noise is unrealistically high (`std = 2`), and the ring noise is somewhat high (`std = 0.6`). The ring solver converges with an average camera positioning error of `0.21`, which isn't too bad, considering how bad our initial camera estimates were. Here's the optimizer log, which tells us that we've converged, but remember that this is just the reprojection error, which often has little bearing on the global accuracy. The ring estimator converges because it "snaps" the cameras to the initial ring in the first iteration (using orthogonal projection).

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  1.419799e+10    0.00e+00    3.38e+10   0.00e+00   0.00e+00  1.00e+04        0    1.21e+01    1.22e+01
   1  1.815157e+09    1.24e+10    4.90e+09   0.00e+00   8.75e-01  1.73e+04        1    1.30e+01    2.52e+01
   2  1.146844e+08    1.70e+09    6.92e+08   4.05e+01   9.42e-01  5.19e+04        1    1.28e+01    3.80e+01
   3  5.338989e+06    1.09e+08    2.61e+08   3.21e+01   9.57e-01  1.56e+05        1    1.30e+01    5.10e+01
   4  1.254573e+04    5.33e+06    1.28e+07   5.70e+00   9.98e-01  4.67e+05        1    1.29e+01    6.39e+01
   5  2.251619e-02    1.25e+04    1.88e+04   3.30e-01   1.00e+00  1.40e+06        1    1.27e+01    7.66e+01
   6  7.283842e-13    2.25e-02    3.86e-02   3.17e-04   1.00e+00  4.20e+06        1    1.28e+01    8.94e+01
```

Without the ring, Levenberg-Marquardt does not converge at all in this case:

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  2.108337e+13    0.00e+00    1.18e+17   0.00e+00   0.00e+00  1.00e+04        0    5.62e+00    5.76e+00
   1  5.267212e+12    1.58e+13    1.47e+16   0.00e+00   7.50e-01  1.14e+04        1    5.56e+00    1.13e+01
   2  2.928386e+13   -2.40e+13    1.47e+16   2.35e+02  -4.56e+00  5.72e+03        1    4.17e-02    1.14e+01
   3  1.342499e+12    3.92e+12    1.84e+15   2.34e+02   7.45e-01  6.48e+03        1    5.64e+00    1.70e+01
   4  5.416423e+12   -4.07e+12    1.84e+15   2.55e+02  -3.04e+00  3.24e+03        1    4.12e-02    1.70e+01
   5  4.398051e+11    9.03e+11    2.30e+14   2.52e+02   6.73e-01  3.38e+03        1    5.50e+00    2.25e+01
   6  1.131124e+11    3.27e+11    2.88e+13   1.88e+02   7.45e-01  3.83e+03        1    5.43e+00    2.80e+01
   7  2.925539e+10    8.39e+10    3.60e+12   1.19e+02   7.49e-01  4.36e+03        1    5.45e+00    3.34e+01
   8  8.057524e+09    2.12e+10    4.54e+11   2.00e+02   7.43e-01  4.93e+03        1    5.55e+00    3.90e+01
   9  1.437565e+11   -1.36e+11    4.54e+11   9.97e+01  -1.80e+01  2.47e+03        1    4.05e-02    3.90e+01
  10  2.895490e+12   -2.89e+12    4.54e+11   7.73e+01  -3.88e+02  6.16e+02        1    3.95e-02    3.91e+01
  11  9.903655e+10   -9.10e+10    4.54e+11   8.13e+01  -1.24e+01  7.70e+01        1    3.80e-02    3.91e+01
  12  7.040723e+09    1.02e+09    4.39e+11   5.52e+01   1.39e-01  5.60e+01        1    5.60e+00    4.47e+01
  13  1.496931e+10   -7.93e+09    4.39e+11   3.25e+01  -1.30e+00  2.80e+01        1    4.03e-02    4.47e+01
  14  1.946917e+10   -1.24e+10    4.39e+11   2.58e+01  -2.09e+00  7.00e+00        1    3.90e-02    4.48e+01
  15  2.455492e+10   -1.75e+10    4.39e+11   1.88e+01  -3.17e+00  8.75e-01        1    4.16e-02    4.48e+01
  16  3.521839e+09    3.52e+09    1.20e+11   9.97e+00   7.23e-01  9.61e-01        1    5.44e+00    5.03e+01
```

<br>

## Realistic noise scenario

<p align="center">
  <img src="diagrams/medium_noise.png" alt="Medium Noise" style="width: 50%; height: auto;"/>
</p>

In the above picture, I've set the positional noise scale to `std = 0.5`, and the noise in all the ring parameters to `std = 0.1`. I think this would be a realistic scenario. The ring estimator converges quickly to a global error of `0.18`.

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.815457e+08    0.00e+00    1.81e+09   0.00e+00   0.00e+00  1.00e+04        0    1.22e+01    1.24e+01
   1  2.701266e+06    3.79e+08    1.74e+08   0.00e+00   9.93e-01  3.00e+04        1    1.29e+01    2.53e+01
   2  1.157033e+02    2.70e+06    6.52e+05   2.82e+00   1.00e+00  9.00e+04        1    1.28e+01    3.81e+01
   3  1.304723e-04    1.16e+02    1.47e+03   4.58e-02   1.00e+00  2.70e+05        1    1.26e+01    5.07e+01
   4  3.832776e-12    1.30e-04    9.16e-03   6.88e-05   1.00e+00  8.10e+05        1    1.32e+01    6.39e+01
```

The basic estimator converges to a global error of `2.43`. We would expect this in the case when the prior estiamte for the ring is comparatively. **Bundle Adjustment minimizes reprojection error, which, without a geometric prior, does not always equate to high geometric accuracy (the points could be in the wrong country entirely, but still have a low reprojection error)**.

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.872072e+08    0.00e+00    1.85e+08   0.00e+00   0.00e+00  1.00e+04        0    5.68e+00    5.80e+00
   1  3.121777e+06    3.84e+08    1.52e+07   0.00e+00   9.92e-01  3.00e+04        1    5.58e+00    1.14e+01
   2  7.910206e+02    3.12e+06    2.61e+05   4.41e+00   1.00e+00  9.00e+04        1    5.56e+00    1.69e+01
   3  8.488959e-04    7.91e+02    3.26e+02   8.72e-02   1.00e+00  2.70e+05        1    5.75e+00    2.27e+01
   4  1.612429e-11    8.49e-04    1.51e-02   1.02e-04   1.00e+00  8.10e+05        1    5.57e+00    2.83e+01
```

<br>

## Terrible initial ring estimate

<p align="center">
  <img src="diagrams/bad_initial_ring.png" alt="Bad Initial Ring" style="width: 50%; height: auto;"/>
</p>

If you don't have any kind of an estimate of where the ring should be (`std = 2` noise on all ring parameters), then the ring estimate will totally fail, while your regular estimate will be okay (consistent with the previous examples). The ring estimator's optimizer log before ceres decided to give up:

```bash
iter      cost      cost_change  |gradient|   |step|    tr_ratio  tr_radius  ls_iter  iter_time  total_time
   0  3.695828e+10    0.00e+00    2.62e+11   0.00e+00   0.00e+00  1.00e+04        0    1.33e+01    1.35e+01
   1  9.833985e+09    2.71e+10    4.04e+10   0.00e+00   7.36e-01  1.12e+04        1    1.33e+01    2.67e+01
   2  9.686036e+15   -9.69e+15    4.04e+10   8.26e+02  -1.01e+06  5.59e+03        1    2.68e-01    2.70e+01
   3  6.193782e+15   -6.19e+15    4.04e+10   6.82e+02  -6.46e+05  1.40e+03        1    2.72e-01    2.72e+01
   4  1.287654e+16   -1.29e+16    4.04e+10   3.50e+02  -1.35e+06  1.75e+02        1    2.54e-01    2.75e+01
   5  1.806553e+09    8.03e+09    5.78e+09   1.08e+02   8.44e-01  2.59e+02        1    1.20e+01    3.95e+01
   6  2.501211e+08    1.56e+09    8.96e+08   5.05e+01   9.34e-01  7.52e+02        1    1.17e+01    5.12e+01
   7  1.726336e+08    7.75e+07    5.63e+08   9.67e+01   5.72e-01  7.54e+02        1    1.17e+01    6.29e+01
   8  1.199832e+08    5.27e+07    1.98e+08   1.23e+02   8.79e-01  1.34e+03        1    1.17e+01    7.46e+01
   9  1.242681e+08   -4.28e+06    1.98e+08   2.07e+02  -3.70e-01  6.68e+02        1    2.13e-01    7.48e+01
  10  1.117673e+08    8.22e+06    1.18e+08   1.20e+02   8.34e-01  9.50e+02        1    1.17e+01    8.65e+01
  11  1.458582e+08   -3.41e+07    1.18e+08   1.48e+02  -4.98e+00  4.75e+02        1    2.06e-01    8.67e+01
  12  1.106560e+08    1.11e+06    2.00e+08   8.60e+01   2.10e-01  3.98e+02        1    1.17e+01    9.84e+01
  13  1.393557e+08   -2.87e+07    2.00e+08   9.48e+01  -2.31e+00  1.99e+02        1    2.26e-01    9.86e+01
  14  1.042820e+08    6.37e+06    2.64e+08   5.24e+01   6.17e-01  2.01e+02        1    1.30e+01    1.12e+02
  15  1.464388e+08   -4.22e+07    2.64e+08   8.23e+01  -1.98e+00  1.01e+02        1    2.52e-01    1.12e+02
  16  9.476664e+07    9.52e+06    3.69e+08   4.56e+01   5.52e-01  1.01e+02        1    1.34e+01    1.25e+02
  17  9.216740e+07    2.60e+06    5.60e+08   3.72e+01   8.55e-02  6.42e+01        1    1.34e+01    1.39e+02
  18  5.040414e+07    4.18e+07    1.54e+08   3.32e+01   9.86e-01  1.93e+02        1    1.35e+01    1.52e+02
  19  4.556504e+07    4.84e+06    5.13e+07   2.56e+01   9.90e-01  5.78e+02        1    1.33e+01    1.65e+02
  20  4.518771e+07    3.77e+05    2.02e+06   6.98e+00   1.03e+00  1.73e+03        1    1.23e+01    1.78e+02
  21  4.518153e+07    6.18e+03    1.15e+05   1.53e+00   1.16e+00  5.20e+03        1    1.36e+01    1.91e+02
  22  4.518110e+07    4.29e+02    6.19e+04   3.28e-01   1.36e+00  1.56e+04        1    1.36e+01    2.05e+02
  23  4.518102e+07    8.21e+01    3.45e+04   9.56e-02   1.50e+00  4.68e+04        1    1.38e+01    2.19e+02
```










