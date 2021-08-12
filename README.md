# StrereoReconstruction

This is software I created during my Master of Science (Computing Science) education at the University of Alberta. It provided the results for my thesis, [Underwater Stereo Matching and its Calibration](https://era.library.ualberta.ca/items/ecc52c46-24d1-48a0-9cb0-91fae234ca8e). Abstract:

>  A fundamental component of stereo vision is that of epipolar geometry. It shows that the corresponding point of a pixel in one image is restricted to a line in another image. When a refractive surface is introduced, such as in underwater imaging, this constraint no longer holds. Instead, the corresponding point of a pixel in one image is now restricted to a curve, not a line, in the other image. In this thesis, we investigate the impact of a planar refractive interface on stereo matching. We address the issue of 3D point projection in a refractive medium, including cases where the refractive interface is not parallel with the camera’s imaging plane. A novel method for calibrating the parameters of a planar refractive interface is proposed. We show how to compute the refractive epipolar curve for a pixel, which allows us to generate a matching cost volume that compensates for the effects of refraction. We implement a multi-view stereo algorithms to test the correctness of our matching cost volume. The experimental results show that our new approach can significantly improve the results of underwater stereo matching over previous approaches using heuristic methods to account for refraction.

I'm not actively working on this, given this was used for research, but hope it can serve as a basis for anyone wanting to learn from or reimplement my work 🙂

StereoReconstruction Project File
-----------------------------------------------------------------------------

This program has to be compiled under the C++11 spec (the latest compilers
should have sufficient support). The following dependencies are required when
building StereoRecontruction:

   * Qt 5.2.1
   * OpenCV 2.2
   * gsl 1.14
   * Boost 1.46
   * Eigen 3
   * GLEW 1.5.8

Additionally, some options (see "Tunable Options" below) will require additional
libraries:

   * OpenEXR 1.6.1 (when hdr option enabled)
   * Intel TBB 3.0 (when tbb option enabled)
   * Point Grey Research FlyCap 2.x SDK (when pgr option enabled)
   * Point Grey Research Digiclops SDK (when pgr option enabled)
   * Point Grey Research Triclops SDK (when pgr option enabled)
   * Middlebury MRF library (when mrf option enabled)

The recommended environment for building StereoReconstruction is Qt Creator 2.x.


Tunable Options
-----------------------------------------------------------------------------

User-specific configuration options should be placed in a file named
`UserConfig.pri` in the same directory as the project file. Various
options that can be added to the CONFIG variable:

  * __hdr__: Enables HDR imaging options
  * __tbb__: Use Intel Thread Building Blocks library for parallel computation
    whenever possible.
  * __openmp__: Use OpenMP for parallel computation whenever possible. If tbb is
    specified then OpenMP is ignored.
  * __mrf__: Use MRF optimization for stereo, with the TRW-S method. If not enabled,
    winner-take-all is used.
  * __splats__: Use splats when rendering point clouds. If not enabled, basic point
    rendering is used.
  * __pgr__: Link against and use the PointGrey FlyCap SDK for image capturing
    functionality.

Also, be sure to specify any non-standard library/include paths in your user
config file. To see which libraries are included for each of the above options,
browse down through this file.


Example workflow
-----------------------------------------------------------------------------

The following workflow shows how you can find depth maps for the example
project provided in the `example/` subdirectory.

### Calibrate the cameras

  1. After starting the program, open `example/project.xml`.
  2. Find the checkboard features.
     2.1. Click the `Stereo -> Find Features...` menu item.
     2.2. In the dialog that appears, select all the image sets that contain a
          checkerboard pattern for calibration. In the example, this is every
          image set besides `bunny`.
     2.3. Click `Find Features`. The dialog will close and a background task will
          start to find the features. You can see this task in the task window.
          If the task window doesn't open, click the `View -> Show Task List`
          menu item.
  3. Ensure features are correctly oriented. Sometimes the feature detector will
     output features in the reverse orientation. You should go through every
     image set for every camera and make sure the features are oriented
     correctly. If you find a camera/image pair with features oriented in the
     wrong direction, you can right-click the image and select the
     `Rotate Features` popup menu item.
  4. Find feature correspondences. Click the `Stereo -> Find Feature Correspondences`
     menu item. Currently, no background task is executed because this task is
     finishes fairly quickly.
  5. Calibrate the cameras.
     5.1. Click the `Calibrate Cameras...` menu item.
     5.2. In the dialog that appears, select the image sets that contain
          checkerboard features. Selecting all image sets is fine as the
          calibration task ignores images with no features.
     5.3. Click `Calibrate`. The dialog will close and a background task will
          start calibrating the cameras.
  6. After the calibration task completes, you can view the result by clicking
     the `View -> View Camera Layout` menu item. If the calibration was
     successful, you should see a semi-circle containing the eight cameras.

### Find the depth maps

  1. In the `Refractive Calibration` tab, select the `bunny` image set.
  2. Once successfully calibrated, you should be able to click an image and see
     a green line in the other image. This is the epipolar line.
     * If you cannot see the line, you may need to adjust the min/max depth
       parameters in the `Stereo` tab. The example project will likely
       require a min and max depth of 300 and 800, respectively.
     * Clicking near a feature will snap the point of interest in that image to
       the feature's location. You can see the feature's reprojection error in
       the `Stereo` tab.
  3. For the example, try setting the following additional parameters:
     * `Cross-check threshold`: 5
     * `Num-depth levels`: 100
     * `Scale`: 0.5
  4. In the `Stereo` tab, click `Compute Depth Maps`.
     4.1. A background task will start to compute the depth maps for the `bunny`
          image set. This is a lengthy process if you compiled the code to use
          geodesic support weights (the default), so head off and do some other
          important research while waiting :)
     4.2. Once complete, you should see the depth maps below the corresponding
          images in the preview area. To see depth maps of other cameras, select
          the camera of interest in the `Refractive Calibration` tab for either
          the `Left View` or `Right View`.
