<a id="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/DTU-PAS/">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h1 align="center">Rebar Segmentation using Ransac</h1>

  <p align="center">
  Rebar segmentation of exposed rebar on construction sites
    <br />
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac?tab=readme-ov-file#results">View Demo</a>
    ·
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac/issues/new?labels=bug&template=bug-report---.md">Report Bug</a>
    ·
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac/issues/new?labels=enhancement&template=feature-request---.md">Request Feature</a>
  </p>
</div>



<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about-the-project">About The Project</a>
      <ul>
        <li><a href="#built-with">Built With</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#results">Results</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Overview: https://www.robetarme-project.eu/

This project uses a poincloud and a depth image to segment exposed rebars and detect potential damages. The sensor used for development is the Intel Realsense D435i. However, the code can be easily be adapted to other sensors such as the D455, L515 or Roboception rc_visard 160 color. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## TODO
Current method: The distance is calculated by taking the pixel of the center of the damaged area out of the depth image and calculating that into a distance from the camera to the rebar. The center pixel is likely hitting the background so the previoulsy calculated average distance from the board background to the rebars is added. However if the pixel is hitting the rebar in the depth image this calculation gives wrong results.

Improved method: A better way would be to create a mesh structure out of the point cloud. 

## Built With
[![build][build]][build-url] 

[![ROS][ROS]][ROS-url] 
Humble

[![OPENCV][OPENCV]][OPENCV-url]
4.5.4

<p align=left><img src="images/pcl.png" height="20">
1.12.1
</p>

[![UBUNTU][UBUNTU]][UBUNTU-url]
22.04.5 LTS

[![C++][C++]][C++-url]
201703L C++17

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started
### Prerequisites

#### Software: 
* ROS: [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* Intel Realsense ROS1 Wrapper (ros1-legacy): [Method 1: The ROS distribution:](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file#method-1-the-ros-distribution)
* Clone the repository: [HTTPS](https://github.com/DTU-PAS/Rebar-segmentation-Ransac.git) or [SSH](git@github.com:DTU-PAS/Rebar-segmentation-Ransac.git)

* Also ensure that the following packages are installed using **sudo apt-get install**
  * python3-colcon-common-extensions
  * ros-$ROS_DISTRO-realsense2-camera
  * ros-$ROS_DISTRO-realsense2-camera-msgs
  * ros-$ROS_DISTRO-realsense2-description
  * ros-$ROS_DISTRO-pcl-conversions
  * ros-$ROS_DISTRO-pcl-msgs
  * ros-$ROS_DISTRO-pcl-ros
  * ros-$ROS_DISTRO-visualization-msgs

#### Hardware
* Camera: Connect the camera sensor using the OEM cable. 3rd party cables have shown to be unreliable at times but can work aswell.

* Test Setup: ~11mm rebar mounted on a wooden board with ~30mm 3d printed spacers. The size of the grid pattern is ~20cm. The camera should be placed ~40-60cm in front of the board.

The Spacers were designed in DesignSpark. The design file aswell as an STL file are included in the repo.

```stl
solid cube_corner
  facet normal -9.911824e-01 -9.297748e-02 -9.440641e-02
    outer loop
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
    endloop
  endfacet
  facet normal -9.977238e-01 6.709370e-02 6.757156e-03
    outer loop
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
    endloop
  endfacet
  facet normal -8.273828e-01 2.490309e-01 5.034097e-01
    outer loop
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
      vertex   -9.271037e+00 1.171937e+01 2.907888e+01
    endloop
  endfacet
  facet normal -9.791211e-01 -1.288650e-01 1.572121e-01
    outer loop
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
    endloop
  endfacet
  facet normal -4.825520e-01 -1.141079e-01 8.684025e-01
    outer loop
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -9.271037e+00 1.171937e+01 2.907888e+01
    endloop
  endfacet
  facet normal 2.790195e-01 1.676339e-01 9.455406e-01
    outer loop
      vertex   -9.271037e+00 1.171937e+01 2.907888e+01
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -8.758865e+00 1.175217e+01 2.892193e+01
    endloop
  endfacet
  facet normal 9.542171e-01 6.706004e-02 2.915007e-01
    outer loop
      vertex   -8.758865e+00 1.175217e+01 2.892193e+01
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
    endloop
  endfacet
  facet normal 5.697450e-01 -1.056883e-01 8.149973e-01
    outer loop
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -8.758865e+00 1.175217e+01 2.892193e+01
    endloop
  endfacet
  facet normal 9.993289e-01 3.610872e-02 -6.160572e-03
    outer loop
      vertex   -8.576172e+00 1.199198e+01 2.648571e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
    endloop
  endfacet
  facet normal 9.857956e-01 -1.483776e-01 7.868347e-02
    outer loop
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
    endloop
  endfacet
  facet normal 9.025291e-01 1.553880e-01 -4.016165e-01
    outer loop
      vertex   -8.576172e+00 1.199198e+01 2.648571e+01
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
      vertex   -8.929135e+00 1.197215e+01 2.568484e+01
    endloop
  endfacet
  facet normal 9.846898e-01 -7.233259e-02 -1.586003e-01
    outer loop
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
      vertex   -8.576172e+00 1.199198e+01 2.648571e+01
    endloop
  endfacet
  facet normal -5.908661e-02 5.623652e-02 -9.966676e-01
    outer loop
      vertex   -8.929135e+00 1.197215e+01 2.568484e+01
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -9.369648e+00 1.197384e+01 2.571105e+01
    endloop
  endfacet
  facet normal 4.911438e-01 -9.413566e-02 -8.659771e-01
    outer loop
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -8.929135e+00 1.197215e+01 2.568484e+01
    endloop
  endfacet
  facet normal -5.213463e-01 -1.300582e-01 -8.433759e-01
    outer loop
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
      vertex   -9.369648e+00 1.197384e+01 2.571105e+01
    endloop
  endfacet
  facet normal -8.068000e-03 9.837677e-01 1.792654e-01
    outer loop
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
      vertex   -9.271037e+00 1.171937e+01 2.907888e+01
      vertex   -8.758865e+00 1.175217e+01 2.892193e+01
    endloop
  endfacet
  facet normal -1.066792e-04 9.859455e-01 1.670670e-01
    outer loop
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
      vertex   -8.758865e+00 1.175217e+01 2.892193e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
    endloop
  endfacet
  facet normal -7.546311e-03 9.971229e-01 7.542467e-02
    outer loop
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
      vertex   -9.683060e+00 1.185128e+01 2.833645e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
    endloop
  endfacet
  facet normal -6.801731e-04 9.974726e-01 7.104954e-02
    outer loop
      vertex   -8.576172e+00 1.199198e+01 2.648571e+01
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
      vertex   -8.560780e+00 1.186652e+01 2.824720e+01
    endloop
  endfacet
  facet normal 2.598525e-03 9.997813e-01 -2.075108e-02
    outer loop
      vertex   -9.369648e+00 1.197384e+01 2.571105e+01
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
      vertex   -8.929135e+00 1.197215e+01 2.568484e+01
    endloop
  endfacet
  facet normal -1.135758e-03 9.997051e-01 -2.425612e-02
    outer loop
      vertex   -8.576172e+00 1.199198e+01 2.648571e+01
      vertex   -8.929135e+00 1.197215e+01 2.568484e+01
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
    endloop
  endfacet
  facet normal -9.165565e-01 1.389196e-01 -3.750006e-01
    outer loop
      vertex   -9.369648e+00 1.197384e+01 2.571105e+01
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
      vertex   -9.686173e+00 1.199085e+01 2.649099e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
      vertex   -8.726539e+00 1.128536e+01 2.587440e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.005260e+01 1.128536e+01 2.514793e+01
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -9.434652e+00 1.128536e+01 2.487799e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -1.005260e+01 1.128536e+01 2.514793e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -9.128359e+00 1.128536e+01 2.992116e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.655397e+00 1.128536e+01 2.862600e+01
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.434652e+00 1.128536e+01 2.487799e+01
      vertex   -9.110171e+00 1.128536e+01 2.565682e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
      vertex   -1.072363e+01 1.128536e+01 2.757326e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -7.655397e+00 1.128536e+01 2.862600e+01
      vertex   -8.547558e+00 1.128536e+01 2.698563e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -9.520811e+00 1.128536e+01 2.888306e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -1.072363e+01 1.128536e+01 2.757326e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -9.726072e+00 1.128536e+01 2.760469e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.128359e+00 1.128536e+01 2.992116e+01
      vertex   -9.113566e+00 1.128536e+01 2.910936e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -9.567469e+00 1.128536e+01 2.593951e+01
      vertex   -1.005260e+01 1.128536e+01 2.514793e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.655397e+00 1.128536e+01 2.862600e+01
      vertex   -8.693637e+00 1.128536e+01 2.881580e+01
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
    endloop
  endfacet
  facet normal 3.195429e-01 8.642947e-02 -9.436219e-01
    outer loop
      vertex   -1.276106e+01 1.198036e+01 2.582374e+01
      vertex   -1.304629e+01 1.128536e+01 2.566350e+01
      vertex   -1.326803e+01 1.196991e+01 2.565111e+01
    endloop
  endfacet
  facet normal 5.749551e-01 -4.763288e-02 -8.167973e-01
    outer loop
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
      vertex   -1.304629e+01 1.128536e+01 2.566350e+01
      vertex   -1.276106e+01 1.198036e+01 2.582374e+01
    endloop
  endfacet
  facet normal -7.373731e-01 1.407131e-01 -6.606668e-01
    outer loop
      vertex   -1.326803e+01 1.196991e+01 2.565111e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
    endloop
  endfacet
  facet normal -3.271671e-01 -1.229368e-01 -9.369356e-01
    outer loop
      vertex   -1.304629e+01 1.128536e+01 2.566350e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.326803e+01 1.196991e+01 2.565111e+01
    endloop
  endfacet
  facet normal -9.682224e-01 -9.014578e-02 -2.332789e-01
    outer loop
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
    endloop
  endfacet
  facet normal -9.018190e-01 -8.575638e-02 4.235190e-01
    outer loop
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
    endloop
  endfacet
  facet normal -9.820055e-01 1.593444e-01 1.013634e-01
    outer loop
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
    endloop
  endfacet
  facet normal -3.983137e-01 1.034613e-01 9.113956e-01
    outer loop
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
      vertex   -1.302555e+01 1.196436e+01 2.742338e+01
    endloop
  endfacet
  facet normal 5.552901e-01 8.553102e-02 8.272469e-01
    outer loop
      vertex   -1.302555e+01 1.196436e+01 2.742338e+01
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
      vertex   -1.261004e+01 1.198214e+01 2.714262e+01
    endloop
  endfacet
  facet normal -9.821061e-02 -1.122886e-01 9.888104e-01
    outer loop
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
      vertex   -1.302555e+01 1.196436e+01 2.742338e+01
    endloop
  endfacet
  facet normal 9.783615e-01 1.950971e-02 2.059807e-01
    outer loop
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
      vertex   -1.261004e+01 1.198214e+01 2.714262e+01
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
    endloop
  endfacet
  facet normal 8.696941e-01 -1.276786e-01 4.767917e-01
    outer loop
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
      vertex   -1.261004e+01 1.198214e+01 2.714262e+01
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
    endloop
  endfacet
  facet normal -5.042476e-03 9.989444e-01 -4.565727e-02
    outer loop
      vertex   -1.276106e+01 1.198036e+01 2.582374e+01
      vertex   -1.326803e+01 1.196991e+01 2.565111e+01
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
    endloop
  endfacet
  facet normal 3.011411e-03 9.997440e-01 -2.242335e-02
    outer loop
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
      vertex   -1.276106e+01 1.198036e+01 2.582374e+01
    endloop
  endfacet
  facet normal -8.923985e-03 9.998980e-01 1.115567e-02
    outer loop
      vertex   -1.371458e+01 1.199064e+01 2.615392e+01
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
    endloop
  endfacet
  facet normal -1.278761e-03 9.996623e-01 2.595576e-02
    outer loop
      vertex   -1.261004e+01 1.198214e+01 2.714262e+01
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
    endloop
  endfacet
  facet normal -4.460827e-04 9.980411e-01 6.255983e-02
    outer loop
      vertex   -1.361187e+01 1.198027e+01 2.716533e+01
      vertex   -1.302555e+01 1.196436e+01 2.742338e+01
      vertex   -1.261004e+01 1.198214e+01 2.714262e+01
    endloop
  endfacet
  facet normal 9.391425e-01 9.275700e-02 -3.307681e-01
    outer loop
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
      vertex   -1.276106e+01 1.198036e+01 2.582374e+01
    endloop
  endfacet
  facet normal 9.855827e-01 -8.510400e-02 -1.462329e-01
    outer loop
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
      vertex   -1.249354e+01 1.199669e+01 2.658790e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
      vertex   -1.408328e+01 1.128536e+01 2.788773e+01
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
      vertex   -1.194296e+01 1.128536e+01 2.537159e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.304629e+01 1.128536e+01 2.566350e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.304629e+01 1.128536e+01 2.566350e+01
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.395440e+01 1.128536e+01 2.505096e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
      vertex   -1.332252e+01 1.128536e+01 2.991381e+01
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.332252e+01 1.128536e+01 2.991381e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
      vertex   -1.371596e+01 1.128536e+01 2.858849e+01
      vertex   -1.464366e+01 1.128536e+01 2.869081e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
      vertex   -1.283439e+01 1.128536e+01 2.736526e+01
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
      vertex   -1.194296e+01 1.128536e+01 2.537159e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
      vertex   -1.265081e+01 1.128536e+01 2.594188e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.453781e+01 1.128536e+01 2.568571e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.395440e+01 1.128536e+01 2.505096e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.371596e+01 1.128536e+01 2.858849e+01
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.252370e+01 1.128536e+01 2.679856e+01
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
      vertex   -1.408328e+01 1.128536e+01 2.788773e+01
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
      vertex   -1.348211e+01 1.128536e+01 2.730093e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.357526e+01 1.128536e+01 2.584821e+01
      vertex   -1.453781e+01 1.128536e+01 2.568571e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
      vertex   -1.377501e+01 1.128536e+01 2.667724e+01
      vertex   -1.453781e+01 1.128536e+01 2.568571e+01
    endloop
  endfacet
  facet normal 1.162087e-17 9.956269e-01 9.341889e-02
    outer loop
      vertex   -5.506548e+00 1.183677e+01 2.847253e+01
      vertex   -4.357163e+00 1.199710e+01 2.676379e+01
      vertex   -5.506548e+00 1.199710e+01 2.676379e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.183677e+01 2.847253e+01
      vertex   -5.506548e+00 1.199710e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
      vertex   -5.506548e+00 1.183677e+01 2.847253e+01
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 8.297500e-01 -0.000000e+00 5.581353e-01
    outer loop
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
      vertex   -4.357163e+00 1.199710e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
    endloop
  endfacet
  facet normal 8.297500e-01 8.860495e-17 5.581353e-01
    outer loop
      vertex   -4.357163e+00 1.199710e+01 2.676379e+01
      vertex   -5.506548e+00 1.183677e+01 2.847253e+01
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.506548e+00 1.199710e+01 2.676379e+01
      vertex   -4.357163e+00 1.199710e+01 2.676379e+01
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.199710e+01 2.676379e+01
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+00 1.128536e+01 2.676379e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+00 1.128536e+01 2.592136e+01
      vertex   -7.054979e+00 1.128536e+01 2.676379e+01
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
      vertex   -5.506548e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.993569e+01
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.491523e+01
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.491523e+01
      vertex   -6.434243e+00 1.128536e+01 2.491523e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -4.357163e+00 1.128536e+01 2.676379e+01
      vertex   -5.506548e+00 1.128536e+01 2.847253e+01
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
      vertex   -2.134557e+01 1.174581e+01 2.895684e+01
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.916617e+01 1.128536e+01 2.895684e+01
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
      vertex   -1.916617e+01 1.128536e+01 2.895684e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+01 1.128536e+01 2.984701e+01
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -1.916617e+01 1.128536e+01 2.895684e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -1.916617e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.244379e+01 1.128536e+01 2.984701e+01
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -1.916617e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+01 1.128536e+01 2.915124e+01
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -2.244379e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+01 1.128536e+01 2.915124e+01
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
    endloop
  endfacet
  facet normal 8.527162e-01 -0.000000e+00 5.223744e-01
    outer loop
      vertex   -2.244379e+01 1.128536e+01 2.915124e+01
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
    endloop
  endfacet
  facet normal 8.570750e-01 1.838920e-01 4.812548e-01
    outer loop
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
      vertex   -2.101804e+01 1.199124e+01 2.650214e+01
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
    endloop
  endfacet
  facet normal 9.872661e-01 2.543663e-18 1.590772e-01
    outer loop
      vertex   -2.101804e+01 1.199124e+01 2.650214e+01
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
      vertex   -2.076235e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 9.558056e-01 -1.786676e-01 2.334813e-01
    outer loop
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
      vertex   -2.101804e+01 1.199124e+01 2.650214e+01
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -2.076235e+01 1.189489e+01 2.491523e+01
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.983806e+01 1.128536e+01 2.491523e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal -9.806832e-01 -1.981135e-17 -1.956030e-01
    outer loop
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -1.983806e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal -9.764444e-01 -4.751433e-02 -2.104728e-01
    outer loop
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.136822e+01 1.128536e+01 2.739549e+01
      vertex   -2.244379e+01 1.128536e+01 2.915124e+01
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.983806e+01 1.128536e+01 2.491523e+01
      vertex   -2.076235e+01 1.128536e+01 2.491523e+01
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
      vertex   -2.244379e+01 1.128536e+01 2.915124e+01
      vertex   -2.244379e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+01 1.128536e+01 2.895684e+01
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
      vertex   -2.244379e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+01 1.128536e+01 2.984701e+01
      vertex   -1.916617e+01 1.128536e+01 2.895684e+01
      vertex   -2.244379e+01 1.128536e+01 2.984701e+01
    endloop
  endfacet
  facet normal -8.832014e-01 -2.012354e-16 -4.689939e-01
    outer loop
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
      vertex   -2.134557e+01 1.174581e+01 2.895684e+01
    endloop
  endfacet
  facet normal -8.873048e-01 3.270144e-02 -4.600227e-01
    outer loop
      vertex   -2.134557e+01 1.128536e+01 2.895684e+01
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -2.020564e+01 1.128536e+01 2.675810e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.851814e+01 1.197988e+01 2.580540e+01
      vertex   -1.661501e+01 1.197988e+01 2.580540e+01
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.851814e+01 1.128536e+01 2.580540e+01
      vertex   -1.851814e+01 1.197988e+01 2.580540e+01
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.851814e+01 1.128536e+01 2.491523e+01
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
      vertex   -1.851814e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
      vertex   -1.851814e+01 1.197988e+01 2.580540e+01
      vertex   -1.851814e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
      vertex   -1.851814e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.515866e+01 1.128536e+01 2.491523e+01
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
      vertex   -1.851814e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal -8.391647e-01 0.000000e+00 -5.438773e-01
    outer loop
      vertex   -1.515866e+01 1.128536e+01 2.491523e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -8.332563e-01 -3.891851e-02 -5.515155e-01
    outer loop
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
    endloop
  endfacet
  facet normal -7.883090e-01 -9.901443e-02 -6.072603e-01
    outer loop
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
    endloop
  endfacet
  facet normal -7.691000e-01 -6.366413e-03 -6.390968e-01
    outer loop
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
    endloop
  endfacet
  facet normal -7.580466e-01 4.309595e-02 6.507750e-01
    outer loop
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
      vertex   -1.719920e+01 1.172296e+01 2.906264e+01
    endloop
  endfacet
  facet normal -9.246984e-01 -2.506330e-01 2.865587e-01
    outer loop
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
    endloop
  endfacet
  facet normal -9.940331e-01 3.755364e-02 -1.024103e-01
    outer loop
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
    endloop
  endfacet
  facet normal 5.867309e-02 1.393006e-01 9.885104e-01
    outer loop
      vertex   -1.719920e+01 1.172296e+01 2.906264e+01
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.660043e+01 1.173049e+01 2.902604e+01
    endloop
  endfacet
  facet normal -1.816702e-01 -1.365658e-01 9.738304e-01
    outer loop
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.719920e+01 1.172296e+01 2.906264e+01
    endloop
  endfacet
  facet normal 8.365634e-01 -1.021078e-01 5.382711e-01
    outer loop
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.660043e+01 1.173049e+01 2.902604e+01
    endloop
  endfacet
  facet normal 8.718134e-01 1.058999e-17 4.898381e-01
    outer loop
      vertex   -1.660043e+01 1.173049e+01 2.902604e+01
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal -9.950283e-02 -2.288322e-18 9.950373e-01
    outer loop
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
      vertex   -1.527121e+01 1.128536e+01 2.845547e+01
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
    endloop
  endfacet
  facet normal -9.950283e-02 0.000000e+00 9.950373e-01
    outer loop
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.527121e+01 1.128536e+01 2.845547e+01
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal -8.578070e-01 -2.241028e-01 -4.625419e-01
    outer loop
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
      vertex   -1.560268e+01 1.128536e+01 2.933871e+01
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
    endloop
  endfacet
  facet normal -9.362433e-01 0.000000e+00 -3.513523e-01
    outer loop
      vertex   -1.527121e+01 1.128536e+01 2.845547e+01
      vertex   -1.560268e+01 1.128536e+01 2.933871e+01
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
    endloop
  endfacet
  facet normal -9.642340e-02 1.797037e-01 -9.789837e-01
    outer loop
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
    endloop
  endfacet
  facet normal -3.443811e-01 -3.079519e-01 -8.868863e-01
    outer loop
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
    endloop
  endfacet
  facet normal -5.588328e-01 1.151705e-01 -8.212440e-01
    outer loop
      vertex   -1.560268e+01 1.128536e+01 2.933871e+01
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
    endloop
  endfacet
  facet normal 6.319030e-01 -9.613938e-02 -7.690617e-01
    outer loop
      vertex   -1.776050e+01 1.155166e+01 2.974787e+01
      vertex   -1.792659e+01 1.128536e+01 2.964469e+01
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
    endloop
  endfacet
  facet normal 3.499949e-01 1.405466e-01 -9.261481e-01
    outer loop
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
      vertex   -1.792659e+01 1.128536e+01 2.964469e+01
      vertex   -1.776050e+01 1.155166e+01 2.974787e+01
    endloop
  endfacet
  facet normal 1.355402e-01 -3.177678e-01 -9.384309e-01
    outer loop
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
      vertex   -1.776050e+01 1.155166e+01 2.974787e+01
    endloop
  endfacet
  facet normal 9.670161e-01 -1.416622e-01 -2.116879e-01
    outer loop
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
    endloop
  endfacet
  facet normal 8.129797e-01 3.122623e-01 -4.914838e-01
    outer loop
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
      vertex   -1.792659e+01 1.128536e+01 2.964469e+01
    endloop
  endfacet
  facet normal 9.669307e-01 1.727258e-01 1.876454e-01
    outer loop
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
    endloop
  endfacet
  facet normal 7.580835e-01 -3.795622e-01 5.303224e-01
    outer loop
      vertex   -1.661501e+01 1.197988e+01 2.580540e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
    endloop
  endfacet
  facet normal 7.452537e-01 -9.709423e-17 6.667810e-01
    outer loop
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
      vertex   -1.661501e+01 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.792659e+01 1.128536e+01 2.964469e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.560268e+01 1.128536e+01 2.933871e+01
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.632894e+01 1.128536e+01 2.983291e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
      vertex   -1.792659e+01 1.128536e+01 2.964469e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.844284e+01 1.128536e+01 2.879075e+01
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.669888e+01 1.128536e+01 2.909461e+01
      vertex   -1.729155e+01 1.128536e+01 2.898405e+01
      vertex   -1.720119e+01 1.128536e+01 2.991882e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
      vertex   -1.515866e+01 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.527121e+01 1.128536e+01 2.845547e+01
      vertex   -1.622619e+01 1.128536e+01 2.835997e+01
      vertex   -1.560268e+01 1.128536e+01 2.933871e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.750632e+01 1.128536e+01 2.829100e+01
      vertex   -1.649938e+01 1.128536e+01 2.698386e+01
      vertex   -1.820929e+01 1.128536e+01 2.758732e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
      vertex   -1.851814e+01 1.128536e+01 2.491523e+01
      vertex   -1.851814e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.515866e+01 1.128536e+01 2.491523e+01
      vertex   -1.851814e+01 1.128536e+01 2.491523e+01
      vertex   -1.661501e+01 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.096298e-01 -1.118711e-17 9.939725e-01
    outer loop
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
      vertex   -1.464366e+01 1.128536e+01 2.869081e+01
    endloop
  endfacet
  facet normal 1.096298e-01 -0.000000e+00 9.939725e-01
    outer loop
      vertex   -1.371596e+01 1.128536e+01 2.858849e+01
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
      vertex   -1.464366e+01 1.128536e+01 2.869081e+01
    endloop
  endfacet
  facet normal -8.004725e-01 0.000000e+00 5.993695e-01
    outer loop
      vertex   -1.371596e+01 1.128536e+01 2.858849e+01
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
    endloop
  endfacet
  facet normal -5.683302e-01 3.158342e-01 7.597694e-01
    outer loop
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.307238e+01 1.171188e+01 2.911354e+01
    endloop
  endfacet
  facet normal 6.367433e-01 2.371605e-01 7.336981e-01
    outer loop
      vertex   -1.307238e+01 1.171188e+01 2.911354e+01
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
    endloop
  endfacet
  facet normal 1.239317e-01 -1.303090e-01 9.836974e-01
    outer loop
      vertex   -1.333989e+01 1.128536e+01 2.909074e+01
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
      vertex   -1.307238e+01 1.171188e+01 2.911354e+01
    endloop
  endfacet
  facet normal 9.856382e-01 -0.000000e+00 1.688708e-01
    outer loop
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
      vertex   -1.238922e+01 1.193584e+01 2.773924e+01
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
    endloop
  endfacet
  facet normal 9.810131e-01 2.768635e-02 1.919553e-01
    outer loop
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
      vertex   -1.238922e+01 1.193584e+01 2.773924e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
    endloop
  endfacet
  facet normal 8.743810e-01 -6.298521e-02 4.811348e-01
    outer loop
      vertex   -1.251845e+01 1.128536e+01 2.849349e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
      vertex   -1.281038e+01 1.128536e+01 2.902403e+01
    endloop
  endfacet
  facet normal -4.462646e-01 1.427448e-02 -8.947872e-01
    outer loop
      vertex   -1.238922e+01 1.193584e+01 2.773924e+01
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
    endloop
  endfacet
  facet normal -4.369507e-01 0.000000e+00 -8.994854e-01
    outer loop
      vertex   -1.238922e+01 1.128536e+01 2.773924e+01
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
      vertex   -1.238922e+01 1.193584e+01 2.773924e+01
    endloop
  endfacet
  facet normal 3.213565e-01 -7.788753e-02 -9.437497e-01
    outer loop
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
      vertex   -1.408328e+01 1.128536e+01 2.788773e+01
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
    endloop
  endfacet
  facet normal 2.598716e-01 2.190648e-02 -9.653947e-01
    outer loop
      vertex   -1.318994e+01 1.128536e+01 2.812821e+01
      vertex   -1.408328e+01 1.128536e+01 2.788773e+01
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
    endloop
  endfacet
  facet normal 8.699514e-01 1.272223e-02 -4.929733e-01
    outer loop
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
      vertex   -1.466144e+01 1.198701e+01 2.696081e+01
    endloop
  endfacet
  facet normal 8.501245e-01 6.321248e-02 -5.227739e-01
    outer loop
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
      vertex   -1.408328e+01 1.128536e+01 2.788773e+01
    endloop
  endfacet
  facet normal 9.955485e-01 -6.632082e-02 6.696818e-02
    outer loop
      vertex   -1.453781e+01 1.128536e+01 2.568571e+01
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
    endloop
  endfacet
  facet normal 9.901585e-01 5.720772e-02 1.277240e-01
    outer loop
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.466144e+01 1.198701e+01 2.696081e+01
      vertex   -1.462651e+01 1.128536e+01 2.700434e+01
    endloop
  endfacet
  facet normal 1.270519e-01 1.286221e-01 9.835213e-01
    outer loop
      vertex   -1.395440e+01 1.128536e+01 2.505096e+01
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
    endloop
  endfacet
  facet normal 5.104091e-01 -2.936757e-01 8.082309e-01
    outer loop
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.395440e+01 1.128536e+01 2.505096e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
    endloop
  endfacet
  facet normal 7.362542e-01 -2.674992e-03 6.766998e-01
    outer loop
      vertex   -1.453781e+01 1.128536e+01 2.568571e+01
      vertex   -1.395440e+01 1.128536e+01 2.505096e+01
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
    endloop
  endfacet
  facet normal -6.642186e-01 -1.331624e-01 7.355823e-01
    outer loop
      vertex   -1.248064e+01 1.190479e+01 2.499821e+01
      vertex   -1.194296e+01 1.128536e+01 2.537159e+01
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
    endloop
  endfacet
  facet normal -4.631734e-01 1.266964e-01 8.771650e-01
    outer loop
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
      vertex   -1.194296e+01 1.128536e+01 2.537159e+01
      vertex   -1.248064e+01 1.190479e+01 2.499821e+01
    endloop
  endfacet
  facet normal -1.156582e-01 -8.406909e-02 9.897250e-01
    outer loop
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
      vertex   -1.282620e+01 1.128536e+01 2.490521e+01
      vertex   -1.248064e+01 1.190479e+01 2.499821e+01
    endloop
  endfacet
  facet normal -9.989863e-01 -4.338876e-02 1.199341e-02
    outer loop
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
      vertex   -1.143478e+01 1.191894e+01 2.780657e+01
    endloop
  endfacet
  facet normal -9.957392e-01 4.972587e-02 7.765778e-02
    outer loop
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
    endloop
  endfacet
  facet normal -9.526420e-01 -3.410445e-02 3.021757e-01
    outer loop
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
    endloop
  endfacet
  facet normal -9.059948e-01 8.621188e-02 4.144163e-01
    outer loop
      vertex   -1.150837e+01 1.128536e+01 2.632168e+01
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
      vertex   -1.194296e+01 1.128536e+01 2.537159e+01
    endloop
  endfacet
  facet normal -9.341593e-01 -1.479497e-01 -3.247417e-01
    outer loop
      vertex   -1.143478e+01 1.191894e+01 2.780657e+01
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
    endloop
  endfacet
  facet normal -9.710071e-01 4.449077e-02 -2.348740e-01
    outer loop
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
      vertex   -1.143478e+01 1.191894e+01 2.780657e+01
      vertex   -1.140994e+01 1.128536e+01 2.758384e+01
    endloop
  endfacet
  facet normal -2.060219e-01 7.378134e-02 -9.757619e-01
    outer loop
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
      vertex   -1.332252e+01 1.128536e+01 2.991381e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
    endloop
  endfacet
  facet normal -1.758830e-01 1.730341e-01 -9.690843e-01
    outer loop
      vertex   -1.248806e+01 1.154280e+01 2.978059e+01
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
    endloop
  endfacet
  facet normal -6.372073e-01 -5.825754e-02 -7.684875e-01
    outer loop
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
      vertex   -1.248806e+01 1.154280e+01 2.978059e+01
    endloop
  endfacet
  facet normal -7.275761e-01 1.685341e-01 -6.650032e-01
    outer loop
      vertex   -1.175976e+01 1.128536e+01 2.903005e+01
      vertex   -1.238696e+01 1.128536e+01 2.971627e+01
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
    endloop
  endfacet
  facet normal 2.604627e-01 1.961337e-01 -9.453522e-01
    outer loop
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
      vertex   -1.411842e+01 1.157923e+01 2.964870e+01
    endloop
  endfacet
  facet normal 3.560267e-01 -3.180776e-01 -8.786761e-01
    outer loop
      vertex   -1.332252e+01 1.128536e+01 2.991381e+01
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
    endloop
  endfacet
  facet normal 8.726962e-01 0.000000e+00 -4.882636e-01
    outer loop
      vertex   -1.464366e+01 1.128536e+01 2.869081e+01
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
    endloop
  endfacet
  facet normal 8.791946e-01 2.821121e-02 -4.756269e-01
    outer loop
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
      vertex   -1.411842e+01 1.157923e+01 2.964870e+01
      vertex   -1.414610e+01 1.128536e+01 2.958011e+01
    endloop
  endfacet
  facet normal -2.314655e-01 -1.816661e-01 -9.557307e-01
    outer loop
      vertex   -8.261960e+00 1.157692e+01 2.965591e+01
      vertex   -9.128359e+00 1.128536e+01 2.992116e+01
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
    endloop
  endfacet
  facet normal -3.166305e-01 8.110994e-02 -9.450748e-01
    outer loop
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
      vertex   -9.128359e+00 1.128536e+01 2.992116e+01
      vertex   -8.261960e+00 1.157692e+01 2.965591e+01
    endloop
  endfacet
  facet normal -8.328776e-01 2.010319e-02 -5.530920e-01
    outer loop
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
      vertex   -8.261960e+00 1.157692e+01 2.965591e+01
    endloop
  endfacet
  facet normal 3.983822e-01 -3.806467e-01 -8.345057e-01
    outer loop
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
    endloop
  endfacet
  facet normal 2.926222e-01 8.871904e-02 -9.521036e-01
    outer loop
      vertex   -9.128359e+00 1.128536e+01 2.992116e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
    endloop
  endfacet
  facet normal 9.883432e-01 9.515684e-03 -1.519447e-01
    outer loop
      vertex   -1.072363e+01 1.128536e+01 2.757326e+01
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
    endloop
  endfacet
  facet normal 8.910542e-01 -3.359746e-01 -3.051941e-01
    outer loop
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
    endloop
  endfacet
  facet normal 7.818143e-01 9.534039e-02 -6.161790e-01
    outer loop
      vertex   -1.050986e+01 1.128536e+01 2.896378e+01
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
      vertex   -9.955715e+00 1.128536e+01 2.966688e+01
    endloop
  endfacet
  facet normal 8.484612e-01 2.934725e-02 5.284432e-01
    outer loop
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -1.005260e+01 1.128536e+01 2.514793e+01
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
    endloop
  endfacet
  facet normal 8.717715e-01 -3.045197e-02 4.889654e-01
    outer loop
      vertex   -1.058573e+01 1.198756e+01 2.605540e+01
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
    endloop
  endfacet
  facet normal 9.953045e-01 4.734291e-02 8.442480e-02
    outer loop
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -1.058573e+01 1.198756e+01 2.605540e+01
    endloop
  endfacet
  facet normal 9.936850e-01 -2.172655e-02 1.100822e-01
    outer loop
      vertex   -1.072363e+01 1.128536e+01 2.757326e+01
      vertex   -1.054200e+01 1.128536e+01 2.593372e+01
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
    endloop
  endfacet
  facet normal 4.145043e-01 -3.276822e-02 9.094572e-01
    outer loop
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
      vertex   -9.434652e+00 1.128536e+01 2.487799e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
    endloop
  endfacet
  facet normal 3.997816e-01 -5.125540e-02 9.151763e-01
    outer loop
      vertex   -1.005260e+01 1.128536e+01 2.514793e+01
      vertex   -9.434652e+00 1.128536e+01 2.487799e+01
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
    endloop
  endfacet
  facet normal -2.653573e-01 -1.815522e-02 9.639792e-01
    outer loop
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
      vertex   -8.281563e+00 1.192151e+01 2.515243e+01
    endloop
  endfacet
  facet normal -2.205851e-01 6.351975e-02 9.732972e-01
    outer loop
      vertex   -9.434652e+00 1.128536e+01 2.487799e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
    endloop
  endfacet
  facet normal -9.938609e-01 2.455706e-02 1.078775e-01
    outer loop
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
      vertex   -7.646195e+00 1.199048e+01 2.612284e+01
    endloop
  endfacet
  facet normal -8.004862e-01 3.294290e-01 5.006979e-01
    outer loop
      vertex   -7.646195e+00 1.199048e+01 2.612284e+01
      vertex   -8.281563e+00 1.192151e+01 2.515243e+01
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
    endloop
  endfacet
  facet normal -9.081143e-01 2.091693e-02 4.181995e-01
    outer loop
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -8.281563e+00 1.192151e+01 2.515243e+01
      vertex   -8.304661e+00 1.128536e+01 2.513409e+01
    endloop
  endfacet
  facet normal -8.543551e-01 1.127731e-01 -5.073062e-01
    outer loop
      vertex   -7.655397e+00 1.128536e+01 2.862600e+01
      vertex   -8.249640e+00 1.128536e+01 2.962676e+01
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
    endloop
  endfacet
  facet normal -9.741594e-01 -2.245626e-01 -2.418758e-02
    outer loop
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -7.655397e+00 1.128536e+01 2.862600e+01
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
    endloop
  endfacet
  facet normal -9.653989e-01 2.332892e-01 -1.165382e-01
    outer loop
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
      vertex   -7.606346e+00 1.128536e+01 2.665047e+01
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -5.506548e+00 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
      vertex   -5.506548e+00 1.198604e+01 2.592136e+01
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
      vertex   -5.506548e+00 1.198604e+01 2.592136e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
      vertex   -5.506548e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -8.797742e-01 0.000000e+00 -4.753919e-01
    outer loop
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal -8.797742e-01 0.000000e+00 -4.753919e-01
    outer loop
      vertex   -3.460164e+00 1.128536e+01 2.592136e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.993569e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -5.629332e+00 1.128536e+01 2.993569e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -6.434243e+00 1.128536e+01 2.993569e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
      vertex   -6.434243e+00 1.199710e+01 2.676379e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -7.054979e+00 1.199710e+01 2.676379e+01
      vertex   -6.434243e+00 1.199710e+01 2.676379e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -7.054979e+00 1.128536e+01 2.676379e+01
      vertex   -7.054979e+00 1.199710e+01 2.676379e+01
      vertex   -6.434243e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+00 1.128536e+01 2.592136e+01
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
      vertex   -7.054979e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
      vertex   -7.054979e+00 1.199710e+01 2.676379e+01
      vertex   -7.054979e+00 1.128536e+01 2.676379e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.434243e+00 1.198604e+01 2.592136e+01
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
      vertex   -7.054979e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
      vertex   -6.434243e+00 1.198604e+01 2.592136e+01
      vertex   -7.054979e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.128536e+01 2.491523e+01
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
      vertex   -6.434243e+00 1.198604e+01 2.592136e+01
      vertex   -6.434243e+00 1.128536e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
      vertex   -6.434243e+00 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.506548e+00 1.128536e+01 2.491523e+01
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -6.434243e+00 1.128536e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
      vertex   -1.076126e+00 1.197988e+01 2.580540e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.979263e+00 1.128536e+01 2.580540e+01
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.979263e+00 1.128536e+01 2.491523e+01
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
      vertex   -2.979263e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
      vertex   -2.979263e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.000000e-01 1.189489e+01 2.491523e+01
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.979263e+00 1.128536e+01 2.491523e+01
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   -3.000000e-01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
      vertex   -2.979263e+00 1.128536e+01 2.491523e+01
      vertex   -3.000000e-01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -8.907719e-01 -1.304915e-01 -4.353131e-01
    outer loop
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   -2.991304e-01 1.128536e+01 2.630536e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
    endloop
  endfacet
  facet normal -8.746575e-01 -0.000000e+00 -4.847414e-01
    outer loop
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
    endloop
  endfacet
  facet normal -7.271223e-01 -9.025756e-02 -6.805489e-01
    outer loop
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -1.844059e+00 1.191301e+01 2.793842e+01
    endloop
  endfacet
  facet normal -7.429037e-01 -3.349903e-02 -6.685596e-01
    outer loop
      vertex   -2.991304e-01 1.128536e+01 2.630536e+01
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
    endloop
  endfacet
  facet normal -9.721457e-01 -4.221610e-02 -2.305439e-01
    outer loop
      vertex   -1.844059e+00 1.191301e+01 2.793842e+01
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
    endloop
  endfacet
  facet normal -9.152749e-01 8.620847e-02 3.934971e-01
    outer loop
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
      vertex   -1.820621e+00 1.173993e+01 2.898423e+01
    endloop
  endfacet
  facet normal -9.799187e-01 -1.538653e-01 -1.268259e-01
    outer loop
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
    endloop
  endfacet
  facet normal 1.099266e-01 -1.403868e-01 9.839755e-01
    outer loop
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -1.346066e+00 1.170765e+01 2.913264e+01
    endloop
  endfacet
  facet normal -2.902573e-01 1.056635e-01 9.510972e-01
    outer loop
      vertex   -1.820621e+00 1.173993e+01 2.898423e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
      vertex   -1.346066e+00 1.170765e+01 2.913264e+01
    endloop
  endfacet
  facet normal -6.497824e-01 -1.178343e-01 7.509314e-01
    outer loop
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
      vertex   -1.820621e+00 1.173993e+01 2.898423e+01
    endloop
  endfacet
  facet normal 3.879869e-01 7.322103e-02 9.187518e-01
    outer loop
      vertex   -1.346066e+00 1.170765e+01 2.913264e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -8.978784e-01 1.174883e+01 2.894009e+01
    endloop
  endfacet
  facet normal 8.853343e-01 -1.317783e-17 4.649550e-01
    outer loop
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -6.873124e-01 1.128536e+01 2.835997e+01
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal 9.276626e-01 -2.292427e-01 2.947710e-01
    outer loop
      vertex   -8.978784e-01 1.174883e+01 2.894009e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal -9.950287e-02 -1.090324e-17 9.950373e-01
    outer loop
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
      vertex   -6.873124e-01 1.128536e+01 2.835997e+01
    endloop
  endfacet
  facet normal -9.950287e-02 -1.120574e-17 9.950373e-01
    outer loop
      vertex   -6.873124e-01 1.128536e+01 2.835997e+01
      vertex   2.676671e-01 1.128536e+01 2.845547e+01
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
    endloop
  endfacet
  facet normal -7.730563e-01 -3.609374e-01 -5.216399e-01
    outer loop
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
      vertex   -1.626059e-01 1.128536e+01 2.947662e+01
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
    endloop
  endfacet
  facet normal -9.215329e-01 0.000000e+00 -3.883002e-01
    outer loop
      vertex   2.676671e-01 1.128536e+01 2.845547e+01
      vertex   -1.626059e-01 1.128536e+01 2.947662e+01
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
    endloop
  endfacet
  facet normal -1.941545e-01 -3.411255e-01 -9.197486e-01
    outer loop
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
    endloop
  endfacet
  facet normal -4.517321e-01 1.251312e-01 -8.833347e-01
    outer loop
      vertex   -1.626059e-01 1.128536e+01 2.947662e+01
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
    endloop
  endfacet
  facet normal 5.488314e-01 4.177914e-02 -8.348883e-01
    outer loop
      vertex   -1.957248e+00 1.128536e+01 2.985450e+01
      vertex   -2.649947e+00 1.128536e+01 2.939914e+01
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
    endloop
  endfacet
  facet normal 3.101723e-01 -4.001418e-01 -8.623686e-01
    outer loop
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
      vertex   -1.957248e+00 1.128536e+01 2.985450e+01
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
    endloop
  endfacet
  facet normal 6.361968e-03 2.243530e-01 -9.744872e-01
    outer loop
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
      vertex   -1.957248e+00 1.128536e+01 2.985450e+01
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
    endloop
  endfacet
  facet normal 8.896009e-01 -1.918532e-01 -4.144907e-01
    outer loop
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -2.950395e+00 1.182873e+01 2.849604e+01
    endloop
  endfacet
  facet normal 8.894298e-01 -2.014136e-01 -4.103014e-01
    outer loop
      vertex   -2.649947e+00 1.128536e+01 2.939914e+01
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
    endloop
  endfacet
  facet normal 8.914482e-01 -2.355658e-01 3.870773e-01
    outer loop
      vertex   -2.950395e+00 1.182873e+01 2.849604e+01
      vertex   -2.831992e+00 1.128536e+01 2.789267e+01
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
    endloop
  endfacet
  facet normal 9.866515e-01 3.959619e-02 1.579585e-01
    outer loop
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -2.831992e+00 1.128536e+01 2.789267e+01
      vertex   -2.950395e+00 1.182873e+01 2.849604e+01
    endloop
  endfacet
  facet normal 7.595681e-01 1.215546e-01 6.389685e-01
    outer loop
      vertex   -2.831992e+00 1.128536e+01 2.789267e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.831992e+00 1.128536e+01 2.789267e+01
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
      vertex   -2.964133e+00 1.128536e+01 2.871806e+01
      vertex   -2.649947e+00 1.128536e+01 2.939914e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.649947e+00 1.128536e+01 2.939914e+01
      vertex   -1.957248e+00 1.128536e+01 2.985450e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.937630e+00 1.128536e+01 2.881165e+01
      vertex   -2.649947e+00 1.128536e+01 2.939914e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.626059e-01 1.128536e+01 2.947662e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.148350e-01 1.128536e+01 2.986130e+01
      vertex   -1.603116e+00 1.128536e+01 2.910111e+01
      vertex   -1.957248e+00 1.128536e+01 2.985450e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.991304e-01 1.128536e+01 2.630536e+01
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.676671e-01 1.128536e+01 2.845547e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   -1.626059e-01 1.128536e+01 2.947662e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.873124e-01 1.128536e+01 2.835997e+01
      vertex   -1.043717e+00 1.128536e+01 2.903861e+01
      vertex   2.676671e-01 1.128536e+01 2.845547e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
      vertex   -2.991304e-01 1.128536e+01 2.630536e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
      vertex   -2.831992e+00 1.128536e+01 2.789267e+01
      vertex   -1.833993e+00 1.128536e+01 2.801090e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
      vertex   -2.979263e+00 1.128536e+01 2.491523e+01
      vertex   -2.979263e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.802183e-01 1.128536e+01 2.491523e+01
      vertex   -2.979263e+00 1.128536e+01 2.491523e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
    endloop
  endfacet
  facet normal 8.244664e-01 -6.253590e-18 5.659110e-01
    outer loop
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
      vertex   -1.076126e+00 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal 7.239280e-01 -4.377890e-02 6.884851e-01
    outer loop
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
      vertex   -1.076126e+00 1.128536e+01 2.580540e+01
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
    endloop
  endfacet
  facet normal -2.440986e-03 5.156922e-01 -8.567704e-01
    outer loop
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
    endloop
  endfacet
  facet normal -5.026873e-03 4.766264e-01 -8.790916e-01
    outer loop
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
    endloop
  endfacet
  facet normal 4.338996e-04 4.035588e-01 -9.149536e-01
    outer loop
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
    endloop
  endfacet
  facet normal -4.882430e-03 2.317489e-01 -9.727634e-01
    outer loop
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
    endloop
  endfacet
  facet normal -2.829830e-03 2.490589e-01 -9.684842e-01
    outer loop
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
    endloop
  endfacet
  facet normal 1.417967e-03 -3.098478e-03 -9.999942e-01
    outer loop
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
    endloop
  endfacet
  facet normal -1.959527e-03 -3.699140e-02 -9.993137e-01
    outer loop
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
    endloop
  endfacet
  facet normal 2.201074e-03 -2.250968e-01 -9.743339e-01
    outer loop
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
    endloop
  endfacet
  facet normal -1.036717e-03 -2.588313e-01 -9.659220e-01
    outer loop
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
    endloop
  endfacet
  facet normal 1.463062e-03 -4.276558e-01 -9.039405e-01
    outer loop
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
    endloop
  endfacet
  facet normal -4.275024e-03 -4.992825e-01 -8.664287e-01
    outer loop
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -3.189542e-03 -5.193622e-01 -8.545483e-01
    outer loop
      vertex   -2.496782e+00 -6.803906e+00 1.661536e+01
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -3.789414e-03 5.740065e-01 -8.188420e-01
    outer loop
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 7.872691e+00 1.744351e+01
    endloop
  endfacet
  facet normal 2.087387e-04 5.995009e-01 -8.003741e-01
    outer loop
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
    endloop
  endfacet
  facet normal -7.431373e-04 4.854398e-01 -8.742698e-01
    outer loop
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
    endloop
  endfacet
  facet normal -2.642340e-04 4.720839e-01 -8.815536e-01
    outer loop
      vertex   2.510000e+01 5.500000e+00 1.583464e+01
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
    endloop
  endfacet
  facet normal 7.050342e-04 7.964725e-01 -6.046743e-01
    outer loop
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
      vertex   -5.694066e-01 9.340868e+00 1.897195e+01
    endloop
  endfacet
  facet normal 5.697352e-03 8.590856e-01 -5.118003e-01
    outer loop
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
    endloop
  endfacet
  facet normal -1.709173e-03 8.238066e-01 -5.668684e-01
    outer loop
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -2.366407e-03 9.724393e-01 -2.331443e-01
    outer loop
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
    endloop
  endfacet
  facet normal 2.716739e-03 9.411686e-01 -3.379264e-01
    outer loop
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
    endloop
  endfacet
  facet normal -7.696178e-03 8.826070e-01 -4.700485e-01
    outer loop
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -4.725986e-03 8.656787e-01 -5.005778e-01
    outer loop
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -1.564446e-03 8.356367e-01 -5.492804e-01
    outer loop
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -1.573076e-03 8.594301e-01 5.112508e-01
    outer loop
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
    endloop
  endfacet
  facet normal -2.767079e-03 8.712469e-01 4.908372e-01
    outer loop
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
    endloop
  endfacet
  facet normal 1.320574e-03 9.063723e-01 4.224778e-01
    outer loop
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
    endloop
  endfacet
  facet normal -1.455234e-03 9.356493e-01 3.529281e-01
    outer loop
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 3.959693e-04 9.301065e-01 3.672897e-01
    outer loop
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
    endloop
  endfacet
  facet normal -1.433419e-04 7.696230e-01 6.384985e-01
    outer loop
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
    endloop
  endfacet
  facet normal 1.897795e-03 8.188465e-01 5.740095e-01
    outer loop
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
    endloop
  endfacet
  facet normal -3.568372e-04 8.365043e-01 5.479603e-01
    outer loop
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
    endloop
  endfacet
  facet normal -9.133076e-04 8.694244e-01 4.940652e-01
    outer loop
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
    endloop
  endfacet
  facet normal -1.809436e-04 8.815549e-01 4.720814e-01
    outer loop
      vertex   2.510000e+01 1.066536e+01 3.200000e+01
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
    endloop
  endfacet
  facet normal 1.010221e-06 6.619165e-01 7.495776e-01
    outer loop
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   -3.000000e-01 7.872708e+00 3.555652e+01
    endloop
  endfacet
  facet normal 1.008662e-06 6.260336e-01 7.797960e-01
    outer loop
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
      vertex   -3.000000e-01 7.872708e+00 3.555652e+01
    endloop
  endfacet
  facet normal 2.354651e-03 5.668672e-01 8.238058e-01
    outer loop
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
    endloop
  endfacet
  facet normal -7.025552e-06 5.405151e-01 8.413343e-01
    outer loop
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
    endloop
  endfacet
  facet normal -8.605693e-04 -5.322150e-01 8.466088e-01
    outer loop
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
    endloop
  endfacet
  facet normal -8.718516e-04 -4.483795e-01 8.938428e-01
    outer loop
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
    endloop
  endfacet
  facet normal -2.926336e-03 -4.217078e-01 9.067270e-01
    outer loop
      vertex   -2.570000e+01 -3.680766e+00 3.791295e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
    endloop
  endfacet
  facet normal 1.989517e-03 -3.050597e-01 9.523312e-01
    outer loop
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
      vertex   -2.570000e+01 -3.680766e+00 3.791295e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
    endloop
  endfacet
  facet normal -2.721121e-03 -2.021395e-01 9.793530e-01
    outer loop
      vertex   -2.570000e+01 -1.345736e+00 3.839490e+01
      vertex   -2.570000e+01 -3.680766e+00 3.791295e+01
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
    endloop
  endfacet
  facet normal 1.248802e-03 -1.234082e-01 9.923552e-01
    outer loop
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
      vertex   -2.570000e+01 -1.345736e+00 3.839490e+01
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
    endloop
  endfacet
  facet normal -4.566117e-03 4.863413e-02 9.988062e-01
    outer loop
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
      vertex   -2.570000e+01 -1.345736e+00 3.839490e+01
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
    endloop
  endfacet
  facet normal 1.203205e-03 1.014205e-01 9.948429e-01
    outer loop
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
    endloop
  endfacet
  facet normal -3.949309e-03 3.668744e-01 9.302621e-01
    outer loop
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
    endloop
  endfacet
  facet normal -7.147029e-04 3.476999e-01 9.376056e-01
    outer loop
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
    endloop
  endfacet
  facet normal -3.050832e-04 5.109715e-01 8.595976e-01
    outer loop
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
    endloop
  endfacet
  facet normal -9.214432e-04 4.965893e-01 8.679851e-01
    outer loop
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
    endloop
  endfacet
  facet normal -5.723008e-04 -6.632998e-01 7.483535e-01
    outer loop
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   2.510000e+01 -7.872691e+00 3.555650e+01
    endloop
  endfacet
  facet normal 1.351356e-03 -5.740101e-01 8.188471e-01
    outer loop
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   2.510000e+01 -7.872691e+00 3.555650e+01
    endloop
  endfacet
  facet normal 1.016762e-03 -5.705992e-01 8.212281e-01
    outer loop
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
    endloop
  endfacet
  facet normal -8.906048e-04 -4.720778e-01 8.815565e-01
    outer loop
      vertex   2.510000e+01 -5.500000e+00 3.716536e+01
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
    endloop
  endfacet
  facet normal -3.883967e-04 -4.862204e-01 8.738361e-01
    outer loop
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
      vertex   2.510000e+01 -5.500000e+00 3.716536e+01
    endloop
  endfacet
  facet normal -2.738334e-03 -7.816137e-01 6.237568e-01
    outer loop
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal 1.669066e-03 -8.238069e-01 5.668681e-01
    outer loop
      vertex   2.510000e+01 -1.058915e+01 3.214535e+01
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -3.317868e-03 -8.574106e-01 -5.146222e-01
    outer loop
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
    endloop
  endfacet
  facet normal -3.822677e-03 -8.632829e-01 -5.047059e-01
    outer loop
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
    endloop
  endfacet
  facet normal 3.817717e-03 -9.108363e-01 -4.127502e-01
    outer loop
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
    endloop
  endfacet
  facet normal -2.067532e-03 -9.660922e-01 -2.581892e-01
    outer loop
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
    endloop
  endfacet
  facet normal 3.692202e-03 -9.827158e-01 -1.850837e-01
    outer loop
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
    endloop
  endfacet
  facet normal -7.354142e-03 -9.995989e-01 2.734823e-02
    outer loop
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
    endloop
  endfacet
  facet normal -4.814805e-03 -9.987615e-01 4.952027e-02
    outer loop
      vertex   -3.000000e-01 -1.184646e+01 2.832333e+01
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
    endloop
  endfacet
  facet normal -7.117231e-04 -9.724885e-01 2.329500e-01
    outer loop
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
      vertex   -3.000000e-01 -1.184646e+01 2.832333e+01
    endloop
  endfacet
  facet normal -9.054802e-03 -9.274623e-01 3.738071e-01
    outer loop
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
    endloop
  endfacet
  facet normal -1.844334e-04 -8.967900e-01 4.424565e-01
    outer loop
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
    endloop
  endfacet
  facet normal -1.085131e-04 -9.024807e-01 4.307303e-01
    outer loop
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -1.101851e-03 -8.576582e-01 5.142190e-01
    outer loop
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   2.510000e+01 -1.058915e+01 3.214535e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -1.090837e-03 -7.603600e-01 -6.495010e-01
    outer loop
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
    endloop
  endfacet
  facet normal 3.347499e-03 -8.360474e-01 -5.486471e-01
    outer loop
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
    endloop
  endfacet
  facet normal 9.604061e-04 -8.188473e-01 -5.740106e-01
    outer loop
      vertex   2.510000e+01 -1.048939e+01 2.067138e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
    endloop
  endfacet
  facet normal -9.488511e-04 -8.689618e-01 -4.948783e-01
    outer loop
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
      vertex   2.510000e+01 -1.048939e+01 2.067138e+01
    endloop
  endfacet
  facet normal -1.849787e-04 -8.815549e-01 -4.720814e-01
    outer loop
      vertex   2.510000e+01 -1.066536e+01 2.100000e+01
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   2.510000e+01 -1.048939e+01 2.067138e+01
    endloop
  endfacet
  facet normal -1.132740e-03 -6.591648e-01 -7.519977e-01
    outer loop
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal 2.856636e-03 -5.538992e-01 -8.325788e-01
    outer loop
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
    endloop
  endfacet
  facet normal 1.544204e-03 -5.668686e-01 -8.238068e-01
    outer loop
      vertex   2.510000e+01 -5.645351e+00 1.591085e+01
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -5.472485e-04 -4.884854e-01 -8.725719e-01
    outer loop
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   2.510000e+01 -5.645351e+00 1.591085e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal 4.201282e-02 -5.753432e-01 8.168324e-01
    outer loop
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
      vertex   -1.181844e+00 -7.056280e+00 3.620490e+01
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
    endloop
  endfacet
  facet normal -3.491045e-04 -6.692525e-01 7.430350e-01
    outer loop
      vertex   2.510000e+01 -8.183972e+00 3.527623e+01
      vertex   2.510000e+01 -7.875061e+00 3.555447e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
    endloop
  endfacet
  facet normal -1.079446e-03 -6.498079e-01 7.600977e-01
    outer loop
      vertex   2.510000e+01 -7.872691e+00 3.555650e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   2.510000e+01 -7.875061e+00 3.555447e+01
    endloop
  endfacet
  facet normal 1.611811e-04 -7.009321e-01 7.132280e-01
    outer loop
      vertex   2.510000e+01 -8.185120e+00 3.527518e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
    endloop
  endfacet
  facet normal -2.443449e-04 -6.758355e-01 7.370525e-01
    outer loop
      vertex   2.510000e+01 -8.183972e+00 3.527623e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   2.510000e+01 -8.185120e+00 3.527518e+01
    endloop
  endfacet
  facet normal 7.975597e-05 -7.062953e-01 7.079173e-01
    outer loop
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
      vertex   -1.504616e-01 -8.451730e+00 3.501870e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -1.896877e-04 -7.252441e-01 6.884918e-01
    outer loop
      vertex   2.510000e+01 -8.632073e+00 3.483590e+01
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal 1.391977e-04 -7.021326e-01 7.120462e-01
    outer loop
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
      vertex   -1.522146e-01 -8.416840e+00 3.505310e+01
      vertex   -1.504616e-01 -8.451730e+00 3.501870e+01
    endloop
  endfacet
  facet normal 1.395751e-04 -7.021104e-01 7.120681e-01
    outer loop
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   -1.522146e-01 -8.416840e+00 3.505310e+01
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
    endloop
  endfacet
  facet normal -1.905881e-04 -7.253063e-01 6.884263e-01
    outer loop
      vertex   2.510000e+01 -8.775182e+00 3.468512e+01
      vertex   2.510000e+01 -8.632073e+00 3.483590e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -5.037811e-04 -7.370524e-01 6.758354e-01
    outer loop
      vertex   2.510000e+01 -8.776235e+00 3.468397e+01
      vertex   2.510000e+01 -8.775182e+00 3.468512e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -9.755439e-04 -7.490274e-01 6.625384e-01
    outer loop
      vertex   2.510000e+01 -8.916174e+00 3.453130e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   2.510000e+01 -8.917723e+00 3.452955e+01
    endloop
  endfacet
  facet normal -5.069387e-04 -7.371693e-01 6.757079e-01
    outer loop
      vertex   2.510000e+01 -8.776235e+00 3.468397e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   2.510000e+01 -8.916174e+00 3.453130e+01
    endloop
  endfacet
  facet normal -1.568205e-03 -7.601838e-01 6.497062e-01
    outer loop
      vertex   2.510000e+01 -9.054470e+00 3.437506e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
    endloop
  endfacet
  facet normal -9.664124e-04 -7.487993e-01 6.627962e-01
    outer loop
      vertex   2.510000e+01 -8.917723e+00 3.452955e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   2.510000e+01 -9.054470e+00 3.437506e+01
    endloop
  endfacet
  facet normal 1.216134e-01 -7.805151e-01 6.131936e-01
    outer loop
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal 1.961917e-02 -8.288684e-01 5.590996e-01
    outer loop
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
      vertex   -1.909845e+00 -9.877253e+00 3.331441e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal 5.710735e-03 -8.767120e-01 4.809818e-01
    outer loop
      vertex   -1.909845e+00 -9.877253e+00 3.331441e+01
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -2.736964e-03 -8.177158e-01 5.756157e-01
    outer loop
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
      vertex   -3.471032e+00 -9.653216e+00 3.362736e+01
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
    endloop
  endfacet
  facet normal -5.373040e-03 -7.715663e-01 6.361262e-01
    outer loop
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -3.471032e+00 -9.653216e+00 3.362736e+01
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
    endloop
  endfacet
  facet normal -3.137062e-03 -7.855946e-01 6.187336e-01
    outer loop
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
    endloop
  endfacet
  facet normal 1.245134e-04 -6.853453e-01 7.282183e-01
    outer loop
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
    endloop
  endfacet
  facet normal 4.813795e-04 -6.734537e-01 7.392292e-01
    outer loop
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
    endloop
  endfacet
  facet normal -4.869153e-03 -6.112186e-01 7.914469e-01
    outer loop
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
    endloop
  endfacet
  facet normal -3.204349e-03 -5.905048e-01 8.070278e-01
    outer loop
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
    endloop
  endfacet
  facet normal 7.007227e-03 -5.512336e-01 8.343215e-01
    outer loop
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
      vertex   -1.181844e+00 -7.056280e+00 3.620490e+01
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
    endloop
  endfacet
  facet normal 3.436638e-02 8.312039e-01 5.549045e-01
    outer loop
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
      vertex   -1.559329e+00 9.822585e+00 3.339262e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
    endloop
  endfacet
  facet normal -7.385438e-05 7.487996e-01 6.627965e-01
    outer loop
      vertex   2.510000e+01 9.054470e+00 3.437506e+01
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   2.510000e+01 8.917723e+00 3.452955e+01
    endloop
  endfacet
  facet normal -1.105634e-04 7.601847e-01 6.497070e-01
    outer loop
      vertex   2.510000e+01 9.054470e+00 3.437506e+01
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
    endloop
  endfacet
  facet normal -9.449024e-04 7.192937e-01 6.947054e-01
    outer loop
      vertex   2.510000e+01 8.916174e+00 3.453130e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.336461e+00 3.513154e+01
    endloop
  endfacet
  facet normal -4.038977e-04 7.118203e-01 7.023615e-01
    outer loop
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.916174e+00 3.453130e+01
    endloop
  endfacet
  facet normal -7.178702e-05 7.490277e-01 6.625387e-01
    outer loop
      vertex   2.510000e+01 8.917723e+00 3.452955e+01
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   2.510000e+01 8.916174e+00 3.453130e+01
    endloop
  endfacet
  facet normal -1.956014e-04 6.891668e-01 7.246027e-01
    outer loop
      vertex   2.510000e+01 8.336461e+00 3.513154e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.335896e+00 3.513207e+01
    endloop
  endfacet
  facet normal -1.517153e-05 6.758355e-01 7.370525e-01
    outer loop
      vertex   2.510000e+01 8.185120e+00 3.527518e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.183972e+00 3.527623e+01
    endloop
  endfacet
  facet normal -1.775319e-04 6.884238e-01 7.253086e-01
    outer loop
      vertex   2.510000e+01 8.335896e+00 3.513207e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.185120e+00 3.527518e+01
    endloop
  endfacet
  facet normal 5.911080e-06 6.636551e-01 7.480387e-01
    outer loop
      vertex   2.510000e+01 8.031304e+00 3.541617e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.029554e+00 3.541773e+01
    endloop
  endfacet
  facet normal -1.354740e-05 6.757077e-01 7.371696e-01
    outer loop
      vertex   2.510000e+01 8.183972e+00 3.527623e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 8.031304e+00 3.541617e+01
    endloop
  endfacet
  facet normal -1.064142e-01 6.817713e-01 7.237844e-01
    outer loop
      vertex   -3.000000e-01 7.872708e+00 3.555652e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
    endloop
  endfacet
  facet normal -1.137805e-04 6.498082e-01 7.600982e-01
    outer loop
      vertex   2.510000e+01 7.875061e+00 3.555447e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
    endloop
  endfacet
  facet normal 7.196774e-06 6.627855e-01 7.488093e-01
    outer loop
      vertex   2.510000e+01 7.875061e+00 3.555447e+01
      vertex   2.510000e+01 8.029554e+00 3.541773e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal 1.633997e-02 6.259501e-01 7.796919e-01
    outer loop
      vertex   -3.000000e-01 7.872708e+00 3.555652e+01
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
    endloop
  endfacet
  facet normal -5.860513e-03 6.156102e-01 7.880290e-01
    outer loop
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
    endloop
  endfacet
  facet normal 2.296148e-02 5.403726e-01 8.411125e-01
    outer loop
      vertex   -3.000000e-01 7.240075e+00 3.606440e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
    endloop
  endfacet
  facet normal 8.557615e-03 5.289858e-01 8.485875e-01
    outer loop
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
    endloop
  endfacet
  facet normal -4.161164e-03 5.951827e-01 8.035797e-01
    outer loop
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
    endloop
  endfacet
  facet normal -4.179594e-03 5.953794e-01 8.034338e-01
    outer loop
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
    endloop
  endfacet
  facet normal 3.519485e-04 6.672725e-01 7.448136e-01
    outer loop
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
    endloop
  endfacet
  facet normal -1.848559e-03 7.479940e-01 6.637030e-01
    outer loop
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
      vertex   -3.890631e+00 9.384056e+00 3.397588e+01
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
    endloop
  endfacet
  facet normal -1.826383e-03 7.481757e-01 6.634982e-01
    outer loop
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
    endloop
  endfacet
  facet normal -3.561341e-04 7.918864e-01 6.106683e-01
    outer loop
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -3.890631e+00 9.384056e+00 3.397588e+01
      vertex   -3.439982e+00 9.673910e+00 3.360027e+01
    endloop
  endfacet
  facet normal -3.577779e-04 8.152121e-01 5.791624e-01
    outer loop
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -3.439982e+00 9.673910e+00 3.360027e+01
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
    endloop
  endfacet
  facet normal 5.127699e-03 8.509841e-01 5.251665e-01
    outer loop
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
      vertex   -1.559329e+00 9.822585e+00 3.339262e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
    endloop
  endfacet
  facet normal 1.053831e-01 5.920734e-01 -7.989640e-01
    outer loop
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
    endloop
  endfacet
  facet normal -3.491045e-04 6.692525e-01 -7.430350e-01
    outer loop
      vertex   2.510000e+01 7.875061e+00 1.744553e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 8.183972e+00 1.772377e+01
    endloop
  endfacet
  facet normal -1.092576e-03 6.494541e-01 -7.604000e-01
    outer loop
      vertex   2.510000e+01 7.872691e+00 1.744351e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 7.875061e+00 1.744553e+01
    endloop
  endfacet
  facet normal 1.611811e-04 7.009321e-01 -7.132280e-01
    outer loop
      vertex   2.510000e+01 8.185120e+00 1.772482e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
    endloop
  endfacet
  facet normal -2.443449e-04 6.758355e-01 -7.370525e-01
    outer loop
      vertex   2.510000e+01 8.183972e+00 1.772377e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   2.510000e+01 8.185120e+00 1.772482e+01
    endloop
  endfacet
  facet normal 7.463916e-05 7.066563e-01 -7.075570e-01
    outer loop
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
      vertex   -1.504616e-01 8.451730e+00 1.798130e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -1.897397e-04 7.252441e-01 -6.884918e-01
    outer loop
      vertex   2.510000e+01 8.632073e+00 1.816410e+01
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal 1.395326e-04 7.021129e-01 -7.120656e-01
    outer loop
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
      vertex   -1.522146e-01 8.416840e+00 1.794690e+01
      vertex   -1.504616e-01 8.451730e+00 1.798130e+01
    endloop
  endfacet
  facet normal 1.395751e-04 7.021104e-01 -7.120681e-01
    outer loop
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   -1.522146e-01 8.416840e+00 1.794690e+01
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
    endloop
  endfacet
  facet normal -1.906401e-04 7.253063e-01 -6.884263e-01
    outer loop
      vertex   2.510000e+01 8.775182e+00 1.831488e+01
      vertex   2.510000e+01 8.632073e+00 1.816410e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -5.038322e-04 7.370524e-01 -6.758354e-01
    outer loop
      vertex   2.510000e+01 8.776235e+00 1.831603e+01
      vertex   2.510000e+01 8.775182e+00 1.831488e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -5.069898e-04 7.371693e-01 -6.757079e-01
    outer loop
      vertex   2.510000e+01 8.916174e+00 1.846870e+01
      vertex   2.510000e+01 8.776235e+00 1.831603e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -9.612945e-04 7.486689e-01 -6.629434e-01
    outer loop
      vertex   2.510000e+01 8.917723e+00 1.847045e+01
      vertex   2.510000e+01 8.916174e+00 1.846870e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -1.582037e-03 7.604420e-01 -6.494040e-01
    outer loop
      vertex   2.510000e+01 9.054470e+00 1.862494e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
    endloop
  endfacet
  facet normal -9.666751e-04 7.488033e-01 -6.627916e-01
    outer loop
      vertex   2.510000e+01 8.917723e+00 1.847045e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   2.510000e+01 9.054470e+00 1.862494e+01
    endloop
  endfacet
  facet normal -7.236473e-04 7.442020e-01 -6.679542e-01
    outer loop
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   -5.694066e-01 9.340868e+00 1.897195e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
    endloop
  endfacet
  facet normal 1.777890e-03 8.605073e-01 -5.094350e-01
    outer loop
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
    endloop
  endfacet
  facet normal -2.444017e-03 8.094086e-01 -5.872408e-01
    outer loop
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
      vertex   -3.700586e+00 9.536608e+00 1.921672e+01
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
    endloop
  endfacet
  facet normal -2.277347e-03 7.648736e-01 -6.441764e-01
    outer loop
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -3.700586e+00 9.536608e+00 1.921672e+01
      vertex   -4.311689e+00 8.776243e+00 1.831605e+01
    endloop
  endfacet
  facet normal -8.250424e-03 6.866271e-01 -7.269630e-01
    outer loop
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -4.311689e+00 8.776243e+00 1.831605e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal -6.545078e-03 7.100264e-01 -7.041446e-01
    outer loop
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
    endloop
  endfacet
  facet normal -1.134957e-03 6.703346e-01 -7.420581e-01
    outer loop
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
      vertex   -3.666887e+00 7.275447e+00 1.695933e+01
    endloop
  endfacet
  facet normal -7.214058e-04 5.896886e-01 -8.076304e-01
    outer loop
      vertex   -3.666887e+00 7.275447e+00 1.695933e+01
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
    endloop
  endfacet
  facet normal 9.993201e-03 5.400709e-01 -8.415602e-01
    outer loop
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
    endloop
  endfacet
  facet normal 1.352284e-02 -8.323081e-01 -5.541483e-01
    outer loop
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
    endloop
  endfacet
  facet normal -3.491464e-04 -7.430356e-01 -6.692518e-01
    outer loop
      vertex   2.510000e+01 -9.054470e+00 1.862494e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   2.510000e+01 -8.776235e+00 1.831603e+01
    endloop
  endfacet
  facet normal -1.094438e-03 -7.604425e-01 -6.494044e-01
    outer loop
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   2.510000e+01 -9.054470e+00 1.862494e+01
    endloop
  endfacet
  facet normal -2.443750e-04 -7.370525e-01 -6.758355e-01
    outer loop
      vertex   2.510000e+01 -8.776235e+00 1.831603e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
    endloop
  endfacet
  facet normal 2.705791e-04 -7.077374e-01 -7.064756e-01
    outer loop
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
      vertex   -1.504616e-01 -8.518702e+00 1.804827e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal 2.838481e-04 -7.071062e-01 -7.071073e-01
    outer loop
      vertex   2.510000e+01 -8.185120e+00 1.772482e+01
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal 1.807296e-04 -7.120571e-01 -7.021216e-01
    outer loop
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
      vertex   -1.522146e-01 -8.553105e+00 1.808316e+01
      vertex   -1.504616e-01 -8.518702e+00 1.804827e+01
    endloop
  endfacet
  facet normal 1.805065e-04 -7.120694e-01 -7.021091e-01
    outer loop
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   -1.522146e-01 -8.553105e+00 1.808316e+01
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
    endloop
  endfacet
  facet normal -5.037788e-04 -6.758354e-01 -7.370524e-01
    outer loop
      vertex   2.510000e+01 -8.185120e+00 1.772482e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
      vertex   2.510000e+01 -8.183972e+00 1.772377e+01
    endloop
  endfacet
  facet normal -5.068177e-04 -6.757127e-01 -7.371649e-01
    outer loop
      vertex   2.510000e+01 -8.031304e+00 1.758383e+01
      vertex   2.510000e+01 -8.183972e+00 1.772377e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal -9.684464e-04 -6.627409e-01 -7.488482e-01
    outer loop
      vertex   2.510000e+01 -8.029554e+00 1.758228e+01
      vertex   2.510000e+01 -8.031304e+00 1.758383e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal -9.666907e-04 -6.627904e-01 -7.488043e-01
    outer loop
      vertex   2.510000e+01 -7.875061e+00 1.744553e+01
      vertex   2.510000e+01 -8.029554e+00 1.758228e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal -1.579733e-03 -6.494537e-01 -7.603995e-01
    outer loop
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   2.510000e+01 -7.875061e+00 1.744553e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal 2.194144e-02 -5.599628e-01 -8.282271e-01
    outer loop
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
      vertex   -1.632213e+00 -6.878993e+00 1.666796e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -1.954505e-03 -5.785267e-01 -8.156611e-01
    outer loop
      vertex   -2.496782e+00 -6.803906e+00 1.661536e+01
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
    endloop
  endfacet
  facet normal 4.568869e-03 -5.378970e-01 -8.429982e-01
    outer loop
      vertex   -1.632213e+00 -6.878993e+00 1.666796e+01
      vertex   -2.496782e+00 -6.803906e+00 1.661536e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -1.781956e-03 -6.301541e-01 -7.764681e-01
    outer loop
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
    endloop
  endfacet
  facet normal -6.385157e-03 -7.037901e-01 -7.103793e-01
    outer loop
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
    endloop
  endfacet
  facet normal -5.788140e-03 -7.074908e-01 -7.066989e-01
    outer loop
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
    endloop
  endfacet
  facet normal -1.398719e-03 -7.775358e-01 -6.288372e-01
    outer loop
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
    endloop
  endfacet
  facet normal -1.812617e-03 -8.153007e-01 -5.790350e-01
    outer loop
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
    endloop
  endfacet
  facet normal -1.518836e-02 9.681239e-01 2.500108e-01
    outer loop
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
      vertex   -2.570000e+01 1.185226e+01 2.837721e+01
    endloop
  endfacet
  facet normal 1.216937e-02 9.901946e-01 1.391641e-01
    outer loop
      vertex   -2.570000e+01 1.185226e+01 2.837721e+01
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
    endloop
  endfacet
  facet normal 1.986295e-02 9.927510e-01 1.185363e-01
    outer loop
      vertex   -2.244379e+01 1.170346e+01 2.915124e+01
      vertex   -2.101804e+01 1.199124e+01 2.650214e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
    endloop
  endfacet
  facet normal -3.797129e-02 9.970535e-01 -6.665227e-02
    outer loop
      vertex   -2.101804e+01 1.199124e+01 2.650214e+01
      vertex   -2.076235e+01 1.189489e+01 2.491523e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
    endloop
  endfacet
  facet normal 1.000498e-19 9.999568e-01 9.293523e-03
    outer loop
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -1.661501e+01 1.197988e+01 2.580540e+01
      vertex   -1.851814e+01 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal -6.764729e-02 9.931932e-01 -9.482184e-02
    outer loop
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -1.851814e+01 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.993341e-01 -3.648836e-02
    outer loop
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 3.966274e-03 9.825672e-01 1.858656e-01
    outer loop
      vertex   -1.660043e+01 1.173049e+01 2.902604e+01
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
    endloop
  endfacet
  facet normal -3.128688e-04 9.804495e-01 1.967706e-01
    outer loop
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
      vertex   -1.719920e+01 1.172296e+01 2.906264e+01
      vertex   -1.660043e+01 1.173049e+01 2.902604e+01
    endloop
  endfacet
  facet normal -3.334473e-03 9.876375e-01 1.567197e-01
    outer loop
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
      vertex   -1.752654e+01 1.180041e+01 2.867621e+01
    endloop
  endfacet
  facet normal -1.359361e-03 9.675172e-01 2.528014e-01
    outer loop
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -1.776050e+01 1.155166e+01 2.974787e+01
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
    endloop
  endfacet
  facet normal -8.254954e-03 9.702423e-01 2.419951e-01
    outer loop
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
    endloop
  endfacet
  facet normal 1.535033e-02 9.825172e-01 1.855378e-01
    outer loop
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
      vertex   -1.827303e+01 1.166467e+01 2.931262e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.857406e-01 1.682722e-01
    outer loop
      vertex   -1.916617e+01 1.174581e+01 2.895684e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
      vertex   -2.134557e+01 1.174581e+01 2.895684e+01
    endloop
  endfacet
  facet normal -1.404890e-02 9.945479e-01 1.033302e-01
    outer loop
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -2.134557e+01 1.174581e+01 2.895684e+01
    endloop
  endfacet
  facet normal 1.686153e-02 9.978574e-01 6.321690e-02
    outer loop
      vertex   -2.028053e+01 1.196923e+01 2.695118e+01
      vertex   -1.845743e+01 1.185005e+01 2.834618e+01
      vertex   -1.661501e+01 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal 1.963077e-03 9.750593e-01 2.219359e-01
    outer loop
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
      vertex   -1.411842e+01 1.157923e+01 2.964870e+01
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
    endloop
  endfacet
  facet normal -1.190794e-02 9.790064e-01 2.034811e-01
    outer loop
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
    endloop
  endfacet
  facet normal 1.379446e-03 9.809146e-01 1.944344e-01
    outer loop
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
      vertex   -1.307238e+01 1.171188e+01 2.911354e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
    endloop
  endfacet
  facet normal -4.868366e-04 9.847401e-01 1.740306e-01
    outer loop
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
      vertex   -1.464366e+01 1.179832e+01 2.869081e+01
    endloop
  endfacet
  facet normal 5.523167e-03 9.944132e-01 1.054132e-01
    outer loop
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
      vertex   -1.371596e+01 1.181686e+01 2.858849e+01
    endloop
  endfacet
  facet normal 5.325737e-03 9.941616e-01 1.077697e-01
    outer loop
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal 5.636791e-03 9.945842e-01 1.037805e-01
    outer loop
      vertex   -1.527121e+01 1.183960e+01 2.845547e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
      vertex   -1.622619e+01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal -3.880004e-04 9.866176e-01 1.630508e-01
    outer loop
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
    endloop
  endfacet
  facet normal -4.281170e-04 9.926633e-01 1.209110e-01
    outer loop
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
    endloop
  endfacet
  facet normal 1.323709e-02 9.891651e-01 1.462097e-01
    outer loop
      vertex   -1.238922e+01 1.193584e+01 2.773924e+01
      vertex   -1.319522e+01 1.188732e+01 2.814045e+01
      vertex   -1.256186e+01 1.180495e+01 2.864040e+01
    endloop
  endfacet
  facet normal -4.038636e-03 9.962079e-01 8.691100e-02
    outer loop
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
      vertex   -1.746854e+01 1.188503e+01 2.814418e+01
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
    endloop
  endfacet
  facet normal 7.225113e-04 9.975947e-01 6.931259e-02
    outer loop
      vertex   -1.418503e+01 1.192836e+01 2.780002e+01
      vertex   -1.466144e+01 1.198701e+01 2.696081e+01
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
    endloop
  endfacet
  facet normal -3.780093e-02 9.973969e-01 -6.140449e-02
    outer loop
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -7.236548e-03 9.998146e-01 -1.784309e-02
    outer loop
      vertex   -1.466144e+01 1.198701e+01 2.696081e+01
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.663551e+01 1.197594e+01 2.714080e+01
    endloop
  endfacet
  facet normal 6.089540e-04 9.952936e-01 -9.690304e-02
    outer loop
      vertex   -1.448918e+01 1.196461e+01 2.563548e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 7.605041e-04 9.684760e-01 2.491058e-01
    outer loop
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
      vertex   -1.248806e+01 1.154280e+01 2.978059e+01
    endloop
  endfacet
  facet normal 2.740674e-02 9.841656e-01 1.751197e-01
    outer loop
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.143478e+01 1.191894e+01 2.780657e+01
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
    endloop
  endfacet
  facet normal 5.025955e-03 9.872101e-01 1.593454e-01
    outer loop
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.191183e+01 1.166768e+01 2.929332e+01
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
    endloop
  endfacet
  facet normal 2.779044e-03 9.979878e-01 6.334529e-02
    outer loop
      vertex   -1.143478e+01 1.191894e+01 2.780657e+01
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
    endloop
  endfacet
  facet normal 3.554664e-02 9.985292e-01 4.093632e-02
    outer loop
      vertex   -1.058573e+01 1.198756e+01 2.605540e+01
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
      vertex   -1.071807e+01 1.192691e+01 2.764964e+01
    endloop
  endfacet
  facet normal -6.370515e-03 9.995647e-01 -2.880528e-02
    outer loop
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
      vertex   -1.145294e+01 1.199705e+01 2.657677e+01
      vertex   -1.058573e+01 1.198756e+01 2.605540e+01
    endloop
  endfacet
  facet normal 5.580658e-03 9.978307e-01 -6.559532e-02
    outer loop
      vertex   -1.058573e+01 1.198756e+01 2.605540e+01
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
    endloop
  endfacet
  facet normal -2.103484e-03 9.957730e-01 -9.182424e-02
    outer loop
      vertex   -1.248064e+01 1.190479e+01 2.499821e+01
      vertex   -1.173553e+01 1.196949e+01 2.568274e+01
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
    endloop
  endfacet
  facet normal -2.940676e-02 9.936981e-01 -1.081630e-01
    outer loop
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
      vertex   -7.646195e+00 1.199048e+01 2.612284e+01
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
    endloop
  endfacet
  facet normal 4.858826e-03 9.972423e-01 -7.405487e-02
    outer loop
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
      vertex   -8.281563e+00 1.192151e+01 2.515243e+01
      vertex   -7.646195e+00 1.199048e+01 2.612284e+01
    endloop
  endfacet
  facet normal 7.223984e-03 9.734916e-01 2.286087e-01
    outer loop
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
      vertex   -8.261960e+00 1.157692e+01 2.965591e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal 7.316580e-02 9.866904e-01 1.452196e-01
    outer loop
      vertex   -7.768911e+00 1.174617e+01 2.891961e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
    endloop
  endfacet
  facet normal 4.848368e-02 9.866674e-01 1.553604e-01
    outer loop
      vertex   -6.434243e+00 1.199710e+01 2.676379e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal -8.009131e-02 9.967017e-01 -1.308397e-02
    outer loop
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
      vertex   -7.054979e+00 1.199710e+01 2.676379e+01
    endloop
  endfacet
  facet normal 1.388289e-02 9.997281e-01 1.873362e-02
    outer loop
      vertex   -7.646195e+00 1.199048e+01 2.612284e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.983730e-01 5.701984e-02
    outer loop
      vertex   -6.434243e+00 1.199710e+01 2.676379e+01
      vertex   -7.054979e+00 1.199710e+01 2.676379e+01
      vertex   -7.519524e+00 1.196675e+01 2.729524e+01
    endloop
  endfacet
  facet normal 3.564720e-04 9.954734e-01 -9.503954e-02
    outer loop
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
    endloop
  endfacet
  facet normal -3.802619e-04 9.989305e-01 -4.623481e-02
    outer loop
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
    endloop
  endfacet
  facet normal 3.679698e-04 9.980468e-01 -6.246995e-02
    outer loop
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
    endloop
  endfacet
  facet normal -1.134567e-03 9.999006e-01 1.405640e-02
    outer loop
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
    endloop
  endfacet
  facet normal -6.903627e-20 9.983401e-01 -5.759464e-02
    outer loop
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
      vertex   -5.506548e+00 1.198604e+01 2.592136e+01
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
    endloop
  endfacet
  facet normal 2.367287e-19 9.987255e-01 -5.047076e-02
    outer loop
      vertex   -1.186332e+00 1.198799e+01 2.596596e+01
      vertex   -1.076126e+00 1.197988e+01 2.580540e+01
      vertex   -2.979263e+00 1.197988e+01 2.580540e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.953729e-01 -9.608697e-02
    outer loop
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -5.506548e+00 1.197716e+01 2.576744e+01
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.888268e-01 -1.490689e-01
    outer loop
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.888268e-01 -1.490689e-01
    outer loop
      vertex   -1.515866e+01 1.189489e+01 2.491523e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
      vertex   -1.851814e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -7.478666e-19 9.779498e-01 -2.088399e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.779498e-01 -2.088399e-01
    outer loop
      vertex   -3.000000e-01 1.189489e+01 2.491523e+01
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   -2.979263e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.908154e-01 -1.352211e-01
    outer loop
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 1.031799e-03 9.789707e-01 -2.039985e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -5.506548e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -3.197407e-04 9.691405e-01 -2.465090e-01
    outer loop
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.689362e-01 -2.473109e-01
    outer loop
      vertex   -2.076235e+01 1.189489e+01 2.491523e+01
      vertex   -1.983806e+01 1.189489e+01 2.491523e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
    endloop
  endfacet
  facet normal -1.049960e-03 9.701222e-01 -2.426147e-01
    outer loop
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
    endloop
  endfacet
  facet normal 7.487383e-05 9.929105e-01 -1.188644e-01
    outer loop
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
    endloop
  endfacet
  facet normal 3.640413e-03 9.879323e-01 -1.548435e-01
    outer loop
      vertex   -1.011406e+01 1.192940e+01 2.521085e+01
      vertex   -1.335788e+01 1.189178e+01 2.489459e+01
      vertex   -1.248064e+01 1.190479e+01 2.499821e+01
    endloop
  endfacet
  facet normal -4.050390e-04 9.934094e-01 -1.146196e-01
    outer loop
      vertex   -9.345317e+00 1.188721e+01 2.485896e+01
      vertex   -8.281563e+00 1.192151e+01 2.515243e+01
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -6.410029e-02 9.919767e-01 -1.089653e-01
    outer loop
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
      vertex   -2.076235e+01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -2.001697e-03 9.987629e-01 4.968471e-02
    outer loop
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
    endloop
  endfacet
  facet normal 6.816113e-04 9.987860e-01 4.925549e-02
    outer loop
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
      vertex   -1.844059e+00 1.191301e+01 2.793842e+01
    endloop
  endfacet
  facet normal 2.114129e-02 9.879941e-01 1.530379e-01
    outer loop
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
      vertex   -1.844059e+00 1.191301e+01 2.793842e+01
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
    endloop
  endfacet
  facet normal -1.335820e-02 9.956971e-01 9.169938e-02
    outer loop
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
    endloop
  endfacet
  facet normal 6.343467e-04 9.775816e-01 2.105557e-01
    outer loop
      vertex   -8.978784e-01 1.174883e+01 2.894009e+01
      vertex   -1.820621e+00 1.173993e+01 2.898423e+01
      vertex   -1.346066e+00 1.170765e+01 2.913264e+01
    endloop
  endfacet
  facet normal 1.468504e-03 9.835743e-01 1.804977e-01
    outer loop
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
      vertex   -8.978784e-01 1.174883e+01 2.894009e+01
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
    endloop
  endfacet
  facet normal -5.707441e-04 9.824828e-01 1.863525e-01
    outer loop
      vertex   -1.990297e+00 1.181793e+01 2.857248e+01
      vertex   -1.820621e+00 1.173993e+01 2.898423e+01
      vertex   -8.978784e-01 1.174883e+01 2.894009e+01
    endloop
  endfacet
  facet normal -3.524110e-02 9.558618e-01 2.916954e-01
    outer loop
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -2.570000e+01 1.185226e+01 2.837721e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.323721e-01 3.615001e-01
    outer loop
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -2.244379e+01 1.152378e+01 2.984701e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
    endloop
  endfacet
  facet normal -2.101932e-03 9.355171e-01 3.532752e-01
    outer loop
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
    endloop
  endfacet
  facet normal -1.115143e-04 9.458518e-01 3.245987e-01
    outer loop
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal -5.252248e-04 9.437029e-01 3.307939e-01
    outer loop
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 1.404436e-04 9.623724e-01 2.717338e-01
    outer loop
      vertex   -8.261960e+00 1.157692e+01 2.965591e+01
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal -1.054636e-04 9.650551e-01 2.620469e-01
    outer loop
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
    endloop
  endfacet
  facet normal -3.953808e-03 9.582714e-01 2.858328e-01
    outer loop
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
      vertex   -1.411842e+01 1.157923e+01 2.964870e+01
      vertex   -1.577829e+01 1.161540e+01 2.950450e+01
    endloop
  endfacet
  facet normal 1.145077e-03 9.679655e-01 2.510804e-01
    outer loop
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
      vertex   -1.023483e+01 1.164698e+01 2.936869e+01
      vertex   -1.248806e+01 1.154280e+01 2.978059e+01
    endloop
  endfacet
  facet normal 1.258284e-05 9.605608e-01 2.780700e-01
    outer loop
      vertex   -1.248806e+01 1.154280e+01 2.978059e+01
      vertex   -1.316370e+01 1.150906e+01 2.989719e+01
      vertex   -9.242210e+00 1.150623e+01 2.990675e+01
    endloop
  endfacet
  facet normal -1.698882e-04 9.653454e-01 2.609757e-01
    outer loop
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal 2.808090e-04 9.465888e-01 3.224430e-01
    outer loop
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal 9.150764e-03 9.525925e-01 3.041114e-01
    outer loop
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
      vertex   -1.523828e+00 1.150584e+01 2.990809e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.456816e-01 3.250944e-01
    outer loop
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -6.434243e+00 1.149765e+01 2.993569e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 2.444938e-04 9.617526e-01 2.739194e-01
    outer loop
      vertex   -1.673022e+01 1.150447e+01 2.991266e+01
      vertex   -1.776050e+01 1.155166e+01 2.974787e+01
      vertex   -1.916617e+01 1.152378e+01 2.984701e+01
    endloop
  endfacet
  facet normal -7.892600e-03 9.776388e-01 2.101428e-01
    outer loop
      vertex   -2.530954e+00 1.161761e+01 2.949398e+01
      vertex   -2.950395e+00 1.182873e+01 2.849604e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
    endloop
  endfacet
  facet normal -8.740771e-02 9.939879e-01 6.593905e-02
    outer loop
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -2.950395e+00 1.182873e+01 2.849604e+01
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
    endloop
  endfacet
  facet normal -7.551109e-02 9.939211e-01 8.011855e-02
    outer loop
      vertex   -3.460164e+00 1.198604e+01 2.592136e+01
      vertex   -5.629332e+00 1.149765e+01 2.993569e+01
      vertex   -2.341456e+00 1.196963e+01 2.717938e+01
    endloop
  endfacet
  facet normal -1.219105e-02 -9.107752e-01 -4.127225e-01
    outer loop
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
      vertex   -3.000000e-01 -1.021250e+01 2.023390e+01
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
    endloop
  endfacet
  facet normal -3.903992e-02 -8.502262e-01 -5.249678e-01
    outer loop
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   6.518182e+00 -1.095275e+01 2.159899e+01
    endloop
  endfacet
  facet normal -1.060433e-02 -9.232567e-01 -3.840372e-01
    outer loop
      vertex   6.518182e+00 -1.095275e+01 2.159899e+01
      vertex   5.320501e+00 -1.128601e+01 2.243324e+01
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
    endloop
  endfacet
  facet normal -5.522665e-03 -9.525922e-01 -3.041998e-01
    outer loop
      vertex   5.320501e+00 -1.128601e+01 2.243324e+01
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
    endloop
  endfacet
  facet normal -2.103082e-02 -9.808179e-01 -1.937882e-01
    outer loop
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
    endloop
  endfacet
  facet normal -2.600862e-02 -9.823901e-01 -1.850223e-01
    outer loop
      vertex   -3.000000e-01 -1.135909e+01 2.276413e+01
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
    endloop
  endfacet
  facet normal 5.763851e-03 -9.983878e-01 -5.646728e-02
    outer loop
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
    endloop
  endfacet
  facet normal -1.741221e-02 -9.986217e-01 4.951333e-02
    outer loop
      vertex   -3.000000e-01 -1.196309e+01 2.597110e+01
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
      vertex   -3.000000e-01 -1.184646e+01 2.832333e+01
    endloop
  endfacet
  facet normal 4.552594e-03 -9.945073e-01 1.045679e-01
    outer loop
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
      vertex   -3.000000e-01 -1.184646e+01 2.832333e+01
    endloop
  endfacet
  facet normal -5.094168e-03 -9.724761e-01 2.329470e-01
    outer loop
      vertex   -3.000000e-01 -1.184646e+01 2.832333e+01
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
    endloop
  endfacet
  facet normal -5.204275e-04 -9.697306e-01 2.441766e-01
    outer loop
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
    endloop
  endfacet
  facet normal -2.137861e-03 -9.306160e-01 3.659907e-01
    outer loop
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
    endloop
  endfacet
  facet normal -1.690144e-02 -8.966619e-01 4.423933e-01
    outer loop
      vertex   -3.000000e-01 -1.137942e+01 3.027305e+01
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -1.185908e-02 -8.887319e-01 4.582739e-01
    outer loop
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   -3.000000e-01 -9.987104e+00 3.309506e+01
    endloop
  endfacet
  facet normal -1.336752e-04 -8.856257e-01 4.643997e-01
    outer loop
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   2.510000e+01 -1.066536e+01 3.200000e+01
      vertex   2.510000e+01 -1.058915e+01 3.214535e+01
    endloop
  endfacet
  facet normal -5.347649e-03 -4.483733e-01 8.938304e-01
    outer loop
      vertex   -3.000000e-01 -6.187097e+00 3.677176e+01
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
    endloop
  endfacet
  facet normal -2.840481e-03 -4.256127e-01 9.049009e-01
    outer loop
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   6.273509e+00 -4.756257e+00 3.751461e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
    endloop
  endfacet
  facet normal -6.450897e-04 -3.364269e-01 9.417094e-01
    outer loop
      vertex   6.273509e+00 -4.756257e+00 3.751461e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
    endloop
  endfacet
  facet normal -8.260540e-03 -3.050499e-01 9.523005e-01
    outer loop
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
      vertex   -3.000000e-01 -4.613617e+00 3.756107e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
    endloop
  endfacet
  facet normal 1.316711e-02 -1.920221e-01 9.813023e-01
    outer loop
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
    endloop
  endfacet
  facet normal -1.000345e-02 -1.234021e-01 9.923063e-01
    outer loop
      vertex   -3.000000e-01 -2.595550e+00 3.820751e+01
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
    endloop
  endfacet
  facet normal 9.292804e-03 -2.606865e-02 9.996170e-01
    outer loop
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
    endloop
  endfacet
  facet normal -3.134147e-02 1.013708e-01 9.943549e-01
    outer loop
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
      vertex   -3.000000e-01 -4.900241e-01 3.846935e+01
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
    endloop
  endfacet
  facet normal -1.516741e-02 1.322577e-01 9.910993e-01
    outer loop
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
      vertex   4.063905e+00 2.392555e+00 3.825325e+01
    endloop
  endfacet
  facet normal -4.812045e-03 2.541350e-01 9.671568e-01
    outer loop
      vertex   4.063905e+00 2.392555e+00 3.825325e+01
      vertex   4.873582e+00 3.598410e+00 3.794042e+01
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
    endloop
  endfacet
  facet normal -2.514367e-02 3.667612e-01 9.299753e-01
    outer loop
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
    endloop
  endfacet
  facet normal -2.464092e-02 3.652783e-01 9.305722e-01
    outer loop
      vertex   4.873582e+00 3.598410e+00 3.794042e+01
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
      vertex   -3.000000e-01 2.741081e+00 3.813995e+01
    endloop
  endfacet
  facet normal -7.371280e-03 4.358652e-01 8.999818e-01
    outer loop
      vertex   -3.000000e-01 6.025719e+00 3.684457e+01
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
    endloop
  endfacet
  facet normal -2.160973e-04 4.643952e-01 8.856281e-01
    outer loop
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   2.510000e+01 5.500000e+00 3.716536e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
    endloop
  endfacet
  facet normal -5.580619e-03 8.949543e-01 4.461229e-01
    outer loop
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
    endloop
  endfacet
  facet normal -2.350907e-03 9.256218e-01 3.784426e-01
    outer loop
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
      vertex   5.334428e+00 1.128117e+01 3.058058e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal -9.999594e-03 9.063278e-01 4.224570e-01
    outer loop
      vertex   -3.000000e-01 1.035895e+01 3.251120e+01
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 3.668027e-04 9.601323e-01 2.795458e-01
    outer loop
      vertex   5.334428e+00 1.128117e+01 3.058058e+01
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal 7.085442e-03 9.525222e-01 3.043865e-01
    outer loop
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
      vertex   -3.000000e-01 1.132405e+01 3.044071e+01
    endloop
  endfacet
  facet normal -2.689375e-05 9.768324e-01 2.140056e-01
    outer loop
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
      vertex   -4.144158e-01 1.157856e+01 2.964692e+01
    endloop
  endfacet
  facet normal 2.722908e-02 9.958217e-01 8.716463e-02
    outer loop
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
    endloop
  endfacet
  facet normal 5.831326e-03 9.947809e-01 1.018669e-01
    outer loop
      vertex   -6.873124e-01 1.185498e+01 2.835997e+01
      vertex   2.676671e-01 1.183960e+01 2.845547e+01
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
    endloop
  endfacet
  facet normal 1.260363e-02 9.918110e-01 1.270909e-01
    outer loop
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   -3.000000e-01 1.192690e+01 2.763546e+01
    endloop
  endfacet
  facet normal 7.736946e-03 9.987038e-01 -5.030776e-02
    outer loop
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 8.870349e-03 9.986018e-01 -5.211290e-02
    outer loop
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
      vertex   -4.856933e-01 1.198412e+01 2.647766e+01
    endloop
  endfacet
  facet normal -1.721002e-04 9.779723e-01 -2.087345e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
    endloop
  endfacet
  facet normal -2.778626e-18 9.779498e-01 -2.088399e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   -3.000000e-01 1.189489e+01 2.491523e+01
      vertex   3.802183e-01 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal -1.262443e-03 9.789832e-01 -2.039370e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
      vertex   4.561703e+00 1.155391e+01 2.327495e+01
    endloop
  endfacet
  facet normal -1.927678e-02 9.376062e-01 -3.471644e-01
    outer loop
      vertex   4.561703e+00 1.155391e+01 2.327495e+01
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
    endloop
  endfacet
  facet normal -1.611616e-02 9.410498e-01 -3.378838e-01
    outer loop
      vertex   -3.000000e-01 1.166815e+01 2.385343e+01
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -2.939517e-03 9.088438e-01 -4.171262e-01
    outer loop
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   -3.000000e-01 1.065144e+01 2.102176e+01
    endloop
  endfacet
  facet normal -7.999785e-05 8.856257e-01 -4.643997e-01
    outer loop
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   2.510000e+01 1.066536e+01 2.100000e+01
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
    endloop
  endfacet
  facet normal -1.245994e-02 4.035275e-01 -9.148827e-01
    outer loop
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
    endloop
  endfacet
  facet normal -5.732332e-03 4.351990e-01 -9.003160e-01
    outer loop
      vertex   -3.000000e-01 5.974222e+00 1.611305e+01
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
    endloop
  endfacet
  facet normal -5.242158e-03 3.651635e-01 -9.309286e-01
    outer loop
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
      vertex   5.134619e+00 3.878275e+00 1.515079e+01
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
    endloop
  endfacet
  facet normal -2.200704e-03 2.595980e-01 -9.657143e-01
    outer loop
      vertex   5.134619e+00 3.878275e+00 1.515079e+01
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
    endloop
  endfacet
  facet normal -1.196626e-02 2.317351e-01 -9.727053e-01
    outer loop
      vertex   -3.000000e-01 3.730690e+00 1.512350e+01
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
    endloop
  endfacet
  facet normal 7.248517e-03 1.174356e-01 -9.930540e-01
    outer loop
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
    endloop
  endfacet
  facet normal -2.459835e-02 -3.097543e-03 -9.996926e-01
    outer loop
      vertex   -3.000000e-01 1.603988e+00 1.461684e+01
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
    endloop
  endfacet
  facet normal -2.486977e-02 -2.557969e-03 -9.996874e-01
    outer loop
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   3.562410e+00 -7.627524e-01 1.452840e+01
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
    endloop
  endfacet
  facet normal -5.955453e-03 -1.257247e-01 -9.920473e-01
    outer loop
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
      vertex   3.562410e+00 -7.627524e-01 1.452840e+01
      vertex   3.966796e+00 -2.181559e+00 1.470578e+01
    endloop
  endfacet
  facet normal -3.495117e-02 -2.715714e-01 -9.617835e-01
    outer loop
      vertex   3.966796e+00 -2.181559e+00 1.470578e+01
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
    endloop
  endfacet
  facet normal -1.326551e-02 -2.250775e-01 -9.742506e-01
    outer loop
      vertex   -3.000000e-01 -1.349836e+00 1.462599e+01
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
    endloop
  endfacet
  facet normal 9.952725e-05 -3.508630e-01 -9.364268e-01
    outer loop
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   6.547458e+00 -4.903864e+00 1.554979e+01
    endloop
  endfacet
  facet normal -1.301745e-02 -4.592501e-01 -8.882116e-01
    outer loop
      vertex   6.547458e+00 -4.903864e+00 1.554979e+01
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
    endloop
  endfacet
  facet normal -7.531297e-03 -4.276442e-01 -9.039158e-01
    outer loop
      vertex   -3.000000e-01 -4.193852e+00 1.528303e+01
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   -3.000000e-01 -6.140663e+00 1.620407e+01
    endloop
  endfacet
  facet normal -9.867102e-05 -4.643997e-01 -8.856257e-01
    outer loop
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   2.510000e+01 -5.500000e+00 1.583464e+01
      vertex   2.510000e+01 -5.645351e+00 1.591085e+01
    endloop
  endfacet
  facet normal 0.000000e+00 9.959217e-01 -9.022193e-02
    outer loop
      vertex   -7.054979e+00 1.198604e+01 2.592136e+01
      vertex   -6.434243e+00 1.198604e+01 2.592136e+01
      vertex   -6.434243e+00 1.189489e+01 2.491523e+01
    endloop
  endfacet
  facet normal 7.074825e-01 6.258991e-01 -3.282056e-01
    outer loop
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
      vertex   2.510000e+01 1.066536e+01 2.100000e+01
    endloop
  endfacet
  facet normal 6.970048e-01 5.770792e-01 -4.256335e-01
    outer loop
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 7.620901e-01 5.333916e-01 -3.670314e-01
    outer loop
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
      vertex   2.510000e+01 1.058915e+01 2.085465e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.747498e-01 4.808036e-01 -4.105978e-01
    outer loop
      vertex   2.510000e+01 9.054470e+00 1.862494e+01
      vertex   2.510000e+01 9.056495e+00 1.862731e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.765257e-01 4.718104e-01 -4.176156e-01
    outer loop
      vertex   2.510000e+01 8.917723e+00 1.847045e+01
      vertex   2.510000e+01 9.054470e+00 1.862494e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.765715e-01 4.716834e-01 -4.176739e-01
    outer loop
      vertex   2.510000e+01 8.916174e+00 1.846870e+01
      vertex   2.510000e+01 8.917723e+00 1.847045e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.803863e-01 4.609503e-01 -4.225186e-01
    outer loop
      vertex   2.510000e+01 8.776235e+00 1.831603e+01
      vertex   2.510000e+01 8.916174e+00 1.846870e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.804458e-01 4.608225e-01 -4.225482e-01
    outer loop
      vertex   2.510000e+01 8.775182e+00 1.831488e+01
      vertex   2.510000e+01 8.776235e+00 1.831603e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.862265e-01 4.481947e-01 -4.254051e-01
    outer loop
      vertex   2.510000e+01 8.632073e+00 1.816410e+01
      vertex   2.510000e+01 8.775182e+00 1.831488e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.862672e-01 4.481187e-01 -4.254099e-01
    outer loop
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
      vertex   2.510000e+01 8.632073e+00 1.816410e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 8.009896e-01 4.196328e-01 -4.269941e-01
    outer loop
      vertex   2.510000e+01 8.185120e+00 1.772482e+01
      vertex   2.510000e+01 8.631537e+00 1.816354e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 8.242581e-01 3.826676e-01 -4.173296e-01
    outer loop
      vertex   2.510000e+01 8.183972e+00 1.772377e+01
      vertex   2.510000e+01 8.185120e+00 1.772482e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 8.295984e-01 3.736842e-01 -4.148814e-01
    outer loop
      vertex   2.510000e+01 7.875061e+00 1.744553e+01
      vertex   2.510000e+01 8.183972e+00 1.772377e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 8.482041e-01 3.439963e-01 -4.027610e-01
    outer loop
      vertex   2.510000e+01 7.872691e+00 1.744351e+01
      vertex   2.510000e+01 7.875061e+00 1.744553e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 8.513020e-01 3.388201e-01 -4.006069e-01
    outer loop
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
      vertex   2.510000e+01 7.872691e+00 1.744351e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
    endloop
  endfacet
  facet normal 7.198003e-01 3.984673e-01 -5.684288e-01
    outer loop
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
      vertex   2.510000e+01 7.872691e+00 1.744351e+01
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 7.058521e-01 3.344050e-01 -6.244567e-01
    outer loop
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
      vertex   2.510000e+01 5.500000e+00 1.583464e+01
      vertex   2.510000e+01 5.828617e+00 1.601061e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 6.500000e+00 2.000000e+01
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 8.787991e+00 1.906765e+01
      vertex   2.550000e+01 6.500000e+00 2.000000e+01
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 4.272184e-02 -2.598052e-01 9.647156e-01
    outer loop
      vertex   9.000000e+00 7.500000e+00 2.100000e+01
      vertex   2.550000e+01 6.500000e+00 2.000000e+01
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 3.973755e-03 -2.416568e-02 9.997001e-01
    outer loop
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   9.000000e+00 7.500000e+00 2.100000e+01
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 4.786451e-03 4.234501e-03 9.999796e-01
    outer loop
      vertex   2.510000e+01 1.066536e+01 2.100000e+01
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   2.550000e+01 1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 8.672227e-03 -9.994455e-01 3.214722e-02
    outer loop
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   2.510000e+01 5.500000e+00 1.583464e+01
    endloop
  endfacet
  facet normal 2.098414e-03 -9.999961e-01 -1.856435e-03
    outer loop
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
      vertex   2.510000e+01 5.500000e+00 1.583464e+01
    endloop
  endfacet
  facet normal 4.448527e-02 -9.646412e-01 2.597852e-01
    outer loop
      vertex   2.550000e+01 6.500000e+00 2.000000e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
      vertex   2.550000e+01 5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 2.047384e-03 -6.900126e-01 7.237944e-01
    outer loop
      vertex   2.550000e+01 6.500000e+00 2.000000e+01
      vertex   9.000000e+00 7.500000e+00 2.100000e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
    endloop
  endfacet
  facet normal 7.074825e-01 -3.282056e-01 -6.258991e-01
    outer loop
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
      vertex   2.510000e+01 -5.645351e+00 1.591085e+01
      vertex   2.510000e+01 -5.500000e+00 1.583464e+01
    endloop
  endfacet
  facet normal 7.000022e-01 -4.048244e-01 -5.883146e-01
    outer loop
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   2.510000e+01 -5.645351e+00 1.591085e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 8.556756e-01 -3.361009e-01 -3.935168e-01
    outer loop
      vertex   2.510000e+01 -7.875061e+00 1.744553e+01
      vertex   2.510000e+01 -7.872691e+00 1.744351e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 8.702031e-01 -3.265525e-01 -3.689310e-01
    outer loop
      vertex   2.510000e+01 -8.029554e+00 1.758228e+01
      vertex   2.510000e+01 -7.875061e+00 1.744553e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 8.701486e-01 -3.265919e-01 -3.690247e-01
    outer loop
      vertex   2.510000e+01 -8.031304e+00 1.758383e+01
      vertex   2.510000e+01 -8.029554e+00 1.758228e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 8.080959e-01 -3.647153e-01 -4.625622e-01
    outer loop
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
      vertex   2.510000e+01 -8.031304e+00 1.758383e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 7.606302e-01 -4.386628e-01 -4.785568e-01
    outer loop
      vertex   2.510000e+01 -8.183972e+00 1.772377e+01
      vertex   2.510000e+01 -8.031304e+00 1.758383e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.605288e-01 -4.388228e-01 -4.785712e-01
    outer loop
      vertex   2.510000e+01 -8.185120e+00 1.772482e+01
      vertex   2.510000e+01 -8.183972e+00 1.772377e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.318995e-01 -4.818310e-01 -4.818318e-01
    outer loop
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
      vertex   2.510000e+01 -8.185120e+00 1.772482e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.290592e-01 -5.044761e-01 -4.625761e-01
    outer loop
      vertex   2.510000e+01 -8.776235e+00 1.831603e+01
      vertex   2.510000e+01 -8.775182e+00 1.831488e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.284023e-01 -5.090906e-01 -4.585377e-01
    outer loop
      vertex   2.510000e+01 -9.054470e+00 1.862494e+01
      vertex   2.510000e+01 -8.776235e+00 1.831603e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.350114e-01 -5.156219e-01 -4.403320e-01
    outer loop
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
      vertex   2.510000e+01 -9.054470e+00 1.862494e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.525083e-01 -5.330681e-01 -3.867424e-01
    outer loop
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
    endloop
  endfacet
  facet normal 7.198013e-01 -5.684278e-01 -3.984670e-01
    outer loop
      vertex   2.510000e+01 -1.048939e+01 2.067138e+01
      vertex   2.510000e+01 -9.056495e+00 1.862731e+01
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 7.058526e-01 -6.244572e-01 -3.344030e-01
    outer loop
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
      vertex   2.510000e+01 -1.066536e+01 2.100000e+01
      vertex   2.510000e+01 -1.048939e+01 2.067138e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
      vertex   2.550000e+01 -6.500000e+00 2.000000e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
      vertex   2.550000e+01 -8.523072e+00 1.867037e+01
      vertex   2.550000e+01 -6.500000e+00 2.000000e+01
    endloop
  endfacet
  facet normal 4.272184e-02 9.647156e-01 2.598052e-01
    outer loop
      vertex   9.000000e+00 -5.500000e+00 1.900000e+01
      vertex   2.550000e+01 -6.500000e+00 2.000000e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 4.997847e-03 9.995255e-01 3.039351e-02
    outer loop
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   9.000000e+00 -5.500000e+00 1.900000e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 6.033307e-03 9.999676e-01 -5.337575e-03
    outer loop
      vertex   2.510000e+01 -5.500000e+00 1.583464e+01
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   2.550000e+01 -5.500000e+00 1.628677e+01
    endloop
  endfacet
  facet normal 6.665089e-03 8.540160e-03 9.999413e-01
    outer loop
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   2.510000e+01 -1.066536e+01 2.100000e+01
    endloop
  endfacet
  facet normal 4.273302e-03 -3.780526e-03 9.999837e-01
    outer loop
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   2.510000e+01 -1.066536e+01 2.100000e+01
    endloop
  endfacet
  facet normal 4.719511e-02 2.597529e-01 9.645212e-01
    outer loop
      vertex   2.550000e+01 -6.500000e+00 2.000000e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   2.550000e+01 -1.021323e+01 2.100000e+01
    endloop
  endfacet
  facet normal 4.236062e-03 7.411838e-01 6.712888e-01
    outer loop
      vertex   2.550000e+01 -6.500000e+00 2.000000e+01
      vertex   9.000000e+00 -5.500000e+00 1.900000e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
    endloop
  endfacet
  facet normal 1.869676e-01 -9.823661e-01 5.407625e-05
    outer loop
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
      vertex   7.424326e+00 5.265554e+00 1.932435e+01
    endloop
  endfacet
  facet normal 3.355463e-01 -9.420196e-01 2.783392e-03
    outer loop
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
    endloop
  endfacet
  facet normal 3.984808e-01 -9.167701e-01 2.730609e-02
    outer loop
      vertex   7.424326e+00 5.265554e+00 1.932435e+01
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
      vertex   7.858228e+00 5.347940e+00 1.575837e+01
    endloop
  endfacet
  facet normal 5.986178e-01 -8.009536e-01 1.139988e-02
    outer loop
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
      vertex   5.134619e+00 3.878275e+00 1.515079e+01
    endloop
  endfacet
  facet normal 5.514098e-01 -8.342343e-01 -5.417657e-04
    outer loop
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
      vertex   6.396455e+00 4.826541e+00 1.551564e+01
    endloop
  endfacet
  facet normal 7.324990e-01 -6.807509e-01 -4.843882e-03
    outer loop
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
      vertex   5.134619e+00 3.878275e+00 1.515079e+01
    endloop
  endfacet
  facet normal 8.158019e-01 -5.782206e-01 1.132284e-02
    outer loop
      vertex   5.134619e+00 3.878275e+00 1.515079e+01
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
    endloop
  endfacet
  facet normal 9.001804e-01 -4.353261e-01 -1.289905e-02
    outer loop
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
    endloop
  endfacet
  facet normal 9.638441e-01 -2.664135e-01 5.322396e-03
    outer loop
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   4.029834e+00 2.311354e+00 1.473209e+01
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
    endloop
  endfacet
  facet normal 9.851562e-01 -1.715386e-01 -6.463999e-03
    outer loop
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
    endloop
  endfacet
  facet normal 9.813452e-01 1.908661e-01 2.305773e-02
    outer loop
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 9.997246e-01 5.201327e-03 -2.288382e-02
    outer loop
      vertex   3.562410e+00 -7.627524e-01 1.452840e+01
      vertex   3.555295e+00 5.904066e-01 1.452511e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal 9.617454e-01 2.739415e-01 -1.383685e-03
    outer loop
      vertex   3.966796e+00 -2.181559e+00 1.470578e+01
      vertex   3.562410e+00 -7.627524e-01 1.452840e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal 9.338946e-01 3.573890e-01 -1.067857e-02
    outer loop
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
      vertex   3.966796e+00 -2.181559e+00 1.470578e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal 8.272421e-01 5.617091e-01 1.238725e-02
    outer loop
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   3.966796e+00 -2.181559e+00 1.470578e+01
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
    endloop
  endfacet
  facet normal 7.749287e-01 6.320487e-01 1.854756e-04
    outer loop
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
      vertex   5.207370e+00 -3.963963e+00 2.109564e+01
    endloop
  endfacet
  facet normal 6.242300e-01 7.811389e-01 1.260951e-02
    outer loop
      vertex   5.207370e+00 -3.963963e+00 2.109564e+01
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
    endloop
  endfacet
  facet normal 6.103567e-01 7.919255e-01 1.785258e-02
    outer loop
      vertex   4.984587e+00 -3.689056e+00 1.509445e+01
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
      vertex   6.547458e+00 -4.903864e+00 1.554979e+01
    endloop
  endfacet
  facet normal 4.382323e-01 8.988547e-01 -3.562954e-03
    outer loop
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
      vertex   6.547458e+00 -4.903864e+00 1.554979e+01
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
    endloop
  endfacet
  facet normal 3.070010e-01 9.515622e-01 1.672781e-02
    outer loop
      vertex   6.547458e+00 -4.903864e+00 1.554979e+01
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
    endloop
  endfacet
  facet normal 1.840687e-01 9.827376e-01 -1.858554e-02
    outer loop
      vertex   8.934804e+00 -5.499603e+00 1.900018e+01
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
    endloop
  endfacet
  facet normal 6.171375e-03 9.995293e-01 3.005250e-02
    outer loop
      vertex   8.064790e+00 -5.397495e+00 1.578278e+01
      vertex   8.934804e+00 -5.499603e+00 1.900018e+01
      vertex   9.000000e+00 -5.500000e+00 1.900000e+01
    endloop
  endfacet
  facet normal 7.074827e-01 3.282023e-01 6.259006e-01
    outer loop
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   2.510000e+01 5.500000e+00 3.716536e+01
    endloop
  endfacet
  facet normal 6.974300e-01 4.228554e-01 5.786058e-01
    outer loop
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 7.649601e-01 3.651075e-01 5.305964e-01
    outer loop
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
      vertex   2.510000e+01 5.645351e+00 3.708915e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.400161e-01 4.370540e-01 5.112338e-01
    outer loop
      vertex   2.510000e+01 7.875061e+00 3.555447e+01
      vertex   2.510000e+01 7.872691e+00 3.555650e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.352704e-01 4.492188e-01 5.075234e-01
    outer loop
      vertex   2.510000e+01 8.029554e+00 3.541773e+01
      vertex   2.510000e+01 7.875061e+00 3.555447e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.351290e-01 4.499098e-01 5.071158e-01
    outer loop
      vertex   2.510000e+01 8.031304e+00 3.541617e+01
      vertex   2.510000e+01 8.029554e+00 3.541773e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.331263e-01 4.595438e-01 5.013435e-01
    outer loop
      vertex   2.510000e+01 8.183972e+00 3.527623e+01
      vertex   2.510000e+01 8.031304e+00 3.541617e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.331326e-01 4.596261e-01 5.012589e-01
    outer loop
      vertex   2.510000e+01 8.185120e+00 3.527518e+01
      vertex   2.510000e+01 8.183972e+00 3.527623e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.337233e-01 4.677485e-01 4.928098e-01
    outer loop
      vertex   2.510000e+01 8.335896e+00 3.513207e+01
      vertex   2.510000e+01 8.185120e+00 3.527518e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.339223e-01 4.681052e-01 4.921744e-01
    outer loop
      vertex   2.510000e+01 8.336461e+00 3.513154e+01
      vertex   2.510000e+01 8.335896e+00 3.513207e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.417197e-01 4.824377e-01 4.659461e-01
    outer loop
      vertex   2.510000e+01 8.916174e+00 3.453130e+01
      vertex   2.510000e+01 8.336461e+00 3.513154e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.737266e-01 4.745238e-01 4.197312e-01
    outer loop
      vertex   2.510000e+01 8.917723e+00 3.452955e+01
      vertex   2.510000e+01 8.916174e+00 3.453130e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.734976e-01 4.745886e-01 4.200799e-01
    outer loop
      vertex   2.510000e+01 9.054470e+00 3.437506e+01
      vertex   2.510000e+01 8.917723e+00 3.452955e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.867479e-01 4.692426e-01 4.010475e-01
    outer loop
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
      vertex   2.510000e+01 9.054470e+00 3.437506e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 8.134165e-01 4.569670e-01 3.599095e-01
    outer loop
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 7.198012e-01 5.684281e-01 3.984667e-01
    outer loop
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
      vertex   2.510000e+01 9.056495e+00 3.437269e+01
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 7.058526e-01 6.244572e-01 3.344030e-01
    outer loop
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
      vertex   2.510000e+01 1.066536e+01 3.200000e+01
      vertex   2.510000e+01 1.048939e+01 3.232862e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
      vertex   2.550000e+01 6.500000e+00 3.300000e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
      vertex   2.550000e+01 7.858755e+00 3.498941e+01
      vertex   2.550000e+01 6.500000e+00 3.300000e+01
    endloop
  endfacet
  facet normal 6.456210e-03 -1.482630e-02 -9.998692e-01
    outer loop
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
      vertex   2.510000e+01 1.066536e+01 3.200000e+01
    endloop
  endfacet
  facet normal 3.057859e-03 2.705241e-03 -9.999917e-01
    outer loop
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
      vertex   2.510000e+01 1.066536e+01 3.200000e+01
    endloop
  endfacet
  facet normal 4.571440e-02 -2.597708e-01 -9.645877e-01
    outer loop
      vertex   2.550000e+01 6.500000e+00 3.300000e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
      vertex   2.550000e+01 1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 4.272184e-02 -9.647156e-01 -2.598052e-01
    outer loop
      vertex   9.000000e+00 5.500000e+00 3.400000e+01
      vertex   2.550000e+01 6.500000e+00 3.300000e+01
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 7.121020e-03 -9.990365e-01 -4.330521e-02
    outer loop
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   9.000000e+00 5.500000e+00 3.400000e+01
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 8.657921e-03 -9.999332e-01 7.659548e-03
    outer loop
      vertex   2.510000e+01 5.500000e+00 3.716536e+01
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   2.550000e+01 5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 3.010900e-03 -7.315071e-01 -6.818272e-01
    outer loop
      vertex   2.550000e+01 6.500000e+00 3.300000e+01
      vertex   9.000000e+00 5.500000e+00 3.400000e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
    endloop
  endfacet
  facet normal 2.562020e-01 6.553531e-03 -9.666011e-01
    outer loop
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
    endloop
  endfacet
  facet normal 2.846845e-01 -3.523609e-03 -9.586148e-01
    outer loop
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
      vertex   8.070495e+00 1.072120e+01 3.188921e+01
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
    endloop
  endfacet
  facet normal 5.322628e-01 5.129807e-03 -8.465636e-01
    outer loop
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
    endloop
  endfacet
  facet normal 5.530714e-01 -1.863158e-03 -8.331318e-01
    outer loop
      vertex   5.334428e+00 1.128117e+01 3.058058e+01
      vertex   6.682450e+00 1.091839e+01 3.147627e+01
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
    endloop
  endfacet
  facet normal 7.819390e-01 5.846678e-03 -6.233276e-01
    outer loop
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   5.334428e+00 1.128117e+01 3.058058e+01
    endloop
  endfacet
  facet normal 7.842663e-01 6.964027e-03 -6.203852e-01
    outer loop
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
    endloop
  endfacet
  facet normal 9.539514e-01 -1.736694e-02 -2.994581e-01
    outer loop
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   4.307649e+00 1.165556e+01 2.929604e+01
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 9.689767e-01 -3.456887e-03 -2.471278e-01
    outer loop
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 9.980125e-01 -1.050487e-03 6.300709e-02
    outer loop
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
    endloop
  endfacet
  facet normal 9.931863e-01 -1.243792e-02 1.158719e-01
    outer loop
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
      vertex   3.564451e+00 1.197072e+01 2.691024e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 9.462671e-01 1.200069e-03 3.233839e-01
    outer loop
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 8.911097e-01 -1.069764e-02 4.536619e-01
    outer loop
      vertex   4.561703e+00 1.155391e+01 2.327495e+01
      vertex   3.815287e+00 1.185987e+01 2.474832e+01
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 8.217604e-01 7.563424e-03 5.697830e-01
    outer loop
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
      vertex   4.561703e+00 1.155391e+01 2.327495e+01
    endloop
  endfacet
  facet normal 6.714773e-01 -1.554168e-02 7.408621e-01
    outer loop
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   4.561703e+00 1.155391e+01 2.327495e+01
    endloop
  endfacet
  facet normal 6.521706e-01 -9.576873e-03 7.580117e-01
    outer loop
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
      vertex   5.916384e+00 6.175006e+00 2.196857e+01
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
    endloop
  endfacet
  facet normal 4.368841e-01 -1.427968e-03 8.995167e-01
    outer loop
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   5.916384e+00 6.175006e+00 2.196857e+01
    endloop
  endfacet
  facet normal 3.471419e-01 -3.666783e-02 9.370955e-01
    outer loop
      vertex   6.060936e+00 1.107791e+01 2.190614e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
    endloop
  endfacet
  facet normal 1.692144e-01 1.066470e-02 9.855216e-01
    outer loop
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   8.917717e+00 7.499607e+00 2.100062e+01
    endloop
  endfacet
  facet normal 7.596350e-03 -2.331878e-02 9.996992e-01
    outer loop
      vertex   8.917717e+00 7.499607e+00 2.100062e+01
      vertex   9.000000e+00 7.500000e+00 2.100000e+01
      vertex   8.250505e+00 1.070604e+01 2.108048e+01
    endloop
  endfacet
  facet normal 1.980672e-01 9.735095e-01 1.142307e-01
    outer loop
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
    endloop
  endfacet
  facet normal 4.314694e-01 9.021260e-01 -1.708278e-03
    outer loop
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
    endloop
  endfacet
  facet normal 5.612182e-01 7.627622e-01 3.212911e-01
    outer loop
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
    endloop
  endfacet
  facet normal 7.544571e-01 5.937713e-01 2.796966e-01
    outer loop
      vertex   -3.828830e+00 7.448618e+00 3.590168e+01
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
    endloop
  endfacet
  facet normal 7.327737e-01 5.458449e-01 4.063202e-01
    outer loop
      vertex   -3.722427e+00 7.852616e+00 3.475738e+01
      vertex   -3.730857e+00 7.887670e+00 3.472549e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
    endloop
  endfacet
  facet normal 7.326336e-01 5.455838e-01 4.069230e-01
    outer loop
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -3.722427e+00 7.852616e+00 3.475738e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
    endloop
  endfacet
  facet normal 7.165218e-01 5.112811e-01 4.745400e-01
    outer loop
      vertex   -3.730857e+00 7.887670e+00 3.472549e+01
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
    endloop
  endfacet
  facet normal 5.425482e-01 2.103412e-01 8.132638e-01
    outer loop
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -3.439982e+00 9.673910e+00 3.360027e+01
      vertex   -3.890631e+00 9.384056e+00 3.397588e+01
    endloop
  endfacet
  facet normal 6.656569e-01 3.500176e-01 6.590817e-01
    outer loop
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -3.890631e+00 9.384056e+00 3.397588e+01
    endloop
  endfacet
  facet normal 6.992073e-01 3.409803e-01 6.283642e-01
    outer loop
      vertex   -4.328816e+00 8.547566e+00 3.491738e+01
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -3.890631e+00 9.384056e+00 3.397588e+01
    endloop
  endfacet
  facet normal 4.103573e-01 2.126316e-01 8.867890e-01
    outer loop
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
      vertex   -3.439982e+00 9.673910e+00 3.360027e+01
    endloop
  endfacet
  facet normal 2.939242e-01 8.751393e-02 9.518140e-01
    outer loop
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
      vertex   -3.439982e+00 9.673910e+00 3.360027e+01
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
    endloop
  endfacet
  facet normal 1.074871e-01 1.206145e-01 9.868631e-01
    outer loop
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
    endloop
  endfacet
  facet normal -7.118347e-02 2.593550e-02 9.971260e-01
    outer loop
      vertex   -2.585710e+00 9.874827e+00 3.331799e+01
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -1.559329e+00 9.822585e+00 3.339262e+01
    endloop
  endfacet
  facet normal -6.674169e-01 2.449291e-01 7.032527e-01
    outer loop
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
      vertex   -7.787513e-01 8.980164e+00 3.397298e+01
    endloop
  endfacet
  facet normal -2.499148e-01 1.423123e-01 9.577525e-01
    outer loop
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -1.559329e+00 9.822585e+00 3.339262e+01
    endloop
  endfacet
  facet normal -5.019406e-01 2.249912e-01 8.351255e-01
    outer loop
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
    endloop
  endfacet
  facet normal -4.466123e-01 1.216987e-01 8.864124e-01
    outer loop
      vertex   -1.559329e+00 9.822585e+00 3.339262e+01
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
    endloop
  endfacet
  facet normal -5.631843e-01 2.476605e-01 7.883449e-01
    outer loop
      vertex   -7.787513e-01 8.980164e+00 3.397298e+01
      vertex   -7.077317e-01 9.468475e+00 3.387031e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
    endloop
  endfacet
  facet normal -6.521517e-01 3.373614e-01 6.788855e-01
    outer loop
      vertex   -7.787513e-01 8.980164e+00 3.397298e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
    endloop
  endfacet
  facet normal -6.550899e-01 3.692511e-01 6.591743e-01
    outer loop
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -7.660639e-01 8.211998e+00 3.440237e+01
    endloop
  endfacet
  facet normal -7.688562e-01 4.336373e-01 4.699136e-01
    outer loop
      vertex   -7.597231e-01 8.179057e+00 3.443792e+01
      vertex   -7.503372e-01 8.041020e+00 3.458065e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -7.663383e-01 4.363108e-01 4.715489e-01
    outer loop
      vertex   -3.002942e-01 9.017429e+00 3.441408e+01
      vertex   -7.660639e-01 8.211998e+00 3.440237e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -8.039265e-01 3.578290e-01 4.750374e-01
    outer loop
      vertex   -7.647290e-01 8.205063e+00 3.440986e+01
      vertex   -7.597231e-01 8.179057e+00 3.443792e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -8.040069e-01 3.576420e-01 4.750423e-01
    outer loop
      vertex   -7.660639e-01 8.211998e+00 3.440237e+01
      vertex   -7.647290e-01 8.205063e+00 3.440986e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -7.447133e-01 4.857640e-01 4.576412e-01
    outer loop
      vertex   -7.511032e-01 8.023731e+00 3.459776e+01
      vertex   -7.517099e-01 8.010039e+00 3.461130e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -7.447405e-01 4.857078e-01 4.576566e-01
    outer loop
      vertex   -7.503372e-01 8.041020e+00 3.458065e+01
      vertex   -7.511032e-01 8.023731e+00 3.459776e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -7.364239e-01 5.028381e-01 4.525855e-01
    outer loop
      vertex   -7.536000e-01 7.988423e+00 3.463224e+01
      vertex   -7.545248e-01 7.977847e+00 3.464249e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -7.364098e-01 5.028674e-01 4.525759e-01
    outer loop
      vertex   -7.517099e-01 8.010039e+00 3.461130e+01
      vertex   -7.536000e-01 7.988423e+00 3.463224e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
    endloop
  endfacet
  facet normal -5.805585e-01 8.133971e-01 3.656592e-02
    outer loop
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
      vertex   -1.143075e+00 7.150934e+00 3.571984e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
    endloop
  endfacet
  facet normal -5.823101e-01 7.706902e-01 2.587502e-01
    outer loop
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -1.143075e+00 7.150934e+00 3.571984e+01
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
    endloop
  endfacet
  facet normal -5.995451e-01 7.165378e-01 3.565379e-01
    outer loop
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
      vertex   -2.455305e-01 8.008673e+00 3.543645e+01
      vertex   -7.545248e-01 7.977847e+00 3.464249e+01
    endloop
  endfacet
  facet normal -6.116053e-01 7.069103e-01 3.552700e-01
    outer loop
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -5.509497e-01 7.569733e+00 3.580501e+01
      vertex   -7.545248e-01 7.977847e+00 3.464249e+01
    endloop
  endfacet
  facet normal -3.481366e-01 9.233321e-01 1.620452e-01
    outer loop
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -1.143075e+00 7.150934e+00 3.571984e+01
    endloop
  endfacet
  facet normal -2.556657e-01 9.516189e-01 1.704597e-01
    outer loop
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
    endloop
  endfacet
  facet normal -1.164852e-01 9.931604e-01 7.973784e-03
    outer loop
      vertex   -1.353515e+00 6.976334e+00 3.626260e+01
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -2.545553e+00 6.835723e+00 3.636228e+01
    endloop
  endfacet
  facet normal 3.089095e-01 -6.672383e-01 6.777669e-01
    outer loop
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
      vertex   -3.115212e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.314370e-01 -7.045204e-01 6.974062e-01
    outer loop
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -7.085681e-01 7.056424e-01
    outer loop
      vertex   -2.236910e+00 4.100000e+00 2.847876e+01
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal -3.092565e-01 -6.697000e-01 6.751758e-01
    outer loop
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -2.106421e+00 8.976593e+00 3.337557e+01
      vertex   -2.236910e+00 4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal -3.078260e-01 -6.701432e-01 6.753897e-01
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -2.236910e+00 4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal -6.624536e-01 -5.334148e-01 5.259505e-01
    outer loop
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -1.515234e+00 8.874423e+00 3.354502e+01
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -8.179804e-01 -3.983414e-01 4.150088e-01
    outer loop
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -9.226186e-01 -2.749887e-01 2.704738e-01
    outer loop
      vertex   -7.660639e-01 8.211998e+00 3.440237e+01
      vertex   -1.038710e+00 8.629962e+00 3.389729e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
    endloop
  endfacet
  facet normal -9.915450e-01 -8.792951e-02 9.543010e-02
    outer loop
      vertex   -7.647290e-01 8.205063e+00 3.440986e+01
      vertex   -7.660639e-01 8.211998e+00 3.440237e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
    endloop
  endfacet
  facet normal -9.915482e-01 -8.791151e-02 9.541326e-02
    outer loop
      vertex   -7.597231e-01 8.179057e+00 3.443792e+01
      vertex   -7.647290e-01 8.205063e+00 3.440986e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
    endloop
  endfacet
  facet normal -9.988552e-01 -2.729466e-02 3.928480e-02
    outer loop
      vertex   -7.503372e-01 8.041020e+00 3.458065e+01
      vertex   -7.597231e-01 8.179057e+00 3.443792e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
    endloop
  endfacet
  facet normal -9.983016e-01 5.054731e-02 -2.896409e-02
    outer loop
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
      vertex   -7.503372e-01 8.041020e+00 3.458065e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
    endloop
  endfacet
  facet normal -9.993607e-01 3.429866e-02 -1.009204e-02
    outer loop
      vertex   -7.511032e-01 8.023731e+00 3.459776e+01
      vertex   -7.503372e-01 8.041020e+00 3.458065e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
    endloop
  endfacet
  facet normal -9.993607e-01 3.429891e-02 -1.009232e-02
    outer loop
      vertex   -7.517099e-01 8.010039e+00 3.461130e+01
      vertex   -7.511032e-01 8.023731e+00 3.459776e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
    endloop
  endfacet
  facet normal -9.979364e-01 5.474319e-02 -3.355709e-02
    outer loop
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
      vertex   -7.536000e-01 7.988423e+00 3.463224e+01
      vertex   -7.517099e-01 8.010039e+00 3.461130e+01
    endloop
  endfacet
  facet normal -9.979361e-01 5.474670e-02 -3.356108e-02
    outer loop
      vertex   -7.545248e-01 7.977847e+00 3.464249e+01
      vertex   -7.536000e-01 7.988423e+00 3.463224e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
    endloop
  endfacet
  facet normal -8.781968e-01 3.289207e-01 -3.472487e-01
    outer loop
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -7.545248e-01 7.977847e+00 3.464249e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
    endloop
  endfacet
  facet normal -7.854530e-01 4.405870e-01 -4.346800e-01
    outer loop
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
    endloop
  endfacet
  facet normal -3.296264e-01 6.639009e-01 -6.712541e-01
    outer loop
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -1.321276e+00 7.157623e+00 3.529888e+01
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
    endloop
  endfacet
  facet normal -2.684700e-01 6.828116e-01 -6.794794e-01
    outer loop
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
    endloop
  endfacet
  facet normal 2.399701e-01 6.857535e-01 -6.871365e-01
    outer loop
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
      vertex   -2.243296e+00 6.878295e+00 3.547538e+01
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
    endloop
  endfacet
  facet normal 2.590296e-01 6.836007e-01 -6.823443e-01
    outer loop
      vertex   -3.006243e+00 1.847755e+00 3.016747e+01
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
    endloop
  endfacet
  facet normal 8.040676e-01 4.186435e-01 -4.221527e-01
    outer loop
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -2.971688e+00 7.037675e+00 3.538006e+01
      vertex   -3.006243e+00 1.847755e+00 3.016747e+01
    endloop
  endfacet
  facet normal 7.871011e-01 4.311311e-01 -4.411325e-01
    outer loop
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -3.006243e+00 1.847755e+00 3.016747e+01
    endloop
  endfacet
  facet normal 9.845394e-01 1.311748e-01 -1.160837e-01
    outer loop
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
      vertex   -3.730857e+00 7.887670e+00 3.472549e+01
      vertex   -3.722427e+00 7.852616e+00 3.475738e+01
    endloop
  endfacet
  facet normal 9.845317e-01 1.312041e-01 -1.161153e-01
    outer loop
      vertex   -3.722272e+00 7.851974e+00 3.475797e+01
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
      vertex   -3.722427e+00 7.852616e+00 3.475738e+01
    endloop
  endfacet
  facet normal 9.984768e-01 4.850904e-02 -2.628686e-02
    outer loop
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -3.730857e+00 7.887670e+00 3.472549e+01
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
    endloop
  endfacet
  facet normal 9.992523e-01 -6.081763e-03 3.818243e-02
    outer loop
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
    endloop
  endfacet
  facet normal 9.177635e-01 -2.878841e-01 2.735558e-01
    outer loop
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -3.747755e+00 8.116003e+00 3.450500e+01
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
    endloop
  endfacet
  facet normal 7.748350e-01 -4.428621e-01 4.511141e-01
    outer loop
      vertex   -3.115212e+00 4.100000e+00 2.886714e+01
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
    endloop
  endfacet
  facet normal 5.437346e-01 -5.999054e-01 5.869124e-01
    outer loop
      vertex   -2.728378e+00 8.932903e+00 3.344865e+01
      vertex   -3.350501e+00 8.704306e+00 3.379135e+01
      vertex   -3.115212e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -1.295016e-01 6.326060e-02 -9.895592e-01
    outer loop
      vertex   -1.694859e+00 8.924407e+00 1.953341e+01
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
    endloop
  endfacet
  facet normal -4.409220e-01 1.324068e-01 -8.877253e-01
    outer loop
      vertex   -1.694859e+00 8.924407e+00 1.953341e+01
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
    endloop
  endfacet
  facet normal -4.969826e-01 1.093738e-01 -8.608401e-01
    outer loop
      vertex   -5.694066e-01 9.340868e+00 1.897195e+01
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
    endloop
  endfacet
  facet normal -6.341707e-01 3.386794e-01 -6.950711e-01
    outer loop
      vertex   -7.681940e-01 8.221496e+00 1.860790e+01
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
      vertex   -5.694066e-01 9.340868e+00 1.897195e+01
    endloop
  endfacet
  facet normal -7.296697e-01 3.252115e-01 -6.015145e-01
    outer loop
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   -7.681940e-01 8.221496e+00 1.860790e+01
      vertex   -5.694066e-01 9.340868e+00 1.897195e+01
    endloop
  endfacet
  facet normal -7.148184e-01 4.397018e-01 -5.437802e-01
    outer loop
      vertex   -7.624360e-01 8.193689e+00 1.857784e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   -7.614633e-01 8.188992e+00 1.857277e+01
    endloop
  endfacet
  facet normal -7.148096e-01 4.397584e-01 -5.437461e-01
    outer loop
      vertex   -7.681940e-01 8.221496e+00 1.860790e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   -7.624360e-01 8.193689e+00 1.857784e+01
    endloop
  endfacet
  facet normal -7.066692e-01 4.858065e-01 -5.144033e-01
    outer loop
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -7.614633e-01 8.188992e+00 1.857277e+01
    endloop
  endfacet
  facet normal -7.070321e-01 5.012972e-01 -4.988053e-01
    outer loop
      vertex   -1.504616e-01 8.451730e+00 1.798130e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -1.504450e-01 8.455477e+00 1.798504e+01
    endloop
  endfacet
  facet normal -7.069906e-01 5.142455e-01 -4.855058e-01
    outer loop
      vertex   -1.522146e-01 8.416840e+00 1.794690e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -1.504616e-01 8.451730e+00 1.798130e+01
    endloop
  endfacet
  facet normal -7.069907e-01 5.142434e-01 -4.855079e-01
    outer loop
      vertex   -1.522146e-01 8.416840e+00 1.794690e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
    endloop
  endfacet
  facet normal -6.818914e-01 6.408191e-01 -3.526683e-01
    outer loop
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
    endloop
  endfacet
  facet normal -6.819283e-01 6.408405e-01 -3.525581e-01
    outer loop
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
      vertex   -8.581301e-01 7.387325e+00 1.745404e+01
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
    endloop
  endfacet
  facet normal -8.452273e-01 4.983345e-01 1.930116e-01
    outer loop
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
      vertex   -8.581301e-01 7.387325e+00 1.745404e+01
      vertex   -1.531095e-01 8.399028e+00 1.792933e+01
    endloop
  endfacet
  facet normal -4.780218e-01 8.599381e-01 -1.788901e-01
    outer loop
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
      vertex   -8.581301e-01 7.387325e+00 1.745404e+01
    endloop
  endfacet
  facet normal -4.522996e-01 8.725590e-01 -1.845695e-01
    outer loop
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
      vertex   -1.685253e+00 6.975463e+00 1.758361e+01
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
    endloop
  endfacet
  facet normal -1.360554e-01 9.907012e-01 1.999583e-04
    outer loop
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
      vertex   -2.409241e+00 6.876049e+00 1.752370e+01
      vertex   -1.685253e+00 6.975463e+00 1.758361e+01
    endloop
  endfacet
  facet normal -1.504500e-01 9.884994e-01 1.528688e-02
    outer loop
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
      vertex   -1.685253e+00 6.975463e+00 1.758361e+01
      vertex   -1.199554e+00 7.061504e+00 1.680013e+01
    endloop
  endfacet
  facet normal 3.551869e-01 9.263004e-01 -1.257373e-01
    outer loop
      vertex   -3.666887e+00 7.275447e+00 1.695933e+01
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
    endloop
  endfacet
  facet normal 3.478273e-01 9.284530e-01 -1.303501e-01
    outer loop
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -2.409241e+00 6.876049e+00 1.752370e+01
      vertex   -2.651193e+00 6.842999e+00 1.664267e+01
    endloop
  endfacet
  facet normal 5.240013e-01 8.206099e-01 -2.280834e-01
    outer loop
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -3.666887e+00 7.275447e+00 1.695933e+01
      vertex   -3.691982e+00 7.440222e+00 1.749451e+01
    endloop
  endfacet
  facet normal 8.763987e-01 4.702886e-01 -1.037009e-01
    outer loop
      vertex   -3.666887e+00 7.275447e+00 1.695933e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
      vertex   -3.691982e+00 7.440222e+00 1.749451e+01
    endloop
  endfacet
  facet normal 5.853130e-01 6.040696e-01 -5.408407e-01
    outer loop
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -3.691982e+00 7.440222e+00 1.749451e+01
    endloop
  endfacet
  facet normal 8.349894e-01 5.412618e-01 9.913771e-02
    outer loop
      vertex   -3.730883e+00 7.887818e+00 1.827458e+01
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal 7.328116e-01 4.977718e-01 -4.639078e-01
    outer loop
      vertex   -3.749224e+00 8.094833e+00 1.847341e+01
      vertex   -3.737802e+00 7.922973e+00 1.830705e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal 7.599670e-01 5.153220e-01 -3.960976e-01
    outer loop
      vertex   -3.737471e+00 7.921290e+00 1.830549e+01
      vertex   -3.730883e+00 7.887818e+00 1.827458e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal 6.733707e-01 3.919380e-01 -6.268624e-01
    outer loop
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
      vertex   -4.311689e+00 8.776243e+00 1.831605e+01
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
    endloop
  endfacet
  facet normal 6.716283e-01 4.122292e-01 -6.156156e-01
    outer loop
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
      vertex   -3.749224e+00 8.094833e+00 1.847341e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal 5.447441e-01 2.807639e-01 -7.902060e-01
    outer loop
      vertex   -3.700586e+00 9.536608e+00 1.921672e+01
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
    endloop
  endfacet
  facet normal 6.762295e-01 2.689248e-01 -6.858521e-01
    outer loop
      vertex   -4.311689e+00 8.776243e+00 1.831605e+01
      vertex   -3.700586e+00 9.536608e+00 1.921672e+01
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
    endloop
  endfacet
  facet normal 2.739978e-01 1.429947e-01 -9.510403e-01
    outer loop
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
    endloop
  endfacet
  facet normal 3.943232e-01 1.165637e-01 -9.115493e-01
    outer loop
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -3.700586e+00 9.536608e+00 1.921672e+01
    endloop
  endfacet
  facet normal -1.719622e-02 -5.732960e-02 -9.982072e-01
    outer loop
      vertex   -1.493224e+00 9.789907e+00 1.956235e+01
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -2.926686e+00 9.806910e+00 1.958607e+01
    endloop
  endfacet
  facet normal 7.600149e-01 5.153526e-01 -3.959659e-01
    outer loop
      vertex   -3.737802e+00 7.922973e+00 1.830705e+01
      vertex   -3.737471e+00 7.921290e+00 1.830549e+01
      vertex   -4.311694e+00 8.776227e+00 1.831604e+01
    endloop
  endfacet
  facet normal 7.523589e-01 -4.688918e-01 -4.627058e-01
    outer loop
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -3.115212e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 8.066592e-01 -4.144095e-01 -4.213854e-01
    outer loop
      vertex   -3.652778e+00 3.440679e+00 2.375221e+01
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
      vertex   -3.115212e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 9.600970e-01 -2.008423e-01 -1.946178e-01
    outer loop
      vertex   -3.749224e+00 8.094833e+00 1.847341e+01
      vertex   -3.571498e+00 8.496142e+00 1.893603e+01
      vertex   -3.652778e+00 3.440679e+00 2.375221e+01
    endloop
  endfacet
  facet normal 9.967304e-01 -5.070076e-02 -6.291193e-02
    outer loop
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
      vertex   -3.749224e+00 8.094833e+00 1.847341e+01
      vertex   -3.652778e+00 3.440679e+00 2.375221e+01
    endloop
  endfacet
  facet normal 9.988583e-01 3.612391e-02 3.125937e-02
    outer loop
      vertex   -3.737802e+00 7.922973e+00 1.830705e+01
      vertex   -3.749224e+00 8.094833e+00 1.847341e+01
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
    endloop
  endfacet
  facet normal 9.897081e-01 1.030672e-01 9.927245e-02
    outer loop
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
      vertex   -3.737471e+00 7.921290e+00 1.830549e+01
      vertex   -3.737802e+00 7.922973e+00 1.830705e+01
    endloop
  endfacet
  facet normal 9.897008e-01 1.031029e-01 9.930869e-02
    outer loop
      vertex   -3.730883e+00 7.887818e+00 1.827458e+01
      vertex   -3.737471e+00 7.921290e+00 1.830549e+01
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
    endloop
  endfacet
  facet normal 8.735481e-01 3.456082e-01 3.427372e-01
    outer loop
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
      vertex   -3.730883e+00 7.887818e+00 1.827458e+01
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
    endloop
  endfacet
  facet normal 8.628328e-01 3.568644e-01 3.580046e-01
    outer loop
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -3.730883e+00 7.887818e+00 1.827458e+01
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
    endloop
  endfacet
  facet normal 4.734143e-01 6.229634e-01 6.227323e-01
    outer loop
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
    endloop
  endfacet
  facet normal 4.310600e-01 6.346039e-01 6.414555e-01
    outer loop
      vertex   -2.409241e+00 6.876049e+00 1.752370e+01
      vertex   -3.231064e+00 7.214461e+00 1.774117e+01
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
    endloop
  endfacet
  facet normal -1.265961e-01 7.051266e-01 6.976890e-01
    outer loop
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
      vertex   -2.409241e+00 6.876049e+00 1.752370e+01
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
    endloop
  endfacet
  facet normal -1.538513e-01 7.004200e-01 6.969516e-01
    outer loop
      vertex   -1.685253e+00 6.975463e+00 1.758361e+01
      vertex   -2.409241e+00 6.876049e+00 1.752370e+01
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
    endloop
  endfacet
  facet normal -5.817812e-01 5.760329e-01 5.742097e-01
    outer loop
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
      vertex   -1.685253e+00 6.975463e+00 1.758361e+01
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
    endloop
  endfacet
  facet normal -7.571080e-01 4.780830e-01 4.452236e-01
    outer loop
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
    endloop
  endfacet
  facet normal -9.310924e-01 2.552448e-01 2.606089e-01
    outer loop
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -1.104424e+00 7.326528e+00 1.781992e+01
    endloop
  endfacet
  facet normal -9.988862e-01 -1.792428e-02 -4.364678e-02
    outer loop
      vertex   -7.614633e-01 8.188992e+00 1.857277e+01
      vertex   -7.509740e-01 8.022473e+00 1.840109e+01
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
    endloop
  endfacet
  facet normal -9.986577e-01 -2.104018e-02 -4.732994e-02
    outer loop
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
      vertex   -7.614633e-01 8.188992e+00 1.857277e+01
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
    endloop
  endfacet
  facet normal -9.902292e-01 -8.911598e-02 -1.072593e-01
    outer loop
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
      vertex   -7.624360e-01 8.193689e+00 1.857784e+01
      vertex   -7.614633e-01 8.188992e+00 1.857277e+01
    endloop
  endfacet
  facet normal -9.902302e-01 -8.911046e-02 -1.072544e-01
    outer loop
      vertex   -7.681940e-01 8.221496e+00 1.860790e+01
      vertex   -7.624360e-01 8.193689e+00 1.857784e+01
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
    endloop
  endfacet
  facet normal -8.849576e-01 -3.343659e-01 -3.241135e-01
    outer loop
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
      vertex   -7.681940e-01 8.221496e+00 1.860790e+01
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
    endloop
  endfacet
  facet normal -7.895774e-01 -4.292407e-01 -4.385430e-01
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
    endloop
  endfacet
  facet normal -5.424131e-01 -5.977047e-01 -5.903704e-01
    outer loop
      vertex   -1.694859e+00 8.924407e+00 1.953341e+01
      vertex   -1.207500e+00 8.747316e+00 1.926493e+01
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -3.050391e-01 -6.673419e-01 -6.794159e-01
    outer loop
      vertex   -2.250000e+00 4.100000e+00 2.452132e+01
      vertex   -1.694859e+00 8.924407e+00 1.953341e+01
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -1.496275e-01 -7.023240e-01 -6.959544e-01
    outer loop
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -1.694859e+00 8.924407e+00 1.953341e+01
      vertex   -2.250000e+00 4.100000e+00 2.452132e+01
    endloop
  endfacet
  facet normal 4.318535e-03 -7.085412e-01 -7.056563e-01
    outer loop
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -2.250000e+00 4.100000e+00 2.452132e+01
    endloop
  endfacet
  facet normal 3.438922e-01 -6.619028e-01 -6.660502e-01
    outer loop
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -2.361688e+00 8.976686e+00 1.962401e+01
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal 3.061627e-01 -6.745589e-01 -6.717401e-01
    outer loop
      vertex   -3.115212e+00 4.100000e+00 2.413286e+01
      vertex   -3.078105e+00 8.833786e+00 1.939613e+01
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal 8.733304e-01 -3.183071e-01 3.687473e-01
    outer loop
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
    endloop
  endfacet
  facet normal 8.775139e-01 -3.162161e-01 3.605229e-01
    outer loop
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
    endloop
  endfacet
  facet normal 8.173195e-01 -4.235142e-01 3.906720e-01
    outer loop
      vertex   5.916384e+00 6.175006e+00 2.196857e+01
      vertex   4.974697e+00 5.092444e+00 2.276509e+01
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
    endloop
  endfacet
  facet normal 7.415867e-01 -4.656609e-01 4.829172e-01
    outer loop
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
      vertex   5.916384e+00 6.175006e+00 2.196857e+01
      vertex   5.497400e+00 4.228896e+00 2.073541e+01
    endloop
  endfacet
  facet normal 6.315963e-01 -5.727766e-01 5.225065e-01
    outer loop
      vertex   5.916384e+00 6.175006e+00 2.196857e+01
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
    endloop
  endfacet
  facet normal 5.727110e-01 -5.826277e-01 5.766689e-01
    outer loop
      vertex   7.424326e+00 5.265554e+00 1.932435e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   6.448088e+00 4.857833e+00 1.988196e+01
    endloop
  endfacet
  facet normal 3.051219e-01 -6.838292e-01 6.627806e-01
    outer loop
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   7.424326e+00 5.265554e+00 1.932435e+01
    endloop
  endfacet
  facet normal 2.827105e-01 -6.955388e-01 6.605305e-01
    outer loop
      vertex   8.917717e+00 7.499607e+00 2.100062e+01
      vertex   7.349678e+00 7.121822e+00 2.127394e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
    endloop
  endfacet
  facet normal 8.711565e-03 -6.909436e-01 7.228561e-01
    outer loop
      vertex   9.000000e+00 7.500000e+00 2.100000e+01
      vertex   8.917717e+00 7.499607e+00 2.100062e+01
      vertex   8.441137e+00 5.459063e+00 1.905590e+01
    endloop
  endfacet
  facet normal 8.822925e-01 -2.994742e-01 3.631461e-01
    outer loop
      vertex   4.581199e+00 3.233393e+00 2.209229e+01
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
    endloop
  endfacet
  facet normal 8.617029e-01 -3.444658e-01 -3.725740e-01
    outer loop
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 8.604065e-01 -3.444900e-01 -3.755359e-01
    outer loop
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
    endloop
  endfacet
  facet normal 7.285363e-01 -5.058423e-01 -4.619073e-01
    outer loop
      vertex   5.444608e+00 5.679151e+00 3.066625e+01
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
    endloop
  endfacet
  facet normal 6.436884e-01 -5.131394e-01 -5.677616e-01
    outer loop
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
    endloop
  endfacet
  facet normal 4.116360e-01 -6.634492e-01 -6.248128e-01
    outer loop
      vertex   6.801887e+00 6.848248e+00 3.152670e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
    endloop
  endfacet
  facet normal 2.851269e-01 -6.595399e-01 -6.954924e-01
    outer loop
      vertex   8.934804e+00 5.499603e+00 3.399983e+01
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
    endloop
  endfacet
  facet normal 6.290076e-03 -7.309765e-01 -6.823736e-01
    outer loop
      vertex   9.000000e+00 5.500000e+00 3.400000e+01
      vertex   8.934804e+00 5.499603e+00 3.399983e+01
      vertex   8.346556e+00 7.417422e+00 3.193998e+01
    endloop
  endfacet
  facet normal 8.868890e-01 -2.894597e-01 -3.600569e-01
    outer loop
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
    endloop
  endfacet
  facet normal -1.361146e-07 -5.000003e-01 8.660252e-01
    outer loop
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 8.516619e-07 -4.999991e-01 8.660259e-01
    outer loop
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
      vertex   -8.671190e-01 2.619374e+00 2.327803e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 3.240081e-08 -4.999995e-01 8.660257e-01
    outer loop
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
      vertex   -3.652778e+00 3.440679e+00 2.375221e+01
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 7.178671e-09 -4.999998e-01 8.660255e-01
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
      vertex   -3.710346e+00 2.781315e+00 2.337152e+01
    endloop
  endfacet
  facet normal 3.337479e-07 -4.999997e-01 8.660256e-01
    outer loop
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -5.000002e-01 8.660253e-01
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -3.652778e+00 3.440679e+00 2.375221e+01
      vertex   -3.115212e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 2.465803e-07 -5.000018e-01 8.660243e-01
    outer loop
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -4.999998e-01 8.660255e-01
    outer loop
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -4.834515e-07 -5.000010e-01 8.660248e-01
    outer loop
      vertex   -1.673463e+00 1.788926e+00 2.279857e+01
      vertex   -2.538924e+00 1.690036e+00 2.274147e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -3.659810e-06 -5.000033e-01 8.660235e-01
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
      vertex   -9.123546e-01 3.553393e+00 2.381728e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -8.243989e-18 -5.000002e-01 8.660253e-01
    outer loop
      vertex   3.776050e+00 1.551615e+00 2.266155e+01
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -2.552695e-08 -5.000000e-01 8.660254e-01
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
      vertex   -3.242240e+00 2.028927e+00 2.293713e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
      vertex   -2.236910e+00 4.100000e+00 2.847876e+01
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.115212e+00 4.100000e+00 2.413286e+01
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.250000e+00 4.100000e+00 2.452132e+01
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
      vertex   -3.115212e+00 4.100000e+00 2.886714e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
      vertex   -2.250000e+00 4.100000e+00 2.452132e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -2.263090e+00 4.100000e+00 2.452124e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   -2.236910e+00 4.100000e+00 2.847876e+01
      vertex   -2.263090e+00 4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.384788e+00 4.100000e+00 2.413286e+01
      vertex   4.035459e+00 4.100000e+00 2.413286e+01
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -1.000000e+00 0.000000e+00
    outer loop
      vertex   3.662636e+00 4.100000e+00 2.522380e+01
      vertex   3.552239e+00 4.100000e+00 2.697245e+01
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 2.643442e-07 -4.999997e-01 -8.660256e-01
    outer loop
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
    endloop
  endfacet
  facet normal 6.080374e-08 -5.000002e-01 -8.660253e-01
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
    endloop
  endfacet
  facet normal 1.374836e-08 -5.000004e-01 -8.660252e-01
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -3.006243e+00 1.847755e+00 3.016747e+01
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
    endloop
  endfacet
  facet normal -6.140039e-09 -4.999998e-01 -8.660255e-01
    outer loop
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -5.000003e-01 -8.660252e-01
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 8.676261e-07 -4.999981e-01 -8.660265e-01
    outer loop
      vertex   -2.291638e+00 1.675974e+00 3.026665e+01
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 1.651614e-07 -4.999999e-01 -8.660255e-01
    outer loop
      vertex   -8.318867e-01 3.391065e+00 2.927644e+01
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
    endloop
  endfacet
  facet normal 0.000000e+00 -4.999998e-01 -8.660255e-01
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
      vertex   -3.115212e+00 4.100000e+00 2.886714e+01
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
    endloop
  endfacet
  facet normal -4.931382e-08 -5.000012e-01 -8.660247e-01
    outer loop
      vertex   -1.419455e+00 1.893771e+00 3.014090e+01
      vertex   -8.924465e-01 2.492340e+00 2.979532e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
    endloop
  endfacet
  facet normal -1.888614e-07 -4.999989e-01 -8.660260e-01
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -3.600953e+00 2.530293e+00 2.977341e+01
      vertex   -3.006243e+00 1.847755e+00 3.016747e+01
    endloop
  endfacet
  facet normal -5.739355e-09 -5.000000e-01 -8.660254e-01
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
      vertex   -3.570900e+00 3.597971e+00 2.915698e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal -0.000000e+00 -5.000002e-01 -8.660253e-01
    outer loop
      vertex   4.035459e+00 4.100000e+00 2.886714e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
      vertex   -1.384788e+00 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.632237e-01 9.865857e-01 -2.582732e-03
    outer loop
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
      vertex   7.707949e+00 -5.336464e+00 3.377389e+01
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
    endloop
  endfacet
  facet normal 3.755446e-01 9.266906e-01 -1.451760e-02
    outer loop
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   7.707949e+00 -5.336464e+00 3.377389e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
    endloop
  endfacet
  facet normal 3.474894e-01 9.376789e-01 -3.076271e-03
    outer loop
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   6.273509e+00 -4.756257e+00 3.751461e+01
    endloop
  endfacet
  facet normal 6.559613e-01 7.547430e-01 8.815100e-03
    outer loop
      vertex   6.273509e+00 -4.756257e+00 3.751461e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
    endloop
  endfacet
  facet normal 6.010912e-01 7.989102e-01 -2.077967e-02
    outer loop
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
    endloop
  endfacet
  facet normal 7.818147e-01 6.235096e-01 -1.232801e-03
    outer loop
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
      vertex   4.366807e+00 -2.942003e+00 3.050974e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
    endloop
  endfacet
  facet normal 9.205549e-01 3.896478e-01 -2.744658e-02
    outer loop
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
      vertex   4.366807e+00 -2.942003e+00 3.050974e+01
    endloop
  endfacet
  facet normal 8.911406e-01 4.536502e-01 -8.363562e-03
    outer loop
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
      vertex   4.826943e+00 -3.504232e+00 3.796091e+01
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
    endloop
  endfacet
  facet normal 9.980220e-01 6.059584e-02 -1.674281e-02
    outer loop
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
    endloop
  endfacet
  facet normal 9.825580e-01 1.858598e-01 5.993429e-03
    outer loop
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   3.698020e+00 -1.278292e+00 3.841163e+01
    endloop
  endfacet
  facet normal 9.847212e-01 -1.739729e-01 7.586102e-03
    outer loop
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 9.585411e-01 -2.848675e-01 -7.033037e-03
    outer loop
      vertex   3.576036e+00 7.456992e-01 3.846555e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
      vertex   4.063905e+00 2.392555e+00 3.825325e+01
    endloop
  endfacet
  facet normal 8.768690e-01 -4.806163e-01 1.042441e-02
    outer loop
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   4.063905e+00 2.392555e+00 3.825325e+01
      vertex   3.821702e+00 1.776182e+00 3.020879e+01
    endloop
  endfacet
  facet normal 8.297951e-01 -5.580576e-01 -3.422943e-03
    outer loop
      vertex   4.063905e+00 2.392555e+00 3.825325e+01
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   4.873582e+00 3.598410e+00 3.794042e+01
    endloop
  endfacet
  facet normal 7.045330e-01 -7.096414e-01 6.514720e-03
    outer loop
      vertex   4.659392e+00 3.322316e+00 3.102921e+01
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
      vertex   4.873582e+00 3.598410e+00 3.794042e+01
    endloop
  endfacet
  facet normal 6.534462e-01 -7.569187e-01 -9.056951e-03
    outer loop
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
      vertex   4.873582e+00 3.598410e+00 3.794042e+01
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
    endloop
  endfacet
  facet normal 4.399553e-01 -8.979168e-01 1.359211e-02
    outer loop
      vertex   5.756785e+00 4.425599e+00 3.253141e+01
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
    endloop
  endfacet
  facet normal 3.341700e-01 -9.421721e-01 -2.534213e-02
    outer loop
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   6.121047e+00 4.680029e+00 3.754888e+01
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
    endloop
  endfacet
  facet normal 1.720895e-01 -9.850673e-01 5.265268e-03
    outer loop
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   7.343870e+00 5.219603e+00 3.361313e+01
      vertex   8.934804e+00 5.499603e+00 3.399983e+01
    endloop
  endfacet
  facet normal 6.203892e-03 -9.990302e-01 -4.359000e-02
    outer loop
      vertex   9.000000e+00 5.500000e+00 3.400000e+01
      vertex   7.993344e+00 5.352443e+00 3.723856e+01
      vertex   8.934804e+00 5.499603e+00 3.399983e+01
    endloop
  endfacet
  facet normal 7.074825e-01 -6.258991e-01 3.282056e-01
    outer loop
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
      vertex   2.510000e+01 -1.058915e+01 3.214535e+01
      vertex   2.510000e+01 -1.066536e+01 3.200000e+01
    endloop
  endfacet
  facet normal 7.000023e-01 -5.883148e-01 4.048241e-01
    outer loop
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
      vertex   2.510000e+01 -1.058915e+01 3.214535e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 8.530065e-01 -3.978601e-01 3.377681e-01
    outer loop
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 8.506476e-01 -3.996567e-01 3.415745e-01
    outer loop
      vertex   2.510000e+01 -9.054470e+00 3.437506e+01
      vertex   2.510000e+01 -9.056495e+00 3.437269e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.373587e-01 -4.093342e-01 3.623203e-01
    outer loop
      vertex   2.510000e+01 -8.917723e+00 3.452955e+01
      vertex   2.510000e+01 -9.054470e+00 3.437506e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.376096e-01 -4.091708e-01 3.619245e-01
    outer loop
      vertex   2.510000e+01 -8.916174e+00 3.453130e+01
      vertex   2.510000e+01 -8.917723e+00 3.452955e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.239474e-01 -4.177290e-01 3.829009e-01
    outer loop
      vertex   2.510000e+01 -8.776235e+00 3.468397e+01
      vertex   2.510000e+01 -8.916174e+00 3.453130e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.238223e-01 -4.177968e-01 3.830961e-01
    outer loop
      vertex   2.510000e+01 -8.775182e+00 3.468512e+01
      vertex   2.510000e+01 -8.776235e+00 3.468397e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.107213e-01 -4.246177e-01 4.030269e-01
    outer loop
      vertex   2.510000e+01 -8.632073e+00 3.483590e+01
      vertex   2.510000e+01 -8.775182e+00 3.468512e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 8.106583e-01 -4.246445e-01 4.031254e-01
    outer loop
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
      vertex   2.510000e+01 -8.632073e+00 3.483590e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.840949e-01 -4.350272e-01 4.426585e-01
    outer loop
      vertex   2.510000e+01 -8.185120e+00 3.527518e+01
      vertex   2.510000e+01 -8.631537e+00 3.483646e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.669356e-01 -4.336999e-01 4.729843e-01
    outer loop
      vertex   2.510000e+01 -8.183972e+00 3.527623e+01
      vertex   2.510000e+01 -8.185120e+00 3.527518e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.621849e-01 -4.332469e-01 4.810107e-01
    outer loop
      vertex   2.510000e+01 -7.875061e+00 3.555447e+01
      vertex   2.510000e+01 -8.183972e+00 3.527623e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.552082e-01 -4.259395e-01 4.982330e-01
    outer loop
      vertex   2.510000e+01 -7.872691e+00 3.555650e+01
      vertex   2.510000e+01 -7.875061e+00 3.555447e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.250205e-01 -3.953368e-01 5.639629e-01
    outer loop
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
      vertex   2.510000e+01 -7.872691e+00 3.555650e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.200689e-01 -3.999581e-01 5.670399e-01
    outer loop
      vertex   2.550000e+01 -5.500000e+00 3.671323e+01
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
    endloop
  endfacet
  facet normal 7.058526e-01 -3.344005e-01 6.244585e-01
    outer loop
      vertex   2.510000e+01 -5.500000e+00 3.716536e+01
      vertex   2.510000e+01 -5.828617e+00 3.698939e+01
      vertex   2.550000e+01 -5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 9.146304e-04 6.995205e-01 -7.146119e-01
    outer loop
      vertex   2.550000e+01 -6.500000e+00 3.300000e+01
      vertex   9.000000e+00 -7.500000e+00 3.200000e+01
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
    endloop
  endfacet
  facet normal 8.066982e-03 9.993212e-01 -3.594394e-02
    outer loop
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
      vertex   7.896347e+00 -5.358569e+00 3.723641e+01
      vertex   2.510000e+01 -5.500000e+00 3.716536e+01
    endloop
  endfacet
  facet normal 9.599039e-04 9.999992e-01 8.492143e-04
    outer loop
      vertex   2.550000e+01 -5.500000e+00 3.671323e+01
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
      vertex   2.510000e+01 -5.500000e+00 3.716536e+01
    endloop
  endfacet
  facet normal 4.311545e-02 9.646992e-01 -2.598008e-01
    outer loop
      vertex   2.550000e+01 -6.500000e+00 3.300000e+01
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
      vertex   2.550000e+01 -5.500000e+00 3.671323e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
      vertex   2.550000e+01 -6.500000e+00 3.300000e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   2.550000e+01 -5.500000e+00 3.671323e+01
      vertex   2.550000e+01 -7.274414e+00 3.546166e+01
      vertex   2.550000e+01 -6.500000e+00 3.300000e+01
    endloop
  endfacet
  facet normal 4.272184e-02 2.598052e-01 -9.647156e-01
    outer loop
      vertex   9.000000e+00 -7.500000e+00 3.200000e+01
      vertex   2.550000e+01 -6.500000e+00 3.300000e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 5.417988e-03 3.294853e-02 -9.994424e-01
    outer loop
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   9.000000e+00 -7.500000e+00 3.200000e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 6.551642e-03 -5.796139e-03 -9.999617e-01
    outer loop
      vertex   2.510000e+01 -1.066536e+01 3.200000e+01
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   2.550000e+01 -1.021323e+01 3.200000e+01
    endloop
  endfacet
  facet normal 2.907764e-01 -5.754928e-03 9.567737e-01
    outer loop
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
    endloop
  endfacet
  facet normal 3.049590e-01 -1.557426e-04 9.523655e-01
    outer loop
      vertex   8.032088e+00 -1.072297e+01 2.111426e+01
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
      vertex   6.518182e+00 -1.095275e+01 2.159899e+01
    endloop
  endfacet
  facet normal 5.725686e-01 -5.418893e-03 8.198389e-01
    outer loop
      vertex   5.320501e+00 -1.128601e+01 2.243324e+01
      vertex   6.518182e+00 -1.095275e+01 2.159899e+01
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
    endloop
  endfacet
  facet normal 5.589143e-01 1.252384e-04 8.292254e-01
    outer loop
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
      vertex   5.320501e+00 -1.128601e+01 2.243324e+01
    endloop
  endfacet
  facet normal 7.494269e-01 4.960119e-04 6.620870e-01
    outer loop
      vertex   5.320501e+00 -1.128601e+01 2.243324e+01
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
    endloop
  endfacet
  facet normal 7.489384e-01 6.489299e-04 6.626393e-01
    outer loop
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
      vertex   4.497651e+00 -4.283071e+00 2.335773e+01
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
    endloop
  endfacet
  facet normal 8.589039e-01 2.348581e-05 5.121369e-01
    outer loop
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
      vertex   4.497651e+00 -4.283071e+00 2.335773e+01
    endloop
  endfacet
  facet normal 9.101431e-01 1.298070e-02 4.140906e-01
    outer loop
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   4.485190e+00 -1.158317e+01 2.337896e+01
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 9.555331e-01 -1.540106e-03 2.948798e-01
    outer loop
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 9.915965e-01 1.110064e-02 1.288921e-01
    outer loop
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
      vertex   3.762417e+00 -1.188341e+01 2.497698e+01
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
    endloop
  endfacet
  facet normal 9.998638e-01 -1.165902e-02 -1.168412e-02
    outer loop
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
    endloop
  endfacet
  facet normal 9.714529e-01 7.683710e-03 -2.371081e-01
    outer loop
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
      vertex   3.525635e+00 -1.198831e+01 2.680764e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
    endloop
  endfacet
  facet normal 9.594167e-01 3.582576e-04 -2.819921e-01
    outer loop
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
    endloop
  endfacet
  facet normal 8.368548e-01 9.396572e-03 -5.473443e-01
    outer loop
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   3.970167e+00 -1.179412e+01 2.863522e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 8.208811e-01 2.284555e-03 -5.710946e-01
    outer loop
      vertex   5.041383e+00 -5.193132e+00 3.030866e+01
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 6.313661e-01 4.968648e-05 -7.754849e-01
    outer loop
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   5.041383e+00 -5.193132e+00 3.030866e+01
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
    endloop
  endfacet
  facet normal 5.864295e-01 1.364188e-02 -8.098854e-01
    outer loop
      vertex   5.080367e+00 -1.136545e+01 3.034000e+01
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
    endloop
  endfacet
  facet normal 4.085147e-01 -1.236779e-02 -9.126680e-01
    outer loop
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
    endloop
  endfacet
  facet normal 2.958026e-01 1.616467e-02 -9.551123e-01
    outer loop
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   6.540314e+00 -1.095031e+01 3.140413e+01
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
    endloop
  endfacet
  facet normal 1.705088e-01 -9.855593e-03 -9.853069e-01
    outer loop
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
      vertex   8.917717e+00 -7.499607e+00 3.199938e+01
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
    endloop
  endfacet
  facet normal 7.637471e-03 3.232378e-02 -9.994483e-01
    outer loop
      vertex   8.093040e+00 -1.072106e+01 3.188890e+01
      vertex   8.917717e+00 -7.499607e+00 3.199938e+01
      vertex   9.000000e+00 -7.500000e+00 3.200000e+01
    endloop
  endfacet
  facet normal 9.040447e-01 2.415435e-01 -3.526470e-01
    outer loop
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   4.366807e+00 -2.942003e+00 3.050974e+01
    endloop
  endfacet
  facet normal 8.722358e-01 3.433868e-01 -3.482675e-01
    outer loop
      vertex   5.041383e+00 -5.193132e+00 3.030866e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
    endloop
  endfacet
  facet normal 8.055544e-01 4.232549e-01 -4.146534e-01
    outer loop
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
      vertex   5.041383e+00 -5.193132e+00 3.030866e+01
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
    endloop
  endfacet
  facet normal 6.076083e-01 5.749645e-01 -5.479306e-01
    outer loop
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
    endloop
  endfacet
  facet normal 5.542545e-01 5.804963e-01 -5.965116e-01
    outer loop
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   7.707949e+00 -5.336464e+00 3.377389e+01
    endloop
  endfacet
  facet normal 2.668998e-01 7.033625e-01 -6.588214e-01
    outer loop
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
      vertex   7.707949e+00 -5.336464e+00 3.377389e+01
    endloop
  endfacet
  facet normal 2.843463e-01 6.940827e-01 -6.613595e-01
    outer loop
      vertex   8.917717e+00 -7.499607e+00 3.199938e+01
      vertex   7.385038e+00 -7.127981e+00 3.173043e+01
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
    endloop
  endfacet
  facet normal 8.688392e-03 7.003055e-01 -7.137904e-01
    outer loop
      vertex   9.000000e+00 -7.500000e+00 3.200000e+01
      vertex   8.917717e+00 -7.499607e+00 3.199938e+01
      vertex   8.587431e+00 -5.481441e+00 3.397540e+01
    endloop
  endfacet
  facet normal 8.715908e-01 3.002789e-01 -3.875074e-01
    outer loop
      vertex   4.366807e+00 -2.942003e+00 3.050974e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
    endloop
  endfacet
  facet normal 7.731801e-01 4.266847e-01 -4.691829e-01
    outer loop
      vertex   5.122217e+00 -3.886657e+00 3.179927e+01
      vertex   6.406964e+00 -4.820304e+00 3.306737e+01
      vertex   6.027231e+00 -6.283179e+00 3.111123e+01
    endloop
  endfacet
  facet normal 1.726485e-07 4.999991e-01 -8.660259e-01
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   -3.602644e+00 -2.466337e+00 2.981033e+01
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
    endloop
  endfacet
  facet normal -3.399511e-07 5.000003e-01 -8.660252e-01
    outer loop
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
    endloop
  endfacet
  facet normal 8.095201e-07 5.000063e-01 -8.660218e-01
    outer loop
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal -1.119056e-06 5.000022e-01 -8.660241e-01
    outer loop
      vertex   -1.499564e+00 -1.862032e+00 3.015923e+01
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal -9.290312e-07 5.000009e-01 -8.660249e-01
    outer loop
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
      vertex   -1.014230e+00 -2.258658e+00 2.993024e+01
    endloop
  endfacet
  facet normal 4.233530e-07 4.999996e-01 -8.660256e-01
    outer loop
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
    endloop
  endfacet
  facet normal -2.983247e-19 5.000019e-01 -8.660243e-01
    outer loop
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
      vertex   -3.115212e+00 -4.100000e+00 2.886714e+01
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 3.798533e-07 4.999974e-01 -8.660269e-01
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
      vertex   -3.602644e+00 -2.466337e+00 2.981033e+01
    endloop
  endfacet
  facet normal 0.000000e+00 4.999993e-01 -8.660258e-01
    outer loop
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 2.250319e-07 4.999999e-01 -8.660254e-01
    outer loop
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
    endloop
  endfacet
  facet normal 7.831301e-07 4.999984e-01 -8.660263e-01
    outer loop
      vertex   -1.014230e+00 -2.258658e+00 2.993024e+01
      vertex   -1.499564e+00 -1.862032e+00 3.015923e+01
      vertex   3.500000e+00 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 8.763672e-18 5.000003e-01 -8.660252e-01
    outer loop
      vertex   3.822249e+00 -1.672449e+00 3.026868e+01
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -4.128868e-08 5.000000e-01 -8.660254e-01
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
    endloop
  endfacet
  facet normal 8.998296e-01 2.925695e-01 3.235890e-01
    outer loop
      vertex   4.497651e+00 -4.283071e+00 2.335773e+01
      vertex   5.207370e+00 -3.963963e+00 2.109564e+01
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
    endloop
  endfacet
  facet normal 8.840624e-01 3.360823e-01 3.247804e-01
    outer loop
      vertex   4.497651e+00 -4.283071e+00 2.335773e+01
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
      vertex   5.207370e+00 -3.963963e+00 2.109564e+01
    endloop
  endfacet
  facet normal 7.939858e-01 4.242959e-01 4.353843e-01
    outer loop
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
      vertex   5.207370e+00 -3.963963e+00 2.109564e+01
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
    endloop
  endfacet
  facet normal 7.490704e-01 4.911101e-01 4.446396e-01
    outer loop
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
      vertex   5.312994e+00 -5.538473e+00 2.243743e+01
    endloop
  endfacet
  facet normal 6.341209e-01 5.411879e-01 5.522738e-01
    outer loop
      vertex   6.118915e+00 -4.676783e+00 2.012797e+01
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
    endloop
  endfacet
  facet normal 4.596519e-01 6.557025e-01 5.989778e-01
    outer loop
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
      vertex   6.590542e+00 -6.707612e+00 2.157652e+01
    endloop
  endfacet
  facet normal 3.094817e-01 6.482969e-01 6.956523e-01
    outer loop
      vertex   7.063134e+00 -5.139632e+00 1.949738e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
      vertex   8.934804e+00 -5.499603e+00 1.900018e+01
    endloop
  endfacet
  facet normal 6.340747e-03 7.407709e-01 6.717278e-01
    outer loop
      vertex   9.000000e+00 -5.500000e+00 1.900000e+01
      vertex   8.934804e+00 -5.499603e+00 1.900018e+01
      vertex   8.195617e+00 -7.383463e+00 2.108465e+01
    endloop
  endfacet
  facet normal 8.495800e-01 3.278441e-01 4.131973e-01
    outer loop
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal 8.408740e-01 3.400285e-01 4.210838e-01
    outer loop
      vertex   4.497651e+00 -4.283071e+00 2.335773e+01
      vertex   4.324999e+00 -2.882557e+00 2.257158e+01
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -9.676180e-07 5.000019e-01 8.660243e-01
    outer loop
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
      vertex   -8.643705e-01 -3.444428e+00 2.375437e+01
    endloop
  endfacet
  facet normal 3.846851e-07 5.000018e-01 8.660244e-01
    outer loop
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
      vertex   -3.623859e+00 -2.512752e+00 2.321647e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 1.403163e-07 5.000011e-01 8.660248e-01
    outer loop
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
      vertex   -2.251508e+00 -1.672190e+00 2.273117e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 3.559743e-07 4.999998e-01 8.660255e-01
    outer loop
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
    endloop
  endfacet
  facet normal 7.804722e-07 4.999970e-01 8.660271e-01
    outer loop
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.999999e-01 8.660255e-01
    outer loop
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
      vertex   -2.251508e+00 -1.672190e+00 2.273117e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -9.072244e-07 5.000022e-01 8.660241e-01
    outer loop
      vertex   -2.251508e+00 -1.672190e+00 2.273117e+01
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
      vertex   3.500000e+00 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -0.000000e+00 4.999982e-01 8.660264e-01
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
      vertex   -3.115212e+00 -4.100000e+00 2.413286e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
    endloop
  endfacet
  facet normal 7.294708e-07 4.999991e-01 8.660259e-01
    outer loop
      vertex   -8.643705e-01 -3.444428e+00 2.375437e+01
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
    endloop
  endfacet
  facet normal -1.661368e-07 4.999988e-01 8.660261e-01
    outer loop
      vertex   -3.623859e+00 -2.512752e+00 2.321647e+01
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -7.967385e-08 5.000000e-01 8.660254e-01
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -1.894150e-17 5.000001e-01 8.660253e-01
    outer loop
      vertex   3.749505e+00 -1.379020e+00 2.256191e+01
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -2.535679e-01 -9.519431e-01 -1.717780e-01
    outer loop
      vertex   -1.358425e+00 -7.135442e+00 1.768498e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -1.632213e+00 -6.878993e+00 1.666796e+01
    endloop
  endfacet
  facet normal -3.955755e-01 -9.101583e-01 -1.230122e-01
    outer loop
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
      vertex   -1.358425e+00 -7.135442e+00 1.768498e+01
      vertex   -1.632213e+00 -6.878993e+00 1.666796e+01
    endloop
  endfacet
  facet normal -6.135172e-01 -7.246783e-01 -3.137482e-01
    outer loop
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -1.358425e+00 -7.135442e+00 1.768498e+01
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
    endloop
  endfacet
  facet normal -7.326072e-01 -6.278903e-01 -2.627556e-01
    outer loop
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -8.281901e-01 -7.266750e+00 1.695142e+01
    endloop
  endfacet
  facet normal -7.148155e-01 -5.437651e-01 -4.397254e-01
    outer loop
      vertex   -7.624360e-01 -7.922156e+00 1.830631e+01
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal -7.148139e-01 -5.437592e-01 -4.397352e-01
    outer loop
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
      vertex   -7.614633e-01 -7.927234e+00 1.831101e+01
      vertex   -7.624360e-01 -7.922156e+00 1.830631e+01
    endloop
  endfacet
  facet normal -7.066699e-01 -5.144034e-01 -4.858053e-01
    outer loop
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -7.614633e-01 -7.927234e+00 1.831101e+01
    endloop
  endfacet
  facet normal -7.070313e-01 -4.989335e-01 -5.011707e-01
    outer loop
      vertex   -1.504616e-01 -8.518702e+00 1.804827e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -1.504450e-01 -8.514960e+00 1.804452e+01
    endloop
  endfacet
  facet normal -7.069905e-01 -4.854986e-01 -5.142525e-01
    outer loop
      vertex   -1.522146e-01 -8.553105e+00 1.808316e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -1.504616e-01 -8.518702e+00 1.804827e+01
    endloop
  endfacet
  facet normal -7.069911e-01 -4.855068e-01 -5.142439e-01
    outer loop
      vertex   -1.522146e-01 -8.553105e+00 1.808316e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
    endloop
  endfacet
  facet normal -6.726630e-01 -3.198913e-01 -6.672286e-01
    outer loop
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
    endloop
  endfacet
  facet normal -6.968498e-01 -2.395470e-01 -6.760307e-01
    outer loop
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
      vertex   -1.531095e-01 -8.570666e+00 1.810097e+01
    endloop
  endfacet
  facet normal -4.162538e-01 -1.693664e-01 -8.933352e-01
    outer loop
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
    endloop
  endfacet
  facet normal -2.771552e-01 -2.766941e-02 -9.604267e-01
    outer loop
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -9.945514e-01 -9.631428e+00 1.934420e+01
    endloop
  endfacet
  facet normal -9.442592e-02 -1.217033e-01 -9.880648e-01
    outer loop
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
    endloop
  endfacet
  facet normal 3.172216e-01 -1.368466e-01 -9.384261e-01
    outer loop
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -3.199527e+00 -8.798140e+00 1.933787e+01
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
    endloop
  endfacet
  facet normal 2.060057e-01 3.444410e-03 -9.785447e-01
    outer loop
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -2.119735e+00 -9.870479e+00 1.967579e+01
    endloop
  endfacet
  facet normal 7.222267e-01 -1.771080e-01 -6.685966e-01
    outer loop
      vertex   -3.199527e+00 -8.798140e+00 1.933787e+01
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -3.730883e+00 -8.225417e+00 1.861218e+01
    endloop
  endfacet
  facet normal 5.554436e-01 -2.839771e-01 -7.815621e-01
    outer loop
      vertex   -3.341888e+00 -9.685494e+00 1.941915e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
      vertex   -3.730883e+00 -8.225417e+00 1.861218e+01
    endloop
  endfacet
  facet normal 7.919266e-01 -4.115866e-01 -4.510528e-01
    outer loop
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -3.737802e+00 -8.192955e+00 1.857703e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
    endloop
  endfacet
  facet normal 7.591947e-01 -3.968531e-01 -5.158789e-01
    outer loop
      vertex   -3.737471e+00 -8.194510e+00 1.857871e+01
      vertex   -3.730883e+00 -8.225417e+00 1.861218e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
    endloop
  endfacet
  facet normal 7.593200e-01 -3.969106e-01 -5.156502e-01
    outer loop
      vertex   -3.737802e+00 -8.192955e+00 1.857703e+01
      vertex   -3.737471e+00 -8.194510e+00 1.857871e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
    endloop
  endfacet
  facet normal 8.107703e-01 -4.123295e-01 -4.154948e-01
    outer loop
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -4.155409e+00 -9.021248e+00 1.859964e+01
    endloop
  endfacet
  facet normal 4.799795e-01 -8.334125e-01 -2.739404e-01
    outer loop
      vertex   -3.344413e+00 -7.142703e+00 1.727437e+01
      vertex   -2.923765e+00 -7.008436e+00 1.760292e+01
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
    endloop
  endfacet
  facet normal 6.666714e-01 -6.418402e-01 -3.789331e-01
    outer loop
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
    endloop
  endfacet
  facet normal 6.593802e-01 -6.876358e-01 -3.039321e-01
    outer loop
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
      vertex   -3.604830e+00 -7.353869e+00 1.742904e+01
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
    endloop
  endfacet
  facet normal 7.071430e-01 -7.058528e-01 -4.148146e-02
    outer loop
      vertex   -4.160347e+00 -7.914057e+00 1.749125e+01
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -3.604830e+00 -7.353869e+00 1.742904e+01
    endloop
  endfacet
  facet normal 4.053432e-01 -8.839480e-01 -2.330941e-01
    outer loop
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -3.604830e+00 -7.353869e+00 1.742904e+01
    endloop
  endfacet
  facet normal 7.855541e-01 -6.059218e-01 -1.255526e-01
    outer loop
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -3.344413e+00 -7.142703e+00 1.727437e+01
    endloop
  endfacet
  facet normal 3.964704e-01 -9.078264e-01 -1.366106e-01
    outer loop
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -2.923765e+00 -7.008436e+00 1.760292e+01
      vertex   -3.344413e+00 -7.142703e+00 1.727437e+01
    endloop
  endfacet
  facet normal 1.643563e-01 -9.863901e-01 4.642320e-03
    outer loop
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -2.923765e+00 -7.008436e+00 1.760292e+01
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
    endloop
  endfacet
  facet normal 2.655347e-01 -9.480731e-01 -1.750680e-01
    outer loop
      vertex   -3.374497e+00 -7.087231e+00 1.681842e+01
      vertex   -2.496782e+00 -6.803906e+00 1.661536e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
    endloop
  endfacet
  facet normal -8.262975e-02 -9.946659e-01 -6.174174e-02
    outer loop
      vertex   -1.632213e+00 -6.878993e+00 1.666796e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -2.496782e+00 -6.803906e+00 1.661536e+01
    endloop
  endfacet
  facet normal -5.276872e-01 6.034580e-01 -5.978165e-01
    outer loop
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -8.138595e-01 4.061718e-01 -4.155203e-01
    outer loop
      vertex   -8.643705e-01 -3.444428e+00 2.375437e+01
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal -9.044722e-01 3.100481e-01 -2.929168e-01
    outer loop
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -1.246221e+00 -8.768609e+00 1.929789e+01
      vertex   -8.643705e-01 -3.444428e+00 2.375437e+01
    endloop
  endfacet
  facet normal -9.994740e-01 1.045363e-02 -3.069885e-02
    outer loop
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -8.643705e-01 -3.444428e+00 2.375437e+01
    endloop
  endfacet
  facet normal -9.990024e-01 -3.746357e-02 2.430589e-02
    outer loop
      vertex   -7.614633e-01 -7.927234e+00 1.831101e+01
      vertex   -7.509740e-01 -8.098906e+00 1.847753e+01
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
    endloop
  endfacet
  facet normal -9.902574e-01 -1.024656e-01 9.429242e-02
    outer loop
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
      vertex   -7.624360e-01 -7.922156e+00 1.830631e+01
      vertex   -7.614633e-01 -7.927234e+00 1.831101e+01
    endloop
  endfacet
  facet normal -9.902570e-01 -1.024675e-01 9.429441e-02
    outer loop
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -7.624360e-01 -7.922156e+00 1.830631e+01
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
    endloop
  endfacet
  facet normal -8.094199e-01 -4.086961e-01 4.216716e-01
    outer loop
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -8.405992e-01 -2.601251e+00 2.326756e+01
    endloop
  endfacet
  facet normal -8.506802e-01 -3.730491e-01 3.703748e-01
    outer loop
      vertex   -7.681940e-01 -7.892100e+00 1.827850e+01
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
      vertex   -1.358425e+00 -7.135442e+00 1.768498e+01
    endloop
  endfacet
  facet normal -3.263986e-01 -6.698155e-01 6.669416e-01
    outer loop
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -1.358425e+00 -7.135442e+00 1.768498e+01
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
    endloop
  endfacet
  facet normal -3.287508e-01 -6.690279e-01 6.665767e-01
    outer loop
      vertex   -2.251508e+00 -1.672190e+00 2.273117e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -1.366780e+00 -1.948175e+00 2.289051e+01
    endloop
  endfacet
  facet normal 2.874970e-01 -6.747272e-01 6.797710e-01
    outer loop
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -2.251508e+00 -1.672190e+00 2.273117e+01
    endloop
  endfacet
  facet normal 1.856220e-01 -7.013719e-01 6.882019e-01
    outer loop
      vertex   -2.923765e+00 -7.008436e+00 1.760292e+01
      vertex   -2.188349e+00 -6.886245e+00 1.752909e+01
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
    endloop
  endfacet
  facet normal 6.678266e-01 -5.246465e-01 5.279713e-01
    outer loop
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -2.923765e+00 -7.008436e+00 1.760292e+01
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
    endloop
  endfacet
  facet normal 7.753183e-01 -4.529134e-01 4.401714e-01
    outer loop
      vertex   -3.623859e+00 -2.512752e+00 2.321647e+01
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -3.054617e+00 -1.888543e+00 2.285608e+01
    endloop
  endfacet
  facet normal 9.374076e-01 -2.408114e-01 2.515491e-01
    outer loop
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -3.445756e+00 -7.398637e+00 1.787543e+01
      vertex   -3.623859e+00 -2.512752e+00 2.321647e+01
    endloop
  endfacet
  facet normal 9.992902e-01 9.164290e-03 -3.654019e-02
    outer loop
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -3.623859e+00 -2.512752e+00 2.321647e+01
    endloop
  endfacet
  facet normal 9.987129e-01 2.076023e-02 -4.627756e-02
    outer loop
      vertex   -3.749224e+00 -8.026592e+00 1.840517e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
      vertex   -3.737802e+00 -8.192955e+00 1.857703e+01
    endloop
  endfacet
  facet normal 9.896963e-01 9.171775e-02 -1.099502e-01
    outer loop
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
      vertex   -3.737471e+00 -8.194510e+00 1.857871e+01
      vertex   -3.737802e+00 -8.192955e+00 1.857703e+01
    endloop
  endfacet
  facet normal 9.896799e-01 9.180298e-02 -1.100266e-01
    outer loop
      vertex   -3.730883e+00 -8.225417e+00 1.861218e+01
      vertex   -3.737471e+00 -8.194510e+00 1.857871e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
    endloop
  endfacet
  facet normal 8.651900e-01 3.586627e-01 -3.504388e-01
    outer loop
      vertex   -3.199527e+00 -8.798140e+00 1.933787e+01
      vertex   -3.730883e+00 -8.225417e+00 1.861218e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
    endloop
  endfacet
  facet normal 8.127361e-01 4.089567e-01 -4.149874e-01
    outer loop
      vertex   -3.115212e+00 -4.100000e+00 2.413286e+01
      vertex   -3.199527e+00 -8.798140e+00 1.933787e+01
      vertex   -3.593962e+00 -3.500049e+00 2.378648e+01
    endloop
  endfacet
  facet normal 3.934855e-01 6.531948e-01 -6.469202e-01
    outer loop
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -3.199527e+00 -8.798140e+00 1.933787e+01
      vertex   -3.115212e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 3.080999e-01 6.694110e-01 -6.759906e-01
    outer loop
      vertex   -2.263090e+00 -4.100000e+00 2.452124e+01
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -3.115212e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 7.113268e-01 -7.028614e-01
    outer loop
      vertex   -2.236910e+00 -4.100000e+00 2.452124e+01
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -2.263090e+00 -4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal -1.171577e-01 7.094983e-01 -6.949001e-01
    outer loop
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -2.495439e+00 -8.962784e+00 1.959989e+01
      vertex   -2.236910e+00 -4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal -3.087049e-01 6.677888e-01 -6.773178e-01
    outer loop
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
      vertex   -1.649557e+00 -8.908794e+00 1.951240e+01
      vertex   -2.236910e+00 -4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal -9.866053e-02 -1.190884e-01 9.879697e-01
    outer loop
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
      vertex   -1.909845e+00 -9.877253e+00 3.331441e+01
    endloop
  endfacet
  facet normal -4.316970e-01 -1.858742e-01 8.826599e-01
    outer loop
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
    endloop
  endfacet
  facet normal -5.414876e-01 -2.175335e-01 8.120778e-01
    outer loop
      vertex   -8.043777e-01 -9.002310e+00 3.394430e+01
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
    endloop
  endfacet
  facet normal -6.568019e-01 -4.288916e-01 6.202123e-01
    outer loop
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -8.043777e-01 -9.002310e+00 3.394430e+01
    endloop
  endfacet
  facet normal -8.244008e-01 -1.036108e-01 5.564424e-01
    outer loop
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   -8.043777e-01 -9.002310e+00 3.394430e+01
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
    endloop
  endfacet
  facet normal -7.327688e-01 -2.262476e-01 6.417646e-01
    outer loop
      vertex   -7.681940e-01 -8.221496e+00 3.439210e+01
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -7.148193e-01 -4.397011e-01 5.437797e-01
    outer loop
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
      vertex   -7.614633e-01 -8.188992e+00 3.442724e+01
      vertex   -7.624360e-01 -8.193689e+00 3.442216e+01
    endloop
  endfacet
  facet normal -7.066693e-01 -4.858094e-01 5.144004e-01
    outer loop
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -7.614633e-01 -8.188992e+00 3.442724e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -7.148140e-01 -4.397347e-01 5.437594e-01
    outer loop
      vertex   -7.624360e-01 -8.193689e+00 3.442216e+01
      vertex   -7.681940e-01 -8.221496e+00 3.439210e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -7.070289e-01 -5.010458e-01 4.990624e-01
    outer loop
      vertex   -1.504616e-01 -8.451730e+00 3.501870e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -1.504450e-01 -8.455477e+00 3.501496e+01
    endloop
  endfacet
  facet normal -7.069888e-01 -5.142597e-01 4.854933e-01
    outer loop
      vertex   -1.522146e-01 -8.416840e+00 3.505310e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -1.504616e-01 -8.451730e+00 3.501870e+01
    endloop
  endfacet
  facet normal -7.069899e-01 -5.142439e-01 4.855084e-01
    outer loop
      vertex   -1.522146e-01 -8.416840e+00 3.505310e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
    endloop
  endfacet
  facet normal -4.209725e-01 -8.911285e-01 1.693283e-01
    outer loop
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -1.181844e+00 -7.056280e+00 3.620490e+01
    endloop
  endfacet
  facet normal -6.830795e-01 -6.370349e-01 3.571959e-01
    outer loop
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -1.531095e-01 -8.399028e+00 3.507067e+01
    endloop
  endfacet
  facet normal -6.402497e-01 -6.755931e-01 3.655877e-01
    outer loop
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
    endloop
  endfacet
  facet normal -5.397173e-01 -8.257670e-01 1.637499e-01
    outer loop
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -5.570037e-01 -7.538383e+00 3.583319e+01
      vertex   -1.181844e+00 -7.056280e+00 3.620490e+01
    endloop
  endfacet
  facet normal -1.534003e-01 -9.878924e-01 2.317249e-02
    outer loop
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -1.181844e+00 -7.056280e+00 3.620490e+01
    endloop
  endfacet
  facet normal -1.034699e-01 -9.920296e-01 7.191086e-02
    outer loop
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
    endloop
  endfacet
  facet normal 3.689211e-01 -9.260052e-01 8.007250e-02
    outer loop
      vertex   -3.295862e+00 -7.239827e+00 3.524381e+01
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
    endloop
  endfacet
  facet normal 3.677472e-01 -9.267651e-01 7.660540e-02
    outer loop
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -2.532766e+00 -6.842936e+00 3.635720e+01
    endloop
  endfacet
  facet normal 8.067778e-01 -5.387628e-01 2.425783e-01
    outer loop
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
    endloop
  endfacet
  facet normal 6.616988e-01 -6.861043e-01 3.023502e-01
    outer loop
      vertex   -3.753324e+00 -7.358867e+00 3.597485e+01
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -3.295862e+00 -7.239827e+00 3.524381e+01
    endloop
  endfacet
  facet normal 7.328116e-01 -4.977718e-01 4.639078e-01
    outer loop
      vertex   -3.749224e+00 -8.094833e+00 3.452659e+01
      vertex   -3.737802e+00 -7.922973e+00 3.469296e+01
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
    endloop
  endfacet
  facet normal 7.599828e-01 -5.153312e-01 3.960553e-01
    outer loop
      vertex   -3.737471e+00 -7.921290e+00 3.469451e+01
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
    endloop
  endfacet
  facet normal 5.146882e-01 -2.249851e-01 8.273317e-01
    outer loop
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
      vertex   -3.471032e+00 -9.653216e+00 3.362736e+01
    endloop
  endfacet
  facet normal 6.503903e-01 -3.857501e-01 6.543618e-01
    outer loop
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
      vertex   -3.749224e+00 -8.094833e+00 3.452659e+01
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
    endloop
  endfacet
  facet normal 6.625251e-01 -4.007400e-01 6.328254e-01
    outer loop
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
      vertex   -3.749224e+00 -8.094833e+00 3.452659e+01
    endloop
  endfacet
  facet normal 6.748399e-01 -2.066464e-01 7.084408e-01
    outer loop
      vertex   -4.311689e+00 -8.776243e+00 3.468395e+01
      vertex   -3.471032e+00 -9.653216e+00 3.362736e+01
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
    endloop
  endfacet
  facet normal 2.875768e-01 -7.126260e-02 9.551027e-01
    outer loop
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -3.471032e+00 -9.653216e+00 3.362736e+01
    endloop
  endfacet
  facet normal 2.449496e-01 -9.778076e-02 9.645925e-01
    outer loop
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
    endloop
  endfacet
  facet normal -4.162090e-02 -8.971024e-02 9.950979e-01
    outer loop
      vertex   -1.909845e+00 -9.877253e+00 3.331441e+01
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
      vertex   -2.416049e+00 -9.892942e+00 3.329182e+01
    endloop
  endfacet
  facet normal 7.598648e-01 -5.152559e-01 3.963796e-01
    outer loop
      vertex   -3.737802e+00 -7.922973e+00 3.469296e+01
      vertex   -3.737471e+00 -7.921290e+00 3.469451e+01
      vertex   -4.311694e+00 -8.776227e+00 3.468396e+01
    endloop
  endfacet
  facet normal -3.708454e-01 -7.206603e-02 9.258942e-01
    outer loop
      vertex   -1.909845e+00 -9.877253e+00 3.331441e+01
      vertex   -8.594272e-01 -9.551497e+00 3.376048e+01
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
    endloop
  endfacet
  facet normal 6.780072e-01 5.229241e-01 5.165817e-01
    outer loop
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -3.115212e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 8.492867e-01 3.642380e-01 3.821553e-01
    outer loop
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
      vertex   -3.115212e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 9.309405e-01 2.614571e-01 2.549313e-01
    outer loop
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
      vertex   -3.749224e+00 -8.094833e+00 3.452659e+01
      vertex   -3.411142e+00 -8.651177e+00 3.386260e+01
    endloop
  endfacet
  facet normal 9.988100e-01 -4.135317e-02 -2.585401e-02
    outer loop
      vertex   -3.749224e+00 -8.094833e+00 3.452659e+01
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
      vertex   -3.737802e+00 -7.922973e+00 3.469296e+01
    endloop
  endfacet
  facet normal 9.896321e-01 -1.132059e-01 -8.838944e-02
    outer loop
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
      vertex   -3.737471e+00 -7.921290e+00 3.469451e+01
      vertex   -3.737802e+00 -7.922973e+00 3.469296e+01
    endloop
  endfacet
  facet normal 9.896367e-01 -1.131819e-01 -8.836855e-02
    outer loop
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -3.737471e+00 -7.921290e+00 3.469451e+01
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
    endloop
  endfacet
  facet normal 9.961936e-01 -7.029467e-02 -5.154553e-02
    outer loop
      vertex   -3.602644e+00 -2.466337e+00 2.981033e+01
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -3.683800e+00 -3.274365e+00 2.934382e+01
    endloop
  endfacet
  facet normal 8.841413e-01 -3.251187e-01 -3.355474e-01
    outer loop
      vertex   -3.295862e+00 -7.239827e+00 3.524381e+01
      vertex   -3.730883e+00 -7.887818e+00 3.472542e+01
      vertex   -3.602644e+00 -2.466337e+00 2.981033e+01
    endloop
  endfacet
  facet normal 7.568440e-01 -4.692391e-01 -4.549745e-01
    outer loop
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
      vertex   -3.295862e+00 -7.239827e+00 3.524381e+01
      vertex   -3.602644e+00 -2.466337e+00 2.981033e+01
    endloop
  endfacet
  facet normal 4.403198e-01 -6.294379e-01 -6.402550e-01
    outer loop
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -3.295862e+00 -7.239827e+00 3.524381e+01
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
    endloop
  endfacet
  facet normal 2.923256e-01 -6.792699e-01 -6.731553e-01
    outer loop
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -3.030582e+00 -1.874792e+00 3.015186e+01
    endloop
  endfacet
  facet normal -1.244097e-01 -6.953590e-01 -7.078122e-01
    outer loop
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -2.514496e+00 -6.910094e+00 3.545702e+01
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
    endloop
  endfacet
  facet normal -2.622406e-01 -6.854503e-01 -6.792553e-01
    outer loop
      vertex   -1.499564e+00 -1.862032e+00 3.015923e+01
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -2.285374e+00 -1.670805e+00 3.026963e+01
    endloop
  endfacet
  facet normal -5.334774e-01 -5.976467e-01 -5.985150e-01
    outer loop
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -1.607252e+00 -7.009221e+00 3.539494e+01
      vertex   -1.499564e+00 -1.862032e+00 3.015923e+01
    endloop
  endfacet
  facet normal -6.745891e-01 -5.270663e-01 -5.168469e-01
    outer loop
      vertex   -1.014230e+00 -2.258658e+00 2.993024e+01
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -1.499564e+00 -1.862032e+00 3.015923e+01
    endloop
  endfacet
  facet normal -9.091793e-01 -2.863800e-01 -3.022905e-01
    outer loop
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -1.225043e+00 -7.215393e+00 3.526013e+01
      vertex   -1.014230e+00 -2.258658e+00 2.993024e+01
    endloop
  endfacet
  facet normal -9.484256e-01 -2.245548e-01 -2.237498e-01
    outer loop
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -1.014230e+00 -2.258658e+00 2.993024e+01
    endloop
  endfacet
  facet normal -9.990362e-01 2.863422e-02 3.326710e-02
    outer loop
      vertex   -7.614633e-01 -8.188992e+00 3.442724e+01
      vertex   -7.509740e-01 -8.022473e+00 3.459891e+01
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
    endloop
  endfacet
  facet normal -9.902551e-01 9.331736e-02 1.033769e-01
    outer loop
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
      vertex   -7.624360e-01 -8.193689e+00 3.442216e+01
      vertex   -7.614633e-01 -8.188992e+00 3.442724e+01
    endloop
  endfacet
  facet normal -9.902554e-01 9.331592e-02 1.033754e-01
    outer loop
      vertex   -7.681940e-01 -8.221496e+00 3.439210e+01
      vertex   -7.624360e-01 -8.193689e+00 3.442216e+01
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
    endloop
  endfacet
  facet normal -9.601328e-01 1.877487e-01 2.071121e-01
    outer loop
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
      vertex   -7.681940e-01 -8.221496e+00 3.439210e+01
      vertex   -7.722978e-01 -2.907316e+00 2.955573e+01
    endloop
  endfacet
  facet normal -8.981185e-01 3.132391e-01 3.086494e-01
    outer loop
      vertex   -7.681940e-01 -8.221496e+00 3.439210e+01
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
    endloop
  endfacet
  facet normal -7.391380e-01 4.740337e-01 4.785050e-01
    outer loop
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -1.047798e+00 -3.768022e+00 2.905880e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -5.562265e-01 5.941429e-01 5.810389e-01
    outer loop
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
      vertex   -1.135209e+00 -8.696037e+00 3.380575e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -3.051549e-01 6.670264e-01 6.796737e-01
    outer loop
      vertex   -2.250000e+00 -4.100000e+00 2.847868e+01
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal -1.183923e-01 7.051818e-01 6.990721e-01
    outer loop
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
      vertex   -1.776666e+00 -8.939592e+00 3.344073e+01
      vertex   -2.250000e+00 -4.100000e+00 2.847868e+01
    endloop
  endfacet
  facet normal 4.318692e-03 7.085155e-01 7.056820e-01
    outer loop
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
      vertex   -2.250000e+00 -4.100000e+00 2.847868e+01
    endloop
  endfacet
  facet normal 2.945896e-01 6.735655e-01 6.778838e-01
    outer loop
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -2.381301e+00 -8.976801e+00 3.337587e+01
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal 3.080830e-01 6.694564e-01 6.759534e-01
    outer loop
      vertex   -3.115212e+00 -4.100000e+00 2.886714e+01
      vertex   -2.993188e+00 -8.875864e+00 3.354148e+01
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.115212e+00 -4.100000e+00 2.886714e+01
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.263090e+00 -4.100000e+00 2.452124e+01
      vertex   -3.115212e+00 -4.100000e+00 2.413286e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
      vertex   -2.263090e+00 -4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.250000e+00 -4.100000e+00 2.847868e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
      vertex   -2.236910e+00 -4.100000e+00 2.452124e+01
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
      vertex   -2.236910e+00 -4.100000e+00 2.452124e+01
      vertex   -2.263090e+00 -4.100000e+00 2.452124e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
      vertex   4.035459e+00 -4.100000e+00 2.413286e+01
      vertex   -1.384788e+00 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.263090e+00 -4.100000e+00 2.847876e+01
      vertex   -2.250000e+00 -4.100000e+00 2.847868e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
      vertex   3.602746e+00 -4.100000e+00 2.553503e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   4.035459e+00 -4.100000e+00 2.886714e+01
      vertex   3.625380e+00 -4.100000e+00 2.747193e+01
      vertex   -1.384788e+00 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.150000e+01 2.548515e+00 2.758547e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.699575e-01 2.922888e+01
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -8.673298e-01 2.913745e+01
      vertex   -2.150000e+01 4.699575e-01 2.922888e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 2.548515e+00 2.758547e+01
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -2.600937e+00 2.555313e+01
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.150000e+01 1.564485e+00 2.419430e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.886714e+01
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.150000e+01 -8.673298e-01 2.913745e+01
    endloop
  endfacet
  facet normal 1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.150000e+01 8.673618e-16 3.123427e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.150000e+01 -4.100000e+00 2.886714e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 4.100000e+00 2.413286e+01
      vertex   -2.150000e+01 1.564485e+00 2.419430e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.150000e+01 -2.600937e+00 2.555313e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -8.053459e-01 2.382945e+01
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.150000e+01 -4.100000e+00 2.413286e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
      vertex   -2.150000e+01 5.624098e-01 2.378088e+01
      vertex   -2.150000e+01 -8.053459e-01 2.382945e+01
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.150000e+01 1.564485e+00 2.419430e+01
      vertex   -2.150000e+01 5.624098e-01 2.378088e+01
      vertex   -2.150000e+01 -3.469447e-15 2.176573e+01
    endloop
  endfacet
  facet normal -6.900465e-01 -2.677314e-02 -7.232697e-01
    outer loop
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
    endloop
  endfacet
  facet normal -7.370336e-01 -5.294936e-02 -6.737787e-01
    outer loop
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
    endloop
  endfacet
  facet normal -6.793071e-01 -1.899445e-01 -7.088462e-01
    outer loop
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.570000e+01 -1.902315e+00 1.469625e+01
    endloop
  endfacet
  facet normal -8.030952e-01 -2.562509e-01 -5.379345e-01
    outer loop
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
    endloop
  endfacet
  facet normal -7.274860e-01 -3.425721e-01 -5.944817e-01
    outer loop
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.570000e+01 -4.527151e+00 1.539961e+01
    endloop
  endfacet
  facet normal -7.162040e-01 -4.911788e-01 -4.957774e-01
    outer loop
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.570000e+01 -7.109334e+00 1.688760e+01
    endloop
  endfacet
  facet normal -7.689177e-01 -4.693979e-01 -4.340866e-01
    outer loop
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
    endloop
  endfacet
  facet normal -6.796099e-01 -6.332863e-01 -3.702417e-01
    outer loop
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.570000e+01 -9.567412e+00 1.932287e+01
    endloop
  endfacet
  facet normal -7.562815e-01 -5.949179e-01 -2.722334e-01
    outer loop
      vertex   -2.650000e+01 -1.070762e+01 2.332185e+01
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
    endloop
  endfacet
  facet normal -6.694086e-01 -7.177060e-01 -1.918078e-01
    outer loop
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
      vertex   -2.650000e+01 -1.070762e+01 2.332185e+01
      vertex   -2.570000e+01 -1.107354e+01 2.189906e+01
    endloop
  endfacet
  facet normal -7.480571e-01 -6.568396e-01 -9.472226e-02
    outer loop
      vertex   -2.650000e+01 -1.114656e+01 2.636563e+01
      vertex   -2.650000e+01 -1.070762e+01 2.332185e+01
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
    endloop
  endfacet
  facet normal -6.117263e-01 -7.907736e-01 2.163493e-02
    outer loop
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
      vertex   -2.650000e+01 -1.114656e+01 2.636563e+01
      vertex   -2.570000e+01 -1.181202e+01 2.466232e+01
    endloop
  endfacet
  facet normal -7.689050e-01 -6.303290e-01 1.071000e-01
    outer loop
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.650000e+01 -1.114656e+01 2.636563e+01
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
    endloop
  endfacet
  facet normal -6.032861e-01 -7.397045e-01 2.981327e-01
    outer loop
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.570000e+01 -1.169690e+01 2.887007e+01
    endloop
  endfacet
  facet normal -7.619786e-01 -5.648089e-01 3.168272e-01
    outer loop
      vertex   -2.650000e+01 -8.597580e+00 3.362261e+01
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
    endloop
  endfacet
  facet normal -6.740260e-01 -5.803276e-01 4.570655e-01
    outer loop
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
      vertex   -2.650000e+01 -8.597580e+00 3.362261e+01
      vertex   -2.570000e+01 -1.004835e+01 3.296034e+01
    endloop
  endfacet
  facet normal -7.510027e-01 -4.471247e-01 4.858749e-01
    outer loop
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.650000e+01 -8.597580e+00 3.362261e+01
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
    endloop
  endfacet
  facet normal -6.751099e-01 -4.509118e-01 5.838708e-01
    outer loop
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.570000e+01 -8.435057e+00 3.500871e+01
    endloop
  endfacet
  facet normal -7.531239e-01 -2.386632e-01 6.130614e-01
    outer loop
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
    endloop
  endfacet
  facet normal -8.282786e-01 -2.362908e-01 5.080563e-01
    outer loop
      vertex   -2.570000e+01 -3.680766e+00 3.791295e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.570000e+01 -6.178879e+00 3.675111e+01
    endloop
  endfacet
  facet normal -7.223454e-01 -1.397866e-01 6.772569e-01
    outer loop
      vertex   -2.570000e+01 -1.345736e+00 3.839490e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.570000e+01 -3.680766e+00 3.791295e+01
    endloop
  endfacet
  facet normal -7.817854e-01 3.032601e-02 6.228097e-01
    outer loop
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.570000e+01 -1.345736e+00 3.839490e+01
    endloop
  endfacet
  facet normal -6.980059e-01 -3.348625e-03 7.160841e-01
    outer loop
      vertex   -2.650000e+01 2.200846e+00 3.743886e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
    endloop
  endfacet
  facet normal -6.850689e-01 2.603748e-01 6.803569e-01
    outer loop
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.650000e+01 2.200846e+00 3.743886e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
    endloop
  endfacet
  facet normal -7.032854e-01 2.471826e-01 6.665512e-01
    outer loop
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.570000e+01 2.267286e+00 3.821898e+01
    endloop
  endfacet
  facet normal -7.255719e-01 4.097118e-01 5.528849e-01
    outer loop
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.570000e+01 6.071994e+00 3.680804e+01
    endloop
  endfacet
  facet normal -7.848526e-01 3.989554e-01 4.741740e-01
    outer loop
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
    endloop
  endfacet
  facet normal -7.078344e-01 5.284960e-01 4.686815e-01
    outer loop
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.570000e+01 8.250125e+00 3.519395e+01
    endloop
  endfacet
  facet normal -7.526733e-01 5.736261e-01 3.231655e-01
    outer loop
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.570000e+01 9.666968e+00 3.359629e+01
    endloop
  endfacet
  facet normal -7.676557e-01 5.627809e-01 3.065655e-01
    outer loop
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
    endloop
  endfacet
  facet normal -6.843539e-01 6.973999e-01 2.128219e-01
    outer loop
      vertex   -2.570000e+01 1.185226e+01 2.837721e+01
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.570000e+01 1.095996e+01 3.130119e+01
    endloop
  endfacet
  facet normal -7.942316e-01 6.017018e-01 8.456447e-02
    outer loop
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.570000e+01 1.185226e+01 2.837721e+01
    endloop
  endfacet
  facet normal -8.178579e-01 5.726476e-01 5.641983e-02
    outer loop
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
    endloop
  endfacet
  facet normal -5.901544e-01 8.024635e-01 -8.814790e-02
    outer loop
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.570000e+01 1.191055e+01 2.796243e+01
    endloop
  endfacet
  facet normal -8.278496e-01 5.261866e-01 -1.944034e-01
    outer loop
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
    endloop
  endfacet
  facet normal -6.861629e-01 6.420696e-01 -3.419459e-01
    outer loop
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.570000e+01 1.133477e+01 2.272073e+01
    endloop
  endfacet
  facet normal -7.632660e-01 4.587468e-01 -4.549466e-01
    outer loop
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.570000e+01 9.464422e+00 1.920879e+01
    endloop
  endfacet
  facet normal -8.450451e-01 3.458103e-01 -4.078161e-01
    outer loop
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
    endloop
  endfacet
  facet normal -6.968713e-01 3.418390e-01 -6.304891e-01
    outer loop
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.570000e+01 7.194510e+00 1.691991e+01
    endloop
  endfacet
  facet normal -7.561443e-01 1.629861e-01 -6.337834e-01
    outer loop
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.570000e+01 4.418604e+00 1.541487e+01
    endloop
  endfacet
  facet normal -7.298580e-01 1.823257e-01 -6.588358e-01
    outer loop
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.570000e+01 1.180441e+00 1.458213e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -1.070762e+01 2.332185e+01
      vertex   -2.650000e+01 -1.114656e+01 2.636563e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.650000e+01 -1.070762e+01 2.332185e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.650000e+01 -9.485726e+00 2.065162e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.650000e+01 -6.683004e+00 1.762091e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -2.405191e-01 2.294351e+01
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.650000e+01 -2.474941e+00 1.561635e+01
      vertex   -2.650000e+01 -2.405191e-01 2.294351e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.650000e+01 -2.405191e-01 2.294351e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.650000e+01 7.884573e-01 1.535989e+01
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.650000e+01 4.664982e+00 1.643268e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.650000e+01 9.161722e+00 2.024572e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.650000e+01 1.103004e+01 2.530266e+01
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.650000e+01 1.056497e+01 3.002307e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.650000e+01 8.655171e+00 3.352900e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.650000e+01 2.200846e+00 3.743886e+01
      vertex   -2.650000e+01 5.516136e+00 3.617009e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.650000e+01 2.200846e+00 3.743886e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.650000e+01 -1.937458e+00 3.741951e+01
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -8.597580e+00 3.362261e+01
      vertex   -2.650000e+01 -6.329691e+00 3.570963e+01
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.650000e+01 -8.597580e+00 3.362261e+01
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.650000e+01 -1.114656e+01 2.636563e+01
      vertex   -2.650000e+01 -1.048531e+01 3.025734e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
    endloop
  endfacet
  facet normal -5.850149e-01 1.393513e-01 -7.989611e-01
    outer loop
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
      vertex   -2.570000e+01 6.374226e-02 2.928379e+01
    endloop
  endfacet
  facet normal -6.461864e-01 8.670089e-02 -7.582388e-01
    outer loop
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
    endloop
  endfacet
  facet normal -7.246576e-01 3.940254e-01 -5.653453e-01
    outer loop
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
    endloop
  endfacet
  facet normal -6.614376e-01 4.919306e-01 -5.661312e-01
    outer loop
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
      vertex   -2.650000e+01 -1.555606e+00 2.971856e+01
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
    endloop
  endfacet
  facet normal -7.184835e-01 6.174547e-01 -3.202049e-01
    outer loop
      vertex   -2.570000e+01 -2.704842e+00 2.712690e+01
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
    endloop
  endfacet
  facet normal -5.508712e-01 8.101435e-01 -2.005203e-01
    outer loop
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
      vertex   -2.650000e+01 -2.895527e+00 2.855425e+01
      vertex   -2.570000e+01 -2.704842e+00 2.712690e+01
    endloop
  endfacet
  facet normal -6.697181e-01 7.411229e-01 -4.705899e-02
    outer loop
      vertex   -2.570000e+01 -2.765951e+00 2.616449e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
      vertex   -2.570000e+01 -2.704842e+00 2.712690e+01
    endloop
  endfacet
  facet normal -6.497134e-01 7.147504e-01 2.588521e-01
    outer loop
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
      vertex   -2.570000e+01 -2.765951e+00 2.616449e+01
    endloop
  endfacet
  facet normal -4.911077e-01 7.606385e-01 4.245495e-01
    outer loop
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
      vertex   -2.650000e+01 -3.489512e+00 2.615443e+01
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
    endloop
  endfacet
  facet normal -6.838856e-01 5.346091e-01 4.964811e-01
    outer loop
      vertex   -2.570000e+01 -1.529558e+00 2.417125e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
    endloop
  endfacet
  facet normal -5.999526e-01 2.096172e-01 7.720864e-01
    outer loop
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
      vertex   -2.570000e+01 -1.529558e+00 2.417125e+01
    endloop
  endfacet
  facet normal -7.249853e-01 2.598310e-01 6.378747e-01
    outer loop
      vertex   -2.650000e+01 -2.405191e-01 2.294351e+01
      vertex   -2.650000e+01 -2.126017e+00 2.371154e+01
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
    endloop
  endfacet
  facet normal -6.976254e-01 -9.074336e-02 7.106930e-01
    outer loop
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
      vertex   -2.650000e+01 -2.405191e-01 2.294351e+01
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
    endloop
  endfacet
  facet normal -7.461822e-01 -1.586237e-01 6.465684e-01
    outer loop
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
    endloop
  endfacet
  facet normal -6.103559e-01 -4.971319e-01 6.167054e-01
    outer loop
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
      vertex   -2.650000e+01 1.163833e+00 2.312282e+01
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
    endloop
  endfacet
  facet normal -6.548878e-01 -5.050442e-01 5.621853e-01
    outer loop
      vertex   -2.570000e+01 2.260143e+00 2.487783e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
    endloop
  endfacet
  facet normal -6.774318e-01 -6.845301e-01 2.692671e-01
    outer loop
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
      vertex   -2.570000e+01 2.260143e+00 2.487783e+01
    endloop
  endfacet
  facet normal -6.337788e-01 -7.350693e-01 2.408269e-01
    outer loop
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
      vertex   -2.650000e+01 2.917599e+00 2.453655e+01
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
    endloop
  endfacet
  facet normal -7.341408e-01 -6.783367e-01 -2.994407e-02
    outer loop
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
    endloop
  endfacet
  facet normal -5.955091e-01 -7.508565e-01 -2.856283e-01
    outer loop
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
      vertex   -2.650000e+01 3.580821e+00 2.656088e+01
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
    endloop
  endfacet
  facet normal -6.769216e-01 -6.497935e-01 -3.457537e-01
    outer loop
      vertex   -2.570000e+01 2.151688e+00 2.825043e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
    endloop
  endfacet
  facet normal -6.347599e-01 -4.784641e-01 -6.067553e-01
    outer loop
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
      vertex   -2.570000e+01 2.151688e+00 2.825043e+01
    endloop
  endfacet
  facet normal -5.315730e-01 -4.585145e-01 -7.121760e-01
    outer loop
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
      vertex   -2.650000e+01 2.820310e+00 2.856010e+01
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
    endloop
  endfacet
  facet normal -7.050644e-01 -1.596689e-01 -6.909341e-01
    outer loop
      vertex   -2.570000e+01 6.374226e-02 2.928379e+01
      vertex   -2.650000e+01 6.323761e-01 2.996874e+01
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
    endloop
  endfacet
  facet normal -1.017943e-02 3.548420e-02 9.993184e-01
    outer loop
      vertex   -2.150000e+01 -8.053459e-01 2.382945e+01
      vertex   -2.150000e+01 5.624098e-01 2.378088e+01
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
    endloop
  endfacet
  facet normal 3.334163e-02 2.618642e-01 9.645286e-01
    outer loop
      vertex   -2.570000e+01 -1.529558e+00 2.417125e+01
      vertex   -2.150000e+01 -8.053459e-01 2.382945e+01
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
    endloop
  endfacet
  facet normal -1.546314e-02 4.987008e-01 8.666363e-01
    outer loop
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.150000e+01 -8.053459e-01 2.382945e+01
      vertex   -2.570000e+01 -1.529558e+00 2.417125e+01
    endloop
  endfacet
  facet normal 9.988948e-03 7.327170e-01 6.804601e-01
    outer loop
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.570000e+01 -1.529558e+00 2.417125e+01
    endloop
  endfacet
  facet normal -1.976269e-02 8.211418e-01 5.703820e-01
    outer loop
      vertex   -2.150000e+01 -2.600937e+00 2.555313e+01
      vertex   -2.150000e+01 -1.802010e+00 2.440297e+01
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
    endloop
  endfacet
  facet normal 1.262388e-02 9.401643e-01 3.404875e-01
    outer loop
      vertex   -2.570000e+01 -2.765951e+00 2.616449e+01
      vertex   -2.150000e+01 -2.600937e+00 2.555313e+01
      vertex   -2.570000e+01 -2.373000e+00 2.507946e+01
    endloop
  endfacet
  facet normal -3.128710e-02 9.980267e-01 5.444019e-02
    outer loop
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.150000e+01 -2.600937e+00 2.555313e+01
      vertex   -2.570000e+01 -2.765951e+00 2.616449e+01
    endloop
  endfacet
  facet normal -4.386553e-03 9.979805e-01 -6.336865e-02
    outer loop
      vertex   -2.570000e+01 -2.704842e+00 2.712690e+01
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.570000e+01 -2.765951e+00 2.616449e+01
    endloop
  endfacet
  facet normal -4.229219e-03 8.877212e-01 -4.603619e-01
    outer loop
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.570000e+01 -2.704842e+00 2.712690e+01
    endloop
  endfacet
  facet normal -1.611634e-02 8.699023e-01 -4.929607e-01
    outer loop
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.150000e+01 -2.686597e+00 2.712349e+01
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
    endloop
  endfacet
  facet normal 1.310975e-02 5.717405e-01 -8.203298e-01
    outer loop
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.570000e+01 -2.083370e+00 2.832529e+01
    endloop
  endfacet
  facet normal -5.219317e-03 5.125464e-01 -8.586437e-01
    outer loop
      vertex   -2.150000e+01 -8.673298e-01 2.913745e+01
      vertex   -2.150000e+01 -1.891899e+00 2.852586e+01
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
    endloop
  endfacet
  facet normal 3.765523e-03 1.718205e-01 -9.851211e-01
    outer loop
      vertex   -2.570000e+01 6.374226e-02 2.928379e+01
      vertex   -2.150000e+01 -8.673298e-01 2.913745e+01
      vertex   -2.570000e+01 -9.657512e-01 2.910423e+01
    endloop
  endfacet
  facet normal -1.963586e-02 6.820069e-02 -9.974784e-01
    outer loop
      vertex   -2.150000e+01 4.699575e-01 2.922888e+01
      vertex   -2.150000e+01 -8.673298e-01 2.913745e+01
      vertex   -2.570000e+01 6.374226e-02 2.928379e+01
    endloop
  endfacet
  facet normal 9.039462e-03 -2.251483e-01 -9.742826e-01
    outer loop
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
      vertex   -2.150000e+01 4.699575e-01 2.922888e+01
      vertex   -2.570000e+01 6.374226e-02 2.928379e+01
    endloop
  endfacet
  facet normal -2.429100e-02 -4.091281e-01 -9.121536e-01
    outer loop
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.150000e+01 4.699575e-01 2.922888e+01
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
    endloop
  endfacet
  facet normal 1.641573e-02 -6.191197e-01 -7.851250e-01
    outer loop
      vertex   -2.570000e+01 2.151688e+00 2.825043e+01
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.570000e+01 1.163558e+00 2.902963e+01
    endloop
  endfacet
  facet normal -1.897131e-02 -8.012698e-01 -5.980024e-01
    outer loop
      vertex   -2.150000e+01 2.548515e+00 2.758547e+01
      vertex   -2.150000e+01 1.750747e+00 2.865441e+01
      vertex   -2.570000e+01 2.151688e+00 2.825043e+01
    endloop
  endfacet
  facet normal 9.039052e-03 -8.827694e-01 -4.697196e-01
    outer loop
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
      vertex   -2.150000e+01 2.548515e+00 2.758547e+01
      vertex   -2.570000e+01 2.151688e+00 2.825043e+01
    endloop
  endfacet
  facet normal -1.651475e-02 -9.814842e-01 -1.908301e-01
    outer loop
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.150000e+01 2.548515e+00 2.758547e+01
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
    endloop
  endfacet
  facet normal 1.394405e-02 -9.989300e-01 -4.409614e-02
    outer loop
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.570000e+01 2.684640e+00 2.724882e+01
    endloop
  endfacet
  facet normal -1.151453e-02 -9.526049e-01 3.039924e-01
    outer loop
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.150000e+01 2.781247e+00 2.638848e+01
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
    endloop
  endfacet
  facet normal 5.855119e-03 -9.305760e-01 3.660519e-01
    outer loop
      vertex   -2.570000e+01 2.260143e+00 2.487783e+01
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.570000e+01 2.735913e+00 2.608733e+01
    endloop
  endfacet
  facet normal -1.523691e-02 -6.682124e-01 7.438145e-01
    outer loop
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.570000e+01 2.260143e+00 2.487783e+01
    endloop
  endfacet
  facet normal 1.438981e-02 -7.342096e-01 6.787703e-01
    outer loop
      vertex   -2.150000e+01 1.564485e+00 2.419430e+01
      vertex   -2.150000e+01 2.353350e+00 2.504760e+01
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
    endloop
  endfacet
  facet normal -4.875234e-03 -3.813811e-01 9.244051e-01
    outer loop
      vertex   -2.150000e+01 5.624098e-01 2.378088e+01
      vertex   -2.150000e+01 1.564485e+00 2.419430e+01
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
    endloop
  endfacet
  facet normal 2.719353e-02 -2.381780e-01 9.708407e-01
    outer loop
      vertex   -2.570000e+01 -8.826345e-03 2.375838e+01
      vertex   -2.150000e+01 5.624098e-01 2.378088e+01
      vertex   -2.570000e+01 1.398309e+00 2.410359e+01
    endloop
  endfacet
endsolid
```

The finished setup can be seen in the pictures below. Some rebars were additionally cut in half and spread appart to demonstrate damages.
<p align=center><img src="images/Setup/spacers.png" height="480">
<img src="images/Setup/setup.png" height="480">
</p>


### Installation

1. Clone the repo
    ```sh
    git clone https://github.com/DTU-PAS/Rebar-segmentation-Ransac.git
    ```
2. Switch the branch
    ```sh
    git switch ros2
    ```
3. Compile the package
    ```sh
    colcon build
    ```
4. Open 2 terminal windows with the following commands
    ```sh
    ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true pointcloud.enable:=true
    ```
    ```sh
    ros2 run rebarsegmentation ransac_node
    ```


<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Results

<p align=center>
<img src="images/Results/Screenshot from 2024-08-13 15-20-29.png" height="480">
<img src="images/Results/Screenshot from 2024-08-13 15-23-08.png" height="480">
<img src="images/Results/Screenshot from 2024-08-13 15-23-38.png" height="480">
<img src="images/Results/Screenshot from 2024-08-13 15-24-11.png" height="480">
</p>

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Lars Arnold Dethlefsen - larde@dtu.dk

<p align="right">(<a href="#readme-top">back to top</a>)</p>


[build]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/actions/workflows/ros2.yml/badge.svg
[build-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/actions/
[ROS]: https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white
[ROS-url]: https://wiki.ros.org/noetic
[OPENCV]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[OPENCV-url]: https://opencv.org/
[PCL-url]: https://pointclouds.org/
[UBUNTU]: https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white
[UBUNTU-url]: https://ubuntu.com/
[C++]: https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white
[C++-url]: https://en.wikipedia.org/wiki/C%2B%2B
