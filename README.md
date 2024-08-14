<a id="readme-top"></a>

<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/DTU-PAS/">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Rebar Segmentation using Ransac</h3>

  <p align="center">
  Rebar segmentation of exposed rebar on construction sites
    <br />
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac"><strong>Explore the docs »</strong></a>
    <br />
    <br />
    <a href="https://github.com/DTU-PAS/Rebar-segmentation-Ransac">View Demo</a>
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

This project uses a poincloud and a depth image to segment exposed rebars and detect potential damages. The 

The sensor used for development is the Intel Realsense D435i. However, the code can be easily be adapted to other sensors such as the D455, L515 or Roboception rc_visard 160 color. 

<p align="right">(<a href="#readme-top">back to top</a>)</p>



## Built With
[![build][build]][build-url] 

[![ROS][ROS]][ROS-url] 
Noetic 1.16.0

[![OPENCV][OPENCV]][OPENCV-url]
4.2.0

<p align=left><img src="images/pcl.png" height="20">
1.10.0
</p>

[![UBUNTU][UBUNTU]][UBUNTU-url]
20.04.6 LTS

[![C++][C++]][C++-url]
201402L C++14

<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- GETTING STARTED -->
## Getting Started
### Prerequisites

#### Software: 
* ROS: [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* Intel Realsense ROS1 Wrapper (ros1-legacy): [Method 1: The ROS distribution:](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file#method-1-the-ros-distribution)
* Clone the repository: [HTTPS](https://github.com/DTU-PAS/Rebar-segmentation-Ransac.git) or [SSH](git@github.com:DTU-PAS/Rebar-segmentation-Ransac.git)

#### Hardware
* Camera: Connect the camera sensor using the OEM cable. 3rd party cables have shown to be unreliable at times but can work aswell.

* Test Setup: ~11mm rebar mounted on a wooden board with ~30mm 3d printed spacers. The size of the grid pattern is ~20cm. The camera should be placed ~40-60cm in front of the board.

The Spacers were designed in DesignSpark. The design file aswell as an STL file are included in the repo.

```stl
solid cube_corner
  facet normal 0.0 -1.0 0.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 1.0 0.0 0.0
      vertex 0.0 0.0 1.0
    endloop
  endfacet
  facet normal 0.0 0.0 -1.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 0.0 1.0 0.0
      vertex 1.0 0.0 0.0
    endloop
  endfacet
  facet normal -1.0 0.0 0.0
    outer loop
      vertex 0.0 0.0 0.0
      vertex 0.0 0.0 1.0
      vertex 0.0 1.0 0.0
    endloop
  endfacet
  facet normal 0.577 0.577 0.577
    outer loop
      vertex 1.0 0.0 0.0
      vertex 0.0 1.0 0.0
      vertex 0.0 0.0 1.0
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
2. Compile the package
   ```sh
   catkin_make
   ```
3. Open 2 terminal windows with the following commands
   ```sh
   roslaunch realsense2_camera rs_camera.launch align_depth:=True filters:=pointcloud
   ```
   ```sh
   roslaunch rebarsegmenation rebarsegmenation.launch
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


[build]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/actions/workflows/main.yml/badge.svg
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
