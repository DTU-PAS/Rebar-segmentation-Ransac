<a id="readme-top"></a>

[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<div align="center">
  <a href="https://github.com/DTU-PAS/">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">Rebar Segmentation using Ransac</h3>

  <p align="center">
  This project uses a poincloud and a depth image to segment exposed rebars and detect potential damages. The sensor used for development is the Intel Realsense D435i. However, the code can be easily be adapted to other sensors such as the D455, L515 or Roboception rc_visard 160 color. 
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
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contact">Contact</a></li>
  </ol>
</details>



<!-- ABOUT THE PROJECT -->
## About The Project

Overview: https://www.robetarme-project.eu/



<p align="right">(<a href="#readme-top">back to top</a>)</p>



## Built With

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

This is an example of how to list things you need to use the software and how to install them.

#### Install: 
* ROS: [Ubuntu install of ROS Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* Intel Realsense ROS1 Wrapper (ros1-legacy): [Method 1: The ROS distribution:](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy?tab=readme-ov-file#method-1-the-ros-distribution)
* Clone the repository: [HTTPS](https://github.com/DTU-PAS/Rebar-segmentation-Ransac.git) or [SSH](git@github.com:DTU-PAS/Rebar-segmentation-Ransac.git)

#### Hardware
Connect 

### Installation

_Below is an example of how you can instruct your audience on installing and setting up your app. This template doesn't rely on any external dependencies or services._

1. Clone the repo
   ```sh
   git clone https://github.com/DTU-PAS/Rebar-segmentation-Ransac.git
   ```
2. Compile the package
   ```sh
   catkin_make
   ```
3. Open 5 terminal windows with the following commands
   ```sh
   roslaunch realsense2_camera rs_camera.launch align_depth:=True filters:=pointcloud
   ```
   ```sh
   roslaunch zPotholeDetection OnlinePotholeDetection.launch
   ```


<p align="right">(<a href="#readme-top">back to top</a>)</p>



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- CONTACT -->
## Contact

Lars Arnold Dethlefsen - larde@dtu.dk

<p align="right">(<a href="#readme-top">back to top</a>)</p>


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/othneildrew/Best-README-Template.svg?style=for-the-badge
[contributors-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/othneildrew/Best-README-Template.svg?style=for-the-badge
[forks-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/network/members
[stars-shield]: https://img.shields.io/github/stars/othneildrew/Best-README-Template.svg?style=for-the-badge
[stars-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/stargazers
[issues-shield]: https://img.shields.io/github/issues/othneildrew/Best-README-Template.svg?style=for-the-badge
[issues-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/issues
[license-shield]: https://img.shields.io/github/license/othneildrew/Best-README-Template.svg?style=for-the-badge
[license-url]: https://github.com/DTU-PAS/Rebar-segmentation-Ransac/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/othneildrew
[product-screenshot]: images/screenshot.png
[ROS]: https://img.shields.io/badge/ROS-22314E?style=for-the-badge&logo=ROS&logoColor=white
[ROS-url]: https://wiki.ros.org/noetic
[OPENCV]: https://img.shields.io/badge/OpenCV-27338e?style=for-the-badge&logo=OpenCV&logoColor=white
[OPENCV-url]: https://opencv.org/
[PCL-url]: https://pointclouds.org/
[UBUNTU]: https://img.shields.io/badge/Ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white
[UBUNTU-url]: https://ubuntu.com/
[C++]: https://img.shields.io/badge/c++-%2300599C.svg?style=for-the-badge&logo=c%2B%2B&logoColor=white
[C++-url]: https://en.wikipedia.org/wiki/C%2B%2B