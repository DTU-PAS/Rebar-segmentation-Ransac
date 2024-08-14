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
solid ASCII
  facet normal -9.993862e-01 -2.883627e-03 -3.491313e-02
    outer loop
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
    endloop
  endfacet
  facet normal -9.992333e-01 -5.980770e-05 -3.915131e-02
    outer loop
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
    endloop
  endfacet
  facet normal -9.914208e-01 2.549546e-03 -1.306843e-01
    outer loop
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
      vertex   -9.665106e+01 1.128536e+02 2.618583e+02
    endloop
  endfacet
  facet normal -9.901849e-01 -4.024526e-03 -1.397056e-01
    outer loop
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
    endloop
  endfacet
  facet normal -9.767826e-01 1.500931e-02 2.137063e-01
    outer loop
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
    endloop
  endfacet
  facet normal -9.949421e-01 -1.428450e-02 9.942924e-02
    outer loop
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
    endloop
  endfacet
  facet normal -9.951766e-01 -1.298008e-02 9.723735e-02
    outer loop
      vertex   -9.726719e+01 1.188361e+02 2.816492e+02
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
    endloop
  endfacet
  facet normal -9.988374e-01 9.128058e-04 4.819807e-02
    outer loop
      vertex   -9.748428e+01 1.193769e+02 2.771400e+02
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -9.726719e+01 1.188361e+02 2.816492e+02
    endloop
  endfacet
  facet normal -9.997195e-01 -1.084053e-02 2.105912e-02
    outer loop
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -9.748428e+01 1.193769e+02 2.771400e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
    endloop
  endfacet
  facet normal -9.999948e-01 3.057444e-03 1.045645e-03
    outer loop
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
      vertex   -9.748428e+01 1.193769e+02 2.771400e+02
    endloop
  endfacet
  facet normal -7.220740e-01 -6.062195e-03 6.917892e-01
    outer loop
      vertex   -9.493885e+01 1.174185e+02 2.897569e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.467372e+01 1.173596e+02 2.900331e+02
    endloop
  endfacet
  facet normal -7.866282e-01 -1.480120e-02 6.172495e-01
    outer loop
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.493885e+01 1.174185e+02 2.897569e+02
    endloop
  endfacet
  facet normal -8.384378e-01 -2.463498e-02 5.444402e-01
    outer loop
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
    endloop
  endfacet
  facet normal -8.391331e-01 -2.486398e-02 5.433576e-01
    outer loop
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
    endloop
  endfacet
  facet normal -9.071332e-01 1.869697e-02 4.204280e-01
    outer loop
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
    endloop
  endfacet
  facet normal -9.393403e-01 -2.695285e-03 3.429761e-01
    outer loop
      vertex   -9.647909e+01 1.128536e+02 2.864552e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
    endloop
  endfacet
  facet normal -9.734755e-01 8.456285e-03 2.286351e-01
    outer loop
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -9.647909e+01 1.128536e+02 2.864552e+02
    endloop
  endfacet
  facet normal -2.800181e-01 1.534945e-02 9.598720e-01
    outer loop
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -9.177265e+01 1.170639e+02 2.913825e+02
    endloop
  endfacet
  facet normal -2.578053e-01 2.257164e-02 9.659332e-01
    outer loop
      vertex   -9.297073e+01 1.128536e+02 2.911181e+02
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
    endloop
  endfacet
  facet normal -4.102357e-01 -6.574749e-04 9.119793e-01
    outer loop
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.297073e+01 1.128536e+02 2.911181e+02
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
    endloop
  endfacet
  facet normal -5.911535e-01 5.183398e-03 8.065424e-01
    outer loop
      vertex   -9.467372e+01 1.173596e+02 2.900331e+02
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
    endloop
  endfacet
  facet normal -6.203138e-01 -5.286697e-03 7.843359e-01
    outer loop
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.467372e+01 1.173596e+02 2.900331e+02
    endloop
  endfacet
  facet normal 6.214553e-01 9.756848e-05 7.834496e-01
    outer loop
      vertex   -8.902290e+01 1.171868e+02 2.908306e+02
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
    endloop
  endfacet
  facet normal 3.977976e-01 -2.268438e-03 9.174704e-01
    outer loop
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.902290e+01 1.171868e+02 2.908306e+02
    endloop
  endfacet
  facet normal 3.138118e-01 1.492217e-02 9.493679e-01
    outer loop
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -8.902290e+01 1.171868e+02 2.908306e+02
    endloop
  endfacet
  facet normal 2.777963e-01 8.553982e-03 9.606019e-01
    outer loop
      vertex   -9.038377e+01 1.128536e+02 2.913239e+02
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
    endloop
  endfacet
  facet normal -7.909107e-03 1.857179e-03 9.999670e-01
    outer loop
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -9.038377e+01 1.128536e+02 2.913239e+02
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
    endloop
  endfacet
  facet normal 5.237701e-02 -2.340988e-02 9.983530e-01
    outer loop
      vertex   -9.177265e+01 1.170639e+02 2.913825e+02
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
    endloop
  endfacet
  facet normal 9.657130e-01 -1.761167e-03 2.596062e-01
    outer loop
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
      vertex   -8.590304e+01 1.128536e+02 2.854036e+02
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
    endloop
  endfacet
  facet normal 9.574965e-01 -1.234909e-02 2.881804e-01
    outer loop
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -8.590304e+01 1.128536e+02 2.854036e+02
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
    endloop
  endfacet
  facet normal 9.166339e-01 8.629495e-03 3.996345e-01
    outer loop
      vertex   -8.718621e+01 1.175653e+02 2.890471e+02
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
    endloop
  endfacet
  facet normal 8.751314e-01 -1.459288e-02 4.836653e-01
    outer loop
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -8.718621e+01 1.175653e+02 2.890471e+02
    endloop
  endfacet
  facet normal 8.013528e-01 5.428894e-03 5.981674e-01
    outer loop
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -8.718621e+01 1.175653e+02 2.890471e+02
    endloop
  endfacet
  facet normal 6.441939e-01 -9.899603e-03 7.647981e-01
    outer loop
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
    endloop
  endfacet
  facet normal 9.999053e-01 3.765113e-03 1.323871e-02
    outer loop
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
    endloop
  endfacet
  facet normal 9.997772e-01 -4.744048e-03 2.056858e-02
    outer loop
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
    endloop
  endfacet
  facet normal 9.940373e-01 9.891538e-03 1.085910e-01
    outer loop
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
    endloop
  endfacet
  facet normal 9.933571e-01 3.574927e-03 1.150172e-01
    outer loop
      vertex   -8.590304e+01 1.128536e+02 2.854036e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
    endloop
  endfacet
  facet normal 9.882999e-01 -4.049888e-03 -1.524691e-01
    outer loop
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
    endloop
  endfacet
  facet normal 9.918230e-01 4.221083e-03 -1.275511e-01
    outer loop
      vertex   -8.535570e+01 1.199915e+02 2.661916e+02
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
    endloop
  endfacet
  facet normal 9.990795e-01 -9.948798e-03 -4.172760e-02
    outer loop
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
      vertex   -8.535570e+01 1.199915e+02 2.661916e+02
    endloop
  endfacet
  facet normal 9.989275e-01 -6.492678e-03 -4.584397e-02
    outer loop
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
    endloop
  endfacet
  facet normal 8.326785e-01 2.397848e-03 -5.537516e-01
    outer loop
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
    endloop
  endfacet
  facet normal 8.264019e-01 -1.758318e-04 -5.630806e-01
    outer loop
      vertex   -8.701311e+01 1.198485e+02 2.589888e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
    endloop
  endfacet
  facet normal 9.186442e-01 -1.727973e-04 -3.950859e-01
    outer loop
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -8.701311e+01 1.198485e+02 2.589888e+02
    endloop
  endfacet
  facet normal 9.202022e-01 -9.903574e-04 -3.914421e-01
    outer loop
      vertex   -8.638935e+01 1.128536e+02 2.604582e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
    endloop
  endfacet
  facet normal 9.586795e-01 1.280600e-03 -2.844854e-01
    outer loop
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
      vertex   -8.638935e+01 1.128536e+02 2.604582e+02
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
    endloop
  endfacet
  facet normal 9.662047e-01 7.894156e-03 -2.576550e-01
    outer loop
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
    endloop
  endfacet
  facet normal 6.574213e-02 1.640290e-03 -9.978353e-01
    outer loop
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.188366e+01 1.196831e+02 2.562851e+02
    endloop
  endfacet
  facet normal 3.543480e-02 8.384550e-03 -9.993368e-01
    outer loop
      vertex   -9.041229e+01 1.128536e+02 2.563309e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
    endloop
  endfacet
  facet normal 1.953445e-01 3.436569e-03 -9.807287e-01
    outer loop
      vertex   -9.009945e+01 1.128536e+02 2.563932e+02
      vertex   -9.041229e+01 1.128536e+02 2.563309e+02
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
    endloop
  endfacet
  facet normal 3.794294e-01 6.347090e-03 -9.251989e-01
    outer loop
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -9.009945e+01 1.128536e+02 2.563932e+02
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
    endloop
  endfacet
  facet normal 3.607441e-01 2.419430e-03 -9.326617e-01
    outer loop
      vertex   -8.884353e+01 1.197278e+02 2.569234e+02
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
    endloop
  endfacet
  facet normal 6.246884e-01 -3.309687e-03 -7.808671e-01
    outer loop
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -8.884353e+01 1.197278e+02 2.569234e+02
    endloop
  endfacet
  facet normal 6.275082e-01 -2.616947e-03 -7.786056e-01
    outer loop
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.884353e+01 1.197278e+02 2.569234e+02
    endloop
  endfacet
  facet normal -5.773859e-01 3.884022e-03 -8.164621e-01
    outer loop
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal -5.773396e-01 3.897470e-03 -8.164948e-01
    outer loop
      vertex   -9.331461e+01 1.197117e+02 2.566882e+02
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal -2.654985e-01 -4.798533e-03 -9.640993e-01
    outer loop
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.331461e+01 1.197117e+02 2.566882e+02
    endloop
  endfacet
  facet normal -2.712570e-01 -6.228876e-03 -9.624868e-01
    outer loop
      vertex   -9.188366e+01 1.196831e+02 2.562851e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.331461e+01 1.197117e+02 2.566882e+02
    endloop
  endfacet
  facet normal -8.501293e-04 9.766754e-01 2.147195e-01
    outer loop
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
      vertex   -8.902290e+01 1.171868e+02 2.908306e+02
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
    endloop
  endfacet
  facet normal -3.131684e-04 9.769849e-01 2.133083e-01
    outer loop
      vertex   -9.177265e+01 1.170639e+02 2.913825e+02
      vertex   -9.048400e+01 1.170790e+02 2.913153e+02
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
    endloop
  endfacet
  facet normal -4.356166e-05 9.775428e-01 2.107368e-01
    outer loop
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
      vertex   -8.902290e+01 1.171868e+02 2.908306e+02
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
    endloop
  endfacet
  facet normal 3.321014e-03 9.783899e-01 2.067416e-01
    outer loop
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
      vertex   -9.177265e+01 1.170639e+02 2.913825e+02
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
    endloop
  endfacet
  facet normal 9.302816e-03 9.798013e-01 1.997572e-01
    outer loop
      vertex   -9.493885e+01 1.174185e+02 2.897569e+02
      vertex   -9.467372e+01 1.173596e+02 2.900331e+02
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
    endloop
  endfacet
  facet normal 1.120122e-02 9.802474e-01 1.974575e-01
    outer loop
      vertex   -9.354033e+01 1.171793e+02 2.908650e+02
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
      vertex   -9.493885e+01 1.174185e+02 2.897569e+02
    endloop
  endfacet
  facet normal -4.734107e-04 9.814719e-01 1.916055e-01
    outer loop
      vertex   -8.718621e+01 1.175653e+02 2.890471e+02
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
    endloop
  endfacet
  facet normal 8.311503e-05 9.790347e-01 2.036933e-01
    outer loop
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
      vertex   -8.718621e+01 1.175653e+02 2.890471e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
    endloop
  endfacet
  facet normal 1.639174e-04 9.793018e-01 2.024053e-01
    outer loop
      vertex   -9.500464e+01 1.174360e+02 2.896735e+02
      vertex   -8.778116e+01 1.173992e+02 2.898456e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
    endloop
  endfacet
  facet normal -2.048735e-04 9.812733e-01 1.926205e-01
    outer loop
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
      vertex   -9.518353e+01 1.174925e+02 2.894006e+02
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
    endloop
  endfacet
  facet normal 1.772292e-04 9.841139e-01 1.775383e-01
    outer loop
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
      vertex   -8.636431e+01 1.179353e+02 2.871539e+02
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
    endloop
  endfacet
  facet normal -1.062435e-03 9.857764e-01 1.680586e-01
    outer loop
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
      vertex   -9.632771e+01 1.179811e+02 2.869101e+02
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
    endloop
  endfacet
  facet normal 4.189452e-03 9.897638e-01 1.426538e-01
    outer loop
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
      vertex   -8.594435e+01 1.182167e+02 2.855936e+02
    endloop
  endfacet
  facet normal 4.750546e-03 9.895250e-01 1.442832e-01
    outer loop
      vertex   -9.726719e+01 1.188361e+02 2.816492e+02
      vertex   -9.709816e+01 1.185879e+02 2.833460e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
    endloop
  endfacet
  facet normal -1.547713e-04 9.928832e-01 1.190923e-01
    outer loop
      vertex   -9.726719e+01 1.188361e+02 2.816492e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
      vertex   -9.748428e+01 1.193769e+02 2.771400e+02
    endloop
  endfacet
  facet normal 7.854567e-03 9.971268e-01 7.534218e-02
    outer loop
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
      vertex   -9.748428e+01 1.193769e+02 2.771400e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
    endloop
  endfacet
  facet normal 1.699959e-03 9.963956e-01 8.481053e-02
    outer loop
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
      vertex   -8.527124e+01 1.191137e+02 2.793503e+02
    endloop
  endfacet
  facet normal -5.896087e-04 9.993143e-01 3.702149e-02
    outer loop
      vertex   -9.748891e+01 1.198060e+02 2.714621e+02
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
    endloop
  endfacet
  facet normal 7.105432e-04 9.994428e-01 3.337151e-02
    outer loop
      vertex   -8.516172e+01 1.198351e+02 2.708732e+02
      vertex   -8.535570e+01 1.199915e+02 2.661916e+02
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
    endloop
  endfacet
  facet normal -1.375059e-03 9.999700e-01 -7.627086e-03
    outer loop
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
    endloop
  endfacet
  facet normal -4.574828e-04 9.999879e-01 -4.888036e-03
    outer loop
      vertex   -8.535570e+01 1.199915e+02 2.661916e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
      vertex   -9.729670e+01 1.199879e+02 2.665561e+02
    endloop
  endfacet
  facet normal 9.053127e-04 9.994342e-01 -3.362102e-02
    outer loop
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
    endloop
  endfacet
  facet normal -7.797994e-04 9.996199e-01 -2.755723e-02
    outer loop
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
      vertex   -8.580387e+01 1.199743e+02 2.627061e+02
    endloop
  endfacet
  facet normal 7.437283e-05 9.987044e-01 -5.088826e-02
    outer loop
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
    endloop
  endfacet
  facet normal -9.350733e-04 9.985058e-01 -5.463825e-02
    outer loop
      vertex   -8.701311e+01 1.198485e+02 2.589888e+02
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
    endloop
  endfacet
  facet normal 4.121322e-05 9.990056e-01 -4.458502e-02
    outer loop
      vertex   -8.643803e+01 1.199082e+02 2.603260e+02
      vertex   -8.701311e+01 1.198485e+02 2.589888e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
    endloop
  endfacet
  facet normal 2.856173e-03 9.981616e-01 -6.054084e-02
    outer loop
      vertex   -9.188366e+01 1.196831e+02 2.562851e+02
      vertex   -9.331461e+01 1.197117e+02 2.566882e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal -3.178990e-04 9.977605e-01 -6.688737e-02
    outer loop
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
      vertex   -9.188366e+01 1.196831e+02 2.562851e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal -6.004002e-04 9.976953e-01 -6.785056e-02
    outer loop
      vertex   -8.884353e+01 1.197278e+02 2.569234e+02
      vertex   -9.020824e+01 1.196911e+02 2.563955e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal -2.815201e-05 9.979710e-01 -6.366963e-02
    outer loop
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
      vertex   -8.790534e+01 1.197760e+02 2.576794e+02
      vertex   -8.884353e+01 1.197278e+02 2.569234e+02
    endloop
  endfacet
  facet normal -9.559499e-01 8.528455e-05 -2.935298e-01
    outer loop
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -9.665106e+01 1.128536e+02 2.618583e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
    endloop
  endfacet
  facet normal -9.564854e-01 -4.847797e-04 -2.917798e-01
    outer loop
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -9.661955e+01 1.199522e+02 2.617577e+02
    endloop
  endfacet
  facet normal -8.907656e-01 5.260899e-03 -4.544325e-01
    outer loop
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
    endloop
  endfacet
  facet normal -8.621782e-01 1.868975e-02 -5.062602e-01
    outer loop
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -9.604173e+01 1.198879e+02 2.598636e+02
    endloop
  endfacet
  facet normal -7.780421e-01 -1.801332e-04 -6.282121e-01
    outer loop
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -9.479383e+01 1.197794e+02 2.577344e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.041229e+01 1.128536e+02 2.563309e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.459057e+01 1.128536e+02 2.497803e+02
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -8.459057e+01 1.128536e+02 2.497803e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.219082e+01 1.128536e+02 2.512783e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
      vertex   -8.638935e+01 1.128536e+02 2.604582e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.404069e+01 1.128536e+02 2.485009e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.807343e+01 1.128536e+02 2.562560e+02
      vertex   -7.891833e+01 1.128536e+02 2.548126e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.638935e+01 1.128536e+02 2.604582e+02
      vertex   -7.807343e+01 1.128536e+02 2.562560e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
      vertex   -7.807343e+01 1.128536e+02 2.562560e+02
      vertex   -8.638935e+01 1.128536e+02 2.604582e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
      vertex   -7.520707e+01 1.128536e+02 2.696024e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -8.595296e+01 1.128536e+02 2.619288e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -8.547558e+01 1.128536e+02 2.650232e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.520707e+01 1.128536e+02 2.696024e+02
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.038377e+01 1.128536e+02 2.913239e+02
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -9.038377e+01 1.128536e+02 2.913239e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.404069e+01 1.128536e+02 2.485009e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.972990e+01 1.128536e+02 2.536926e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -7.891833e+01 1.128536e+02 2.548126e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.701413e+01 1.128536e+02 2.589895e+02
      vertex   -7.972990e+01 1.128536e+02 2.536926e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.009945e+01 1.128536e+02 2.563932e+02
      vertex   -8.895720e+01 1.128536e+02 2.568616e+02
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.172426e+01 1.128536e+02 2.562843e+02
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -9.346951e+01 1.128536e+02 2.567649e+02
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.010847e+02 1.128536e+02 2.517679e+02
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -1.010847e+02 1.128536e+02 2.517679e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -9.472809e+01 1.128536e+02 2.576550e+02
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.550117e+01 1.128536e+02 2.983370e+02
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.978793e+01 1.128536e+02 2.911516e+02
      vertex   -8.550117e+01 1.128536e+02 2.983370e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.327047e+01 1.128536e+02 2.972094e+02
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.550117e+01 1.128536e+02 2.983370e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.327047e+01 1.128536e+02 2.972094e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -8.899161e+01 1.128536e+02 2.908064e+02
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.010957e+01 1.128536e+02 2.532541e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -7.972990e+01 1.128536e+02 2.536926e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
      vertex   -1.072000e+02 1.128536e+02 2.808092e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.075199e+02 1.128536e+02 2.721049e+02
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
      vertex   -8.515148e+01 1.128536e+02 2.720852e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.528823e+01 1.128536e+02 2.792031e+02
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.072000e+02 1.128536e+02 2.808092e+02
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.579833e+01 1.128536e+02 2.834219e+02
      vertex   -7.528823e+01 1.128536e+02 2.792031e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.041229e+01 1.128536e+02 2.563309e+02
      vertex   -9.009945e+01 1.128536e+02 2.563932e+02
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -9.041229e+01 1.128536e+02 2.563309e+02
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -9.735413e+01 1.128536e+02 2.799609e+02
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -7.579833e+01 1.128536e+02 2.834219e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.590304e+01 1.128536e+02 2.854036e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -8.532229e+01 1.128536e+02 2.803879e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.219082e+01 1.128536e+02 2.512783e+02
      vertex   -8.782904e+01 1.128536e+02 2.577641e+02
      vertex   -8.010957e+01 1.128536e+02 2.532541e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -9.789488e+01 1.128536e+02 2.980077e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.006555e+02 1.128536e+02 2.962476e+02
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.647909e+01 1.128536e+02 2.864552e+02
      vertex   -9.687252e+01 1.128536e+02 2.847801e+02
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -8.590304e+01 1.128536e+02 2.854036e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.788317e+01 1.128536e+02 2.906925e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.041648e+02 1.128536e+02 2.918922e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
      vertex   -1.041648e+02 1.128536e+02 2.918922e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.647909e+01 1.128536e+02 2.864552e+02
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
      vertex   -9.647909e+01 1.128536e+02 2.864552e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.789488e+01 1.128536e+02 2.980077e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.297073e+01 1.128536e+02 2.911181e+02
      vertex   -9.364954e+01 1.128536e+02 2.908128e+02
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.073034e+02 1.128536e+02 2.679186e+02
      vertex   -1.075199e+02 1.128536e+02 2.721049e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
      vertex   -1.073034e+02 1.128536e+02 2.679186e+02
      vertex   -9.750896e+01 1.128536e+02 2.726104e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -1.073034e+02 1.128536e+02 2.679186e+02
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.665106e+01 1.128536e+02 2.618583e+02
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -9.730725e+01 1.128536e+02 2.668364e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -9.665106e+01 1.128536e+02 2.618583e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -9.665106e+01 1.128536e+02 2.618583e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.049158e+02 1.128536e+02 2.573190e+02
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -9.586601e+01 1.128536e+02 2.881343e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
      vertex   -9.297073e+01 1.128536e+02 2.911181e+02
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
      vertex   -9.297073e+01 1.128536e+02 2.911181e+02
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -9.534851e+01 1.128536e+02 2.584234e+02
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.049158e+02 1.128536e+02 2.573190e+02
      vertex   -9.597107e+01 1.128536e+02 2.596437e+02
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -8.757992e+01 1.128536e+02 2.896173e+02
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.788317e+01 1.128536e+02 2.906925e+02
      vertex   -8.668261e+01 1.128536e+02 2.879937e+02
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.006555e+02 1.128536e+02 2.962476e+02
      vertex   -9.463768e+01 1.128536e+02 2.900313e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
      vertex   -9.225510e+01 1.128536e+02 2.913091e+02
    endloop
  endfacet
  facet normal 2.583781e-01 -2.971817e-02 -9.655867e-01
    outer loop
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.305886e+02 1.196907e+02 2.563909e+02
    endloop
  endfacet
  facet normal 3.904722e-01 3.394602e-03 -9.206085e-01
    outer loop
      vertex   -1.287009e+02 1.197451e+02 2.571918e+02
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.305886e+02 1.196907e+02 2.563909e+02
    endloop
  endfacet
  facet normal 5.720115e-01 -9.418551e-03 -8.201915e-01
    outer loop
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.287009e+02 1.197451e+02 2.571918e+02
    endloop
  endfacet
  facet normal 6.399973e-01 9.567879e-03 -7.683176e-01
    outer loop
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
      vertex   -1.287009e+02 1.197451e+02 2.571918e+02
    endloop
  endfacet
  facet normal 7.400836e-01 -1.563793e-03 -6.725131e-01
    outer loop
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
    endloop
  endfacet
  facet normal -7.457006e-01 -1.580310e-02 -6.660938e-01
    outer loop
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
    endloop
  endfacet
  facet normal -5.741232e-01 1.661997e-02 -8.186002e-01
    outer loop
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
    endloop
  endfacet
  facet normal -4.829931e-01 -6.779308e-03 -8.755979e-01
    outer loop
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
    endloop
  endfacet
  facet normal -2.349119e-01 9.448254e-03 -9.719708e-01
    outer loop
      vertex   -1.321996e+02 1.196817e+02 2.562663e+02
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
    endloop
  endfacet
  facet normal -1.522122e-01 -7.910861e-03 -9.883162e-01
    outer loop
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.321996e+02 1.196817e+02 2.562663e+02
    endloop
  endfacet
  facet normal 7.712672e-02 4.092153e-03 -9.970129e-01
    outer loop
      vertex   -1.305886e+02 1.196907e+02 2.563909e+02
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.321996e+02 1.196817e+02 2.562663e+02
    endloop
  endfacet
  facet normal -9.990111e-01 -5.207223e-03 -4.415538e-02
    outer loop
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
    endloop
  endfacet
  facet normal -9.998085e-01 6.962094e-03 -1.829129e-02
    outer loop
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
    endloop
  endfacet
  facet normal -9.837195e-01 -4.103741e-03 -1.796638e-01
    outer loop
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
    endloop
  endfacet
  facet normal -9.213555e-01 -6.306335e-03 -3.886699e-01
    outer loop
      vertex   -1.369550e+02 1.198745e+02 2.595366e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
    endloop
  endfacet
  facet normal -9.444260e-01 6.426778e-03 -3.286612e-01
    outer loop
      vertex   -1.376210e+02 1.199471e+02 2.614520e+02
      vertex   -1.369550e+02 1.198745e+02 2.595366e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
    endloop
  endfacet
  facet normal -9.868565e-01 -9.138059e-03 -1.613404e-01
    outer loop
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.376210e+02 1.199471e+02 2.614520e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
    endloop
  endfacet
  facet normal -8.262303e-01 7.538044e-03 -5.632821e-01
    outer loop
      vertex   -1.369550e+02 1.198745e+02 2.595366e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
    endloop
  endfacet
  facet normal -8.094600e-01 1.314994e-02 5.870277e-01
    outer loop
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
    endloop
  endfacet
  facet normal -9.243699e-01 -2.327933e-02 3.807864e-01
    outer loop
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
    endloop
  endfacet
  facet normal -9.430671e-01 -7.261613e-03 3.325234e-01
    outer loop
      vertex   -1.375161e+02 1.199233e+02 2.692541e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
    endloop
  endfacet
  facet normal -9.849359e-01 1.093735e-03 1.729167e-01
    outer loop
      vertex   -1.378809e+02 1.199797e+02 2.671760e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.375161e+02 1.199233e+02 2.692541e+02
    endloop
  endfacet
  facet normal -9.921514e-01 -1.091867e-02 1.245645e-01
    outer loop
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.378809e+02 1.199797e+02 2.671760e+02
    endloop
  endfacet
  facet normal -9.986356e-01 3.099024e-04 5.221969e-02
    outer loop
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.378809e+02 1.199797e+02 2.671760e+02
    endloop
  endfacet
  facet normal -2.537761e-01 1.013561e-02 9.672099e-01
    outer loop
      vertex   -1.338351e+02 1.196626e+02 2.739880e+02
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.317854e+02 1.196213e+02 2.745263e+02
    endloop
  endfacet
  facet normal -3.209299e-01 -6.210998e-03 9.470826e-01
    outer loop
      vertex   -1.342556e+02 1.128536e+02 2.738009e+02
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.338351e+02 1.196626e+02 2.739880e+02
    endloop
  endfacet
  facet normal -4.947495e-01 6.669788e-03 8.690101e-01
    outer loop
      vertex   -1.350043e+02 1.197111e+02 2.733220e+02
      vertex   -1.342556e+02 1.128536e+02 2.738009e+02
      vertex   -1.338351e+02 1.196626e+02 2.739880e+02
    endloop
  endfacet
  facet normal -6.730454e-01 -2.185730e-02 7.392781e-01
    outer loop
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.342556e+02 1.128536e+02 2.738009e+02
      vertex   -1.350043e+02 1.197111e+02 2.733220e+02
    endloop
  endfacet
  facet normal -6.836915e-01 -1.772904e-02 7.295558e-01
    outer loop
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.350043e+02 1.197111e+02 2.733220e+02
    endloop
  endfacet
  facet normal 6.405241e-01 7.250899e-03 7.679039e-01
    outer loop
      vertex   -1.280801e+02 1.196883e+02 2.736399e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
      vertex   -1.267940e+02 1.197608e+02 2.725664e+02
    endloop
  endfacet
  facet normal 4.957381e-01 -1.751929e-02 8.682953e-01
    outer loop
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
      vertex   -1.280801e+02 1.196883e+02 2.736399e+02
    endloop
  endfacet
  facet normal 4.230971e-01 -4.070430e-04 9.060842e-01
    outer loop
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.280801e+02 1.196883e+02 2.736399e+02
    endloop
  endfacet
  facet normal 1.166346e-01 -1.560635e-04 9.931749e-01
    outer loop
      vertex   -1.317854e+02 1.196213e+02 2.745263e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
    endloop
  endfacet
  facet normal 7.289913e-02 -1.589135e-02 9.972127e-01
    outer loop
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.317854e+02 1.196213e+02 2.745263e+02
    endloop
  endfacet
  facet normal 9.984949e-01 -1.354112e-02 5.314751e-02
    outer loop
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.247137e+02 1.199972e+02 2.657742e+02
    endloop
  endfacet
  facet normal 9.990062e-01 -9.569726e-03 4.353227e-02
    outer loop
      vertex   -1.249217e+02 1.128536e+02 2.685278e+02
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
    endloop
  endfacet
  facet normal 9.912170e-01 -2.491973e-04 1.322456e-01
    outer loop
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
      vertex   -1.249217e+02 1.128536e+02 2.685278e+02
    endloop
  endfacet
  facet normal 9.359261e-01 1.316627e-03 3.521941e-01
    outer loop
      vertex   -1.252377e+02 1.199017e+02 2.698366e+02
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
    endloop
  endfacet
  facet normal 9.689431e-01 -2.153228e-03 2.472745e-01
    outer loop
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
      vertex   -1.252377e+02 1.199017e+02 2.698366e+02
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
    endloop
  endfacet
  facet normal 9.692974e-01 -2.444490e-03 2.458791e-01
    outer loop
      vertex   -1.249217e+02 1.128536e+02 2.685278e+02
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
    endloop
  endfacet
  facet normal 8.329471e-01 -2.260410e-02 5.528908e-01
    outer loop
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.267940e+02 1.197608e+02 2.725664e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
    endloop
  endfacet
  facet normal 7.669542e-01 2.308549e-02 6.412865e-01
    outer loop
      vertex   -1.259015e+02 1.128536e+02 2.713772e+02
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
    endloop
  endfacet
  facet normal 9.129369e-01 -5.047180e-03 4.080696e-01
    outer loop
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.259015e+02 1.128536e+02 2.713772e+02
    endloop
  endfacet
  facet normal -8.274101e-05 9.974843e-01 -7.088817e-02
    outer loop
      vertex   -1.305886e+02 1.196907e+02 2.563909e+02
      vertex   -1.321996e+02 1.196817e+02 2.562663e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
    endloop
  endfacet
  facet normal 2.993052e-04 9.980387e-01 -6.259911e-02
    outer loop
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
    endloop
  endfacet
  facet normal 1.706544e-04 9.976803e-01 -6.807381e-02
    outer loop
      vertex   -1.287009e+02 1.197451e+02 2.571918e+02
      vertex   -1.305886e+02 1.196907e+02 2.563909e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
    endloop
  endfacet
  facet normal -7.059052e-04 9.982667e-01 -5.884814e-02
    outer loop
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
      vertex   -1.287009e+02 1.197451e+02 2.571918e+02
      vertex   -1.339665e+02 1.197120e+02 2.566936e+02
    endloop
  endfacet
  facet normal -1.864327e-04 9.992791e-01 -3.796358e-02
    outer loop
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
      vertex   -1.369550e+02 1.198745e+02 2.595366e+02
      vertex   -1.376210e+02 1.199471e+02 2.614520e+02
    endloop
  endfacet
  facet normal 1.041617e-04 9.986953e-01 -5.106632e-02
    outer loop
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
      vertex   -1.369550e+02 1.198745e+02 2.595366e+02
    endloop
  endfacet
  facet normal -1.881821e-04 9.986332e-01 -5.226548e-02
    outer loop
      vertex   -1.266449e+02 1.198526e+02 2.590545e+02
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
    endloop
  endfacet
  facet normal -9.215279e-04 9.989848e-01 -4.503880e-02
    outer loop
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
      vertex   -1.266449e+02 1.198526e+02 2.590545e+02
      vertex   -1.359826e+02 1.198014e+02 2.581094e+02
    endloop
  endfacet
  facet normal 2.008065e-03 9.997337e-01 -2.299061e-02
    outer loop
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
      vertex   -1.376210e+02 1.199471e+02 2.614520e+02
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
    endloop
  endfacet
  facet normal -6.838476e-04 9.994346e-01 -3.361557e-02
    outer loop
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.252311e+02 1.199606e+02 2.619869e+02
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
    endloop
  endfacet
  facet normal -7.565856e-04 9.999789e-01 -6.450991e-03
    outer loop
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
    endloop
  endfacet
  facet normal 3.845700e-04 9.998807e-01 -1.544269e-02
    outer loop
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
      vertex   -1.252311e+02 1.199606e+02 2.619869e+02
      vertex   -1.378298e+02 1.199768e+02 2.627272e+02
    endloop
  endfacet
  facet normal 5.615132e-04 9.999440e-01 1.057165e-02
    outer loop
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
      vertex   -1.378809e+02 1.199797e+02 2.671760e+02
    endloop
  endfacet
  facet normal -2.415241e-04 9.998792e-01 1.554071e-02
    outer loop
      vertex   -1.247137e+02 1.199972e+02 2.657742e+02
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
    endloop
  endfacet
  facet normal -1.161365e-04 1.000000e+00 1.158795e-04
    outer loop
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
      vertex   -1.379598e+02 1.199957e+02 2.656665e+02
      vertex   -1.247137e+02 1.199972e+02 2.657742e+02
    endloop
  endfacet
  facet normal -2.192382e-04 9.996304e-01 2.718645e-02
    outer loop
      vertex   -1.378809e+02 1.199797e+02 2.671760e+02
      vertex   -1.375161e+02 1.199233e+02 2.692541e+02
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
    endloop
  endfacet
  facet normal -4.642408e-04 9.996857e-01 2.506463e-02
    outer loop
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
      vertex   -1.248214e+02 1.199659e+02 2.677894e+02
      vertex   -1.375161e+02 1.199233e+02 2.692541e+02
    endloop
  endfacet
  facet normal 3.288839e-04 9.991224e-01 4.188461e-02
    outer loop
      vertex   -1.375161e+02 1.199233e+02 2.692541e+02
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
    endloop
  endfacet
  facet normal -8.589073e-04 9.993616e-01 3.571616e-02
    outer loop
      vertex   -1.252377e+02 1.199017e+02 2.698366e+02
      vertex   -1.249377e+02 1.199440e+02 2.686617e+02
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
    endloop
  endfacet
  facet normal -9.093341e-05 9.990467e-01 4.365513e-02
    outer loop
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.252377e+02 1.199017e+02 2.698366e+02
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
    endloop
  endfacet
  facet normal 8.461674e-05 9.975491e-01 6.996965e-02
    outer loop
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
      vertex   -1.267940e+02 1.197608e+02 2.725664e+02
    endloop
  endfacet
  facet normal -7.438642e-03 9.982624e-01 5.845328e-02
    outer loop
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
      vertex   -1.280801e+02 1.196883e+02 2.736399e+02
      vertex   -1.267940e+02 1.197608e+02 2.725664e+02
    endloop
  endfacet
  facet normal -1.086883e-04 9.970353e-01 7.694585e-02
    outer loop
      vertex   -1.338351e+02 1.196626e+02 2.739880e+02
      vertex   -1.317854e+02 1.196213e+02 2.745263e+02
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
    endloop
  endfacet
  facet normal 1.579487e-04 9.973830e-01 7.229862e-02
    outer loop
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
      vertex   -1.350043e+02 1.197111e+02 2.733220e+02
      vertex   -1.338351e+02 1.196626e+02 2.739880e+02
    endloop
  endfacet
  facet normal 1.512020e-03 9.979452e-01 6.405548e-02
    outer loop
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
      vertex   -1.350043e+02 1.197111e+02 2.733220e+02
      vertex   -1.293711e+02 1.196434e+02 2.742427e+02
    endloop
  endfacet
  facet normal -2.054264e-04 9.984707e-01 5.528250e-02
    outer loop
      vertex   -1.267940e+02 1.197608e+02 2.725664e+02
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
    endloop
  endfacet
  facet normal 1.831911e-04 9.983613e-01 5.722469e-02
    outer loop
      vertex   -1.369126e+02 1.198514e+02 2.709640e+02
      vertex   -1.356262e+02 1.197493e+02 2.727401e+02
      vertex   -1.255759e+02 1.198624e+02 2.707355e+02
    endloop
  endfacet
  facet normal 7.577854e-01 -1.174896e-04 -6.525039e-01
    outer loop
      vertex   -1.266449e+02 1.198526e+02 2.590545e+02
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
      vertex   -1.270990e+02 1.198249e+02 2.585271e+02
    endloop
  endfacet
  facet normal 8.345178e-01 -5.896190e-03 -5.509495e-01
    outer loop
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
      vertex   -1.266449e+02 1.198526e+02 2.590545e+02
    endloop
  endfacet
  facet normal 8.399948e-01 -7.579262e-03 -5.425415e-01
    outer loop
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
    endloop
  endfacet
  facet normal 9.205197e-01 8.971257e-03 -3.905932e-01
    outer loop
      vertex   -1.252311e+02 1.199606e+02 2.619869e+02
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
      vertex   -1.261688e+02 1.198855e+02 2.597753e+02
    endloop
  endfacet
  facet normal 9.480781e-01 -9.950862e-03 -3.178820e-01
    outer loop
      vertex   -1.250340e+02 1.128536e+02 2.627971e+02
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
      vertex   -1.252311e+02 1.199606e+02 2.619869e+02
    endloop
  endfacet
  facet normal 9.833426e-01 6.558833e-03 -1.816433e-01
    outer loop
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
      vertex   -1.250340e+02 1.128536e+02 2.627971e+02
      vertex   -1.252311e+02 1.199606e+02 2.619869e+02
    endloop
  endfacet
  facet normal 9.913392e-01 -4.899075e-03 -1.312346e-01
    outer loop
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.250340e+02 1.128536e+02 2.627971e+02
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
    endloop
  endfacet
  facet normal 9.985419e-01 2.435419e-04 -5.398187e-02
    outer loop
      vertex   -1.247137e+02 1.199972e+02 2.657742e+02
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.247889e+02 1.199974e+02 2.643820e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
      vertex   -1.357778e+02 1.128536e+02 2.812942e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.236629e+02 1.128536e+02 2.501765e+02
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.274842e+02 1.128536e+02 2.581195e+02
      vertex   -1.236629e+02 1.128536e+02 2.501765e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.260810e+02 1.128536e+02 2.491131e+02
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.236629e+02 1.128536e+02 2.501765e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.260810e+02 1.128536e+02 2.491131e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.357778e+02 1.128536e+02 2.812942e+02
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.329444e+02 1.128536e+02 2.483819e+02
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.291810e+02 1.128536e+02 2.810693e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.347096e+02 1.128536e+02 2.486036e+02
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.329444e+02 1.128536e+02 2.483819e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.347096e+02 1.128536e+02 2.486036e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.285073e+02 1.128536e+02 2.808243e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.291810e+02 1.128536e+02 2.810693e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.347096e+02 1.128536e+02 2.486036e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.274599e+02 1.128536e+02 2.803344e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.285073e+02 1.128536e+02 2.808243e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.264038e+02 1.128536e+02 2.796764e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.274599e+02 1.128536e+02 2.803344e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.328729e+02 1.128536e+02 2.913542e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.384173e+02 1.128536e+02 2.981731e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.384173e+02 1.128536e+02 2.981731e+02
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.318423e+02 1.128536e+02 2.562659e+02
      vertex   -1.290835e+02 1.128536e+02 2.570041e+02
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.328729e+02 1.128536e+02 2.913542e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.335801e+02 1.128536e+02 2.565335e+02
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.393483e+02 1.128536e+02 2.502708e+02
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.371596e+02 1.128536e+02 2.858849e+02
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.264038e+02 1.128536e+02 2.796764e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.250340e+02 1.128536e+02 2.627971e+02
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.363123e+02 1.128536e+02 2.888378e+02
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
      vertex   -1.363123e+02 1.128536e+02 2.888378e+02
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.404915e+02 1.128536e+02 2.792846e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.420764e+02 1.128536e+02 2.779608e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.404915e+02 1.128536e+02 2.792846e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.342556e+02 1.128536e+02 2.738009e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.259015e+02 1.128536e+02 2.713772e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.250340e+02 1.128536e+02 2.627971e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
      vertex   -1.258475e+02 1.128536e+02 2.603710e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
      vertex   -1.293768e+02 1.128536e+02 2.742423e+02
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.273109e+02 1.128536e+02 2.730629e+02
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.393483e+02 1.128536e+02 2.502708e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.352511e+02 1.128536e+02 2.574553e+02
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.437046e+02 1.128536e+02 2.539164e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.247354e+02 1.128536e+02 2.977665e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.247354e+02 1.128536e+02 2.977665e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.469279e+02 1.128536e+02 2.694970e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.139897e+02 1.128536e+02 2.729216e+02
      vertex   -1.142001e+02 1.128536e+02 2.680310e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.450958e+02 1.128536e+02 2.559302e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.437046e+02 1.128536e+02 2.539164e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.142001e+02 1.128536e+02 2.680310e+02
      vertex   -1.147521e+02 1.128536e+02 2.640201e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
      vertex   -1.259015e+02 1.128536e+02 2.713772e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.469279e+02 1.128536e+02 2.694970e+02
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
      vertex   -1.363123e+02 1.128536e+02 2.888378e+02
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.147521e+02 1.128536e+02 2.640201e+02
      vertex   -1.152208e+02 1.128536e+02 2.618900e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.268534e+02 1.128536e+02 2.588137e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
      vertex   -1.328729e+02 1.128536e+02 2.913542e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.323183e+02 1.128536e+02 2.744574e+02
      vertex   -1.342556e+02 1.128536e+02 2.738009e+02
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.450958e+02 1.128536e+02 2.559302e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
      vertex   -1.367249e+02 1.128536e+02 2.591053e+02
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
      vertex   -1.420764e+02 1.128536e+02 2.779608e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.375857e+02 1.128536e+02 2.689023e+02
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
      vertex   -1.363552e+02 1.128536e+02 2.718894e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.152208e+02 1.128536e+02 2.618900e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.249217e+02 1.128536e+02 2.685278e+02
      vertex   -1.253072e+02 1.128536e+02 2.700478e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.239723e+02 1.128536e+02 2.785219e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.262512e+02 1.128536e+02 2.883359e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.183710e+02 1.128536e+02 2.923696e+02
      vertex   -1.262512e+02 1.128536e+02 2.883359e+02
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
      vertex   -1.262512e+02 1.128536e+02 2.883359e+02
      vertex   -1.183710e+02 1.128536e+02 2.923696e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
      vertex   -1.139897e+02 1.128536e+02 2.729216e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
      vertex   -1.239723e+02 1.128536e+02 2.785219e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.148130e+02 1.128536e+02 2.833353e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.466127e+02 1.128536e+02 2.595722e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.148130e+02 1.128536e+02 2.833353e+02
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
      vertex   -1.247616e+02 1.128536e+02 2.648550e+02
      vertex   -1.249217e+02 1.128536e+02 2.685278e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
      vertex   -1.183710e+02 1.128536e+02 2.923696e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
      vertex   -1.374544e+02 1.128536e+02 2.608344e+02
      vertex   -1.466127e+02 1.128536e+02 2.595722e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.471473e+02 1.128536e+02 2.621216e+02
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
      vertex   -1.466127e+02 1.128536e+02 2.595722e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
      vertex   -1.471473e+02 1.128536e+02 2.621216e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.379406e+02 1.128536e+02 2.660753e+02
      vertex   -1.378882e+02 1.128536e+02 2.632098e+02
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.986717e-01 5.152547e-02
    outer loop
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
      vertex   -4.357162e+01 1.199710e+02 2.676379e+02
      vertex   -5.506548e+01 1.199710e+02 2.676379e+02
    endloop
  endfacet
  facet normal -8.806358e-03 9.994100e-01 3.319829e-02
    outer loop
      vertex   -4.582985e+01 1.198396e+02 2.709951e+02
      vertex   -4.357162e+01 1.199710e+02 2.676379e+02
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
    endloop
  endfacet
  facet normal 5.963292e-03 9.953414e-01 9.622873e-02
    outer loop
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -4.582985e+01 1.198396e+02 2.709951e+02
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
    endloop
  endfacet
  facet normal -3.435907e-03 9.950054e-01 9.976221e-02
    outer loop
      vertex   -5.506548e+01 1.188490e+02 2.815043e+02
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
    endloop
  endfacet
  facet normal 1.309767e-02 9.889370e-01 1.477570e-01
    outer loop
      vertex   -5.506548e+01 1.183677e+02 2.847252e+02
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -5.506548e+01 1.188490e+02 2.815043e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.188490e+02 2.815043e+02
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
      vertex   -5.506548e+01 1.188490e+02 2.815043e+02
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.196866e+02 2.731494e+02
      vertex   -5.506548e+01 1.199710e+02 2.676379e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
      vertex   -5.506548e+01 1.183677e+02 2.847252e+02
      vertex   -5.506548e+01 1.188490e+02 2.815043e+02
    endloop
  endfacet
  facet normal 8.297500e-01 7.145621e-07 5.581353e-01
    outer loop
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
    endloop
  endfacet
  facet normal 8.297502e-01 1.785836e-06 5.581350e-01
    outer loop
      vertex   -4.582985e+01 1.198396e+02 2.709951e+02
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 8.297485e-01 0.000000e+00 5.581375e-01
    outer loop
      vertex   -4.357162e+01 1.199710e+02 2.676379e+02
      vertex   -4.582985e+01 1.198396e+02 2.709951e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 8.297504e-01 0.000000e+00 5.581346e-01
    outer loop
      vertex   -5.225709e+01 1.189543e+02 2.805502e+02
      vertex   -5.506548e+01 1.183677e+02 2.847252e+02
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.506548e+01 1.199710e+02 2.676379e+02
      vertex   -4.357162e+01 1.199710e+02 2.676379e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.199710e+02 2.676379e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+01 1.128536e+02 2.676379e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+01 1.128536e+02 2.592136e+02
      vertex   -7.054979e+01 1.128536e+02 2.676379e+02
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
      vertex   -5.506548e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.993569e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.491523e+02
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -4.357162e+01 1.128536e+02 2.676379e+02
      vertex   -5.506548e+01 1.128536e+02 2.847252e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -2.134557e+02 1.174581e+02 2.895683e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+02 1.128536e+02 2.984701e+02
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -1.916617e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
      vertex   -1.916617e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+02 1.128536e+02 2.915124e+02
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+02 1.128536e+02 2.915124e+02
      vertex   -2.244379e+02 1.170346e+02 2.915124e+02
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal 8.600420e-01 -1.396968e-03 5.102213e-01
    outer loop
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
      vertex   -2.159969e+02 1.190939e+02 2.797139e+02
    endloop
  endfacet
  facet normal 8.265776e-01 -5.044081e-03 5.628002e-01
    outer loop
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
    endloop
  endfacet
  facet normal 8.262253e-01 -4.488605e-03 5.633219e-01
    outer loop
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
      vertex   -2.209847e+02 1.178682e+02 2.875105e+02
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
    endloop
  endfacet
  facet normal 7.738185e-01 -1.933114e-03 6.334044e-01
    outer loop
      vertex   -2.230574e+02 1.173561e+02 2.900411e+02
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
      vertex   -2.209847e+02 1.178682e+02 2.875105e+02
    endloop
  endfacet
  facet normal 7.712985e-01 -4.679352e-03 6.364564e-01
    outer loop
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
      vertex   -2.230574e+02 1.173561e+02 2.900411e+02
    endloop
  endfacet
  facet normal 7.289114e-01 3.223665e-03 6.846005e-01
    outer loop
      vertex   -2.244379e+02 1.170346e+02 2.915124e+02
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
      vertex   -2.230574e+02 1.173561e+02 2.900411e+02
    endloop
  endfacet
  facet normal 7.225776e-01 0.000000e+00 6.912898e-01
    outer loop
      vertex   -2.244379e+02 1.170346e+02 2.915124e+02
      vertex   -2.244379e+02 1.128536e+02 2.915124e+02
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
    endloop
  endfacet
  facet normal 9.689247e-01 4.044889e-03 2.473228e-01
    outer loop
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
    endloop
  endfacet
  facet normal 9.570986e-01 -1.277020e-02 2.894808e-01
    outer loop
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
    endloop
  endfacet
  facet normal 9.465284e-01 6.259124e-03 3.225599e-01
    outer loop
      vertex   -2.121507e+02 1.198201e+02 2.714612e+02
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
    endloop
  endfacet
  facet normal 9.315834e-01 -3.003949e-03 3.635152e-01
    outer loop
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
      vertex   -2.121507e+02 1.198201e+02 2.714612e+02
    endloop
  endfacet
  facet normal 9.149903e-01 1.015258e-02 4.033480e-01
    outer loop
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
      vertex   -2.121507e+02 1.198201e+02 2.714612e+02
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
    endloop
  endfacet
  facet normal 9.077398e-01 -2.981072e-04 4.195334e-01
    outer loop
      vertex   -2.146719e+02 1.128536e+02 2.771746e+02
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
    endloop
  endfacet
  facet normal 8.880920e-01 1.519638e-03 4.596632e-01
    outer loop
      vertex   -2.146719e+02 1.128536e+02 2.771746e+02
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
      vertex   -2.159969e+02 1.190939e+02 2.797139e+02
    endloop
  endfacet
  facet normal 8.706533e-01 -1.520438e-02 4.916621e-01
    outer loop
      vertex   -2.146719e+02 1.128536e+02 2.771746e+02
      vertex   -2.159969e+02 1.190939e+02 2.797139e+02
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
    endloop
  endfacet
  facet normal 9.990972e-01 0.000000e+00 4.248383e-02
    outer loop
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
      vertex   -2.076235e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 9.997911e-01 1.835989e-02 8.984745e-03
    outer loop
      vertex   -2.076326e+02 1.128536e+02 2.501731e+02
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
    endloop
  endfacet
  facet normal 9.981227e-01 -2.083626e-03 6.121080e-02
    outer loop
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -2.076326e+02 1.128536e+02 2.501731e+02
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
    endloop
  endfacet
  facet normal 9.944831e-01 -1.862902e-03 1.048802e-01
    outer loop
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
    endloop
  endfacet
  facet normal 9.928670e-01 -8.218076e-03 1.189439e-01
    outer loop
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
    endloop
  endfacet
  facet normal 9.836537e-01 8.086135e-03 1.798887e-01
    outer loop
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
    endloop
  endfacet
  facet normal 9.770361e-01 -9.270918e-03 2.128718e-01
    outer loop
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -2.076235e+02 1.189489e+02 2.491523e+02
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.983806e+02 1.128536e+02 2.491523e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal -9.490573e-01 9.342628e-03 -3.149650e-01
    outer loop
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
    endloop
  endfacet
  facet normal -9.639659e-01 -1.278721e-02 -2.657182e-01
    outer loop
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
    endloop
  endfacet
  facet normal -9.715138e-01 7.446636e-03 -2.368658e-01
    outer loop
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
    endloop
  endfacet
  facet normal -9.822267e-01 -5.323002e-03 -1.876230e-01
    outer loop
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
    endloop
  endfacet
  facet normal -9.898294e-01 6.744020e-03 -1.420996e-01
    outer loop
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
      vertex   -1.986816e+02 1.128536e+02 2.537089e+02
    endloop
  endfacet
  facet normal -9.936685e-01 -7.202221e-03 -1.121206e-01
    outer loop
      vertex   -1.985206e+02 1.192734e+02 2.518699e+02
      vertex   -1.986816e+02 1.128536e+02 2.537089e+02
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
    endloop
  endfacet
  facet normal -9.970390e-01 2.989328e-03 -7.683953e-02
    outer loop
      vertex   -1.986816e+02 1.128536e+02 2.537089e+02
      vertex   -1.985206e+02 1.192734e+02 2.518699e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
    endloop
  endfacet
  facet normal -9.986889e-01 -2.710892e-03 -5.111783e-02
    outer loop
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
      vertex   -1.985206e+02 1.192734e+02 2.518699e+02
    endloop
  endfacet
  facet normal -9.992611e-01 -0.000000e+00 -3.843594e-02
    outer loop
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -1.983806e+02 1.128536e+02 2.491523e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
      vertex   -2.009401e+02 1.128536e+02 2.649913e+02
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
      vertex   -2.097086e+02 1.128536e+02 2.637151e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
      vertex   -2.116321e+02 1.128536e+02 2.700748e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.146719e+02 1.128536e+02 2.771746e+02
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.129423e+02 1.128536e+02 2.734324e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.146719e+02 1.128536e+02 2.771746e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.111965e+02 1.128536e+02 2.865287e+02
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.111965e+02 1.128536e+02 2.865287e+02
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.180610e+02 1.128536e+02 2.831763e+02
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -2.083663e+02 1.128536e+02 2.575544e+02
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.986816e+02 1.128536e+02 2.537089e+02
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -1.993864e+02 1.128536e+02 2.586188e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -1.986816e+02 1.128536e+02 2.537089e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.076326e+02 1.128536e+02 2.501731e+02
      vertex   -2.077905e+02 1.128536e+02 2.527481e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
      vertex   -2.244379e+02 1.128536e+02 2.915124e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.210854e+02 1.128536e+02 2.876181e+02
      vertex   -2.234265e+02 1.128536e+02 2.904552e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
      vertex   -2.076326e+02 1.128536e+02 2.501731e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.983806e+02 1.128536e+02 2.491523e+02
      vertex   -2.076235e+02 1.128536e+02 2.491523e+02
      vertex   -1.984307e+02 1.128536e+02 2.504529e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.916617e+02 1.128536e+02 2.984701e+02
      vertex   -1.916617e+02 1.128536e+02 2.895683e+02
      vertex   -2.244379e+02 1.128536e+02 2.984701e+02
    endloop
  endfacet
  facet normal -8.018796e-01 0.000000e+00 -5.974856e-01
    outer loop
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.134557e+02 1.174581e+02 2.895683e+02
    endloop
  endfacet
  facet normal -8.026000e-01 7.635590e-04 -5.965171e-01
    outer loop
      vertex   -2.111965e+02 1.128536e+02 2.865287e+02
      vertex   -2.134557e+02 1.128536e+02 2.895683e+02
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
    endloop
  endfacet
  facet normal -8.280333e-01 -4.022229e-03 -5.606644e-01
    outer loop
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
      vertex   -2.111965e+02 1.128536e+02 2.865287e+02
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
    endloop
  endfacet
  facet normal -8.454130e-01 1.692947e-02 -5.338448e-01
    outer loop
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.111965e+02 1.128536e+02 2.865287e+02
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
    endloop
  endfacet
  facet normal -8.695289e-01 -8.318032e-03 -4.938119e-01
    outer loop
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
    endloop
  endfacet
  facet normal -8.881467e-01 7.883242e-03 -4.594924e-01
    outer loop
      vertex   -2.075443e+02 1.128536e+02 2.807450e+02
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
    endloop
  endfacet
  facet normal -9.066498e-01 -1.106566e-02 -4.217389e-01
    outer loop
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
    endloop
  endfacet
  facet normal -9.200700e-01 5.557294e-03 -3.917147e-01
    outer loop
      vertex   -2.050059e+02 1.128536e+02 2.758385e+02
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
    endloop
  endfacet
  facet normal -9.368430e-01 -6.989899e-03 -3.496804e-01
    outer loop
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
      vertex   -2.028930e+02 1.128536e+02 2.708756e+02
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
      vertex   -1.661501e+02 1.197988e+02 2.580540e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -1.851814e+02 1.128536e+02 2.580540e+02
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.851814e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.851814e+02 1.189489e+02 2.491523e+02
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
      vertex   -1.851814e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
      vertex   -1.851814e+02 1.189489e+02 2.491523e+02
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -1.515866e+02 1.128536e+02 2.491523e+02
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal -8.569938e-01 -5.045309e-03 -5.153019e-01
    outer loop
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
      vertex   -1.550308e+02 1.128536e+02 2.589531e+02
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
    endloop
  endfacet
  facet normal -8.836052e-01 9.481685e-03 -4.681367e-01
    outer loop
      vertex   -1.534372e+02 1.128536e+02 2.559451e+02
      vertex   -1.550308e+02 1.128536e+02 2.589531e+02
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
    endloop
  endfacet
  facet normal -9.221855e-01 -1.085587e-02 -3.865955e-01
    outer loop
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
      vertex   -1.534372e+02 1.128536e+02 2.559451e+02
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
    endloop
  endfacet
  facet normal -9.429744e-01 8.552263e-03 -3.327553e-01
    outer loop
      vertex   -1.521815e+02 1.128536e+02 2.523868e+02
      vertex   -1.534372e+02 1.128536e+02 2.559451e+02
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
    endloop
  endfacet
  facet normal -9.683410e-01 -1.182184e-02 -2.493510e-01
    outer loop
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
      vertex   -1.521815e+02 1.128536e+02 2.523868e+02
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
    endloop
  endfacet
  facet normal -9.834920e-01 4.848627e-03 -1.808866e-01
    outer loop
      vertex   -1.515866e+02 1.128536e+02 2.491523e+02
      vertex   -1.521815e+02 1.128536e+02 2.523868e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal -9.864541e-01 -0.000000e+00 -1.640376e-01
    outer loop
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
      vertex   -1.515866e+02 1.128536e+02 2.491523e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal -6.972212e-01 -4.544106e-03 -7.168417e-01
    outer loop
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
    endloop
  endfacet
  facet normal -7.116832e-01 4.418050e-03 -7.024866e-01
    outer loop
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
    endloop
  endfacet
  facet normal -7.588589e-01 6.395546e-03 -6.512237e-01
    outer loop
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
      vertex   -1.574234e+02 1.199666e+02 2.622346e+02
      vertex   -1.566913e+02 1.128536e+02 2.613116e+02
    endloop
  endfacet
  facet normal -7.396011e-01 -7.546143e-03 -6.730031e-01
    outer loop
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
      vertex   -1.574234e+02 1.199666e+02 2.622346e+02
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
    endloop
  endfacet
  facet normal -7.961923e-01 -3.440520e-03 -6.050339e-01
    outer loop
      vertex   -1.574234e+02 1.199666e+02 2.622346e+02
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
      vertex   -1.566913e+02 1.128536e+02 2.613116e+02
    endloop
  endfacet
  facet normal -8.176671e-01 5.939119e-03 -5.756606e-01
    outer loop
      vertex   -1.566913e+02 1.128536e+02 2.613116e+02
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
      vertex   -1.550308e+02 1.128536e+02 2.589531e+02
    endloop
  endfacet
  facet normal -8.173671e-01 -1.629381e-02 -5.758867e-01
    outer loop
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.736218e+02 1.191845e+02 2.789627e+02
    endloop
  endfacet
  facet normal -7.646793e-01 9.168969e-03 -6.443458e-01
    outer loop
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
      vertex   -1.736218e+02 1.191845e+02 2.789627e+02
    endloop
  endfacet
  facet normal -7.217381e-01 -3.694252e-03 -6.921564e-01
    outer loop
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
    endloop
  endfacet
  facet normal -7.021357e-01 5.004632e-03 -7.120256e-01
    outer loop
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
    endloop
  endfacet
  facet normal -6.949261e-01 -2.629814e-03 -7.190763e-01
    outer loop
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
    endloop
  endfacet
  facet normal -9.790271e-01 4.067240e-03 -2.036895e-01
    outer loop
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
    endloop
  endfacet
  facet normal -9.840543e-01 -8.174639e-03 -1.776805e-01
    outer loop
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
    endloop
  endfacet
  facet normal -9.252966e-01 4.317388e-03 -3.792196e-01
    outer loop
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
    endloop
  endfacet
  facet normal -9.080244e-01 -6.261024e-03 -4.188705e-01
    outer loop
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
    endloop
  endfacet
  facet normal -8.681628e-01 7.966164e-04 -4.962789e-01
    outer loop
      vertex   -1.736218e+02 1.191845e+02 2.789627e+02
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
    endloop
  endfacet
  facet normal -8.332343e-01 2.246495e-02 5.524635e-01
    outer loop
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
      vertex   -1.736923e+02 1.174104e+02 2.897917e+02
    endloop
  endfacet
  facet normal -8.637945e-01 2.789170e-03 5.038366e-01
    outer loop
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
    endloop
  endfacet
  facet normal -9.602739e-01 -9.161541e-03 2.789087e-01
    outer loop
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
    endloop
  endfacet
  facet normal -9.659853e-01 -1.534699e-02 2.581411e-01
    outer loop
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
    endloop
  endfacet
  facet normal -9.941512e-01 5.497158e-03 1.078567e-01
    outer loop
      vertex   -1.755582e+02 1.183090e+02 2.850740e+02
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
    endloop
  endfacet
  facet normal -9.999506e-01 -8.634169e-03 4.917772e-03
    outer loop
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
      vertex   -1.755582e+02 1.183090e+02 2.850740e+02
    endloop
  endfacet
  facet normal -9.996132e-01 -2.754745e-03 -2.767418e-02
    outer loop
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
      vertex   -1.755582e+02 1.183090e+02 2.850740e+02
    endloop
  endfacet
  facet normal -4.566253e-03 -9.946278e-03 9.999401e-01
    outer loop
      vertex   -1.696300e+02 1.170633e+02 2.913853e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.686295e+02 1.170623e+02 2.913898e+02
    endloop
  endfacet
  facet normal -4.236929e-02 -2.101496e-02 9.988810e-01
    outer loop
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.696300e+02 1.170633e+02 2.913853e+02
    endloop
  endfacet
  facet normal -1.445862e-01 7.042955e-03 9.894671e-01
    outer loop
      vertex   -1.709677e+02 1.171071e+02 2.911895e+02
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.696300e+02 1.170633e+02 2.913853e+02
    endloop
  endfacet
  facet normal -2.802978e-01 4.183343e-04 9.599130e-01
    outer loop
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.709677e+02 1.171071e+02 2.911895e+02
    endloop
  endfacet
  facet normal -4.014040e-01 2.491409e-02 9.155622e-01
    outer loop
      vertex   -1.732960e+02 1.173322e+02 2.901626e+02
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.709677e+02 1.171071e+02 2.911895e+02
    endloop
  endfacet
  facet normal -5.808722e-01 -5.573880e-02 8.120842e-01
    outer loop
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.732960e+02 1.173322e+02 2.901626e+02
    endloop
  endfacet
  facet normal -6.849989e-01 -1.676140e-02 7.283513e-01
    outer loop
      vertex   -1.736923e+02 1.174104e+02 2.897917e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
      vertex   -1.732960e+02 1.173322e+02 2.901626e+02
    endloop
  endfacet
  facet normal 6.391416e-01 -2.025895e-02 7.688222e-01
    outer loop
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
    endloop
  endfacet
  facet normal 4.418292e-01 2.404018e-02 8.967770e-01
    outer loop
      vertex   -1.669383e+02 1.171277e+02 2.910972e+02
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
    endloop
  endfacet
  facet normal 2.750286e-01 -3.721073e-02 9.607157e-01
    outer loop
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.669383e+02 1.171277e+02 2.910972e+02
    endloop
  endfacet
  facet normal 1.705235e-01 -1.074266e-04 9.853536e-01
    outer loop
      vertex   -1.686295e+02 1.170623e+02 2.913898e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.669383e+02 1.171277e+02 2.910972e+02
    endloop
  endfacet
  facet normal 9.931280e-01 0.000000e+00 1.170332e-01
    outer loop
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.622619e+02 1.128536e+02 2.835997e+02
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 9.949188e-01 -5.390968e-03 1.005359e-01
    outer loop
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 9.864539e-01 -1.851244e-04 1.640385e-01
    outer loop
      vertex   -1.625170e+02 1.182053e+02 2.856751e+02
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
    endloop
  endfacet
  facet normal 9.715416e-01 -3.523360e-03 2.368427e-01
    outer loop
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.625170e+02 1.182053e+02 2.856751e+02
    endloop
  endfacet
  facet normal 9.747161e-01 -3.477707e-04 2.234465e-01
    outer loop
      vertex   -1.627876e+02 1.128536e+02 2.867861e+02
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
    endloop
  endfacet
  facet normal 9.358724e-01 4.512551e-03 3.523103e-01
    outer loop
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.627876e+02 1.128536e+02 2.867861e+02
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
    endloop
  endfacet
  facet normal 9.244944e-01 1.370123e-02 3.809493e-01
    outer loop
      vertex   -1.635117e+02 1.176793e+02 2.884797e+02
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
    endloop
  endfacet
  facet normal 8.463664e-01 -3.470397e-03 5.325898e-01
    outer loop
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.635117e+02 1.176793e+02 2.884797e+02
    endloop
  endfacet
  facet normal 7.573955e-01 2.437038e-02 6.525014e-01
    outer loop
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.635117e+02 1.176793e+02 2.884797e+02
    endloop
  endfacet
  facet normal -9.950312e-02 0.000000e+00 9.950373e-01
    outer loop
      vertex   -1.605388e+02 1.185220e+02 2.837720e+02
      vertex   -1.527121e+02 1.128536e+02 2.845547e+02
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
    endloop
  endfacet
  facet normal -9.950283e-02 3.977757e-07 9.950373e-01
    outer loop
      vertex   -1.622619e+02 1.128536e+02 2.835997e+02
      vertex   -1.527121e+02 1.128536e+02 2.845547e+02
      vertex   -1.605388e+02 1.185220e+02 2.837720e+02
    endloop
  endfacet
  facet normal -9.950154e-02 0.000000e+00 9.950374e-01
    outer loop
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
      vertex   -1.622619e+02 1.128536e+02 2.835997e+02
      vertex   -1.605388e+02 1.185220e+02 2.837720e+02
    endloop
  endfacet
  facet normal -6.482177e-01 2.656962e-03 -7.614504e-01
    outer loop
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
      vertex   -1.583518e+02 1.128536e+02 2.961079e+02
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
    endloop
  endfacet
  facet normal -6.712660e-01 1.876714e-02 -7.409789e-01
    outer loop
      vertex   -1.565168e+02 1.128536e+02 2.944455e+02
      vertex   -1.583518e+02 1.128536e+02 2.961079e+02
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
    endloop
  endfacet
  facet normal -7.670204e-01 -1.157093e-02 -6.415184e-01
    outer loop
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
      vertex   -1.565168e+02 1.128536e+02 2.944455e+02
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
    endloop
  endfacet
  facet normal -8.050527e-01 1.362397e-02 -5.930468e-01
    outer loop
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
      vertex   -1.565168e+02 1.128536e+02 2.944455e+02
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
    endloop
  endfacet
  facet normal -8.526636e-01 -3.207941e-03 -5.224504e-01
    outer loop
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
    endloop
  endfacet
  facet normal -8.984066e-01 1.198139e-02 -4.390012e-01
    outer loop
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
    endloop
  endfacet
  facet normal -9.196979e-01 -1.089239e-02 -3.924757e-01
    outer loop
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
    endloop
  endfacet
  facet normal -9.618107e-01 1.634288e-02 -2.732271e-01
    outer loop
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
    endloop
  endfacet
  facet normal -9.657330e-01 8.586125e-03 -2.593955e-01
    outer loop
      vertex   -1.531785e+02 1.178889e+02 2.873992e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
    endloop
  endfacet
  facet normal -9.833297e-01 2.798444e-03 -1.818101e-01
    outer loop
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
      vertex   -1.531785e+02 1.178889e+02 2.873992e+02
    endloop
  endfacet
  facet normal -9.920832e-01 1.840572e-02 -1.242258e-01
    outer loop
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
    endloop
  endfacet
  facet normal -9.863943e-01 -0.000000e+00 -1.643965e-01
    outer loop
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
      vertex   -1.527121e+02 1.128536e+02 2.845547e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
    endloop
  endfacet
  facet normal -1.449730e-02 5.193744e-02 -9.985451e-01
    outer loop
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
    endloop
  endfacet
  facet normal -3.564085e-02 2.914251e-02 -9.989397e-01
    outer loop
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
      vertex   -1.679633e+02 1.149894e+02 2.993138e+02
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
    endloop
  endfacet
  facet normal -7.590701e-02 7.945502e-03 -9.970832e-01
    outer loop
      vertex   -1.679633e+02 1.149894e+02 2.993138e+02
      vertex   -1.668203e+02 1.150153e+02 2.992270e+02
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
    endloop
  endfacet
  facet normal -1.849594e-01 -1.316072e-02 -9.826580e-01
    outer loop
      vertex   -1.668203e+02 1.150153e+02 2.992270e+02
      vertex   -1.635742e+02 1.151959e+02 2.986136e+02
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
    endloop
  endfacet
  facet normal -1.667624e-01 8.846441e-03 -9.859574e-01
    outer loop
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
      vertex   -1.668203e+02 1.150153e+02 2.992270e+02
    endloop
  endfacet
  facet normal -3.235409e-01 3.164508e-02 -9.456849e-01
    outer loop
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
      vertex   -1.635742e+02 1.151959e+02 2.986136e+02
    endloop
  endfacet
  facet normal -3.225608e-01 3.055011e-02 -9.460556e-01
    outer loop
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal -4.853779e-01 -2.788839e-02 -8.738596e-01
    outer loop
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal -5.044633e-01 -5.074687e-03 -8.634183e-01
    outer loop
      vertex   -1.583518e+02 1.128536e+02 2.961079e+02
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
    endloop
  endfacet
  facet normal 6.964863e-01 1.513488e-02 -7.174105e-01
    outer loop
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
      vertex   -1.815025e+02 1.162394e+02 2.948031e+02
    endloop
  endfacet
  facet normal 6.821055e-01 3.995205e-03 -7.312429e-01
    outer loop
      vertex   -1.801562e+02 1.159113e+02 2.960571e+02
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.815025e+02 1.162394e+02 2.948031e+02
    endloop
  endfacet
  facet normal 5.429170e-01 1.014156e-03 -8.397857e-01
    outer loop
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.801562e+02 1.159113e+02 2.960571e+02
    endloop
  endfacet
  facet normal 5.844499e-01 -4.062758e-02 -8.104121e-01
    outer loop
      vertex   -1.790268e+02 1.128536e+02 2.968645e+02
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
    endloop
  endfacet
  facet normal 4.441551e-01 1.337228e-02 -8.958501e-01
    outer loop
      vertex   -1.763255e+02 1.128536e+02 2.982038e+02
      vertex   -1.790268e+02 1.128536e+02 2.968645e+02
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
    endloop
  endfacet
  facet normal 4.022843e-01 -2.264400e-02 -9.152347e-01
    outer loop
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
      vertex   -1.763255e+02 1.128536e+02 2.982038e+02
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
    endloop
  endfacet
  facet normal 2.449859e-01 -6.053769e-04 -9.695265e-01
    outer loop
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
      vertex   -1.763255e+02 1.128536e+02 2.982038e+02
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
    endloop
  endfacet
  facet normal 1.736586e-01 -9.873474e-02 -9.798439e-01
    outer loop
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
    endloop
  endfacet
  facet normal 1.135903e-01 -2.652628e-02 -9.931735e-01
    outer loop
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
    endloop
  endfacet
  facet normal 2.149264e-02 1.616331e-02 -9.996383e-01
    outer loop
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
    endloop
  endfacet
  facet normal 9.987984e-01 7.506518e-03 -4.842853e-02
    outer loop
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.851712e+02 1.128536e+02 2.850059e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
    endloop
  endfacet
  facet normal 9.931328e-01 -7.557655e-03 -1.167480e-01
    outer loop
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
    endloop
  endfacet
  facet normal 9.687096e-01 2.214381e-02 -2.472072e-01
    outer loop
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
    endloop
  endfacet
  facet normal 9.571952e-01 -1.332495e-03 -2.894402e-01
    outer loop
      vertex   -1.840521e+02 1.171387e+02 2.910473e+02
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
    endloop
  endfacet
  facet normal 8.589998e-01 1.722595e-02 -5.116861e-01
    outer loop
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
      vertex   -1.840521e+02 1.171387e+02 2.910473e+02
    endloop
  endfacet
  facet normal 9.057251e-01 -6.530046e-02 -4.188054e-01
    outer loop
      vertex   -1.835039e+02 1.128536e+02 2.921373e+02
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
    endloop
  endfacet
  facet normal 7.954159e-01 1.816073e-02 -6.057918e-01
    outer loop
      vertex   -1.835039e+02 1.128536e+02 2.921373e+02
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
    endloop
  endfacet
  facet normal 7.817866e-01 8.046424e-03 -6.234941e-01
    outer loop
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
      vertex   -1.815025e+02 1.162394e+02 2.948031e+02
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
    endloop
  endfacet
  facet normal 9.307476e-01 -8.401129e-03 3.655658e-01
    outer loop
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
      vertex   -1.841271e+02 1.128536e+02 2.794556e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
    endloop
  endfacet
  facet normal 9.623772e-01 7.780943e-03 2.716057e-01
    outer loop
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
      vertex   -1.841271e+02 1.128536e+02 2.794556e+02
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
    endloop
  endfacet
  facet normal 9.757122e-01 -5.080935e-03 2.189975e-01
    outer loop
      vertex   -1.849592e+02 1.186873e+02 2.826886e+02
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
    endloop
  endfacet
  facet normal 9.865342e-01 3.453105e-03 1.635189e-01
    outer loop
      vertex   -1.850795e+02 1.128536e+02 2.835379e+02
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
      vertex   -1.849592e+02 1.186873e+02 2.826886e+02
    endloop
  endfacet
  facet normal 9.981429e-01 -1.189472e-02 5.974360e-02
    outer loop
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
      vertex   -1.850795e+02 1.128536e+02 2.835379e+02
      vertex   -1.849592e+02 1.186873e+02 2.826886e+02
    endloop
  endfacet
  facet normal 9.979687e-01 -1.310218e-02 6.234387e-02
    outer loop
      vertex   -1.851712e+02 1.128536e+02 2.850059e+02
      vertex   -1.850795e+02 1.128536e+02 2.835379e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
    endloop
  endfacet
  facet normal 8.122597e-01 1.712152e-03 5.832934e-01
    outer loop
      vertex   -1.817685e+02 1.128536e+02 2.748989e+02
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
      vertex   -1.810216e+02 1.196693e+02 2.738388e+02
    endloop
  endfacet
  facet normal 8.613185e-01 -1.540068e-02 5.078318e-01
    outer loop
      vertex   -1.810216e+02 1.196693e+02 2.738388e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
      vertex   -1.817685e+02 1.128536e+02 2.748989e+02
    endloop
  endfacet
  facet normal 8.662816e-01 -9.898060e-03 4.994579e-01
    outer loop
      vertex   -1.817685e+02 1.128536e+02 2.748989e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
      vertex   -1.833329e+02 1.128536e+02 2.776124e+02
    endloop
  endfacet
  facet normal 9.183742e-01 -7.883804e-04 3.957123e-01
    outer loop
      vertex   -1.833329e+02 1.128536e+02 2.776124e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
      vertex   -1.841271e+02 1.128536e+02 2.794556e+02
    endloop
  endfacet
  facet normal 6.950639e-01 -1.471252e-04 7.189479e-01
    outer loop
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
    endloop
  endfacet
  facet normal 6.908854e-01 -7.664760e-03 7.229237e-01
    outer loop
      vertex   -1.780649e+02 1.128536e+02 2.703070e+02
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
    endloop
  endfacet
  facet normal 7.489624e-01 1.031402e-03 6.626117e-01
    outer loop
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
      vertex   -1.780649e+02 1.128536e+02 2.703070e+02
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
    endloop
  endfacet
  facet normal 7.715296e-01 -1.154441e-02 6.360887e-01
    outer loop
      vertex   -1.810216e+02 1.196693e+02 2.738388e+02
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
    endloop
  endfacet
  facet normal 7.051275e-01 6.602448e-03 7.090498e-01
    outer loop
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
      vertex   -1.681614e+02 1.199184e+02 2.606984e+02
    endloop
  endfacet
  facet normal 6.979676e-01 -2.183336e-04 7.161293e-01
    outer loop
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.851712e+02 1.128536e+02 2.850059e+02
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
      vertex   -1.851712e+02 1.128536e+02 2.850059e+02
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.850795e+02 1.128536e+02 2.835379e+02
      vertex   -1.851712e+02 1.128536e+02 2.850059e+02
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
      vertex   -1.850795e+02 1.128536e+02 2.835379e+02
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.755074e+02 1.128536e+02 2.858200e+02
      vertex   -1.850673e+02 1.128536e+02 2.871506e+02
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.841528e+02 1.128536e+02 2.907340e+02
      vertex   -1.835039e+02 1.128536e+02 2.921373e+02
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.668437e+02 1.128536e+02 2.992116e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.835039e+02 1.128536e+02 2.921373e+02
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
      vertex   -1.749401e+02 1.128536e+02 2.879432e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.790268e+02 1.128536e+02 2.968645e+02
      vertex   -1.763255e+02 1.128536e+02 2.982038e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
      vertex   -1.790268e+02 1.128536e+02 2.968645e+02
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.790268e+02 1.128536e+02 2.968645e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.816005e+02 1.128536e+02 2.946365e+02
      vertex   -1.801107e+02 1.128536e+02 2.960828e+02
      vertex   -1.742034e+02 1.128536e+02 2.892062e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.642873e+02 1.128536e+02 2.987792e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.655812e+02 1.128536e+02 2.905431e+02
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.583518e+02 1.128536e+02 2.961079e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.610133e+02 1.128536e+02 2.976629e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.833329e+02 1.128536e+02 2.776124e+02
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.817685e+02 1.128536e+02 2.748989e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.763255e+02 1.128536e+02 2.982038e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.833329e+02 1.128536e+02 2.776124e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.841271e+02 1.128536e+02 2.794556e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.833329e+02 1.128536e+02 2.776124e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.565168e+02 1.128536e+02 2.944455e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.583518e+02 1.128536e+02 2.961079e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.717456e+02 1.128536e+02 2.909642e+02
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.841271e+02 1.128536e+02 2.794556e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.755159e+02 1.128536e+02 2.840903e+02
      vertex   -1.750340e+02 1.128536e+02 2.817739e+02
      vertex   -1.847953e+02 1.128536e+02 2.818232e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.565168e+02 1.128536e+02 2.944455e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.640058e+02 1.128536e+02 2.892335e+02
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.727751e+02 1.128536e+02 2.991009e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.683964e+02 1.128536e+02 2.913490e+02
      vertex   -1.707729e+02 1.128536e+02 2.912482e+02
      vertex   -1.712486e+02 1.128536e+02 2.992755e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
      vertex   -1.550308e+02 1.128536e+02 2.589531e+02
      vertex   -1.534372e+02 1.128536e+02 2.559451e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.551593e+02 1.128536e+02 2.926027e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.627876e+02 1.128536e+02 2.867861e+02
      vertex   -1.632670e+02 1.128536e+02 2.880593e+02
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.534372e+02 1.128536e+02 2.559451e+02
      vertex   -1.521815e+02 1.128536e+02 2.523868e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.780649e+02 1.128536e+02 2.703070e+02
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
      vertex   -1.780649e+02 1.128536e+02 2.703070e+02
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
      vertex   -1.780649e+02 1.128536e+02 2.703070e+02
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
      vertex   -1.627876e+02 1.128536e+02 2.867861e+02
      vertex   -1.540792e+02 1.128536e+02 2.903923e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.627876e+02 1.128536e+02 2.867861e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.622619e+02 1.128536e+02 2.835997e+02
      vertex   -1.624785e+02 1.128536e+02 2.854376e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.527121e+02 1.128536e+02 2.845547e+02
      vertex   -1.622619e+02 1.128536e+02 2.835997e+02
      vertex   -1.531256e+02 1.128536e+02 2.870356e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.701306e+02 1.128536e+02 2.750655e+02
      vertex   -1.633593e+02 1.128536e+02 2.685217e+02
      vertex   -1.719948e+02 1.128536e+02 2.645060e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.566913e+02 1.128536e+02 2.613116e+02
      vertex   -1.550308e+02 1.128536e+02 2.589531e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.521815e+02 1.128536e+02 2.523868e+02
      vertex   -1.515866e+02 1.128536e+02 2.491523e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
      vertex   -1.724642e+02 1.128536e+02 2.774989e+02
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.596791e+02 1.128536e+02 2.647933e+02
      vertex   -1.566913e+02 1.128536e+02 2.613116e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.802215e+02 1.128536e+02 2.727447e+02
      vertex   -1.817685e+02 1.128536e+02 2.748989e+02
      vertex   -1.741953e+02 1.128536e+02 2.799559e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
      vertex   -1.851814e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.515866e+02 1.128536e+02 2.491523e+02
      vertex   -1.851814e+02 1.128536e+02 2.491523e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 7.959316e-01 0.000000e+00 6.053866e-01
    outer loop
      vertex   -1.681614e+02 1.199184e+02 2.606984e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
      vertex   -1.661501e+02 1.197988e+02 2.580540e+02
    endloop
  endfacet
  facet normal 7.935120e-01 -1.873566e-03 6.085518e-01
    outer loop
      vertex   -1.684442e+02 1.128536e+02 2.610454e+02
      vertex   -1.661501e+02 1.128536e+02 2.580540e+02
      vertex   -1.681614e+02 1.199184e+02 2.606984e+02
    endloop
  endfacet
  facet normal 1.096298e-01 0.000000e+00 9.939725e-01
    outer loop
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.464366e+02 1.179832e+02 2.869081e+02
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
    endloop
  endfacet
  facet normal 1.096298e-01 -0.000000e+00 9.939725e-01
    outer loop
      vertex   -1.371596e+02 1.128536e+02 2.858849e+02
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
    endloop
  endfacet
  facet normal -8.092629e-01 -5.160371e-03 5.874240e-01
    outer loop
      vertex   -1.363123e+02 1.128536e+02 2.888378e+02
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
    endloop
  endfacet
  facet normal -9.184718e-01 1.948516e-02 3.950062e-01
    outer loop
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.363123e+02 1.128536e+02 2.888378e+02
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
    endloop
  endfacet
  facet normal -8.987948e-01 3.270388e-03 4.383574e-01
    outer loop
      vertex   -1.367704e+02 1.178245e+02 2.877448e+02
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
    endloop
  endfacet
  facet normal -9.671261e-01 -1.113803e-03 2.542947e-01
    outer loop
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
      vertex   -1.367368e+02 1.128536e+02 2.878507e+02
      vertex   -1.367704e+02 1.178245e+02 2.877448e+02
    endloop
  endfacet
  facet normal -9.785301e-01 6.587739e-03 2.059985e-01
    outer loop
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
      vertex   -1.367704e+02 1.178245e+02 2.877448e+02
    endloop
  endfacet
  facet normal -9.844653e-01 0.000000e+00 1.755791e-01
    outer loop
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.371596e+02 1.128536e+02 2.858849e+02
      vertex   -1.369612e+02 1.128536e+02 2.869972e+02
    endloop
  endfacet
  facet normal -1.805396e-02 -7.536852e-03 9.998086e-01
    outer loop
      vertex   -1.328729e+02 1.128536e+02 2.913542e+02
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
    endloop
  endfacet
  facet normal -3.312783e-01 3.836349e-02 9.427528e-01
    outer loop
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
      vertex   -1.328729e+02 1.128536e+02 2.913542e+02
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
    endloop
  endfacet
  facet normal -2.726987e-01 6.108174e-03 9.620801e-01
    outer loop
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
    endloop
  endfacet
  facet normal -5.913151e-01 -3.881278e-04 8.064405e-01
    outer loop
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
      vertex   -1.343906e+02 1.128536e+02 2.908209e+02
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
    endloop
  endfacet
  facet normal -6.872754e-01 3.412029e-02 7.255952e-01
    outer loop
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.354215e+02 1.128536e+02 2.900650e+02
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
    endloop
  endfacet
  facet normal 8.404196e-01 -2.479013e-02 5.413690e-01
    outer loop
      vertex   -1.266291e+02 1.176020e+02 2.888696e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
    endloop
  endfacet
  facet normal 7.518769e-01 -3.919447e-03 6.592919e-01
    outer loop
      vertex   -1.270642e+02 1.175000e+02 2.893652e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.266291e+02 1.176020e+02 2.888696e+02
    endloop
  endfacet
  facet normal 6.371232e-01 -3.322120e-03 7.707548e-01
    outer loop
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.270642e+02 1.175000e+02 2.893652e+02
    endloop
  endfacet
  facet normal 6.591800e-01 1.003426e-02 7.519183e-01
    outer loop
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
    endloop
  endfacet
  facet normal 4.840528e-01 -6.646831e-03 8.750136e-01
    outer loop
      vertex   -1.289850e+02 1.171816e+02 2.908545e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
    endloop
  endfacet
  facet normal 3.003463e-01 -4.064760e-02 9.529637e-01
    outer loop
      vertex   -1.304878e+02 1.170770e+02 2.913237e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.289850e+02 1.171816e+02 2.908545e+02
    endloop
  endfacet
  facet normal 3.380024e-01 -1.913351e-02 9.409508e-01
    outer loop
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.283475e+02 1.128536e+02 2.904690e+02
      vertex   -1.304878e+02 1.170770e+02 2.913237e+02
    endloop
  endfacet
  facet normal 4.105481e-02 1.148578e-02 9.990909e-01
    outer loop
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
      vertex   -1.309104e+02 1.128536e+02 2.913896e+02
      vertex   -1.304878e+02 1.170770e+02 2.913237e+02
    endloop
  endfacet
  facet normal 9.974948e-01 1.967648e-03 7.071182e-02
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
      vertex   -1.239723e+02 1.128536e+02 2.785219e+02
    endloop
  endfacet
  facet normal 9.968124e-01 0.000000e+00 7.978137e-02
    outer loop
      vertex   -1.238922e+02 1.193584e+02 2.773924e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
    endloop
  endfacet
  facet normal 9.935003e-01 3.212962e-04 1.138290e-01
    outer loop
      vertex   -1.239723e+02 1.128536e+02 2.785219e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
    endloop
  endfacet
  facet normal 9.920506e-01 6.041439e-03 1.256943e-01
    outer loop
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
    endloop
  endfacet
  facet normal 9.848407e-01 -3.291648e-03 1.734300e-01
    outer loop
      vertex   -1.243456e+02 1.128536e+02 2.817804e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
    endloop
  endfacet
  facet normal 9.744163e-01 1.094522e-02 2.244841e-01
    outer loop
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
    endloop
  endfacet
  facet normal 9.003969e-01 -6.047903e-03 4.350273e-01
    outer loop
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.262512e+02 1.128536e+02 2.883359e+02
    endloop
  endfacet
  facet normal 8.907791e-01 -1.152846e-02 4.542903e-01
    outer loop
      vertex   -1.257413e+02 1.178939e+02 2.873837e+02
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
    endloop
  endfacet
  facet normal 9.400114e-01 3.621306e-03 3.411238e-01
    outer loop
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
      vertex   -1.257413e+02 1.178939e+02 2.873837e+02
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
    endloop
  endfacet
  facet normal 9.590035e-01 -6.860166e-03 2.833112e-01
    outer loop
      vertex   -1.248148e+02 1.128536e+02 2.844444e+02
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
      vertex   -1.255131e+02 1.128536e+02 2.868082e+02
    endloop
  endfacet
  facet normal 7.756958e-01 1.474691e-02 6.309347e-01
    outer loop
      vertex   -1.262512e+02 1.128536e+02 2.883359e+02
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.270884e+02 1.128536e+02 2.893652e+02
    endloop
  endfacet
  facet normal -7.101616e-02 -5.443275e-04 -9.974750e-01
    outer loop
      vertex   -1.307346e+02 1.188688e+02 2.814380e+02
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.330877e+02 1.188455e+02 2.816055e+02
    endloop
  endfacet
  facet normal -1.049679e-01 1.221061e-02 -9.944006e-01
    outer loop
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.307346e+02 1.188688e+02 2.814380e+02
    endloop
  endfacet
  facet normal -2.131260e-01 -1.508292e-03 -9.770236e-01
    outer loop
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
      vertex   -1.307346e+02 1.188688e+02 2.814380e+02
    endloop
  endfacet
  facet normal -2.571738e-01 1.641243e-03 -9.663638e-01
    outer loop
      vertex   -1.291810e+02 1.128536e+02 2.810693e+02
      vertex   -1.299960e+02 1.128536e+02 2.812862e+02
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
    endloop
  endfacet
  facet normal -3.417075e-01 -4.555623e-03 -9.397953e-01
    outer loop
      vertex   -1.285073e+02 1.128536e+02 2.808243e+02
      vertex   -1.291810e+02 1.128536e+02 2.810693e+02
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
    endloop
  endfacet
  facet normal -3.707893e-01 -1.041631e-02 -9.286586e-01
    outer loop
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
      vertex   -1.285073e+02 1.128536e+02 2.808243e+02
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
    endloop
  endfacet
  facet normal -4.236535e-01 -1.869209e-03 -9.058224e-01
    outer loop
      vertex   -1.274599e+02 1.128536e+02 2.803344e+02
      vertex   -1.285073e+02 1.128536e+02 2.808243e+02
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
    endloop
  endfacet
  facet normal -5.288166e-01 -6.570803e-03 -8.487107e-01
    outer loop
      vertex   -1.264038e+02 1.128536e+02 2.796764e+02
      vertex   -1.274599e+02 1.128536e+02 2.803344e+02
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
    endloop
  endfacet
  facet normal -5.786631e-01 -2.102986e-02 -8.152955e-01
    outer loop
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
      vertex   -1.264038e+02 1.128536e+02 2.796764e+02
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
    endloop
  endfacet
  facet normal -6.626374e-01 1.109845e-02 -7.488581e-01
    outer loop
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
      vertex   -1.264038e+02 1.128536e+02 2.796764e+02
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
    endloop
  endfacet
  facet normal -7.195815e-01 -2.365759e-03 -6.944040e-01
    outer loop
      vertex   -1.238922e+02 1.193584e+02 2.773924e+02
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
    endloop
  endfacet
  facet normal -7.463082e-01 0.000000e+00 -6.656006e-01
    outer loop
      vertex   -1.238922e+02 1.128536e+02 2.773924e+02
      vertex   -1.241529e+02 1.128536e+02 2.776847e+02
      vertex   -1.238922e+02 1.193584e+02 2.773924e+02
    endloop
  endfacet
  facet normal 6.892721e-01 -6.484478e-03 -7.244735e-01
    outer loop
      vertex   -1.410653e+02 1.191958e+02 2.788660e+02
      vertex   -1.420764e+02 1.128536e+02 2.779608e+02
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
    endloop
  endfacet
  facet normal 6.410543e-01 7.338340e-03 -7.674604e-01
    outer loop
      vertex   -1.404915e+02 1.128536e+02 2.792846e+02
      vertex   -1.420764e+02 1.128536e+02 2.779608e+02
      vertex   -1.410653e+02 1.191958e+02 2.788660e+02
    endloop
  endfacet
  facet normal 5.476307e-01 -5.683351e-03 -8.367008e-01
    outer loop
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
      vertex   -1.404915e+02 1.128536e+02 2.792846e+02
      vertex   -1.410653e+02 1.191958e+02 2.788660e+02
    endloop
  endfacet
  facet normal 4.915210e-01 6.921604e-03 -8.708382e-01
    outer loop
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
      vertex   -1.404915e+02 1.128536e+02 2.792846e+02
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
    endloop
  endfacet
  facet normal 4.006357e-01 -7.475719e-03 -9.162069e-01
    outer loop
      vertex   -1.370341e+02 1.189316e+02 2.809763e+02
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
    endloop
  endfacet
  facet normal 3.224203e-01 1.714150e-02 -9.464414e-01
    outer loop
      vertex   -1.357778e+02 1.128536e+02 2.812942e+02
      vertex   -1.386868e+02 1.128536e+02 2.803032e+02
      vertex   -1.370341e+02 1.189316e+02 2.809763e+02
    endloop
  endfacet
  facet normal 2.328437e-01 -2.736306e-03 -9.725103e-01
    outer loop
      vertex   -1.355207e+02 1.188824e+02 2.813388e+02
      vertex   -1.357778e+02 1.128536e+02 2.812942e+02
      vertex   -1.370341e+02 1.189316e+02 2.809763e+02
    endloop
  endfacet
  facet normal 1.090192e-01 2.703544e-03 -9.940360e-01
    outer loop
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.357778e+02 1.128536e+02 2.812942e+02
      vertex   -1.355207e+02 1.188824e+02 2.813388e+02
    endloop
  endfacet
  facet normal 1.090119e-01 2.700431e-03 -9.940368e-01
    outer loop
      vertex   -1.330877e+02 1.188455e+02 2.816055e+02
      vertex   -1.329796e+02 1.128536e+02 2.816011e+02
      vertex   -1.355207e+02 1.188824e+02 2.813388e+02
    endloop
  endfacet
  facet normal 9.992970e-01 2.932028e-03 -3.737519e-02
    outer loop
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
      vertex   -1.473336e+02 1.199941e+02 2.656736e+02
    endloop
  endfacet
  facet normal 9.974945e-01 -5.287992e-03 -7.054558e-02
    outer loop
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.473336e+02 1.199941e+02 2.656736e+02
    endloop
  endfacet
  facet normal 9.882280e-01 1.676951e-03 -1.529789e-01
    outer loop
      vertex   -1.469279e+02 1.128536e+02 2.694970e+02
      vertex   -1.472466e+02 1.128536e+02 2.674384e+02
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
    endloop
  endfacet
  facet normal 9.776495e-01 -1.045100e-02 -2.099814e-01
    outer loop
      vertex   -1.464924e+02 1.198395e+02 2.711772e+02
      vertex   -1.469279e+02 1.128536e+02 2.694970e+02
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
    endloop
  endfacet
  facet normal 9.615054e-01 6.125017e-03 -2.747176e-01
    outer loop
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.469279e+02 1.128536e+02 2.694970e+02
      vertex   -1.464924e+02 1.198395e+02 2.711772e+02
    endloop
  endfacet
  facet normal 9.388454e-01 1.761583e-04 -3.443389e-01
    outer loop
      vertex   -1.460302e+02 1.197676e+02 2.724373e+02
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.464924e+02 1.198395e+02 2.711772e+02
    endloop
  endfacet
  facet normal 8.919662e-01 1.329058e-02 -4.519067e-01
    outer loop
      vertex   -1.448983e+02 1.196092e+02 2.746668e+02
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.460302e+02 1.197676e+02 2.724373e+02
    endloop
  endfacet
  facet normal 8.932834e-01 1.189320e-02 -4.493365e-01
    outer loop
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
      vertex   -1.462957e+02 1.128536e+02 2.717100e+02
      vertex   -1.448983e+02 1.196092e+02 2.746668e+02
    endloop
  endfacet
  facet normal 7.983350e-01 -3.426784e-03 -6.022038e-01
    outer loop
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
      vertex   -1.448983e+02 1.196092e+02 2.746668e+02
    endloop
  endfacet
  facet normal 7.490429e-01 1.855484e-02 -6.622617e-01
    outer loop
      vertex   -1.445703e+02 1.128536e+02 2.751401e+02
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
      vertex   -1.420764e+02 1.128536e+02 2.779608e+02
    endloop
  endfacet
  facet normal 7.668019e-01 -4.758630e-03 6.418662e-01
    outer loop
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.437046e+02 1.128536e+02 2.539164e+02
      vertex   -1.428166e+02 1.193885e+02 2.529041e+02
    endloop
  endfacet
  facet normal 8.612446e-01 -1.075913e-02 5.080768e-01
    outer loop
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
      vertex   -1.450958e+02 1.128536e+02 2.559302e+02
    endloop
  endfacet
  facet normal 8.227348e-01 6.619537e-03 5.683868e-01
    outer loop
      vertex   -1.437046e+02 1.128536e+02 2.539164e+02
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.450958e+02 1.128536e+02 2.559302e+02
    endloop
  endfacet
  facet normal 9.344101e-01 -6.794299e-03 3.561342e-01
    outer loop
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
    endloop
  endfacet
  facet normal 9.005778e-01 5.350787e-03 4.346620e-01
    outer loop
      vertex   -1.450958e+02 1.128536e+02 2.559302e+02
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
    endloop
  endfacet
  facet normal 9.500414e-01 1.117895e-03 3.121218e-01
    outer loop
      vertex   -1.460993e+02 1.128536e+02 2.580094e+02
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.466127e+02 1.128536e+02 2.595722e+02
    endloop
  endfacet
  facet normal 9.785729e-01 -5.863903e-03 2.058169e-01
    outer loop
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.471473e+02 1.128536e+02 2.621216e+02
    endloop
  endfacet
  facet normal 9.786997e-01 -6.137175e-03 2.052052e-01
    outer loop
      vertex   -1.466127e+02 1.128536e+02 2.595722e+02
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.471473e+02 1.128536e+02 2.621216e+02
    endloop
  endfacet
  facet normal 9.989643e-01 -4.039685e-03 4.532213e-02
    outer loop
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.473336e+02 1.199941e+02 2.656736e+02
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
    endloop
  endfacet
  facet normal 9.979751e-01 2.482689e-03 6.355748e-02
    outer loop
      vertex   -1.471473e+02 1.128536e+02 2.621216e+02
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.473351e+02 1.128536e+02 2.650716e+02
    endloop
  endfacet
  facet normal -1.277116e-02 7.454360e-03 9.998907e-01
    outer loop
      vertex   -1.329444e+02 1.128536e+02 2.483819e+02
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal 5.791885e-02 -2.785116e-02 9.979327e-01
    outer loop
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
      vertex   -1.329444e+02 1.128536e+02 2.483819e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal 1.246188e-01 -5.401765e-03 9.921900e-01
    outer loop
      vertex   -1.347096e+02 1.128536e+02 2.486036e+02
      vertex   -1.329444e+02 1.128536e+02 2.483819e+02
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
    endloop
  endfacet
  facet normal 2.719532e-01 7.656117e-04 9.623102e-01
    outer loop
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
      vertex   -1.347096e+02 1.128536e+02 2.486036e+02
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
    endloop
  endfacet
  facet normal 3.054962e-01 -1.478499e-02 9.520785e-01
    outer loop
      vertex   -1.384771e+02 1.190334e+02 2.498008e+02
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
    endloop
  endfacet
  facet normal 4.309958e-01 7.866772e-03 9.023197e-01
    outer loop
      vertex   -1.393483e+02 1.128536e+02 2.502708e+02
      vertex   -1.375213e+02 1.128536e+02 2.493982e+02
      vertex   -1.384771e+02 1.190334e+02 2.498008e+02
    endloop
  endfacet
  facet normal 4.985797e-01 -4.360034e-03 8.668329e-01
    outer loop
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
      vertex   -1.393483e+02 1.128536e+02 2.502708e+02
      vertex   -1.384771e+02 1.190334e+02 2.498008e+02
    endloop
  endfacet
  facet normal 5.735019e-01 1.360023e-02 8.190913e-01
    outer loop
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
      vertex   -1.393483e+02 1.128536e+02 2.502708e+02
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
    endloop
  endfacet
  facet normal 6.383022e-01 -1.107263e-02 7.697062e-01
    outer loop
      vertex   -1.428166e+02 1.193885e+02 2.529041e+02
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
    endloop
  endfacet
  facet normal 7.211842e-01 9.311722e-03 6.926808e-01
    outer loop
      vertex   -1.437046e+02 1.128536e+02 2.539164e+02
      vertex   -1.419583e+02 1.128536e+02 2.520983e+02
      vertex   -1.428166e+02 1.193885e+02 2.529041e+02
    endloop
  endfacet
  facet normal -7.469476e-01 1.321699e-02 6.647516e-01
    outer loop
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
    endloop
  endfacet
  facet normal -7.262580e-01 -4.885454e-04 6.874221e-01
    outer loop
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
    endloop
  endfacet
  facet normal -5.701330e-01 9.093799e-03 8.215020e-01
    outer loop
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
    endloop
  endfacet
  facet normal -5.815712e-01 1.376526e-03 8.134943e-01
    outer loop
      vertex   -1.236629e+02 1.128536e+02 2.501765e+02
      vertex   -1.206845e+02 1.128536e+02 2.523058e+02
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
    endloop
  endfacet
  facet normal -4.025616e-01 -5.609828e-03 9.153757e-01
    outer loop
      vertex   -1.260810e+02 1.128536e+02 2.491131e+02
      vertex   -1.236629e+02 1.128536e+02 2.501765e+02
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
    endloop
  endfacet
  facet normal -4.039232e-01 -4.930827e-03 9.147796e-01
    outer loop
      vertex   -1.263468e+02 1.189322e+02 2.490285e+02
      vertex   -1.260810e+02 1.128536e+02 2.491131e+02
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
    endloop
  endfacet
  facet normal -1.982676e-01 4.970741e-03 9.801353e-01
    outer loop
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
      vertex   -1.260810e+02 1.128536e+02 2.491131e+02
      vertex   -1.263468e+02 1.189322e+02 2.490285e+02
    endloop
  endfacet
  facet normal -2.313750e-01 2.276626e-02 9.725982e-01
    outer loop
      vertex   -1.284992e+02 1.188627e+02 2.485181e+02
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
      vertex   -1.263468e+02 1.189322e+02 2.490285e+02
    endloop
  endfacet
  facet normal -9.765390e-02 6.628383e-04 9.952202e-01
    outer loop
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
      vertex   -1.294769e+02 1.128536e+02 2.484262e+02
      vertex   -1.284992e+02 1.188627e+02 2.485181e+02
    endloop
  endfacet
  facet normal -9.990482e-01 -4.962264e-03 4.333695e-02
    outer loop
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
      vertex   -1.139897e+02 1.128536e+02 2.729216e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
    endloop
  endfacet
  facet normal -9.990625e-01 -5.222477e-03 4.297510e-02
    outer loop
      vertex   -1.142001e+02 1.128536e+02 2.680310e+02
      vertex   -1.139897e+02 1.128536e+02 2.729216e+02
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
    endloop
  endfacet
  facet normal -9.906604e-01 -2.475723e-03 1.363301e-01
    outer loop
      vertex   -1.142001e+02 1.128536e+02 2.680310e+02
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
      vertex   -1.147521e+02 1.128536e+02 2.640201e+02
    endloop
  endfacet
  facet normal -9.906598e-01 -2.477874e-03 1.363340e-01
    outer loop
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.147521e+02 1.128536e+02 2.640201e+02
    endloop
  endfacet
  facet normal -9.766374e-01 1.066611e-03 2.148914e-01
    outer loop
      vertex   -1.147521e+02 1.128536e+02 2.640201e+02
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.152208e+02 1.128536e+02 2.618900e+02
    endloop
  endfacet
  facet normal -9.695281e-01 -6.986196e-03 2.448804e-01
    outer loop
      vertex   -1.152208e+02 1.128536e+02 2.618900e+02
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
    endloop
  endfacet
  facet normal -9.353814e-01 -1.231996e-03 3.536384e-01
    outer loop
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
    endloop
  endfacet
  facet normal -9.530726e-01 8.599155e-03 3.026195e-01
    outer loop
      vertex   -1.152208e+02 1.128536e+02 2.618900e+02
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
    endloop
  endfacet
  facet normal -8.622967e-01 -1.016232e-02 5.063015e-01
    outer loop
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
    endloop
  endfacet
  facet normal -8.836252e-01 1.225276e-02 4.680346e-01
    outer loop
      vertex   -1.161619e+02 1.128536e+02 2.589259e+02
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.181746e+02 1.128536e+02 2.551260e+02
    endloop
  endfacet
  facet normal -8.628169e-01 9.514867e-03 -5.054270e-01
    outer loop
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.183710e+02 1.128536e+02 2.923696e+02
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
    endloop
  endfacet
  facet normal -8.558215e-01 1.765440e-02 -5.169699e-01
    outer loop
      vertex   -1.172102e+02 1.172536e+02 2.905241e+02
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
    endloop
  endfacet
  facet normal -9.097049e-01 3.220262e-04 -4.152553e-01
    outer loop
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.172102e+02 1.172536e+02 2.905241e+02
    endloop
  endfacet
  facet normal -9.344890e-01 1.990165e-02 -3.554353e-01
    outer loop
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.169358e+02 1.128536e+02 2.899197e+02
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
    endloop
  endfacet
  facet normal -9.529527e-01 -1.343407e-02 -3.028210e-01
    outer loop
      vertex   -1.150962e+02 1.183726e+02 2.846805e+02
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
    endloop
  endfacet
  facet normal -9.743983e-01 4.777380e-03 -2.247779e-01
    outer loop
      vertex   -1.148130e+02 1.128536e+02 2.833353e+02
      vertex   -1.154011e+02 1.128536e+02 2.858846e+02
      vertex   -1.150962e+02 1.183726e+02 2.846805e+02
    endloop
  endfacet
  facet normal -9.824641e-01 -4.994695e-03 -1.863848e-01
    outer loop
      vertex   -1.148130e+02 1.128536e+02 2.833353e+02
      vertex   -1.150962e+02 1.183726e+02 2.846805e+02
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
    endloop
  endfacet
  facet normal -9.913488e-01 1.170580e-02 -1.307310e-01
    outer loop
      vertex   -1.148130e+02 1.128536e+02 2.833353e+02
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
    endloop
  endfacet
  facet normal -9.941161e-01 9.779759e-04 -1.083156e-01
    outer loop
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
      vertex   -1.141438e+02 1.192663e+02 2.782145e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
    endloop
  endfacet
  facet normal -9.993288e-01 7.135350e-03 -3.593018e-02
    outer loop
      vertex   -1.141438e+02 1.192663e+02 2.782145e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
    endloop
  endfacet
  facet normal -9.992767e-01 5.958906e-03 -3.755835e-02
    outer loop
      vertex   -1.142090e+02 1.128536e+02 2.787552e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
      vertex   -1.139897e+02 1.128536e+02 2.729216e+02
    endloop
  endfacet
  facet normal -7.838599e-02 -1.895028e-02 -9.967430e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
    endloop
  endfacet
  facet normal -2.037117e-01 8.620940e-02 -9.752279e-01
    outer loop
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
    endloop
  endfacet
  facet normal -2.087751e-01 7.908774e-02 -9.747605e-01
    outer loop
      vertex   -1.274703e+02 1.151530e+02 2.987608e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
    endloop
  endfacet
  facet normal -3.217024e-01 3.531917e-03 -9.468342e-01
    outer loop
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.274703e+02 1.151530e+02 2.987608e+02
    endloop
  endfacet
  facet normal -4.204137e-01 2.774193e-02 -9.069083e-01
    outer loop
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
    endloop
  endfacet
  facet normal -3.572351e-01 -2.279052e-02 -9.337364e-01
    outer loop
      vertex   -1.247354e+02 1.128536e+02 2.977665e+02
      vertex   -1.260531e+02 1.128536e+02 2.982707e+02
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
    endloop
  endfacet
  facet normal -5.210880e-01 1.481714e-02 -8.533743e-01
    outer loop
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
      vertex   -1.247354e+02 1.128536e+02 2.977665e+02
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
    endloop
  endfacet
  facet normal -5.519355e-01 -1.622435e-02 -8.337289e-01
    outer loop
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
    endloop
  endfacet
  facet normal -6.456920e-01 2.634442e-02 -7.631434e-01
    outer loop
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
      vertex   -1.223449e+02 1.128536e+02 2.963068e+02
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
    endloop
  endfacet
  facet normal -6.692125e-01 7.255728e-03 -7.430357e-01
    outer loop
      vertex   -1.201478e+02 1.163197e+02 2.944910e+02
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
    endloop
  endfacet
  facet normal -7.717036e-01 -3.622880e-03 -6.359720e-01
    outer loop
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
      vertex   -1.201478e+02 1.163197e+02 2.944910e+02
    endloop
  endfacet
  facet normal -7.692948e-01 -6.318911e-03 -6.388627e-01
    outer loop
      vertex   -1.183710e+02 1.128536e+02 2.923696e+02
      vertex   -1.199764e+02 1.128536e+02 2.943029e+02
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
    endloop
  endfacet
  facet normal 6.400031e-01 6.777180e-03 -7.683425e-01
    outer loop
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
    endloop
  endfacet
  facet normal 6.281099e-01 -1.931022e-03 -7.781223e-01
    outer loop
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
    endloop
  endfacet
  facet normal 4.773499e-01 -1.209597e-04 -8.787133e-01
    outer loop
      vertex   -1.384173e+02 1.128536e+02 2.981731e+02
      vertex   -1.405187e+02 1.128536e+02 2.970315e+02
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
    endloop
  endfacet
  facet normal 4.053962e-01 -6.705750e-02 -9.116782e-01
    outer loop
      vertex   -1.367380e+02 1.151557e+02 2.987505e+02
      vertex   -1.384173e+02 1.128536e+02 2.981731e+02
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
    endloop
  endfacet
  facet normal 2.768069e-01 3.889661e-02 -9.601380e-01
    outer loop
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.384173e+02 1.128536e+02 2.981731e+02
      vertex   -1.367380e+02 1.151557e+02 2.987505e+02
    endloop
  endfacet
  facet normal 2.027923e-01 -8.935269e-04 -9.792214e-01
    outer loop
      vertex   -1.344416e+02 1.150155e+02 2.992262e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.367380e+02 1.151557e+02 2.987505e+02
    endloop
  endfacet
  facet normal 4.715253e-02 8.162956e-02 -9.955467e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.344416e+02 1.150155e+02 2.992262e+02
    endloop
  endfacet
  facet normal 6.062701e-02 5.980181e-02 -9.963675e-01
    outer loop
      vertex   -1.308868e+02 1.128536e+02 2.992804e+02
      vertex   -1.355637e+02 1.128536e+02 2.989958e+02
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
    endloop
  endfacet
  facet normal 9.796010e-01 0.000000e+00 -2.009525e-01
    outer loop
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
      vertex   -1.464366e+02 1.179832e+02 2.869081e+02
    endloop
  endfacet
  facet normal 9.770280e-01 3.334279e-03 -2.130847e-01
    outer loop
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
      vertex   -1.464366e+02 1.128536e+02 2.869081e+02
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
    endloop
  endfacet
  facet normal 9.682735e-01 5.522736e-04 -2.498923e-01
    outer loop
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
    endloop
  endfacet
  facet normal 9.334997e-01 2.170640e-02 -3.579205e-01
    outer loop
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
      vertex   -1.448091e+02 1.169003e+02 2.920947e+02
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
    endloop
  endfacet
  facet normal 9.332796e-01 2.123929e-02 -3.585221e-01
    outer loop
      vertex   -1.460805e+02 1.128536e+02 2.885408e+02
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
    endloop
  endfacet
  facet normal 8.512285e-01 4.697052e-04 -5.247950e-01
    outer loop
      vertex   -1.448091e+02 1.169003e+02 2.920947e+02
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
    endloop
  endfacet
  facet normal 7.211191e-01 -1.581648e-02 -6.926305e-01
    outer loop
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
    endloop
  endfacet
  facet normal 8.050621e-01 4.768431e-02 -5.912708e-01
    outer loop
      vertex   -1.445639e+02 1.128536e+02 2.924887e+02
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
      vertex   -1.423443e+02 1.128536e+02 2.955109e+02
    endloop
  endfacet
  facet normal -7.626777e-02 -2.936062e-02 -9.966550e-01
    outer loop
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
      vertex   -8.795835e+01 1.150710e+02 2.990388e+02
    endloop
  endfacet
  facet normal -2.992540e-01 1.053820e-01 -9.483363e-01
    outer loop
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
      vertex   -8.795835e+01 1.150710e+02 2.990388e+02
    endloop
  endfacet
  facet normal -2.244907e-01 -2.875637e-02 -9.740518e-01
    outer loop
      vertex   -8.550117e+01 1.128536e+02 2.983370e+02
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
    endloop
  endfacet
  facet normal -4.511448e-01 4.695376e-03 -8.924384e-01
    outer loop
      vertex   -8.327047e+01 1.128536e+02 2.972094e+02
      vertex   -8.550117e+01 1.128536e+02 2.983370e+02
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
    endloop
  endfacet
  facet normal -5.161660e-01 -6.116342e-02 -8.543019e-01
    outer loop
      vertex   -8.206047e+01 1.158527e+02 2.962636e+02
      vertex   -8.327047e+01 1.128536e+02 2.972094e+02
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
    endloop
  endfacet
  facet normal -6.150854e-01 -4.895698e-04 -7.884603e-01
    outer loop
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
      vertex   -8.327047e+01 1.128536e+02 2.972094e+02
      vertex   -8.206047e+01 1.158527e+02 2.962636e+02
    endloop
  endfacet
  facet normal -6.838176e-01 -1.452260e-02 -7.295085e-01
    outer loop
      vertex   -8.028574e+01 1.162942e+02 2.945912e+02
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
      vertex   -8.206047e+01 1.158527e+02 2.962636e+02
    endloop
  endfacet
  facet normal -7.455647e-01 3.656526e-02 -6.654294e-01
    outer loop
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
      vertex   -8.169266e+01 1.128536e+02 2.959785e+02
      vertex   -8.028574e+01 1.162942e+02 2.945912e+02
    endloop
  endfacet
  facet normal -8.025383e-01 2.986470e-03 -5.965932e-01
    outer loop
      vertex   -7.948557e+01 1.165605e+02 2.935162e+02
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
      vertex   -8.028574e+01 1.162942e+02 2.945912e+02
    endloop
  endfacet
  facet normal 7.668843e-01 -2.286196e-02 -6.413781e-01
    outer loop
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
    endloop
  endfacet
  facet normal 7.214476e-01 2.087214e-02 -6.921544e-01
    outer loop
      vertex   -1.006555e+02 1.128536e+02 2.962476e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
    endloop
  endfacet
  facet normal 5.879554e-01 -2.535072e-02 -8.084960e-01
    outer loop
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -1.006555e+02 1.128536e+02 2.962476e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
    endloop
  endfacet
  facet normal 5.999911e-01 -1.352105e-02 -7.998924e-01
    outer loop
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
    endloop
  endfacet
  facet normal 4.808212e-01 -4.865358e-03 -8.768052e-01
    outer loop
      vertex   -9.789488e+01 1.128536e+02 2.980077e+02
      vertex   -9.927891e+01 1.128536e+02 2.972487e+02
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
    endloop
  endfacet
  facet normal 3.611578e-01 -7.265772e-02 -9.296698e-01
    outer loop
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
      vertex   -9.789488e+01 1.128536e+02 2.980077e+02
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
    endloop
  endfacet
  facet normal 3.584157e-01 -7.583438e-02 -9.304770e-01
    outer loop
      vertex   -9.498749e+01 1.150972e+02 2.989498e+02
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
    endloop
  endfacet
  facet normal 1.975498e-01 3.043114e-02 -9.798204e-01
    outer loop
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
      vertex   -9.636308e+01 1.128536e+02 2.986027e+02
      vertex   -9.498749e+01 1.150972e+02 2.989498e+02
    endloop
  endfacet
  facet normal 1.162330e-01 -3.211025e-02 -9.927028e-01
    outer loop
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
      vertex   -9.498749e+01 1.150972e+02 2.989498e+02
    endloop
  endfacet
  facet normal -3.816121e-03 5.551785e-02 -9.984504e-01
    outer loop
      vertex   -8.926248e+01 1.128536e+02 2.992039e+02
      vertex   -9.330469e+01 1.128536e+02 2.992194e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
    endloop
  endfacet
  facet normal 9.999240e-01 4.284272e-03 -1.155779e-02
    outer loop
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
      vertex   -1.075199e+02 1.128536e+02 2.721049e+02
      vertex   -1.075421e+02 1.197464e+02 2.727375e+02
    endloop
  endfacet
  facet normal 9.989204e-01 -1.737511e-02 -4.308368e-02
    outer loop
      vertex   -1.075421e+02 1.197464e+02 2.727375e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
    endloop
  endfacet
  facet normal 9.970299e-01 -1.205912e-03 -7.700568e-02
    outer loop
      vertex   -1.072000e+02 1.128536e+02 2.808092e+02
      vertex   -1.074579e+02 1.128536e+02 2.774704e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
    endloop
  endfacet
  facet normal 9.891698e-01 -6.041462e-03 -1.466512e-01
    outer loop
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
      vertex   -1.072000e+02 1.128536e+02 2.808092e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
    endloop
  endfacet
  facet normal 9.899090e-01 -2.646800e-03 -1.416795e-01
    outer loop
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
    endloop
  endfacet
  facet normal 9.716814e-01 -4.130402e-05 -2.362947e-01
    outer loop
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.058501e+02 1.178188e+02 2.877612e+02
    endloop
  endfacet
  facet normal 9.710567e-01 -1.577297e-03 -2.388440e-01
    outer loop
      vertex   -1.066536e+02 1.128536e+02 2.844949e+02
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
    endloop
  endfacet
  facet normal 9.405200e-01 8.849345e-03 -3.396231e-01
    outer loop
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
    endloop
  endfacet
  facet normal 9.465728e-01 1.107146e-03 -3.224884e-01
    outer loop
      vertex   -1.058501e+02 1.178188e+02 2.877612e+02
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
      vertex   -1.058654e+02 1.128536e+02 2.876994e+02
    endloop
  endfacet
  facet normal 9.006439e-01 -3.455046e-02 -4.331823e-01
    outer loop
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
      vertex   -1.041648e+02 1.128536e+02 2.918922e+02
    endloop
  endfacet
  facet normal 8.899941e-01 -1.430487e-02 -4.557476e-01
    outer loop
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
      vertex   -1.049134e+02 1.128536e+02 2.903357e+02
    endloop
  endfacet
  facet normal 8.310537e-01 1.632202e-02 -5.559526e-01
    outer loop
      vertex   -1.041648e+02 1.128536e+02 2.918922e+02
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
      vertex   -1.026231e+02 1.128536e+02 2.941967e+02
    endloop
  endfacet
  facet normal 8.298226e-01 4.566116e-03 5.580085e-01
    outer loop
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
    endloop
  endfacet
  facet normal 8.619981e-01 -8.691021e-03 5.068369e-01
    outer loop
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
    endloop
  endfacet
  facet normal 8.948912e-01 3.995894e-03 4.462665e-01
    outer loop
      vertex   -1.049158e+02 1.128536e+02 2.573190e+02
      vertex   -1.039196e+02 1.128536e+02 2.553213e+02
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
    endloop
  endfacet
  facet normal 9.267464e-01 -7.041771e-03 3.756215e-01
    outer loop
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
      vertex   -1.049158e+02 1.128536e+02 2.573190e+02
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
    endloop
  endfacet
  facet normal 9.452660e-01 8.045064e-03 3.262015e-01
    outer loop
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -1.049158e+02 1.128536e+02 2.573190e+02
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
    endloop
  endfacet
  facet normal 9.698661e-01 -8.493989e-03 2.434906e-01
    outer loop
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
    endloop
  endfacet
  facet normal 9.762551e-01 2.711981e-03 2.166070e-01
    outer loop
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -1.060061e+02 1.128536e+02 2.604783e+02
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
    endloop
  endfacet
  facet normal 9.910659e-01 -8.652210e-03 1.330920e-01
    outer loop
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
    endloop
  endfacet
  facet normal 9.921455e-01 -3.936044e-03 1.250270e-01
    outer loop
      vertex   -1.073034e+02 1.128536e+02 2.679186e+02
      vertex   -1.068386e+02 1.128536e+02 2.642302e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
    endloop
  endfacet
  facet normal 9.986650e-01 1.454459e-04 5.165430e-02
    outer loop
      vertex   -1.073034e+02 1.128536e+02 2.679186e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
      vertex   -1.075199e+02 1.128536e+02 2.721049e+02
    endloop
  endfacet
  facet normal 9.987979e-01 -1.277358e-03 4.900221e-02
    outer loop
      vertex   -1.075421e+02 1.197464e+02 2.727375e+02
      vertex   -1.075199e+02 1.128536e+02 2.721049e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
    endloop
  endfacet
  facet normal 3.500131e-02 1.383762e-02 9.992915e-01
    outer loop
      vertex   -9.252737e+01 1.188395e+02 2.483512e+02
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
    endloop
  endfacet
  facet normal 3.877145e-02 1.517391e-02 9.991329e-01
    outer loop
      vertex   -9.404069e+01 1.128536e+02 2.485009e+02
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -9.252737e+01 1.188395e+02 2.483512e+02
    endloop
  endfacet
  facet normal 1.068826e-01 -2.167357e-03 9.942693e-01
    outer loop
      vertex   -9.431639e+01 1.188662e+02 2.485436e+02
      vertex   -9.404069e+01 1.128536e+02 2.485009e+02
      vertex   -9.252737e+01 1.188395e+02 2.483512e+02
    endloop
  endfacet
  facet normal 2.193635e-01 3.122717e-03 9.756382e-01
    outer loop
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
      vertex   -9.404069e+01 1.128536e+02 2.485009e+02
      vertex   -9.431639e+01 1.188662e+02 2.485436e+02
    endloop
  endfacet
  facet normal 2.449054e-01 -4.620799e-03 9.695360e-01
    outer loop
      vertex   -9.657965e+01 1.189439e+02 2.491157e+02
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
      vertex   -9.431639e+01 1.188662e+02 2.485436e+02
    endloop
  endfacet
  facet normal 3.840389e-01 8.915509e-03 9.232739e-01
    outer loop
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
      vertex   -9.604194e+01 1.128536e+02 2.489508e+02
      vertex   -9.657965e+01 1.189439e+02 2.491157e+02
    endloop
  endfacet
  facet normal 3.998943e-01 2.374265e-03 9.165582e-01
    outer loop
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
      vertex   -9.657965e+01 1.189439e+02 2.491157e+02
    endloop
  endfacet
  facet normal 5.514542e-01 -1.743526e-03 8.342034e-01
    outer loop
      vertex   -1.004897e+02 1.192095e+02 2.512542e+02
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
    endloop
  endfacet
  facet normal 5.826588e-01 1.113999e-02 8.126405e-01
    outer loop
      vertex   -1.010847e+02 1.128536e+02 2.517679e+02
      vertex   -9.869469e+01 1.128536e+02 2.500543e+02
      vertex   -1.004897e+02 1.192095e+02 2.512542e+02
    endloop
  endfacet
  facet normal 6.996780e-01 -7.753357e-03 7.144163e-01
    outer loop
      vertex   -1.023758e+02 1.194081e+02 2.531035e+02
      vertex   -1.010847e+02 1.128536e+02 2.517679e+02
      vertex   -1.004897e+02 1.192095e+02 2.512542e+02
    endloop
  endfacet
  facet normal 7.344490e-01 6.387382e-03 6.786338e-01
    outer loop
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
      vertex   -1.010847e+02 1.128536e+02 2.517679e+02
      vertex   -1.023758e+02 1.194081e+02 2.531035e+02
    endloop
  endfacet
  facet normal 7.951932e-01 -1.612106e-03 6.063540e-01
    outer loop
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
      vertex   -1.027210e+02 1.128536e+02 2.535388e+02
      vertex   -1.023758e+02 1.194081e+02 2.531035e+02
    endloop
  endfacet
  facet normal -7.893239e-01 -5.572764e-03 6.139517e-01
    outer loop
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
      vertex   -7.891833e+01 1.128536e+02 2.548126e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
    endloop
  endfacet
  facet normal -8.096268e-01 -1.862977e-02 5.866492e-01
    outer loop
      vertex   -7.972990e+01 1.128536e+02 2.536926e+02
      vertex   -7.891833e+01 1.128536e+02 2.548126e+02
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
    endloop
  endfacet
  facet normal -7.560239e-01 -3.723260e-03 6.545335e-01
    outer loop
      vertex   -8.010957e+01 1.128536e+02 2.532541e+02
      vertex   -7.972990e+01 1.128536e+02 2.536926e+02
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
    endloop
  endfacet
  facet normal -6.828089e-01 5.135632e-03 7.305790e-01
    outer loop
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
      vertex   -8.010957e+01 1.128536e+02 2.532541e+02
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
    endloop
  endfacet
  facet normal -6.884841e-01 1.812910e-03 7.252492e-01
    outer loop
      vertex   -8.219082e+01 1.128536e+02 2.512783e+02
      vertex   -8.010957e+01 1.128536e+02 2.532541e+02
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
    endloop
  endfacet
  facet normal -5.777048e-01 -1.595025e-03 8.162442e-01
    outer loop
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
      vertex   -8.219082e+01 1.128536e+02 2.512783e+02
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
    endloop
  endfacet
  facet normal -5.242833e-01 -1.407874e-02 8.514275e-01
    outer loop
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
    endloop
  endfacet
  facet normal -4.983079e-01 -6.030632e-03 8.669792e-01
    outer loop
      vertex   -8.459057e+01 1.128536e+02 2.497803e+02
      vertex   -8.308308e+01 1.128536e+02 2.506468e+02
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
    endloop
  endfacet
  facet normal -3.754410e-01 -8.032859e-03 9.268115e-01
    outer loop
      vertex   -8.676811e+01 1.189217e+02 2.489508e+02
      vertex   -8.459057e+01 1.128536e+02 2.497803e+02
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
    endloop
  endfacet
  facet normal -3.437057e-01 5.030474e-03 9.390639e-01
    outer loop
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
      vertex   -8.459057e+01 1.128536e+02 2.497803e+02
      vertex   -8.676811e+01 1.189217e+02 2.489508e+02
    endloop
  endfacet
  facet normal -1.882619e-01 -6.133582e-03 9.820997e-01
    outer loop
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
      vertex   -8.676811e+01 1.189217e+02 2.489508e+02
    endloop
  endfacet
  facet normal -1.455460e-01 1.220980e-02 9.892761e-01
    outer loop
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -8.717199e+01 1.128536e+02 2.488355e+02
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
    endloop
  endfacet
  facet normal -6.413480e-02 2.179629e-03 9.979389e-01
    outer loop
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -9.040541e+01 1.128536e+02 2.483598e+02
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
    endloop
  endfacet
  facet normal -9.998136e-01 -2.708287e-04 1.930779e-02
    outer loop
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
      vertex   -7.520707e+01 1.128536e+02 2.696024e+02
    endloop
  endfacet
  facet normal -9.997793e-01 -1.603420e-03 2.094561e-02
    outer loop
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.520707e+01 1.128536e+02 2.696024e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
    endloop
  endfacet
  facet normal -9.934773e-01 -5.675560e-03 1.138885e-01
    outer loop
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.585338e+01 1.199791e+02 2.637552e+02
    endloop
  endfacet
  facet normal -9.798175e-01 -6.311065e-04 1.998932e-01
    outer loop
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -7.585338e+01 1.199791e+02 2.637552e+02
    endloop
  endfacet
  facet normal -9.719414e-01 -1.721063e-02 2.345926e-01
    outer loop
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -7.585338e+01 1.199791e+02 2.637552e+02
    endloop
  endfacet
  facet normal -9.513022e-01 3.427006e-03 3.082409e-01
    outer loop
      vertex   -7.651282e+01 1.128536e+02 2.605003e+02
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
    endloop
  endfacet
  facet normal -9.169019e-01 -1.926451e-02 3.986473e-01
    outer loop
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -7.807343e+01 1.128536e+02 2.562560e+02
    endloop
  endfacet
  facet normal -9.089385e-01 -1.025438e-02 4.168041e-01
    outer loop
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -7.734573e+01 1.128536e+02 2.579298e+02
    endloop
  endfacet
  facet normal -8.630169e-01 2.760833e-03 5.051675e-01
    outer loop
      vertex   -7.807343e+01 1.128536e+02 2.562560e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -7.891833e+01 1.128536e+02 2.548126e+02
    endloop
  endfacet
  facet normal -9.947587e-01 2.785877e-03 1.022124e-01
    outer loop
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.576634e+01 1.128536e+02 2.641593e+02
      vertex   -7.520707e+01 1.128536e+02 2.696024e+02
    endloop
  endfacet
  facet normal -8.425738e-01 3.833336e-03 -5.385672e-01
    outer loop
      vertex   -7.858759e+01 1.168965e+02 2.921137e+02
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
      vertex   -7.948557e+01 1.165605e+02 2.935162e+02
    endloop
  endfacet
  facet normal -8.669792e-01 2.391952e-02 -4.977700e-01
    outer loop
      vertex   -7.788317e+01 1.128536e+02 2.906925e+02
      vertex   -7.952127e+01 1.128536e+02 2.935456e+02
      vertex   -7.858759e+01 1.168965e+02 2.921137e+02
    endloop
  endfacet
  facet normal -8.974580e-01 -1.312818e-03 -4.410980e-01
    outer loop
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
      vertex   -7.788317e+01 1.128536e+02 2.906925e+02
      vertex   -7.858759e+01 1.168965e+02 2.921137e+02
    endloop
  endfacet
  facet normal -9.409412e-01 -7.646078e-03 -3.384838e-01
    outer loop
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -7.788317e+01 1.128536e+02 2.906925e+02
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
    endloop
  endfacet
  facet normal -9.363437e-01 3.638975e-03 -3.510659e-01
    outer loop
      vertex   -7.671030e+01 1.178709e+02 2.874870e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
    endloop
  endfacet
  facet normal -9.699483e-01 4.169579e-03 -2.432753e-01
    outer loop
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -7.671030e+01 1.178709e+02 2.874870e+02
    endloop
  endfacet
  facet normal -9.745222e-01 1.355842e-02 -2.238807e-01
    outer loop
      vertex   -7.579833e+01 1.128536e+02 2.834219e+02
      vertex   -7.673605e+01 1.128536e+02 2.875037e+02
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
    endloop
  endfacet
  facet normal -9.855809e-01 -2.945619e-03 -1.691792e-01
    outer loop
      vertex   -7.564920e+01 1.187178e+02 2.824510e+02
      vertex   -7.579833e+01 1.128536e+02 2.834219e+02
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
    endloop
  endfacet
  facet normal -9.927552e-01 5.373153e-03 -1.200347e-01
    outer loop
      vertex   -7.528823e+01 1.128536e+02 2.792031e+02
      vertex   -7.579833e+01 1.128536e+02 2.834219e+02
      vertex   -7.564920e+01 1.187178e+02 2.824510e+02
    endloop
  endfacet
  facet normal -9.947435e-01 -4.574333e-03 -1.022958e-01
    outer loop
      vertex   -7.522482e+01 1.192562e+02 2.783002e+02
      vertex   -7.528823e+01 1.128536e+02 2.792031e+02
      vertex   -7.564920e+01 1.187178e+02 2.824510e+02
    endloop
  endfacet
  facet normal -9.991515e-01 4.116052e-03 -4.097865e-02
    outer loop
      vertex   -7.528823e+01 1.128536e+02 2.792031e+02
      vertex   -7.522482e+01 1.192562e+02 2.783002e+02
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
    endloop
  endfacet
  facet normal -9.993012e-01 2.102969e-03 -3.731787e-02
    outer loop
      vertex   -7.522482e+01 1.192562e+02 2.783002e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
      vertex   -7.510700e+01 1.128536e+02 2.747842e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
      vertex   -5.506548e+01 1.193837e+02 2.529787e+02
      vertex   -5.506548e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal -1.000000e+00 -0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
      vertex   -5.506548e+01 1.193837e+02 2.529787e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
      vertex   -5.506548e+01 1.198604e+02 2.592136e+02
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -5.506548e+01 1.193837e+02 2.529787e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -5.506548e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
      vertex   -5.506548e+01 1.198604e+02 2.592136e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
      vertex   -5.506548e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
      vertex   -3.460164e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal -1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
      vertex   -3.460164e+01 1.199717e+02 2.676038e+02
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
    endloop
  endfacet
  facet normal -8.257205e-01 -9.894278e-07 -5.640795e-01
    outer loop
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal -8.257199e-01 -2.943148e-06 -5.640804e-01
    outer loop
      vertex   -5.541862e+01 1.153514e+02 2.980765e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal -8.257226e-01 -0.000000e+00 -5.640765e-01
    outer loop
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -5.541862e+01 1.153514e+02 2.980765e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal -8.257203e-01 6.560432e-07 -5.640797e-01
    outer loop
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal -8.257206e-01 -1.541838e-06 -5.640793e-01
    outer loop
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
      vertex   -4.387629e+01 1.188915e+02 2.811804e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal -8.257197e-01 2.651108e-06 -5.640806e-01
    outer loop
      vertex   -4.387629e+01 1.188915e+02 2.811804e+02
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal -8.257211e-01 0.000000e+00 -5.640787e-01
    outer loop
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
      vertex   -3.460164e+01 1.199717e+02 2.676038e+02
    endloop
  endfacet
  facet normal -8.257205e-01 4.120540e-07 -5.640794e-01
    outer loop
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
      vertex   -3.460164e+01 1.128536e+02 2.676038e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.993569e+02
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -5.629332e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
      vertex   -6.434243e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.195434e+02 2.754587e+02
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.197945e+02 2.718024e+02
      vertex   -6.434243e+01 1.195434e+02 2.754587e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.199710e+02 2.676379e+02
      vertex   -6.434243e+01 1.197945e+02 2.718024e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -6.434243e+01 1.128536e+02 2.993569e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -7.054979e+01 1.199710e+02 2.676379e+02
      vertex   -6.434243e+01 1.199710e+02 2.676379e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -7.054979e+01 1.128536e+02 2.676379e+02
      vertex   -7.054979e+01 1.199710e+02 2.676379e+02
      vertex   -6.434243e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -7.054979e+01 1.128536e+02 2.592136e+02
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
      vertex   -7.054979e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.054979e+01 1.198604e+02 2.592136e+02
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
      vertex   -7.054979e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
      vertex   -7.054979e+01 1.199710e+02 2.676379e+02
      vertex   -7.054979e+01 1.128536e+02 2.676379e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.434243e+01 1.198604e+02 2.592136e+02
      vertex   -7.054979e+01 1.198604e+02 2.592136e+02
      vertex   -7.054979e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
      vertex   -6.434243e+01 1.198604e+02 2.592136e+02
      vertex   -7.054979e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
      vertex   -6.434243e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.434243e+01 1.197716e+02 2.576744e+02
      vertex   -6.434243e+01 1.198604e+02 2.592136e+02
      vertex   -6.434243e+01 1.128536e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 -0.000000e+00 1.000000e+00
    outer loop
      vertex   -5.506548e+01 1.128536e+02 2.491523e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -6.434243e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
      vertex   -1.076126e+01 1.197988e+02 2.580540e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 -1.000000e+00
    outer loop
      vertex   -2.979263e+01 1.128536e+02 2.580540e+02
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 0.000000e+00
    outer loop
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
      vertex   -2.979263e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.979263e+01 1.189489e+02 2.491523e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
    endloop
  endfacet
  facet normal 1.000000e+00 0.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
      vertex   -2.979263e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -3.000000e+00 1.189489e+02 2.491523e+02
      vertex   3.802183e+00 1.128536e+02 2.491523e+02
      vertex   3.802183e+00 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
      vertex   3.802183e+00 1.128536e+02 2.491523e+02
      vertex   -3.000000e+00 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -0.000000e+00 0.000000e+00 1.000000e+00
    outer loop
      vertex   -2.979263e+01 1.189489e+02 2.491523e+02
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
      vertex   -3.000000e+00 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -8.664076e-01 1.405403e-02 -4.991396e-01
    outer loop
      vertex   1.825630e+00 1.128536e+02 2.562442e+02
      vertex   -3.087892e-01 1.128536e+02 2.599492e+02
      vertex   1.029275e+00 1.197828e+02 2.578217e+02
    endloop
  endfacet
  facet normal -9.108654e-01 -1.076441e-02 -4.125631e-01
    outer loop
      vertex   2.615688e+00 1.195235e+02 2.543259e+02
      vertex   1.825630e+00 1.128536e+02 2.562442e+02
      vertex   1.029275e+00 1.197828e+02 2.578217e+02
    endloop
  endfacet
  facet normal -9.378775e-01 1.135497e-02 -3.467808e-01
    outer loop
      vertex   2.972715e+00 1.128536e+02 2.531419e+02
      vertex   1.825630e+00 1.128536e+02 2.562442e+02
      vertex   2.615688e+00 1.195235e+02 2.543259e+02
    endloop
  endfacet
  facet normal -9.548842e-01 1.603324e-03 -2.969742e-01
    outer loop
      vertex   3.104197e+00 1.193715e+02 2.527543e+02
      vertex   2.972715e+00 1.128536e+02 2.531419e+02
      vertex   2.615688e+00 1.195235e+02 2.543259e+02
    endloop
  endfacet
  facet normal -9.750410e-01 6.472312e-03 -2.219305e-01
    outer loop
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   2.972715e+00 1.128536e+02 2.531419e+02
      vertex   3.104197e+00 1.193715e+02 2.527543e+02
    endloop
  endfacet
  facet normal -9.752691e-01 6.028415e-03 -2.209387e-01
    outer loop
      vertex   3.621309e+00 1.191163e+02 2.504647e+02
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   3.104197e+00 1.193715e+02 2.527543e+02
    endloop
  endfacet
  facet normal -9.906506e-01 -7.875815e-04 -1.364214e-01
    outer loop
      vertex   3.802183e+00 1.189489e+02 2.491523e+02
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   3.621309e+00 1.191163e+02 2.504647e+02
    endloop
  endfacet
  facet normal -9.914254e-01 0.000000e+00 -1.306739e-01
    outer loop
      vertex   3.802183e+00 1.128536e+02 2.491523e+02
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   3.802183e+00 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -7.141664e-01 2.406470e-03 -6.999718e-01
    outer loop
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
      vertex   -3.713972e+00 1.199948e+02 2.641674e+02
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
    endloop
  endfacet
  facet normal -7.167571e-01 -6.904263e-04 -6.973226e-01
    outer loop
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
    endloop
  endfacet
  facet normal -7.643155e-01 7.187642e-04 -6.448420e-01
    outer loop
      vertex   -3.713972e+00 1.199948e+02 2.641674e+02
      vertex   -1.336844e+00 1.199410e+02 2.613498e+02
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
    endloop
  endfacet
  facet normal -7.778646e-01 1.133930e-02 -6.283297e-01
    outer loop
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
      vertex   -1.336844e+00 1.199410e+02 2.613498e+02
      vertex   -3.087892e-01 1.128536e+02 2.599492e+02
    endloop
  endfacet
  facet normal -8.306949e-01 -1.049369e-02 -5.566291e-01
    outer loop
      vertex   -1.336844e+00 1.199410e+02 2.613498e+02
      vertex   1.029275e+00 1.197828e+02 2.578217e+02
      vertex   -3.087892e-01 1.128536e+02 2.599492e+02
    endloop
  endfacet
  facet normal -7.534926e-01 -7.497369e-03 -6.574136e-01
    outer loop
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
    endloop
  endfacet
  facet normal -7.282242e-01 1.230040e-02 -6.852285e-01
    outer loop
      vertex   -1.339072e+01 1.196755e+02 2.737113e+02
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
    endloop
  endfacet
  facet normal -6.978226e-01 -7.954817e-03 -7.162265e-01
    outer loop
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
      vertex   -1.339072e+01 1.196755e+02 2.737113e+02
    endloop
  endfacet
  facet normal -6.932754e-01 -6.419322e-04 -7.206725e-01
    outer loop
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
      vertex   -1.339072e+01 1.196755e+02 2.737113e+02
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
    endloop
  endfacet
  facet normal -9.978178e-01 3.411120e-03 -6.593928e-02
    outer loop
      vertex   -2.005551e+01 1.185879e+02 2.833532e+02
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -2.014447e+01 1.183714e+02 2.846882e+02
    endloop
  endfacet
  facet normal -9.831173e-01 -1.363045e-02 -1.824678e-01
    outer loop
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -2.005551e+01 1.185879e+02 2.833532e+02
    endloop
  endfacet
  facet normal -9.743552e-01 -3.221170e-03 -2.249925e-01
    outer loop
      vertex   -1.979407e+01 1.187579e+02 2.822186e+02
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -2.005551e+01 1.185879e+02 2.833532e+02
    endloop
  endfacet
  facet normal -9.247378e-01 2.645695e-03 -3.805957e-01
    outer loop
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -1.979407e+01 1.187579e+02 2.822186e+02
    endloop
  endfacet
  facet normal -9.379483e-01 1.373025e-02 -3.465032e-01
    outer loop
      vertex   -1.938246e+01 1.128536e+02 2.810817e+02
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
    endloop
  endfacet
  facet normal -8.684957e-01 -9.614083e-03 -4.956035e-01
    outer loop
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
      vertex   -1.938246e+01 1.128536e+02 2.810817e+02
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
    endloop
  endfacet
  facet normal -8.513594e-01 3.826874e-04 -5.245828e-01
    outer loop
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
    endloop
  endfacet
  facet normal -7.992694e-01 -1.133078e-03 6.009718e-01
    outer loop
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
      vertex   -1.865998e+01 1.174848e+02 2.894377e+02
    endloop
  endfacet
  facet normal -8.752347e-01 2.712857e-02 4.829371e-01
    outer loop
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
      vertex   -1.865998e+01 1.174848e+02 2.894377e+02
    endloop
  endfacet
  facet normal -9.050417e-01 8.783621e-03 4.252322e-01
    outer loop
      vertex   -1.974450e+01 1.128536e+02 2.875657e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
    endloop
  endfacet
  facet normal -9.665914e-01 2.008230e-03 2.563143e-01
    outer loop
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
      vertex   -1.974450e+01 1.128536e+02 2.875657e+02
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
    endloop
  endfacet
  facet normal -9.743152e-01 6.876117e-03 2.250836e-01
    outer loop
      vertex   -2.009433e+01 1.181431e+02 2.860262e+02
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
    endloop
  endfacet
  facet normal -9.982809e-01 -1.319180e-02 5.710669e-02
    outer loop
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
      vertex   -2.009433e+01 1.181431e+02 2.860262e+02
    endloop
  endfacet
  facet normal -9.993184e-01 -5.976789e-03 3.642889e-02
    outer loop
      vertex   -2.014447e+01 1.183714e+02 2.846882e+02
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -2.009433e+01 1.181431e+02 2.860262e+02
    endloop
  endfacet
  facet normal -4.212737e-02 -2.554955e-02 9.987855e-01
    outer loop
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -1.385739e+01 1.170593e+02 2.914029e+02
    endloop
  endfacet
  facet normal -6.249201e-02 -1.854004e-02 9.978732e-01
    outer loop
      vertex   -1.481712e+01 1.170728e+02 2.913431e+02
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.385739e+01 1.170593e+02 2.914029e+02
    endloop
  endfacet
  facet normal -1.519425e-01 -8.111952e-03 9.883560e-01
    outer loop
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.481712e+01 1.170728e+02 2.913431e+02
    endloop
  endfacet
  facet normal -2.063205e-01 -8.646653e-03 9.784463e-01
    outer loop
      vertex   -1.623726e+01 1.128536e+02 2.910364e+02
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
    endloop
  endfacet
  facet normal -4.262251e-01 4.168829e-02 9.036560e-01
    outer loop
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
      vertex   -1.623726e+01 1.128536e+02 2.910364e+02
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
    endloop
  endfacet
  facet normal -3.867821e-01 2.042941e-02 9.219448e-01
    outer loop
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
    endloop
  endfacet
  facet normal -6.090901e-01 -1.092981e-02 7.930257e-01
    outer loop
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
    endloop
  endfacet
  facet normal -6.648844e-01 6.099837e-03 7.469213e-01
    outer loop
      vertex   -1.865998e+01 1.174848e+02 2.894377e+02
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
    endloop
  endfacet
  facet normal 5.465757e-01 -1.532424e-02 8.372695e-01
    outer loop
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
    endloop
  endfacet
  facet normal 3.407630e-01 3.662943e-02 9.394354e-01
    outer loop
      vertex   -1.245231e+01 1.170824e+02 2.913000e+02
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
    endloop
  endfacet
  facet normal 2.418106e-01 -6.720957e-03 9.703002e-01
    outer loop
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -1.245231e+01 1.170824e+02 2.913000e+02
    endloop
  endfacet
  facet normal 7.296955e-02 5.045333e-03 9.973214e-01
    outer loop
      vertex   -1.385739e+01 1.170593e+02 2.914029e+02
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -1.245231e+01 1.170824e+02 2.913000e+02
    endloop
  endfacet
  facet normal 9.951501e-01 0.000000e+00 9.836789e-02
    outer loop
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 9.889690e-01 7.509951e-03 1.479321e-01
    outer loop
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 9.825445e-01 -4.055301e-03 1.859835e-01
    outer loop
      vertex   -7.382220e+00 1.128536e+02 2.866966e+02
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal 9.351159e-01 1.898078e-02 3.538333e-01
    outer loop
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   -7.382220e+00 1.128536e+02 2.866966e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal 9.421343e-01 1.011254e-02 3.350831e-01
    outer loop
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal 8.503447e-01 -5.299142e-04 5.262258e-01
    outer loop
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
    endloop
  endfacet
  facet normal 7.173124e-01 3.083112e-02 6.960692e-01
    outer loop
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
    endloop
  endfacet
  facet normal 7.292892e-01 2.417838e-02 6.837782e-01
    outer loop
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
    endloop
  endfacet
  facet normal -9.950212e-02 0.000000e+00 9.950373e-01
    outer loop
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
      vertex   -4.074998e+00 1.185047e+02 2.838795e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal -9.950317e-02 5.237818e-07 9.950372e-01
    outer loop
      vertex   2.676671e+00 1.183960e+02 2.845547e+02
      vertex   -4.074998e+00 1.185047e+02 2.838795e+02
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
    endloop
  endfacet
  facet normal -9.950287e-02 0.000000e+00 9.950373e-01
    outer loop
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
      vertex   2.676671e+00 1.128536e+02 2.845547e+02
      vertex   2.676671e+00 1.183960e+02 2.845547e+02
    endloop
  endfacet
  facet normal -7.108571e-01 -6.683251e-03 -7.033047e-01
    outer loop
      vertex   -7.808797e-01 1.164357e+02 2.940230e+02
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
      vertex   -2.472520e+00 1.159962e+02 2.957370e+02
    endloop
  endfacet
  facet normal -6.974323e-01 -1.801851e-02 -7.164242e-01
    outer loop
      vertex   -1.009835e+00 1.128536e+02 2.943360e+02
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
      vertex   -7.808797e-01 1.164357e+02 2.940230e+02
    endloop
  endfacet
  facet normal -8.101114e-01 5.551004e-04 -5.862757e-01
    outer loop
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
      vertex   -1.009835e+00 1.128536e+02 2.943360e+02
      vertex   -7.808797e-01 1.164357e+02 2.940230e+02
    endloop
  endfacet
  facet normal -8.288256e-01 -1.339199e-02 -5.593467e-01
    outer loop
      vertex   3.913928e-01 1.168555e+02 2.922759e+02
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
      vertex   -7.808797e-01 1.164357e+02 2.940230e+02
    endloop
  endfacet
  facet normal -8.918675e-01 4.715154e-03 -4.522721e-01
    outer loop
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
      vertex   3.913928e-01 1.168555e+02 2.922759e+02
    endloop
  endfacet
  facet normal -9.154656e-01 -2.265587e-02 -4.017580e-01
    outer loop
      vertex   1.678921e+00 1.175079e+02 2.893053e+02
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
      vertex   3.913928e-01 1.168555e+02 2.922759e+02
    endloop
  endfacet
  facet normal -9.446995e-01 -8.606046e-04 -3.279361e-01
    outer loop
      vertex   1.824653e+00 1.128536e+02 2.888977e+02
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
      vertex   1.678921e+00 1.175079e+02 2.893053e+02
    endloop
  endfacet
  facet normal -9.761881e-01 -1.159547e-02 -2.166155e-01
    outer loop
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
      vertex   1.824653e+00 1.128536e+02 2.888977e+02
      vertex   1.678921e+00 1.175079e+02 2.893053e+02
    endloop
  endfacet
  facet normal -9.715720e-01 3.557095e-03 -2.367176e-01
    outer loop
      vertex   2.466444e+00 1.181325e+02 2.860824e+02
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
      vertex   1.678921e+00 1.175079e+02 2.893053e+02
    endloop
  endfacet
  facet normal -9.906990e-01 1.567331e-03 -1.360624e-01
    outer loop
      vertex   2.676671e+00 1.183960e+02 2.845547e+02
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
      vertex   2.466444e+00 1.181325e+02 2.860824e+02
    endloop
  endfacet
  facet normal -9.898608e-01 -0.000000e+00 -1.420405e-01
    outer loop
      vertex   2.676671e+00 1.183960e+02 2.845547e+02
      vertex   2.676671e+00 1.128536e+02 2.845547e+02
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
    endloop
  endfacet
  facet normal -9.803521e-02 -3.710942e-03 -9.951760e-01
    outer loop
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
    endloop
  endfacet
  facet normal -6.487730e-02 5.233257e-02 -9.965201e-01
    outer loop
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
      vertex   -1.073784e+01 1.150429e+02 2.991338e+02
    endloop
  endfacet
  facet normal -2.130958e-01 -1.376054e-02 -9.769344e-01
    outer loop
      vertex   -7.682700e+00 1.152392e+02 2.984647e+02
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
      vertex   -1.073784e+01 1.150429e+02 2.991338e+02
    endloop
  endfacet
  facet normal -2.637280e-01 3.352501e-02 -9.640143e-01
    outer loop
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
      vertex   -7.682700e+00 1.152392e+02 2.984647e+02
    endloop
  endfacet
  facet normal -3.675627e-01 -2.029655e-02 -9.297772e-01
    outer loop
      vertex   -5.640534e+00 1.154711e+02 2.976523e+02
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
      vertex   -7.682700e+00 1.152392e+02 2.984647e+02
    endloop
  endfacet
  facet normal -4.661388e-01 2.222208e-02 -8.844325e-01
    outer loop
      vertex   -3.319502e+00 1.128536e+02 2.963632e+02
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
      vertex   -5.640534e+00 1.154711e+02 2.976523e+02
    endloop
  endfacet
  facet normal -5.131343e-01 -3.262233e-02 -8.576881e-01
    outer loop
      vertex   -2.472520e+00 1.159962e+02 2.957370e+02
      vertex   -3.319502e+00 1.128536e+02 2.963632e+02
      vertex   -5.640534e+00 1.154711e+02 2.976523e+02
    endloop
  endfacet
  facet normal -6.048463e-01 4.322877e-03 -7.963305e-01
    outer loop
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
      vertex   -3.319502e+00 1.128536e+02 2.963632e+02
      vertex   -2.472520e+00 1.159962e+02 2.957370e+02
    endloop
  endfacet
  facet normal 6.595333e-01 -1.546550e-02 -7.515162e-01
    outer loop
      vertex   -2.446381e+01 1.158529e+02 2.962714e+02
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
      vertex   -2.610656e+01 1.162351e+02 2.948219e+02
    endloop
  endfacet
  facet normal 6.317061e-01 -2.126494e-04 -7.752079e-01
    outer loop
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
      vertex   -2.446381e+01 1.158529e+02 2.962714e+02
    endloop
  endfacet
  facet normal 5.048806e-01 8.067675e-03 -8.631515e-01
    outer loop
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -2.446381e+01 1.158529e+02 2.962714e+02
    endloop
  endfacet
  facet normal 4.935685e-01 2.516184e-02 -8.693428e-01
    outer loop
      vertex   -2.117292e+01 1.128536e+02 2.981096e+02
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
    endloop
  endfacet
  facet normal 3.064262e-01 -1.908413e-02 -9.517031e-01
    outer loop
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
      vertex   -2.117292e+01 1.128536e+02 2.981096e+02
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
    endloop
  endfacet
  facet normal 2.623851e-01 2.779026e-02 -9.645630e-01
    outer loop
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -2.117292e+01 1.128536e+02 2.981096e+02
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
    endloop
  endfacet
  facet normal 1.907300e-01 -3.646976e-03 -9.816358e-01
    outer loop
      vertex   -1.690145e+01 1.150292e+02 2.991803e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
    endloop
  endfacet
  facet normal 1.136967e-01 3.529509e-02 -9.928884e-01
    outer loop
      vertex   -1.637812e+01 1.150115e+02 2.992396e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -1.690145e+01 1.150292e+02 2.991803e+02
    endloop
  endfacet
  facet normal 8.692063e-02 5.528814e-02 -9.946799e-01
    outer loop
      vertex   -1.582586e+01 1.149974e+02 2.992870e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -1.637812e+01 1.150115e+02 2.992396e+02
    endloop
  endfacet
  facet normal 8.269702e-02 5.953300e-02 -9.947950e-01
    outer loop
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -1.582586e+01 1.149974e+02 2.992870e+02
    endloop
  endfacet
  facet normal 6.749305e-03 -1.727471e-02 -9.998280e-01
    outer loop
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
      vertex   -1.582586e+01 1.149974e+02 2.992870e+02
    endloop
  endfacet
  facet normal 9.999326e-01 -6.822133e-03 9.396013e-03
    outer loop
      vertex   -2.973875e+01 1.180351e+02 2.866187e+02
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
    endloop
  endfacet
  facet normal 9.926057e-01 5.513927e-03 -1.212578e-01
    outer loop
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -2.973875e+01 1.180351e+02 2.866187e+02
    endloop
  endfacet
  facet normal 9.900242e-01 -7.050380e-04 -1.408954e-01
    outer loop
      vertex   -2.945709e+01 1.176557e+02 2.885998e+02
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -2.973875e+01 1.180351e+02 2.866187e+02
    endloop
  endfacet
  facet normal 9.534592e-01 1.207252e-02 -3.012802e-01
    outer loop
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -2.945709e+01 1.176557e+02 2.885998e+02
    endloop
  endfacet
  facet normal 9.588294e-01 6.006302e-04 -2.839820e-01
    outer loop
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
    endloop
  endfacet
  facet normal 9.041506e-01 1.865277e-03 -4.272097e-01
    outer loop
      vertex   -2.831703e+01 1.169801e+02 2.917471e+02
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
    endloop
  endfacet
  facet normal 8.033410e-01 -1.072323e-03 -5.955184e-01
    outer loop
      vertex   -2.742692e+01 1.128536e+02 2.932240e+02
      vertex   -2.702544e+01 1.165023e+02 2.937590e+02
      vertex   -2.703027e+01 1.128536e+02 2.937591e+02
    endloop
  endfacet
  facet normal 8.400535e-01 -1.290802e-02 -5.423499e-01
    outer loop
      vertex   -2.831703e+01 1.169801e+02 2.917471e+02
      vertex   -2.702544e+01 1.165023e+02 2.937590e+02
      vertex   -2.742692e+01 1.128536e+02 2.932240e+02
    endloop
  endfacet
  facet normal 8.764498e-01 1.682978e-02 -4.811990e-01
    outer loop
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -2.831703e+01 1.169801e+02 2.917471e+02
      vertex   -2.742692e+01 1.128536e+02 2.932240e+02
    endloop
  endfacet
  facet normal 7.334627e-01 1.323858e-02 -6.796008e-01
    outer loop
      vertex   -2.703027e+01 1.128536e+02 2.937591e+02
      vertex   -2.610656e+01 1.162351e+02 2.948219e+02
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
    endloop
  endfacet
  facet normal 7.563552e-01 -1.011071e-03 -6.541604e-01
    outer loop
      vertex   -2.702544e+01 1.165023e+02 2.937590e+02
      vertex   -2.610656e+01 1.162351e+02 2.948219e+02
      vertex   -2.703027e+01 1.128536e+02 2.937591e+02
    endloop
  endfacet
  facet normal 9.272582e-01 -7.836446e-03 3.743406e-01
    outer loop
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
    endloop
  endfacet
  facet normal 9.690940e-01 2.887291e-03 2.466747e-01
    outer loop
      vertex   -2.948985e+01 1.128536e+02 2.822215e+02
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
    endloop
  endfacet
  facet normal 9.842046e-01 -1.608106e-02 1.763029e-01
    outer loop
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
      vertex   -2.948985e+01 1.128536e+02 2.822215e+02
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
    endloop
  endfacet
  facet normal 9.973075e-01 1.698162e-02 7.134016e-02
    outer loop
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -2.948985e+01 1.128536e+02 2.822215e+02
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
    endloop
  endfacet
  facet normal 7.902923e-01 -1.004227e-02 6.126477e-01
    outer loop
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -2.342848e+01 1.198403e+02 2.711217e+02
    endloop
  endfacet
  facet normal 8.086618e-01 5.140644e-03 5.882513e-01
    outer loop
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
    endloop
  endfacet
  facet normal 8.728525e-01 3.740283e-03 4.879698e-01
    outer loop
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
      vertex   -2.802554e+01 1.128536e+02 2.777526e+02
    endloop
  endfacet
  facet normal 8.578856e-01 -3.832946e-03 5.138265e-01
    outer loop
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
    endloop
  endfacet
  facet normal 9.319179e-01 -1.465210e-02 3.623733e-01
    outer loop
      vertex   -2.802554e+01 1.128536e+02 2.777526e+02
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
    endloop
  endfacet
  facet normal 6.998767e-01 -8.027773e-03 7.142186e-01
    outer loop
      vertex   -2.342848e+01 1.198403e+02 2.711217e+02
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
    endloop
  endfacet
  facet normal 7.243590e-01 1.227966e-02 6.893136e-01
    outer loop
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
      vertex   -2.342848e+01 1.198403e+02 2.711217e+02
    endloop
  endfacet
  facet normal 6.884436e-01 -4.456713e-03 7.252762e-01
    outer loop
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
      vertex   -1.343593e+01 1.128536e+02 2.614384e+02
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
    endloop
  endfacet
  facet normal 6.901493e-01 -2.213907e-03 7.236636e-01
    outer loop
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
      vertex   -1.343593e+01 1.128536e+02 2.614384e+02
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.948985e+01 1.128536e+02 2.822215e+02
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -2.976952e+01 1.128536e+02 2.861312e+02
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
      vertex   -2.951313e+01 1.128536e+02 2.882300e+02
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.998914e+01 1.128536e+02 2.866432e+02
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -1.974450e+01 1.128536e+02 2.875657e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -2.742692e+01 1.128536e+02 2.932240e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.974450e+01 1.128536e+02 2.875657e+02
      vertex   -2.874515e+01 1.128536e+02 2.908230e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.742692e+01 1.128536e+02 2.932240e+02
      vertex   -2.703027e+01 1.128536e+02 2.937591e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -1.274021e+01 1.128536e+02 2.913425e+02
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -2.117292e+01 1.128536e+02 2.981096e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
      vertex   -2.458929e+01 1.128536e+02 2.961700e+02
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
      vertex   -1.855413e+01 1.128536e+02 2.895697e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.703027e+01 1.128536e+02 2.937591e+02
      vertex   -2.543482e+01 1.128536e+02 2.954810e+02
      vertex   -1.921742e+01 1.128536e+02 2.886876e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -9.785009e+00 1.128536e+02 2.989568e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -1.074180e+01 1.128536e+02 2.908444e+02
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.319502e+00 1.128536e+02 2.963632e+02
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -6.573566e+00 1.128536e+02 2.980783e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -3.319502e+00 1.128536e+02 2.963632e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.938246e+01 1.128536e+02 2.810817e+02
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
      vertex   -2.117292e+01 1.128536e+02 2.981096e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.623726e+01 1.128536e+02 2.910364e+02
      vertex   -1.729250e+01 1.128536e+02 2.905387e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.623726e+01 1.128536e+02 2.910364e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.802554e+01 1.128536e+02 2.777526e+02
      vertex   -1.938246e+01 1.128536e+02 2.810817e+02
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -1.938246e+01 1.128536e+02 2.810817e+02
      vertex   -2.802554e+01 1.128536e+02 2.777526e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -2.802554e+01 1.128536e+02 2.777526e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   -9.289949e+00 1.128536e+02 2.898967e+02
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.009835e+00 1.128536e+02 2.943360e+02
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   -2.285723e+00 1.128536e+02 2.955780e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -1.972801e+01 1.128536e+02 2.820170e+02
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.948985e+01 1.128536e+02 2.822215e+02
      vertex   -2.012991e+01 1.128536e+02 2.841824e+02
      vertex   -2.897164e+01 1.128536e+02 2.801857e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   -1.009835e+00 1.128536e+02 2.943360e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   -8.466074e+00 1.128536e+02 2.890476e+02
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.366126e+01 1.128536e+02 2.993387e+02
      vertex   -1.529745e+01 1.128536e+02 2.912346e+02
      vertex   -1.797203e+01 1.128536e+02 2.989803e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.087892e-01 1.128536e+02 2.599492e+02
      vertex   1.825630e+00 1.128536e+02 2.562442e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   1.108248e-01 1.128536e+02 2.927874e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -7.382220e+00 1.128536e+02 2.866966e+02
      vertex   -7.966106e+00 1.128536e+02 2.882397e+02
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.824653e+00 1.128536e+02 2.888977e+02
      vertex   -7.382220e+00 1.128536e+02 2.866966e+02
      vertex   1.263289e+00 1.128536e+02 2.905148e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   1.825630e+00 1.128536e+02 2.562442e+02
      vertex   2.972715e+00 1.128536e+02 2.531419e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -1.343593e+01 1.128536e+02 2.614384e+02
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   -7.382220e+00 1.128536e+02 2.866966e+02
      vertex   1.824653e+00 1.128536e+02 2.888977e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   1.824653e+00 1.128536e+02 2.888977e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
      vertex   -6.957398e+00 1.128536e+02 2.844523e+02
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.676671e+00 1.128536e+02 2.845547e+02
      vertex   -6.873124e+00 1.128536e+02 2.835997e+02
      vertex   2.472272e+00 1.128536e+02 2.859791e+02
    endloop
  endfacet
  facet normal -0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
      vertex   -1.343593e+01 1.128536e+02 2.614384e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.563201e+01 1.128536e+02 2.759708e+02
      vertex   -7.734220e+00 1.128536e+02 2.682759e+02
      vertex   -2.056629e+01 1.128536e+02 2.682385e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   2.972715e+00 1.128536e+02 2.531419e+02
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -3.613418e+00 1.128536e+02 2.640403e+02
      vertex   -3.087892e-01 1.128536e+02 2.599492e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 -0.000000e+00
    outer loop
      vertex   -2.442790e+01 1.128536e+02 2.722964e+02
      vertex   -2.679266e+01 1.128536e+02 2.755473e+02
      vertex   -1.804255e+01 1.128536e+02 2.787336e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
      vertex   -2.979263e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.802183e+00 1.128536e+02 2.491523e+02
      vertex   -2.979263e+01 1.128536e+02 2.491523e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 0.000000e+00 1.000000e+00 0.000000e+00
    outer loop
      vertex   3.694017e+00 1.128536e+02 2.499729e+02
      vertex   3.802183e+00 1.128536e+02 2.491523e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
    endloop
  endfacet
  facet normal 8.244664e-01 0.000000e+00 5.659110e-01
    outer loop
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
      vertex   -1.076126e+01 1.197988e+02 2.580540e+02
    endloop
  endfacet
  facet normal 8.260060e-01 7.556855e-04 5.636608e-01
    outer loop
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -1.076126e+01 1.128536e+02 2.580540e+02
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
    endloop
  endfacet
  facet normal 7.406022e-01 1.273477e-03 6.719426e-01
    outer loop
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
    endloop
  endfacet
  facet normal 7.467410e-01 5.392928e-03 6.650931e-01
    outer loop
      vertex   -1.343593e+01 1.128536e+02 2.614384e+02
      vertex   -1.187405e+01 1.128536e+02 2.596848e+02
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
    endloop
  endfacet
  facet normal -1.598961e-04 5.590316e-01 -8.291464e-01
    outer loop
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -2.400293e+01 6.782562e+01 1.660067e+02
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal -1.201829e-05 5.470773e-01 -8.370821e-01
    outer loop
      vertex   -3.000000e+00 6.447350e+01 1.637969e+02
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal -7.995907e-05 5.191883e-01 -8.546599e-01
    outer loop
      vertex   -3.000000e+00 6.044658e+01 1.613506e+02
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -3.000000e+00 6.447350e+01 1.637969e+02
    endloop
  endfacet
  facet normal -1.251396e-04 5.173135e-01 -8.557960e-01
    outer loop
      vertex   -2.570000e+02 5.975858e+01 1.609719e+02
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -3.000000e+00 6.044658e+01 1.613506e+02
    endloop
  endfacet
  facet normal 1.147199e-05 4.789998e-01 -8.778150e-01
    outer loop
      vertex   -3.000000e+00 5.470285e+01 1.582164e+02
      vertex   -2.570000e+02 5.975858e+01 1.609719e+02
      vertex   -3.000000e+00 6.044658e+01 1.613506e+02
    endloop
  endfacet
  facet normal -2.074317e-04 4.705007e-01 -8.823996e-01
    outer loop
      vertex   -2.570000e+02 5.283089e+01 1.572780e+02
      vertex   -2.570000e+02 5.975858e+01 1.609719e+02
      vertex   -3.000000e+00 5.470285e+01 1.582164e+02
    endloop
  endfacet
  facet normal 1.989950e-04 4.264411e-01 -9.045153e-01
    outer loop
      vertex   -3.000000e+00 4.736433e+01 1.547566e+02
      vertex   -2.570000e+02 5.283089e+01 1.572780e+02
      vertex   -3.000000e+00 5.470285e+01 1.582164e+02
    endloop
  endfacet
  facet normal -2.146569e-04 4.105902e-01 -9.118200e-01
    outer loop
      vertex   -2.570000e+02 4.543310e+01 1.539468e+02
      vertex   -2.570000e+02 5.283089e+01 1.572780e+02
      vertex   -3.000000e+00 4.736433e+01 1.547566e+02
    endloop
  endfacet
  facet normal 1.625864e-04 3.684499e-01 -9.296476e-01
    outer loop
      vertex   -3.000000e+00 4.128146e+01 1.523458e+02
      vertex   -2.570000e+02 4.543310e+01 1.539468e+02
      vertex   -3.000000e+00 4.736433e+01 1.547566e+02
    endloop
  endfacet
  facet normal -2.409876e-04 3.469370e-01 -9.378884e-01
    outer loop
      vertex   -2.570000e+02 3.806931e+01 1.512228e+02
      vertex   -2.570000e+02 4.543310e+01 1.539468e+02
      vertex   -3.000000e+00 4.128146e+01 1.523458e+02
    endloop
  endfacet
  facet normal 3.001188e-04 3.087796e-01 -9.511336e-01
    outer loop
      vertex   -3.000000e+00 3.256941e+01 1.495175e+02
      vertex   -2.570000e+02 3.806931e+01 1.512228e+02
      vertex   -3.000000e+00 4.128146e+01 1.523458e+02
    endloop
  endfacet
  facet normal -2.395189e-05 2.951516e-01 -9.554504e-01
    outer loop
      vertex   -2.570000e+02 3.206738e+01 1.493687e+02
      vertex   -2.570000e+02 3.806931e+01 1.512228e+02
      vertex   -3.000000e+00 3.256941e+01 1.495175e+02
    endloop
  endfacet
  facet normal 6.397849e-05 2.541348e-01 -9.671688e-01
    outer loop
      vertex   -3.000000e+00 2.801238e+01 1.483200e+02
      vertex   -2.570000e+02 3.206738e+01 1.493687e+02
      vertex   -3.000000e+00 3.256941e+01 1.495175e+02
    endloop
  endfacet
  facet normal -6.398587e-05 2.466218e-01 -9.691118e-01
    outer loop
      vertex   -2.570000e+02 2.750120e+01 1.482067e+02
      vertex   -2.570000e+02 3.206738e+01 1.493687e+02
      vertex   -3.000000e+00 2.801238e+01 1.483200e+02
    endloop
  endfacet
  facet normal 1.632047e-05 2.086849e-01 -9.779829e-01
    outer loop
      vertex   -3.000000e+00 2.238051e+01 1.471183e+02
      vertex   -2.570000e+02 2.750120e+01 1.482067e+02
      vertex   -3.000000e+00 2.801238e+01 1.483200e+02
    endloop
  endfacet
  facet normal -8.990053e-05 2.036418e-01 -9.790454e-01
    outer loop
      vertex   -2.570000e+02 2.135102e+01 1.469275e+02
      vertex   -2.570000e+02 2.750120e+01 1.482067e+02
      vertex   -3.000000e+00 2.238051e+01 1.471183e+02
    endloop
  endfacet
  facet normal 5.596938e-05 1.688741e-01 -9.856376e-01
    outer loop
      vertex   -3.000000e+00 1.773645e+01 1.463226e+02
      vertex   -2.570000e+02 2.135102e+01 1.469275e+02
      vertex   -3.000000e+00 2.238051e+01 1.471183e+02
    endloop
  endfacet
  facet normal -1.888670e-04 1.521249e-01 -9.883613e-01
    outer loop
      vertex   -2.570000e+02 1.514232e+01 1.459719e+02
      vertex   -2.570000e+02 2.135102e+01 1.469275e+02
      vertex   -3.000000e+00 1.773645e+01 1.463226e+02
    endloop
  endfacet
  facet normal 1.177556e-04 1.226552e-01 -9.924493e-01
    outer loop
      vertex   -3.000000e+00 1.202128e+01 1.456163e+02
      vertex   -2.570000e+02 1.514232e+01 1.459719e+02
      vertex   -3.000000e+00 1.773645e+01 1.463226e+02
    endloop
  endfacet
  facet normal -1.618459e-04 1.001874e-01 -9.949686e-01
    outer loop
      vertex   -2.570000e+02 8.892117e+00 1.453425e+02
      vertex   -2.570000e+02 1.514232e+01 1.459719e+02
      vertex   -3.000000e+00 1.202128e+01 1.456163e+02
    endloop
  endfacet
  facet normal 2.680410e-04 6.554475e-02 -9.978496e-01
    outer loop
      vertex   -3.000000e+00 3.925515e+00 1.450845e+02
      vertex   -2.570000e+02 8.892117e+00 1.453425e+02
      vertex   -3.000000e+00 1.202128e+01 1.456163e+02
    endloop
  endfacet
  facet normal -1.871765e-04 4.232934e-02 -9.991037e-01
    outer loop
      vertex   -2.570000e+02 1.832184e+00 1.450434e+02
      vertex   -2.570000e+02 8.892117e+00 1.453425e+02
      vertex   -3.000000e+00 3.925515e+00 1.450845e+02
    endloop
  endfacet
  facet normal 6.935270e-05 1.121905e-02 -9.999371e-01
    outer loop
      vertex   -3.000000e+00 -1.570557e+00 1.450228e+02
      vertex   -2.570000e+02 1.832184e+00 1.450434e+02
      vertex   -3.000000e+00 3.925515e+00 1.450845e+02
    endloop
  endfacet
  facet normal -2.991461e-04 -1.628823e-02 -9.998673e-01
    outer loop
      vertex   -2.570000e+02 -6.017731e+00 1.451713e+02
      vertex   -2.570000e+02 1.832184e+00 1.450434e+02
      vertex   -3.000000e+00 -1.570557e+00 1.450228e+02
    endloop
  endfacet
  facet normal 4.090333e-05 -3.569329e-02 -9.993628e-01
    outer loop
      vertex   -3.000000e+00 -7.325546e+00 1.452284e+02
      vertex   -2.570000e+02 -6.017731e+00 1.451713e+02
      vertex   -3.000000e+00 -1.570557e+00 1.450228e+02
    endloop
  endfacet
  facet normal -1.998223e-04 -8.232595e-02 -9.966054e-01
    outer loop
      vertex   -3.000000e+00 -1.202128e+01 1.456163e+02
      vertex   -2.570000e+02 -6.017731e+00 1.451713e+02
      vertex   -3.000000e+00 -7.325546e+00 1.452284e+02
    endloop
  endfacet
  facet normal -3.206076e-04 -8.740411e-02 -9.961729e-01
    outer loop
      vertex   -2.570000e+02 -1.514232e+01 1.459719e+02
      vertex   -2.570000e+02 -6.017731e+00 1.451713e+02
      vertex   -3.000000e+00 -1.202128e+01 1.456163e+02
    endloop
  endfacet
  facet normal 1.619086e-04 -1.261979e-01 -9.920051e-01
    outer loop
      vertex   -3.000000e+00 -1.825287e+01 1.464090e+02
      vertex   -2.570000e+02 -1.514232e+01 1.459719e+02
      vertex   -3.000000e+00 -1.202128e+01 1.456163e+02
    endloop
  endfacet
  facet normal -2.357836e-04 -1.580302e-01 -9.874343e-01
    outer loop
      vertex   -2.570000e+02 -2.211969e+01 1.470885e+02
      vertex   -2.570000e+02 -1.514232e+01 1.459719e+02
      vertex   -3.000000e+00 -1.825287e+01 1.464090e+02
    endloop
  endfacet
  facet normal 2.749144e-04 -1.905643e-01 -9.816747e-01
    outer loop
      vertex   -3.000000e+00 -2.724466e+01 1.481545e+02
      vertex   -2.570000e+02 -2.211969e+01 1.470885e+02
      vertex   -3.000000e+00 -1.825287e+01 1.464090e+02
    endloop
  endfacet
  facet normal -2.233472e-04 -2.142408e-01 -9.767808e-01
    outer loop
      vertex   -2.570000e+02 -3.004446e+01 1.488267e+02
      vertex   -2.570000e+02 -2.211969e+01 1.470885e+02
      vertex   -3.000000e+00 -2.724466e+01 1.481545e+02
    endloop
  endfacet
  facet normal 1.696024e-04 -2.479660e-01 -9.687687e-01
    outer loop
      vertex   -3.000000e+00 -3.256941e+01 1.495175e+02
      vertex   -2.570000e+02 -3.004446e+01 1.488267e+02
      vertex   -3.000000e+00 -2.724466e+01 1.481545e+02
    endloop
  endfacet
  facet normal -1.177264e-04 -2.748760e-01 -9.614797e-01
    outer loop
      vertex   -2.570000e+02 -3.558129e+01 1.504096e+02
      vertex   -2.570000e+02 -3.004446e+01 1.488267e+02
      vertex   -3.000000e+00 -3.256941e+01 1.495175e+02
    endloop
  endfacet
  facet normal 1.618576e-04 -2.965407e-01 -9.550202e-01
    outer loop
      vertex   -3.000000e+00 -3.856867e+01 1.513803e+02
      vertex   -2.570000e+02 -3.558129e+01 1.504096e+02
      vertex   -3.000000e+00 -3.256941e+01 1.495175e+02
    endloop
  endfacet
  facet normal -1.387607e-04 -3.196669e-01 -9.475300e-01
    outer loop
      vertex   -2.570000e+02 -4.078931e+01 1.521666e+02
      vertex   -2.570000e+02 -3.558129e+01 1.504096e+02
      vertex   -3.000000e+00 -3.856867e+01 1.513803e+02
    endloop
  endfacet
  facet normal 1.618636e-04 -3.502077e-01 -9.366721e-01
    outer loop
      vertex   -3.000000e+00 -4.543310e+01 1.539468e+02
      vertex   -2.570000e+02 -4.078931e+01 1.521666e+02
      vertex   -3.000000e+00 -3.856867e+01 1.513803e+02
    endloop
  endfacet
  facet normal -4.199684e-04 -3.778845e-01 -9.258526e-01
    outer loop
      vertex   -2.570000e+02 -4.975371e+01 1.558254e+02
      vertex   -2.570000e+02 -4.078931e+01 1.521666e+02
      vertex   -3.000000e+00 -4.543310e+01 1.539468e+02
    endloop
  endfacet
  facet normal -7.194659e-05 -3.951902e-01 -9.185993e-01
    outer loop
      vertex   -3.000000e+00 -4.976130e+01 1.558088e+02
      vertex   -2.570000e+02 -4.975371e+01 1.558254e+02
      vertex   -3.000000e+00 -4.543310e+01 1.539468e+02
    endloop
  endfacet
  facet normal -7.195833e-05 -4.252525e-01 -9.050747e-01
    outer loop
      vertex   -3.000000e+00 -5.260454e+01 1.571447e+02
      vertex   -2.570000e+02 -4.975371e+01 1.558254e+02
      vertex   -3.000000e+00 -4.976130e+01 1.558088e+02
    endloop
  endfacet
  facet normal -5.753894e-04 -4.617475e-01 -8.870112e-01
    outer loop
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
      vertex   -2.570000e+02 -4.975371e+01 1.558254e+02
      vertex   -3.000000e+00 -5.260454e+01 1.571447e+02
    endloop
  endfacet
  facet normal 1.918627e-04 -4.409815e-01 -8.975162e-01
    outer loop
      vertex   -2.570000e+02 -5.609572e+01 1.589415e+02
      vertex   -2.570000e+02 -4.975371e+01 1.558254e+02
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
    endloop
  endfacet
  facet normal -1.798077e-04 -5.045270e-01 -8.633959e-01
    outer loop
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
      vertex   -2.570000e+02 -5.609572e+01 1.589415e+02
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
    endloop
  endfacet
  facet normal 2.802417e-04 -4.934715e-01 -8.697619e-01
    outer loop
      vertex   -2.570000e+02 -6.269744e+01 1.626871e+02
      vertex   -2.570000e+02 -5.609572e+01 1.589415e+02
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
    endloop
  endfacet
  facet normal -1.439189e-04 -5.519367e-01 -8.338860e-01
    outer loop
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -2.570000e+02 -6.269744e+01 1.626871e+02
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
    endloop
  endfacet
  facet normal -3.595957e-04 -5.435970e-01 -8.393463e-01
    outer loop
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
    endloop
  endfacet
  facet normal -2.370028e-04 -5.577913e-01 -8.299812e-01
    outer loop
      vertex   -2.143748e+01 -6.779947e+01 1.659887e+02
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal -1.277570e-04 6.443924e-01 -7.646950e-01
    outer loop
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
      vertex   -2.580776e+00 7.980245e+01 1.753838e+02
      vertex   2.510000e+02 7.872691e+01 1.744351e+02
    endloop
  endfacet
  facet normal 2.065408e-04 6.305079e-01 -7.761828e-01
    outer loop
      vertex   2.510000e+02 7.308392e+01 1.698512e+02
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
      vertex   2.510000e+02 7.872691e+01 1.744351e+02
    endloop
  endfacet
  facet normal -7.263201e-05 6.087606e-01 -7.933540e-01
    outer loop
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
      vertex   2.510000e+02 7.308392e+01 1.698512e+02
    endloop
  endfacet
  facet normal 3.430314e-04 5.769888e-01 -8.167520e-01
    outer loop
      vertex   2.510000e+02 6.520527e+01 1.642853e+02
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   2.510000e+02 7.308392e+01 1.698512e+02
    endloop
  endfacet
  facet normal 3.615910e-04 5.775409e-01 -8.163617e-01
    outer loop
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   2.510000e+02 6.520527e+01 1.642853e+02
    endloop
  endfacet
  facet normal 3.361629e-05 5.470773e-01 -8.370821e-01
    outer loop
      vertex   -3.000000e+00 6.447350e+01 1.637969e+02
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
      vertex   2.510000e+02 6.520527e+01 1.642853e+02
    endloop
  endfacet
  facet normal 1.565564e-04 5.170160e-01 -8.559757e-01
    outer loop
      vertex   2.510000e+02 5.828617e+01 1.601061e+02
      vertex   -3.000000e+00 6.447350e+01 1.637969e+02
      vertex   2.510000e+02 6.520527e+01 1.642853e+02
    endloop
  endfacet
  facet normal 2.285926e-04 5.191883e-01 -8.546599e-01
    outer loop
      vertex   -3.000000e+00 6.044658e+01 1.613506e+02
      vertex   -3.000000e+00 6.447350e+01 1.637969e+02
      vertex   2.510000e+02 5.828617e+01 1.601061e+02
    endloop
  endfacet
  facet normal -2.266809e-04 4.789998e-01 -8.778150e-01
    outer loop
      vertex   -3.000000e+00 5.470285e+01 1.582164e+02
      vertex   -3.000000e+00 6.044658e+01 1.613506e+02
      vertex   2.510000e+02 5.828617e+01 1.601061e+02
    endloop
  endfacet
  facet normal -3.171806e-04 4.839737e-01 -8.750825e-01
    outer loop
      vertex   8.856785e+01 5.496808e+01 1.583299e+02
      vertex   -3.000000e+00 5.470285e+01 1.582164e+02
      vertex   2.510000e+02 5.828617e+01 1.601061e+02
    endloop
  endfacet
  facet normal -3.538375e-06 4.720839e-01 -8.815536e-01
    outer loop
      vertex   2.510000e+02 5.500000e+01 1.583464e+02
      vertex   8.856785e+01 5.496808e+01 1.583299e+02
      vertex   2.510000e+02 5.828617e+01 1.601061e+02
    endloop
  endfacet
  facet normal 1.779965e-05 7.650433e-01 -6.439788e-01
    outer loop
      vertex   -3.000000e+00 9.259142e+01 1.886735e+02
      vertex   2.510000e+02 9.056495e+01 1.862731e+02
      vertex   -3.297513e+00 9.089682e+01 1.866603e+02
    endloop
  endfacet
  facet normal 3.323574e-04 7.811748e-01 -6.243123e-01
    outer loop
      vertex   2.510000e+02 9.624274e+01 1.933775e+02
      vertex   2.510000e+02 9.056495e+01 1.862731e+02
      vertex   -3.000000e+00 9.259142e+01 1.886735e+02
    endloop
  endfacet
  facet normal 4.148370e-04 7.789732e-01 -6.270571e-01
    outer loop
      vertex   -3.000000e+00 9.456129e+01 1.911206e+02
      vertex   2.510000e+02 9.624274e+01 1.933775e+02
      vertex   -3.000000e+00 9.259142e+01 1.886735e+02
    endloop
  endfacet
  facet normal 4.657046e-05 7.993857e-01 -6.008182e-01
    outer loop
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
      vertex   2.510000e+02 9.624274e+01 1.933775e+02
      vertex   -3.000000e+00 9.456129e+01 1.911206e+02
    endloop
  endfacet
  facet normal 1.618760e-04 8.196057e-01 -5.729280e-01
    outer loop
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
      vertex   2.510000e+02 9.624274e+01 1.933775e+02
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
    endloop
  endfacet
  facet normal 2.957859e-04 8.225437e-01 -5.687018e-01
    outer loop
      vertex   2.510000e+02 1.011355e+02 2.004541e+02
      vertex   2.510000e+02 9.624274e+01 1.933775e+02
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
    endloop
  endfacet
  facet normal -3.097773e-05 8.454691e-01 -5.340243e-01
    outer loop
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
      vertex   2.510000e+02 1.011355e+02 2.004541e+02
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
    endloop
  endfacet
  facet normal 3.112245e-04 8.621320e-01 -5.066836e-01
    outer loop
      vertex   2.510000e+02 1.058915e+02 2.085465e+02
      vertex   2.510000e+02 1.011355e+02 2.004541e+02
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
    endloop
  endfacet
  facet normal -1.212419e-04 8.703555e-01 -4.924239e-01
    outer loop
      vertex   -3.000000e+00 1.061874e+02 2.091321e+02
      vertex   2.510000e+02 1.058915e+02 2.085465e+02
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
    endloop
  endfacet
  facet normal -1.657280e-05 9.873705e-01 -1.584280e-01
    outer loop
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -3.000000e+00 1.185433e+02 2.464897e+02
    endloop
  endfacet
  facet normal 1.311209e-04 9.824243e-01 -1.866616e-01
    outer loop
      vertex   -3.000000e+00 1.170288e+02 2.385191e+02
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -3.000000e+00 1.185433e+02 2.464897e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.814989e-01 -1.914676e-01
    outer loop
      vertex   -2.570000e+02 1.170288e+02 2.385191e+02
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -3.000000e+00 1.170288e+02 2.385191e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.681479e-01 -2.503791e-01
    outer loop
      vertex   -3.000000e+00 1.151939e+02 2.314240e+02
      vertex   -2.570000e+02 1.170288e+02 2.385191e+02
      vertex   -3.000000e+00 1.170288e+02 2.385191e+02
    endloop
  endfacet
  facet normal -1.624437e-04 9.667228e-01 -2.558259e-01
    outer loop
      vertex   -2.570000e+02 1.149534e+02 2.306764e+02
      vertex   -2.570000e+02 1.170288e+02 2.385191e+02
      vertex   -3.000000e+00 1.151939e+02 2.314240e+02
    endloop
  endfacet
  facet normal -4.885320e-05 9.566675e-01 -2.911825e-01
    outer loop
      vertex   -3.000000e+00 1.141268e+02 2.279180e+02
      vertex   -2.570000e+02 1.149534e+02 2.306764e+02
      vertex   -3.000000e+00 1.151939e+02 2.314240e+02
    endloop
  endfacet
  facet normal -4.726726e-04 9.451122e-01 -3.267457e-01
    outer loop
      vertex   -3.000000e+00 1.129293e+02 2.244543e+02
      vertex   -2.570000e+02 1.149534e+02 2.306764e+02
      vertex   -3.000000e+00 1.141268e+02 2.279180e+02
    endloop
  endfacet
  facet normal -1.598176e-05 9.507565e-01 -3.099387e-01
    outer loop
      vertex   -2.570000e+02 1.127631e+02 2.239576e+02
      vertex   -2.570000e+02 1.149534e+02 2.306764e+02
      vertex   -3.000000e+00 1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal 9.994823e-05 9.318315e-01 -3.628912e-01
    outer loop
      vertex   -2.570000e+02 1.110532e+02 2.195669e+02
      vertex   -2.570000e+02 1.127631e+02 2.239576e+02
      vertex   -3.000000e+00 1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal 1.798569e-04 9.304175e-01 -3.665013e-01
    outer loop
      vertex   -3.000000e+00 1.102434e+02 2.176357e+02
      vertex   -2.570000e+02 1.110532e+02 2.195669e+02
      vertex   -3.000000e+00 1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal -1.284587e-04 9.160637e-01 -4.010327e-01
    outer loop
      vertex   -2.570000e+02 1.088490e+02 2.145318e+02
      vertex   -2.570000e+02 1.110532e+02 2.195669e+02
      vertex   -3.000000e+00 1.102434e+02 2.176357e+02
    endloop
  endfacet
  facet normal 2.004457e-05 9.115619e-01 -4.111629e-01
    outer loop
      vertex   -3.000000e+00 1.083061e+02 2.133406e+02
      vertex   -2.570000e+02 1.088490e+02 2.145318e+02
      vertex   -3.000000e+00 1.102434e+02 2.176357e+02
    endloop
  endfacet
  facet normal -1.997500e-04 8.931989e-01 -4.496618e-01
    outer loop
      vertex   -3.000000e+00 1.061874e+02 2.091321e+02
      vertex   -2.570000e+02 1.088490e+02 2.145318e+02
      vertex   -3.000000e+00 1.083061e+02 2.133406e+02
    endloop
  endfacet
  facet normal -2.630167e-04 8.919953e-01 -4.520446e-01
    outer loop
      vertex   -2.570000e+02 1.050633e+02 2.070618e+02
      vertex   -2.570000e+02 1.088490e+02 2.145318e+02
      vertex   -3.000000e+00 1.061874e+02 2.091321e+02
    endloop
  endfacet
  facet normal 1.618718e-04 8.703555e-01 -4.924239e-01
    outer loop
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
      vertex   -2.570000e+02 1.050633e+02 2.070618e+02
      vertex   -3.000000e+00 1.061874e+02 2.091321e+02
    endloop
  endfacet
  facet normal -2.034470e-04 8.610168e-01 -5.085764e-01
    outer loop
      vertex   -2.570000e+02 1.012031e+02 2.005265e+02
      vertex   -2.570000e+02 1.050633e+02 2.070618e+02
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
    endloop
  endfacet
  facet normal 1.038917e-04 8.454691e-01 -5.340243e-01
    outer loop
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
      vertex   -2.570000e+02 1.012031e+02 2.005265e+02
      vertex   -3.000000e+00 1.025787e+02 2.027537e+02
    endloop
  endfacet
  facet normal -7.199876e-05 8.338855e-01 -5.519375e-01
    outer loop
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
      vertex   -2.570000e+02 1.012031e+02 2.005265e+02
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
    endloop
  endfacet
  facet normal -2.693068e-05 8.308561e-01 -5.564873e-01
    outer loop
      vertex   -2.398978e+01 9.898121e+01 1.971567e+02
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
    endloop
  endfacet
  facet normal -5.187206e-05 8.832702e-01 -4.688643e-01
    outer loop
      vertex   8.943205e+01 1.066575e+02 2.100076e+02
      vertex   2.510000e+02 1.058915e+02 2.085465e+02
      vertex   -3.000000e+00 1.061874e+02 2.091321e+02
    endloop
  endfacet
  facet normal -7.122285e-05 8.334900e-01 5.525345e-01
    outer loop
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
      vertex   -2.386830e+01 9.894104e+01 3.329017e+02
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
    endloop
  endfacet
  facet normal -1.079012e-04 8.526412e-01 5.224968e-01
    outer loop
      vertex   -3.000000e+00 1.041730e+02 3.245397e+02
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
    endloop
  endfacet
  facet normal -1.394968e-04 8.531933e-01 5.215949e-01
    outer loop
      vertex   -2.570000e+02 1.046885e+02 3.236284e+02
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
      vertex   -3.000000e+00 1.041730e+02 3.245397e+02
    endloop
  endfacet
  facet normal 3.083078e-05 8.740142e-01 4.859003e-01
    outer loop
      vertex   -3.000000e+00 1.059537e+02 3.213366e+02
      vertex   -2.570000e+02 1.046885e+02 3.236284e+02
      vertex   -3.000000e+00 1.041730e+02 3.245397e+02
    endloop
  endfacet
  facet normal -3.117627e-04 8.896821e-01 4.565804e-01
    outer loop
      vertex   -3.000000e+00 1.073880e+02 3.185417e+02
      vertex   -2.570000e+02 1.046885e+02 3.236284e+02
      vertex   -3.000000e+00 1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal -2.589989e-04 8.886170e-01 4.586499e-01
    outer loop
      vertex   -2.570000e+02 1.082896e+02 3.166515e+02
      vertex   -2.570000e+02 1.046885e+02 3.236284e+02
      vertex   -3.000000e+00 1.073880e+02 3.185417e+02
    endloop
  endfacet
  facet normal 6.540115e-05 9.059727e-01 4.233361e-01
    outer loop
      vertex   -3.000000e+00 1.098259e+02 3.133245e+02
      vertex   -2.570000e+02 1.082896e+02 3.166515e+02
      vertex   -3.000000e+00 1.073880e+02 3.185417e+02
    endloop
  endfacet
  facet normal -2.397724e-04 9.147242e-01 4.040786e-01
    outer loop
      vertex   -2.570000e+02 1.111449e+02 3.101879e+02
      vertex   -2.570000e+02 1.082896e+02 3.166515e+02
      vertex   -3.000000e+00 1.098259e+02 3.133245e+02
    endloop
  endfacet
  facet normal 7.196103e-05 9.238791e-01 3.826846e-01
    outer loop
      vertex   -3.000000e+00 1.118292e+02 3.084880e+02
      vertex   -2.570000e+02 1.111449e+02 3.101879e+02
      vertex   -3.000000e+00 1.098259e+02 3.133245e+02
    endloop
  endfacet
  facet normal -3.590515e-04 9.450871e-01 3.268183e-01
    outer loop
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
      vertex   -2.570000e+02 1.111449e+02 3.101879e+02
      vertex   -3.000000e+00 1.118292e+02 3.084880e+02
    endloop
  endfacet
  facet normal 2.953254e-04 9.397782e-01 3.417847e-01
    outer loop
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
      vertex   -2.570000e+02 1.111449e+02 3.101879e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 9.823161e-05 9.521689e-01 3.055721e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 9.188568e-06 9.536021e-01 3.010699e-01
    outer loop
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
    endloop
  endfacet
  facet normal -1.490888e-04 7.562867e-01 6.542404e-01
    outer loop
      vertex   -3.000000e+00 9.259142e+01 3.413265e+02
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
      vertex   2.510000e+02 9.056495e+01 3.437269e+02
    endloop
  endfacet
  facet normal 4.839762e-04 7.887758e-01 6.146808e-01
    outer loop
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
      vertex   -3.000000e+00 9.259142e+01 3.413265e+02
      vertex   2.510000e+02 9.056495e+01 3.437269e+02
    endloop
  endfacet
  facet normal 6.802703e-04 7.923117e-01 6.101162e-01
    outer loop
      vertex   2.510000e+02 9.881516e+01 3.330130e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
      vertex   2.510000e+02 9.056495e+01 3.437269e+02
    endloop
  endfacet
  facet normal -1.288925e-04 8.172484e-01 5.762856e-01
    outer loop
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
      vertex   2.510000e+02 9.881516e+01 3.330130e+02
    endloop
  endfacet
  facet normal 4.620099e-04 8.480201e-01 5.299639e-01
    outer loop
      vertex   2.510000e+02 1.048939e+02 3.232862e+02
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
      vertex   2.510000e+02 9.881516e+01 3.330130e+02
    endloop
  endfacet
  facet normal 1.586782e-04 8.526412e-01 5.224968e-01
    outer loop
      vertex   -3.000000e+00 1.041730e+02 3.245397e+02
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
      vertex   2.510000e+02 1.048939e+02 3.232862e+02
    endloop
  endfacet
  facet normal -8.259435e-05 8.740142e-01 4.859003e-01
    outer loop
      vertex   -3.000000e+00 1.059537e+02 3.213366e+02
      vertex   -3.000000e+00 1.041730e+02 3.245397e+02
      vertex   2.510000e+02 1.048939e+02 3.232862e+02
    endloop
  endfacet
  facet normal 9.000771e-05 8.834430e-01 4.685387e-01
    outer loop
      vertex   8.865777e+01 1.066675e+02 3.199730e+02
      vertex   -3.000000e+00 1.059537e+02 3.213366e+02
      vertex   2.510000e+02 1.048939e+02 3.232862e+02
    endloop
  endfacet
  facet normal -2.921907e-06 8.815550e-01 4.720814e-01
    outer loop
      vertex   2.510000e+02 1.066536e+02 3.200000e+02
      vertex   8.865777e+01 1.066675e+02 3.199730e+02
      vertex   2.510000e+02 1.048939e+02 3.232862e+02
    endloop
  endfacet
  facet normal 1.010221e-06 6.619165e-01 7.495776e-01
    outer loop
      vertex   2.510000e+02 7.872691e+01 3.555650e+02
      vertex   -2.455305e+00 8.008673e+01 3.543645e+02
      vertex   -3.000000e+00 7.872708e+01 3.555651e+02
    endloop
  endfacet
  facet normal 1.009925e-06 6.494441e-01 7.604093e-01
    outer loop
      vertex   -3.000000e+00 7.713451e+01 3.569253e+02
      vertex   2.510000e+02 7.872691e+01 3.555650e+02
      vertex   -3.000000e+00 7.872708e+01 3.555651e+02
    endloop
  endfacet
  facet normal 1.928486e-04 6.315766e-01 7.753134e-01
    outer loop
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
      vertex   2.510000e+02 7.872691e+01 3.555650e+02
      vertex   -3.000000e+00 7.713451e+01 3.569253e+02
    endloop
  endfacet
  facet normal 1.825377e-04 6.319644e-01 7.749974e-01
    outer loop
      vertex   2.510000e+02 7.369886e+01 3.596650e+02
      vertex   2.510000e+02 7.872691e+01 3.555650e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal -2.559541e-06 6.024509e-01 7.981559e-01
    outer loop
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
      vertex   2.510000e+02 7.369886e+01 3.596650e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal 1.907556e-04 5.937161e-01 8.046745e-01
    outer loop
      vertex   2.510000e+02 6.803359e+01 3.638451e+02
      vertex   2.510000e+02 7.369886e+01 3.596650e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -1.052821e-04 5.695981e-01 8.219234e-01
    outer loop
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
      vertex   2.510000e+02 6.803359e+01 3.638451e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal 1.388130e-04 5.459513e-01 8.378169e-01
    outer loop
      vertex   2.510000e+02 6.322684e+01 3.669773e+02
      vertex   2.510000e+02 6.803359e+01 3.638451e+02
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal -5.989111e-05 5.340242e-01 8.454692e-01
    outer loop
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
      vertex   2.510000e+02 6.322684e+01 3.669773e+02
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal 1.183377e-04 5.003416e-01 8.658281e-01
    outer loop
      vertex   2.510000e+02 5.645351e+01 3.708915e+02
      vertex   2.510000e+02 6.322684e+01 3.669773e+02
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
    endloop
  endfacet
  facet normal -1.213052e-04 4.924232e-01 8.703559e-01
    outer loop
      vertex   -3.000000e+00 5.586786e+01 3.711874e+02
      vertex   2.510000e+02 5.645351e+01 3.708915e+02
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
    endloop
  endfacet
  facet normal 1.695944e-04 -5.386940e-01 8.425015e-01
    outer loop
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
      vertex   -2.310793e+01 -6.778645e+01 3.640202e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
    endloop
  endfacet
  facet normal 1.307816e-04 -5.439728e-01 8.391028e-01
    outer loop
      vertex   -3.000000e+00 -6.269744e+01 3.673129e+02
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
    endloop
  endfacet
  facet normal -3.249332e-04 -5.205774e-01 8.538144e-01
    outer loop
      vertex   -2.570000e+02 -5.930280e+01 3.692860e+02
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
      vertex   -3.000000e+00 -6.269744e+01 3.673129e+02
    endloop
  endfacet
  facet normal -2.996949e-05 -5.041904e-01 8.635925e-01
    outer loop
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
      vertex   -2.570000e+02 -5.930280e+01 3.692860e+02
      vertex   -3.000000e+00 -6.269744e+01 3.673129e+02
    endloop
  endfacet
  facet normal -2.136053e-04 -4.519928e-01 8.920215e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   -2.570000e+02 -5.930280e+01 3.692860e+02
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
    endloop
  endfacet
  facet normal 9.963298e-04 -4.793474e-01 8.776247e-01
    outer loop
      vertex   -2.570000e+02 -5.447886e+01 3.719208e+02
      vertex   -2.570000e+02 -5.930280e+01 3.692860e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal -3.737984e-05 -4.267314e-01 9.043784e-01
    outer loop
      vertex   -2.570000e+02 -4.856081e+01 3.747132e+02
      vertex   -2.570000e+02 -5.447886e+01 3.719208e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal 1.658739e-04 -4.043055e-01 9.146240e-01
    outer loop
      vertex   -3.000000e+00 -4.495279e+01 3.762621e+02
      vertex   -2.570000e+02 -4.856081e+01 3.747132e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal -4.198941e-04 -3.693621e-01 9.292855e-01
    outer loop
      vertex   -2.570000e+02 -4.029278e+01 3.779995e+02
      vertex   -2.570000e+02 -4.856081e+01 3.747132e+02
      vertex   -3.000000e+00 -4.495279e+01 3.762621e+02
    endloop
  endfacet
  facet normal -2.458697e-04 -3.610871e-01 9.325321e-01
    outer loop
      vertex   -3.000000e+00 -4.202329e+01 3.773964e+02
      vertex   -2.570000e+02 -4.029278e+01 3.779995e+02
      vertex   -3.000000e+00 -4.495279e+01 3.762621e+02
    endloop
  endfacet
  facet normal 7.593199e-05 -3.191424e-01 9.477068e-01
    outer loop
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
      vertex   -2.570000e+02 -4.029278e+01 3.779995e+02
      vertex   -3.000000e+00 -4.202329e+01 3.773964e+02
    endloop
  endfacet
  facet normal 8.738664e-05 -3.196691e-01 9.475292e-01
    outer loop
      vertex   -2.570000e+02 -3.508461e+01 3.797566e+02
      vertex   -2.570000e+02 -4.029278e+01 3.779995e+02
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
    endloop
  endfacet
  facet normal 1.362149e-04 -2.744307e-01 9.616069e-01
    outer loop
      vertex   -2.570000e+02 -3.156047e+01 3.807623e+02
      vertex   -2.570000e+02 -3.508461e+01 3.797566e+02
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
    endloop
  endfacet
  facet normal 1.579149e-04 -2.730803e-01 9.619912e-01
    outer loop
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
      vertex   -2.570000e+02 -3.156047e+01 3.807623e+02
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
    endloop
  endfacet
  facet normal -3.463206e-04 -2.137589e-01 9.768864e-01
    outer loop
      vertex   -2.570000e+02 -2.018628e+01 3.832512e+02
      vertex   -2.570000e+02 -3.156047e+01 3.807623e+02
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
    endloop
  endfacet
  facet normal -9.599204e-04 -2.296725e-01 9.732675e-01
    outer loop
      vertex   -3.000000e+00 -2.494940e+01 3.823777e+02
      vertex   -2.570000e+02 -2.018628e+01 3.832512e+02
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
    endloop
  endfacet
  facet normal -3.598653e-05 -1.822341e-01 9.832552e-01
    outer loop
      vertex   -3.000000e+00 -1.980194e+01 3.833317e+02
      vertex   -2.570000e+02 -2.018628e+01 3.832512e+02
      vertex   -3.000000e+00 -2.494940e+01 3.823777e+02
    endloop
  endfacet
  facet normal -1.252727e-04 -1.250895e-01 9.921455e-01
    outer loop
      vertex   -3.000000e+00 -1.071570e+01 3.844773e+02
      vertex   -2.570000e+02 -2.018628e+01 3.832512e+02
      vertex   -3.000000e+00 -1.980194e+01 3.833317e+02
    endloop
  endfacet
  facet normal 2.996843e-04 -1.362955e-01 9.906682e-01
    outer loop
      vertex   -2.570000e+02 -1.240990e+01 3.843211e+02
      vertex   -2.570000e+02 -2.018628e+01 3.832512e+02
      vertex   -3.000000e+00 -1.071570e+01 3.844773e+02
    endloop
  endfacet
  facet normal -1.947411e-04 -6.284792e-02 9.980231e-01
    outer loop
      vertex   -3.000000e+00 -3.141114e+00 3.849543e+02
      vertex   -2.570000e+02 -1.240990e+01 3.843211e+02
      vertex   -3.000000e+00 -1.071570e+01 3.844773e+02
    endloop
  endfacet
  facet normal -4.001228e-04 -5.724271e-02 9.983602e-01
    outer loop
      vertex   -2.570000e+02 -1.046985e+00 3.849726e+02
      vertex   -2.570000e+02 -1.240990e+01 3.843211e+02
      vertex   -3.000000e+00 -3.141114e+00 3.849543e+02
    endloop
  endfacet
  facet normal 1.624119e-04 1.097417e-02 9.999398e-01
    outer loop
      vertex   -3.000000e+00 4.971544e+00 3.848653e+02
      vertex   -2.570000e+02 -1.046985e+00 3.849726e+02
      vertex   -3.000000e+00 -3.141114e+00 3.849543e+02
    endloop
  endfacet
  facet normal -3.278452e-04 3.165663e-02 9.994987e-01
    outer loop
      vertex   -2.570000e+02 8.106601e+00 3.846827e+02
      vertex   -2.570000e+02 -1.046985e+00 3.849726e+02
      vertex   -3.000000e+00 4.971544e+00 3.848653e+02
    endloop
  endfacet
  facet normal 2.756539e-04 8.039235e-02 9.967633e-01
    outer loop
      vertex   -3.000000e+00 1.436103e+01 3.841080e+02
      vertex   -2.570000e+02 8.106601e+00 3.846827e+02
      vertex   -3.000000e+00 4.971544e+00 3.848653e+02
    endloop
  endfacet
  facet normal -2.682968e-04 1.022982e-01 9.947537e-01
    outer loop
      vertex   -2.570000e+02 1.721734e+01 3.837457e+02
      vertex   -2.570000e+02 8.106601e+00 3.846827e+02
      vertex   -3.000000e+00 1.436103e+01 3.841080e+02
    endloop
  endfacet
  facet normal 3.680796e-04 1.579616e-01 9.874452e-01
    outer loop
      vertex   -3.000000e+00 2.314578e+01 3.827027e+02
      vertex   -2.570000e+02 1.721734e+01 3.837457e+02
      vertex   -3.000000e+00 1.436103e+01 3.841080e+02
    endloop
  endfacet
  facet normal -2.286047e-04 1.827714e-01 9.831554e-01
    outer loop
      vertex   -2.570000e+02 2.544990e+01 3.822153e+02
      vertex   -2.570000e+02 1.721734e+01 3.837457e+02
      vertex   -3.000000e+00 2.314578e+01 3.827027e+02
    endloop
  endfacet
  facet normal 1.684334e-04 2.246939e-01 9.744294e-01
    outer loop
      vertex   -3.000000e+00 3.156047e+01 3.807623e+02
      vertex   -2.570000e+02 2.544990e+01 3.822153e+02
      vertex   -3.000000e+00 2.314578e+01 3.827027e+02
    endloop
  endfacet
  facet normal -1.055613e-03 2.726459e-01 9.621139e-01
    outer loop
      vertex   -2.570000e+02 3.991759e+01 3.781154e+02
      vertex   -2.570000e+02 2.544990e+01 3.822153e+02
      vertex   -3.000000e+00 3.156047e+01 3.807623e+02
    endloop
  endfacet
  facet normal -9.916043e-04 2.744306e-01 9.616064e-01
    outer loop
      vertex   -3.000000e+00 3.508461e+01 3.797566e+02
      vertex   -2.570000e+02 3.991759e+01 3.781154e+02
      vertex   -3.000000e+00 3.156047e+01 3.807623e+02
    endloop
  endfacet
  facet normal -1.835476e-04 3.128860e-01 9.497907e-01
    outer loop
      vertex   -3.000000e+00 3.881389e+01 3.785281e+02
      vertex   -2.570000e+02 3.991759e+01 3.781154e+02
      vertex   -3.000000e+00 3.508461e+01 3.797566e+02
    endloop
  endfacet
  facet normal 9.562259e-05 3.694524e-01 9.292496e-01
    outer loop
      vertex   -3.000000e+00 4.951185e+01 3.742747e+02
      vertex   -2.570000e+02 3.991759e+01 3.781154e+02
      vertex   -3.000000e+00 3.881389e+01 3.785281e+02
    endloop
  endfacet
  facet normal -1.195454e-05 3.719081e-01 9.282695e-01
    outer loop
      vertex   -2.570000e+02 4.975371e+01 3.741746e+02
      vertex   -2.570000e+02 3.991759e+01 3.781154e+02
      vertex   -3.000000e+00 4.951185e+01 3.742747e+02
    endloop
  endfacet
  facet normal 6.129797e-05 4.369202e-01 8.995003e-01
    outer loop
      vertex   -3.000000e+00 5.586786e+01 3.711874e+02
      vertex   -2.570000e+02 4.975371e+01 3.741746e+02
      vertex   -3.000000e+00 4.951185e+01 3.742747e+02
    endloop
  endfacet
  facet normal -3.226126e-04 4.497649e-01 8.931469e-01
    outer loop
      vertex   -2.570000e+02 5.793393e+01 3.700552e+02
      vertex   -2.570000e+02 4.975371e+01 3.741746e+02
      vertex   -3.000000e+00 5.586786e+01 3.711874e+02
    endloop
  endfacet
  facet normal 1.259624e-04 4.924232e-01 8.703559e-01
    outer loop
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
      vertex   -2.570000e+02 5.793393e+01 3.700552e+02
      vertex   -3.000000e+00 5.586786e+01 3.711874e+02
    endloop
  endfacet
  facet normal -2.629061e-04 5.096124e-01 8.604041e-01
    outer loop
      vertex   -2.570000e+02 6.446368e+01 3.661877e+02
      vertex   -2.570000e+02 5.793393e+01 3.700552e+02
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
    endloop
  endfacet
  facet normal 3.199181e-05 5.340242e-01 8.454692e-01
    outer loop
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
      vertex   -2.570000e+02 6.446368e+01 3.661877e+02
      vertex   -3.000000e+00 6.224630e+01 3.675787e+02
    endloop
  endfacet
  facet normal -1.726803e-04 5.548467e-01 8.319526e-01
    outer loop
      vertex   -2.334514e+01 6.778671e+01 3.640200e+02
      vertex   -2.570000e+02 6.446368e+01 3.661877e+02
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal -5.117286e-05 4.686025e-01 8.834091e-01
    outer loop
      vertex   8.821533e+01 5.496034e+01 3.716741e+02
      vertex   2.510000e+02 5.645351e+01 3.708915e+02
      vertex   -3.000000e+00 5.586786e+01 3.711874e+02
    endloop
  endfacet
  facet normal -2.297161e-04 -6.370609e-01 7.708134e-01
    outer loop
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   -2.516175e+00 -7.999101e+01 3.544447e+02
      vertex   2.510000e+02 -7.872691e+01 3.555650e+02
    endloop
  endfacet
  facet normal 5.492515e-04 -6.134634e-01 7.897230e-01
    outer loop
      vertex   2.510000e+02 -6.845808e+01 3.635419e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   2.510000e+02 -7.872691e+01 3.555650e+02
    endloop
  endfacet
  facet normal -1.377365e-04 -5.919189e-01 8.059975e-01
    outer loop
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   2.510000e+02 -6.845808e+01 3.635419e+02
    endloop
  endfacet
  facet normal 1.959082e-04 -5.296726e-01 8.482021e-01
    outer loop
      vertex   2.510000e+02 -5.828617e+01 3.698939e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
      vertex   2.510000e+02 -6.845808e+01 3.635419e+02
    endloop
  endfacet
  facet normal 9.209917e-04 -5.439726e-01 8.391025e-01
    outer loop
      vertex   -3.000000e+00 -6.269744e+01 3.673129e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
      vertex   2.510000e+02 -5.828617e+01 3.698939e+02
    endloop
  endfacet
  facet normal -1.876310e-05 -5.041904e-01 8.635925e-01
    outer loop
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
      vertex   -3.000000e+00 -6.269744e+01 3.673129e+02
      vertex   2.510000e+02 -5.828617e+01 3.698939e+02
    endloop
  endfacet
  facet normal -6.655494e-05 -4.776859e-01 8.785307e-01
    outer loop
      vertex   2.510000e+02 -5.639486e+01 3.709222e+02
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
      vertex   2.510000e+02 -5.828617e+01 3.698939e+02
    endloop
  endfacet
  facet normal -1.112461e-04 -4.737727e-01 8.806471e-01
    outer loop
      vertex   8.769390e+01 -5.492844e+01 3.716905e+02
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
      vertex   2.510000e+02 -5.639486e+01 3.709222e+02
    endloop
  endfacet
  facet normal -3.450258e-06 -4.643815e-01 8.856353e-01
    outer loop
      vertex   2.510000e+02 -5.500000e+01 3.716536e+02
      vertex   8.769390e+01 -5.492844e+01 3.716905e+02
      vertex   2.510000e+02 -5.639486e+01 3.709222e+02
    endloop
  endfacet
  facet normal 2.911407e-04 -7.816406e-01 6.237290e-01
    outer loop
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
      vertex   2.510000e+02 -9.056495e+01 3.437269e+02
      vertex   -4.174363e+00 -9.211172e+01 3.419077e+02
    endloop
  endfacet
  facet normal 2.039805e-04 -7.791219e-01 6.268724e-01
    outer loop
      vertex   2.510000e+02 -9.593956e+01 3.370470e+02
      vertex   2.510000e+02 -9.056495e+01 3.437269e+02
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
    endloop
  endfacet
  facet normal -1.926180e-04 -7.992194e-01 6.010394e-01
    outer loop
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
      vertex   2.510000e+02 -9.593956e+01 3.370470e+02
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
    endloop
  endfacet
  facet normal 4.138011e-04 -8.218243e-01 5.697409e-01
    outer loop
      vertex   2.510000e+02 -1.005438e+02 3.304056e+02
      vertex   2.510000e+02 -9.593956e+01 3.370470e+02
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
    endloop
  endfacet
  facet normal 3.486346e-04 -8.241216e-01 5.664128e-01
    outer loop
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
      vertex   2.510000e+02 -1.005438e+02 3.304056e+02
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
    endloop
  endfacet
  facet normal 1.103761e-04 -8.408808e-01 5.412203e-01
    outer loop
      vertex   -3.000000e+00 -1.016096e+02 3.288015e+02
      vertex   2.510000e+02 -1.005438e+02 3.304056e+02
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
    endloop
  endfacet
  facet normal 3.784512e-04 -8.594966e-01 5.111413e-01
    outer loop
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
      vertex   2.510000e+02 -1.005438e+02 3.304056e+02
      vertex   -3.000000e+00 -1.016096e+02 3.288015e+02
    endloop
  endfacet
  facet normal 3.128123e-04 -8.584885e-01 5.128327e-01
    outer loop
      vertex   2.510000e+02 -1.058915e+02 3.214535e+02
      vertex   2.510000e+02 -1.005438e+02 3.304056e+02
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
    endloop
  endfacet
  facet normal -8.778004e-04 -8.776600e-01 -4.792830e-01
    outer loop
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
      vertex   -2.265247e+01 -9.902140e+01 1.972153e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal -1.799155e-04 -8.338854e-01 -5.519376e-01
    outer loop
      vertex   -2.570000e+02 -1.020316e+02 2.018610e+02
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal 1.798494e-04 -8.433922e-01 -5.372984e-01
    outer loop
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
      vertex   -2.570000e+02 -1.020316e+02 2.018610e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal -1.079122e-04 -8.616290e-01 -5.075386e-01
    outer loop
      vertex   -2.570000e+02 -1.046885e+02 2.063716e+02
      vertex   -2.570000e+02 -1.020316e+02 2.018610e+02
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
    endloop
  endfacet
  facet normal 2.680413e-04 -8.766588e-01 -4.811125e-01
    outer loop
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
      vertex   -2.570000e+02 -1.046885e+02 2.063716e+02
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
    endloop
  endfacet
  facet normal -2.302157e-04 -8.886170e-01 -4.586499e-01
    outer loop
      vertex   -2.570000e+02 -1.082896e+02 2.133485e+02
      vertex   -2.570000e+02 -1.046885e+02 2.063716e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
    endloop
  endfacet
  facet normal 1.496293e-04 -9.034857e-01 -4.286181e-01
    outer loop
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
      vertex   -2.570000e+02 -1.082896e+02 2.133485e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
    endloop
  endfacet
  facet normal -7.191958e-05 -9.099610e-01 -4.146939e-01
    outer loop
      vertex   -2.570000e+02 -1.104606e+02 2.181123e+02
      vertex   -2.570000e+02 -1.082896e+02 2.133485e+02
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
    endloop
  endfacet
  facet normal 8.225907e-05 -9.200166e-01 -3.918794e-01
    outer loop
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
      vertex   -2.570000e+02 -1.104606e+02 2.181123e+02
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
    endloop
  endfacet
  facet normal -1.354432e-04 -9.300416e-01 -3.674542e-01
    outer loop
      vertex   -2.570000e+02 -1.120957e+02 2.222507e+02
      vertex   -2.570000e+02 -1.104606e+02 2.181123e+02
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
    endloop
  endfacet
  facet normal -3.084388e-08 -9.353070e-01 -3.538373e-01
    outer loop
      vertex   -3.000000e+00 -1.129293e+02 2.244543e+02
      vertex   -2.570000e+02 -1.120957e+02 2.222507e+02
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
    endloop
  endfacet
  facet normal -4.001119e-04 -9.496988e-01 -3.131646e-01
    outer loop
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
      vertex   -2.570000e+02 -1.120957e+02 2.222507e+02
      vertex   -3.000000e+00 -1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal -4.725175e-04 -9.503636e-01 -3.111413e-01
    outer loop
      vertex   -2.570000e+02 -1.155960e+02 2.329424e+02
      vertex   -2.570000e+02 -1.120957e+02 2.222507e+02
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
    endloop
  endfacet
  facet normal 1.240284e-04 -9.667774e-01 -2.556199e-01
    outer loop
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
      vertex   -2.570000e+02 -1.155960e+02 2.329424e+02
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
    endloop
  endfacet
  facet normal -4.067804e-04 -9.733524e-01 -2.293142e-01
    outer loop
      vertex   -2.570000e+02 -1.178160e+02 2.423654e+02
      vertex   -2.570000e+02 -1.155960e+02 2.329424e+02
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
    endloop
  endfacet
  facet normal 1.378935e-04 -9.796667e-01 -2.006316e-01
    outer loop
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
      vertex   -2.570000e+02 -1.178160e+02 2.423654e+02
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
    endloop
  endfacet
  facet normal -3.596007e-04 -9.890154e-01 -1.478123e-01
    outer loop
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
      vertex   -2.570000e+02 -1.178160e+02 2.423654e+02
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
    endloop
  endfacet
  facet normal -2.404847e-04 -9.884183e-01 -1.517540e-01
    outer loop
      vertex   -2.570000e+02 -1.192059e+02 2.514182e+02
      vertex   -2.570000e+02 -1.178160e+02 2.423654e+02
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
    endloop
  endfacet
  facet normal 8.091582e-05 -9.949686e-01 -1.001876e-01
    outer loop
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
      vertex   -2.570000e+02 -1.192059e+02 2.514182e+02
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
    endloop
  endfacet
  facet normal -3.237126e-04 -9.969172e-01 -7.845986e-02
    outer loop
      vertex   -2.570000e+02 -1.198630e+02 2.597667e+02
      vertex   -2.570000e+02 -1.192059e+02 2.514182e+02
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
    endloop
  endfacet
  facet normal 1.496425e-04 -9.989546e-01 -4.571310e-02
    outer loop
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
      vertex   -2.570000e+02 -1.198630e+02 2.597667e+02
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
    endloop
  endfacet
  facet normal -3.318900e-04 -9.999947e-01 -3.242719e-03
    outer loop
      vertex   -2.570000e+02 -1.198927e+02 2.689248e+02
      vertex   -2.570000e+02 -1.198630e+02 2.597667e+02
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
    endloop
  endfacet
  facet normal -6.170489e-06 -9.999507e-01 9.929674e-03
    outer loop
      vertex   -3.000000e+00 -1.198812e+02 2.702341e+02
      vertex   -2.570000e+02 -1.198927e+02 2.689248e+02
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
    endloop
  endfacet
  facet normal -3.105967e-04 -9.976190e-01 6.896532e-02
    outer loop
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
      vertex   -2.570000e+02 -1.198927e+02 2.689248e+02
      vertex   -3.000000e+00 -1.198812e+02 2.702341e+02
    endloop
  endfacet
  facet normal -3.654980e-04 -9.974809e-01 7.093412e-02
    outer loop
      vertex   -2.570000e+02 -1.192059e+02 2.785818e+02
      vertex   -2.570000e+02 -1.198927e+02 2.689248e+02
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal 3.196786e-05 -9.940032e-01 1.093514e-01
    outer loop
      vertex   -3.000000e+00 -1.189688e+02 2.806625e+02
      vertex   -2.570000e+02 -1.192059e+02 2.785818e+02
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal -3.196949e-04 -9.884350e-01 1.516450e-01
    outer loop
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
      vertex   -2.570000e+02 -1.192059e+02 2.785818e+02
      vertex   -3.000000e+00 -1.189688e+02 2.806625e+02
    endloop
  endfacet
  facet normal -1.222763e-04 -9.895257e-01 1.443570e-01
    outer loop
      vertex   -2.570000e+02 -1.180725e+02 2.863510e+02
      vertex   -2.570000e+02 -1.192059e+02 2.785818e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
    endloop
  endfacet
  facet normal 7.523476e-05 -9.814758e-01 1.915863e-01
    outer loop
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
      vertex   -2.570000e+02 -1.180725e+02 2.863510e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
    endloop
  endfacet
  facet normal -3.643337e-04 -9.766604e-01 2.147889e-01
    outer loop
      vertex   -2.570000e+02 -1.163300e+02 2.942741e+02
      vertex   -2.570000e+02 -1.180725e+02 2.863510e+02
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
    endloop
  endfacet
  facet normal -2.177085e-05 -9.708490e-01 2.396920e-01
    outer loop
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
      vertex   -2.570000e+02 -1.163300e+02 2.942741e+02
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
    endloop
  endfacet
  facet normal -3.056228e-04 -9.588191e-01 2.840173e-01
    outer loop
      vertex   -2.570000e+02 -1.133575e+02 3.043091e+02
      vertex   -2.570000e+02 -1.163300e+02 2.942741e+02
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
    endloop
  endfacet
  facet normal -4.523990e-04 -9.600007e-01 2.799972e-01
    outer loop
      vertex   -3.000000e+00 -1.141268e+02 3.020820e+02
      vertex   -2.570000e+02 -1.133575e+02 3.043091e+02
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
    endloop
  endfacet
  facet normal 8.993400e-05 -9.419933e-01 3.356318e-01
    outer loop
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
      vertex   -2.570000e+02 -1.133575e+02 3.043091e+02
      vertex   -3.000000e+00 -1.141268e+02 3.020820e+02
    endloop
  endfacet
  facet normal -3.083887e-05 -9.375420e-01 3.478721e-01
    outer loop
      vertex   -2.570000e+02 -1.114455e+02 3.094622e+02
      vertex   -2.570000e+02 -1.133575e+02 3.043091e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal 4.531583e-04 -9.232097e-01 3.842964e-01
    outer loop
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
      vertex   -2.570000e+02 -1.114455e+02 3.094622e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal 2.288834e-04 -9.201945e-01 3.914615e-01
    outer loop
      vertex   -2.570000e+02 -1.091912e+02 3.147613e+02
      vertex   -2.570000e+02 -1.114455e+02 3.094622e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -5.996883e-05 -9.009086e-01 4.340089e-01
    outer loop
      vertex   -2.570000e+02 -1.071463e+02 3.190061e+02
      vertex   -2.570000e+02 -1.091912e+02 3.147613e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -1.544523e-05 -8.993402e-01 4.372497e-01
    outer loop
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
      vertex   -2.570000e+02 -1.071463e+02 3.190061e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -7.198320e-05 -8.870116e-01 4.617471e-01
    outer loop
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
      vertex   -2.570000e+02 -1.071463e+02 3.190061e+02
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
    endloop
  endfacet
  facet normal -2.518902e-04 -8.788157e-01 4.771613e-01
    outer loop
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
      vertex   -2.570000e+02 -1.071463e+02 3.190061e+02
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal -3.165380e-04 -8.771473e-01 4.802213e-01
    outer loop
      vertex   -2.570000e+02 -1.033758e+02 3.258930e+02
      vertex   -2.570000e+02 -1.071463e+02 3.190061e+02
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
    endloop
  endfacet
  facet normal 1.237663e-04 -8.594967e-01 5.111414e-01
    outer loop
      vertex   -3.000000e+00 -1.016096e+02 3.288015e+02
      vertex   -2.570000e+02 -1.033758e+02 3.258930e+02
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
    endloop
  endfacet
  facet normal -5.399749e-05 -8.526387e-01 5.225010e-01
    outer loop
      vertex   -2.570000e+02 -1.006405e+02 3.303567e+02
      vertex   -2.570000e+02 -1.033758e+02 3.258930e+02
      vertex   -3.000000e+00 -1.016096e+02 3.288015e+02
    endloop
  endfacet
  facet normal 1.054767e-04 -8.408808e-01 5.412203e-01
    outer loop
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
      vertex   -2.570000e+02 -1.006405e+02 3.303567e+02
      vertex   -3.000000e+00 -1.016096e+02 3.288015e+02
    endloop
  endfacet
  facet normal -1.354686e-04 -8.247085e-01 5.655581e-01
    outer loop
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
      vertex   -2.570000e+02 -1.006405e+02 3.303567e+02
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
    endloop
  endfacet
  facet normal -2.258667e-04 -8.301822e-01 5.574921e-01
    outer loop
      vertex   -2.241685e+01 -9.903052e+01 3.327713e+02
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
    endloop
  endfacet
  facet normal -4.246044e-06 -8.788157e-01 4.771613e-01
    outer loop
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
      vertex   2.510000e+02 -1.058915e+02 3.214535e+02
      vertex   -3.000000e+00 -1.049544e+02 3.231771e+02
    endloop
  endfacet
  facet normal 3.692431e-06 -8.859772e-01 4.637288e-01
    outer loop
      vertex   8.762442e+01 -1.066968e+02 3.199162e+02
      vertex   2.510000e+02 -1.058915e+02 3.214535e+02
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal -6.163368e-05 -7.473288e-01 -6.644544e-01
    outer loop
      vertex   2.510000e+02 -9.056495e+01 1.862731e+02
      vertex   -3.434055e+00 -9.108093e+01 1.868770e+02
      vertex   -2.100292e+00 -8.835109e+01 1.838066e+02
    endloop
  endfacet
  facet normal 2.461332e-05 -7.653980e-01 -6.435572e-01
    outer loop
      vertex   -3.000000e+00 -9.259142e+01 1.886735e+02
      vertex   -3.434055e+00 -9.108093e+01 1.868770e+02
      vertex   2.510000e+02 -9.056495e+01 1.862731e+02
    endloop
  endfacet
  facet normal 3.244048e-04 -7.807730e-01 -6.248147e-01
    outer loop
      vertex   2.510000e+02 -9.592141e+01 1.929666e+02
      vertex   -3.000000e+00 -9.259142e+01 1.886735e+02
      vertex   2.510000e+02 -9.056495e+01 1.862731e+02
    endloop
  endfacet
  facet normal 1.785083e-04 -7.850168e-01 -6.194744e-01
    outer loop
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
      vertex   -3.000000e+00 -9.259142e+01 1.886735e+02
      vertex   2.510000e+02 -9.592141e+01 1.929666e+02
    endloop
  endfacet
  facet normal 5.310826e-05 -8.115726e-01 -5.842516e-01
    outer loop
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
      vertex   2.510000e+02 -9.592141e+01 1.929666e+02
    endloop
  endfacet
  facet normal 3.197353e-04 -8.186298e-01 -5.743214e-01
    outer loop
      vertex   2.510000e+02 -1.008625e+02 2.000096e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
      vertex   2.510000e+02 -9.592141e+01 1.929666e+02
    endloop
  endfacet
  facet normal -2.143672e-04 -8.433922e-01 -5.372984e-01
    outer loop
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
      vertex   2.510000e+02 -1.008625e+02 2.000096e+02
    endloop
  endfacet
  facet normal 2.215362e-04 -8.569972e-01 -5.153211e-01
    outer loop
      vertex   2.510000e+02 -1.048939e+02 2.067138e+02
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
      vertex   2.510000e+02 -1.008625e+02 2.000096e+02
    endloop
  endfacet
  facet normal -3.283939e-04 -8.766588e-01 -4.811125e-01
    outer loop
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
      vertex   -3.000000e+00 -1.031180e+02 2.036513e+02
      vertex   2.510000e+02 -1.048939e+02 2.067138e+02
    endloop
  endfacet
  facet normal -5.954907e-04 -8.694253e-01 -4.940641e-01
    outer loop
      vertex   8.757386e+01 -1.066949e+02 2.100802e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
      vertex   2.510000e+02 -1.048939e+02 2.067138e+02
    endloop
  endfacet
  facet normal -8.991256e-06 -8.815550e-01 -4.720814e-01
    outer loop
      vertex   2.510000e+02 -1.066536e+02 2.100000e+02
      vertex   8.757386e+01 -1.066949e+02 2.100802e+02
      vertex   2.510000e+02 -1.048939e+02 2.067138e+02
    endloop
  endfacet
  facet normal 3.056343e-05 -6.402630e-01 -7.681557e-01
    outer loop
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
      vertex   2.510000e+02 -7.872691e+01 1.744351e+02
      vertex   -3.312455e+00 -7.831074e+01 1.740781e+02
    endloop
  endfacet
  facet normal 2.223407e-04 -6.299964e-01 -7.765981e-01
    outer loop
      vertex   2.510000e+02 -7.267581e+01 1.695263e+02
      vertex   2.510000e+02 -7.872691e+01 1.744351e+02
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
    endloop
  endfacet
  facet normal 3.359529e-06 -6.194746e-01 -7.850167e-01
    outer loop
      vertex   -3.000000e+00 -7.221780e+01 1.691637e+02
      vertex   2.510000e+02 -7.267581e+01 1.695263e+02
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
    endloop
  endfacet
  facet normal 8.806744e-05 -5.901371e-01 -8.073030e-01
    outer loop
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
      vertex   2.510000e+02 -7.267581e+01 1.695263e+02
      vertex   -3.000000e+00 -7.221780e+01 1.691637e+02
    endloop
  endfacet
  facet normal 2.497026e-04 -5.812035e-01 -8.137582e-01
    outer loop
      vertex   2.510000e+02 -6.673434e+01 1.652827e+02
      vertex   2.510000e+02 -7.267581e+01 1.695263e+02
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal 7.714877e-05 -5.711890e-01 -8.208186e-01
    outer loop
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
      vertex   2.510000e+02 -6.673434e+01 1.652827e+02
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal 2.010568e-05 -5.435970e-01 -8.393463e-01
    outer loop
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
      vertex   2.510000e+02 -6.673434e+01 1.652827e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal 2.689246e-04 -5.270632e-01 -8.498260e-01
    outer loop
      vertex   2.510000e+02 -5.986516e+01 1.610225e+02
      vertex   2.510000e+02 -6.673434e+01 1.652827e+02
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
    endloop
  endfacet
  facet normal -2.339473e-04 -5.045270e-01 -8.633959e-01
    outer loop
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
      vertex   2.510000e+02 -5.986516e+01 1.610225e+02
      vertex   -3.000000e+00 -6.402706e+01 1.635233e+02
    endloop
  endfacet
  facet normal -2.641929e-05 -4.892626e-01 -8.721365e-01
    outer loop
      vertex   2.510000e+02 -5.645351e+01 1.591085e+02
      vertex   2.510000e+02 -5.986516e+01 1.610225e+02
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
    endloop
  endfacet
  facet normal -9.744039e-05 -4.719702e-01 -8.816145e-01
    outer loop
      vertex   8.717284e+01 -5.489552e+01 1.582926e+02
      vertex   2.510000e+02 -5.645351e+01 1.591085e+02
      vertex   -3.000000e+00 -5.724815e+01 1.595620e+02
    endloop
  endfacet
  facet normal 1.012921e-02 -5.918886e-01 8.059562e-01
    outer loop
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
      vertex   -1.146481e+01 -7.054294e+01 3.620737e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
    endloop
  endfacet
  facet normal 2.163831e-03 -5.770355e-01 8.167162e-01
    outer loop
      vertex   -1.594969e+01 -6.875972e+01 3.633455e+02
      vertex   -1.146481e+01 -7.054294e+01 3.620737e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
    endloop
  endfacet
  facet normal 2.899871e-02 -5.582575e-01 8.291608e-01
    outer loop
      vertex   -1.146481e+01 -7.054294e+01 3.620737e+02
      vertex   -9.636137e+00 -7.160015e+01 3.612980e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
    endloop
  endfacet
  facet normal 9.757520e-03 -6.089408e-01 7.931556e-01
    outer loop
      vertex   -7.616774e+00 -7.311148e+01 3.601557e+02
      vertex   -6.510307e+00 -7.415342e+01 3.593421e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
    endloop
  endfacet
  facet normal 1.183915e-02 -5.981127e-01 8.013245e-01
    outer loop
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   -8.248649e+00 -7.259507e+01 3.605505e+02
      vertex   -7.616774e+00 -7.311148e+01 3.601557e+02
    endloop
  endfacet
  facet normal 1.548700e-02 -5.867213e-01 8.096408e-01
    outer loop
      vertex   -8.248649e+00 -7.259507e+01 3.605505e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   -9.636137e+00 -7.160015e+01 3.612980e+02
    endloop
  endfacet
  facet normal 1.318481e-02 -6.243779e-01 7.810111e-01
    outer loop
      vertex   -6.510307e+00 -7.415342e+01 3.593421e+02
      vertex   -4.024838e+00 -7.717656e+01 3.568833e+02
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
    endloop
  endfacet
  facet normal 6.716568e-02 -6.325638e-01 7.715905e-01
    outer loop
      vertex   -3.000000e+00 -7.364636e+01 3.596882e+02
      vertex   -4.024838e+00 -7.717656e+01 3.568833e+02
      vertex   -2.516175e+00 -7.999101e+01 3.544447e+02
    endloop
  endfacet
  facet normal 5.037290e-05 -6.692526e-01 7.430350e-01
    outer loop
      vertex   2.510000e+02 -8.183971e+01 3.527623e+02
      vertex   2.510000e+02 -7.875061e+01 3.555447e+02
      vertex   -2.516175e+00 -7.999101e+01 3.544447e+02
    endloop
  endfacet
  facet normal -1.188038e-04 -6.498082e-01 7.600982e-01
    outer loop
      vertex   2.510000e+02 -7.872691e+01 3.555650e+02
      vertex   -2.516175e+00 -7.999101e+01 3.544447e+02
      vertex   2.510000e+02 -7.875061e+01 3.555447e+02
    endloop
  endfacet
  facet normal -1.316540e-04 -6.828558e-01 7.305532e-01
    outer loop
      vertex   2.510000e+02 -8.183971e+01 3.527623e+02
      vertex   -2.516175e+00 -7.999101e+01 3.544447e+02
      vertex   -1.531095e+00 -8.399028e+01 3.507067e+02
    endloop
  endfacet
  facet normal -2.443449e-04 -6.758355e-01 7.370525e-01
    outer loop
      vertex   2.510000e+02 -8.183971e+01 3.527623e+02
      vertex   -1.531095e+00 -8.399028e+01 3.507067e+02
      vertex   2.510000e+02 -8.185120e+01 3.527518e+02
    endloop
  endfacet
  facet normal 2.668399e-04 -7.062949e-01 7.079177e-01
    outer loop
      vertex   2.510000e+02 -8.185120e+01 3.527518e+02
      vertex   -1.504616e+00 -8.451730e+01 3.501870e+02
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
    endloop
  endfacet
  facet normal 1.546942e-04 -7.009321e-01 7.132280e-01
    outer loop
      vertex   2.510000e+02 -8.631536e+01 3.483646e+02
      vertex   2.510000e+02 -8.185120e+01 3.527518e+02
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
    endloop
  endfacet
  facet normal -1.896877e-04 -7.252441e-01 6.884918e-01
    outer loop
      vertex   2.510000e+02 -8.632072e+01 3.483590e+02
      vertex   2.510000e+02 -8.631536e+01 3.483646e+02
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
    endloop
  endfacet
  facet normal 1.809350e-04 -7.021316e-01 7.120472e-01
    outer loop
      vertex   2.510000e+02 -8.185120e+01 3.527518e+02
      vertex   -1.522146e+00 -8.416840e+01 3.505310e+02
      vertex   -1.504616e+00 -8.451730e+01 3.501870e+02
    endloop
  endfacet
  facet normal 1.805382e-04 -7.021093e-01 7.120691e-01
    outer loop
      vertex   -1.531095e+00 -8.399028e+01 3.507067e+02
      vertex   -1.522146e+00 -8.416840e+01 3.505310e+02
      vertex   2.510000e+02 -8.185120e+01 3.527518e+02
    endloop
  endfacet
  facet normal -1.905881e-04 -7.253063e-01 6.884263e-01
    outer loop
      vertex   2.510000e+02 -8.775182e+01 3.468512e+02
      vertex   2.510000e+02 -8.632072e+01 3.483590e+02
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
    endloop
  endfacet
  facet normal -5.037811e-04 -7.370524e-01 6.758354e-01
    outer loop
      vertex   2.510000e+02 -8.776235e+01 3.468397e+02
      vertex   2.510000e+02 -8.775182e+01 3.468512e+02
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
    endloop
  endfacet
  facet normal -8.174785e-05 -7.490277e-01 6.625387e-01
    outer loop
      vertex   2.510000e+02 -8.916174e+01 3.453130e+02
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   2.510000e+02 -8.917723e+01 3.452955e+02
    endloop
  endfacet
  facet normal -9.487881e-06 -7.371694e-01 6.757079e-01
    outer loop
      vertex   2.510000e+02 -8.776235e+01 3.468397e+02
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   2.510000e+02 -8.916174e+01 3.453130e+02
    endloop
  endfacet
  facet normal -1.007886e-04 -7.219510e-01 6.919442e-01
    outer loop
      vertex   -1.504450e+00 -8.455477e+01 3.501496e+02
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   2.510000e+02 -8.776235e+01 3.468397e+02
    endloop
  endfacet
  facet normal -2.940103e-04 -7.601847e-01 6.497070e-01
    outer loop
      vertex   2.510000e+02 -9.054470e+01 3.437506e+02
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   2.510000e+02 -9.056495e+01 3.437269e+02
    endloop
  endfacet
  facet normal -8.031268e-05 -7.487996e-01 6.627965e-01
    outer loop
      vertex   2.510000e+02 -8.917723e+01 3.452955e+02
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   2.510000e+02 -9.054470e+01 3.437506e+02
    endloop
  endfacet
  facet normal -1.399832e-04 -7.520827e-01 6.590687e-01
    outer loop
      vertex   -2.150407e+00 -8.846244e+01 3.460724e+02
      vertex   -4.174363e+00 -9.211172e+01 3.419077e+02
      vertex   2.510000e+02 -9.056495e+01 3.437269e+02
    endloop
  endfacet
  facet normal 1.356058e-02 -7.784330e-01 6.275812e-01
    outer loop
      vertex   -4.174363e+00 -9.211172e+01 3.419077e+02
      vertex   -5.596422e+00 -9.361256e+01 3.400768e+02
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
    endloop
  endfacet
  facet normal 9.550604e-03 -7.871984e-01 6.166258e-01
    outer loop
      vertex   -5.596422e+00 -9.361256e+01 3.400768e+02
      vertex   -6.290087e+00 -9.419487e+01 3.393441e+02
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
    endloop
  endfacet
  facet normal 1.046826e-02 -7.967305e-01 6.042441e-01
    outer loop
      vertex   -6.290087e+00 -9.419487e+01 3.393441e+02
      vertex   -8.405305e+00 -9.562607e+01 3.374937e+02
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
    endloop
  endfacet
  facet normal 7.663352e-03 -7.991959e-01 6.010217e-01
    outer loop
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
      vertex   -1.071648e+01 -9.676842e+01 3.359618e+02
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
    endloop
  endfacet
  facet normal 3.285673e-02 -8.243325e-01 5.651517e-01
    outer loop
      vertex   -3.000000e+00 -9.405082e+01 3.394771e+02
      vertex   -8.405305e+00 -9.562607e+01 3.374937e+02
      vertex   -1.071648e+01 -9.676842e+01 3.359618e+02
    endloop
  endfacet
  facet normal -1.650388e-04 -8.241216e-01 5.664129e-01
    outer loop
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
      vertex   -1.532520e+01 -9.823047e+01 3.339242e+02
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
    endloop
  endfacet
  facet normal 2.622894e-05 -8.125138e-01 5.829420e-01
    outer loop
      vertex   -1.071648e+01 -9.676842e+01 3.359618e+02
      vertex   -1.532520e+01 -9.823047e+01 3.339242e+02
      vertex   -3.000000e+00 -9.829824e+01 3.338292e+02
    endloop
  endfacet
  facet normal -1.459490e-04 -8.230829e-01 5.679213e-01
    outer loop
      vertex   -2.473293e+01 -9.895531e+01 3.328811e+02
      vertex   -2.775075e+01 -9.859607e+01 3.334010e+02
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
    endloop
  endfacet
  facet normal -1.755655e-04 -8.258344e-01 5.639127e-01
    outer loop
      vertex   -2.415133e+01 -9.898752e+01 3.328341e+02
      vertex   -2.473293e+01 -9.895531e+01 3.328811e+02
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
    endloop
  endfacet
  facet normal -1.950372e-04 -8.275679e-01 5.613657e-01
    outer loop
      vertex   -2.241685e+01 -9.903052e+01 3.327713e+02
      vertex   -2.415133e+01 -9.898752e+01 3.328341e+02
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
    endloop
  endfacet
  facet normal 1.225325e-04 -8.254846e-01 5.644246e-01
    outer loop
      vertex   -2.012587e+01 -9.893602e+01 3.329090e+02
      vertex   -2.241685e+01 -9.903052e+01 3.327713e+02
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
    endloop
  endfacet
  facet normal 4.571699e-04 -8.221520e-01 5.692678e-01
    outer loop
      vertex   -3.000000e+00 -9.948451e+01 3.321032e+02
      vertex   -1.532520e+01 -9.823047e+01 3.339242e+02
      vertex   -2.012587e+01 -9.893602e+01 3.329090e+02
    endloop
  endfacet
  facet normal -1.032239e-04 -8.159160e-01 5.781704e-01
    outer loop
      vertex   -2.775075e+01 -9.859607e+01 3.334010e+02
      vertex   -3.330800e+01 -9.718194e+01 3.353956e+02
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
    endloop
  endfacet
  facet normal -3.904627e-04 -7.993146e-01 6.009127e-01
    outer loop
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
      vertex   -3.521233e+01 -9.637436e+01 3.364977e+02
      vertex   -3.701296e+01 -9.539036e+01 3.378054e+02
    endloop
  endfacet
  facet normal -2.191240e-04 -8.067951e-01 5.908313e-01
    outer loop
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
      vertex   -3.330800e+01 -9.718194e+01 3.353956e+02
      vertex   -3.521233e+01 -9.637436e+01 3.364977e+02
    endloop
  endfacet
  facet normal -1.931050e-04 -7.876512e-01 6.161213e-01
    outer loop
      vertex   -2.570000e+02 -9.306978e+01 3.407031e+02
      vertex   -3.701296e+01 -9.539036e+01 3.378054e+02
      vertex   -3.880718e+01 -9.405334e+01 3.395141e+02
    endloop
  endfacet
  facet normal -4.606287e-04 -7.973210e-01 6.035553e-01
    outer loop
      vertex   -2.570000e+02 -9.812386e+01 3.340265e+02
      vertex   -3.701296e+01 -9.539036e+01 3.378054e+02
      vertex   -2.570000e+02 -9.306978e+01 3.407031e+02
    endloop
  endfacet
  facet normal -4.409897e-05 -7.744939e-01 6.325813e-01
    outer loop
      vertex   -2.570000e+02 -9.306978e+01 3.407031e+02
      vertex   -3.880718e+01 -9.405334e+01 3.395141e+02
      vertex   -4.119671e+01 -9.161777e+01 3.424959e+02
    endloop
  endfacet
  facet normal 3.798893e-04 -7.628647e-01 6.465581e-01
    outer loop
      vertex   -2.570000e+02 -9.306978e+01 3.407031e+02
      vertex   -4.311689e+01 -8.776243e+01 3.468395e+02
      vertex   -2.570000e+02 -8.917738e+01 3.452957e+02
    endloop
  endfacet
  facet normal -4.809204e-04 -7.479967e-01 6.637022e-01
    outer loop
      vertex   -2.570000e+02 -9.306978e+01 3.407031e+02
      vertex   -4.119671e+01 -9.161777e+01 3.424959e+02
      vertex   -4.311689e+01 -8.776243e+01 3.468395e+02
    endloop
  endfacet
  facet normal 9.968363e-07 -7.372728e-01 6.755951e-01
    outer loop
      vertex   -2.570000e+02 -8.917738e+01 3.452957e+02
      vertex   -4.311689e+01 -8.776243e+01 3.468395e+02
      vertex   -2.570000e+02 -8.776244e+01 3.468398e+02
    endloop
  endfacet
  facet normal 1.069580e-06 -6.853645e-01 7.282001e-01
    outer loop
      vertex   -2.570000e+02 -8.776244e+01 3.468398e+02
      vertex   -4.311689e+01 -8.776243e+01 3.468395e+02
      vertex   -4.311694e+01 -8.776227e+01 3.468396e+02
    endloop
  endfacet
  facet normal 1.072048e-06 -7.078483e-01 7.063645e-01
    outer loop
      vertex   -2.570000e+02 -8.776244e+01 3.468398e+02
      vertex   -4.311694e+01 -8.776227e+01 3.468396e+02
      vertex   -2.570000e+02 -8.314054e+01 3.514714e+02
    endloop
  endfacet
  facet normal -2.562820e-04 -7.137654e-01 7.003849e-01
    outer loop
      vertex   -4.311694e+01 -8.776227e+01 3.468396e+02
      vertex   -4.339487e+01 -8.376194e+01 3.509163e+02
      vertex   -2.570000e+02 -8.314054e+01 3.514714e+02
    endloop
  endfacet
  facet normal -1.252418e-04 -6.898091e-01 7.239913e-01
    outer loop
      vertex   -2.570000e+02 -8.314054e+01 3.514714e+02
      vertex   -4.339487e+01 -8.376194e+01 3.509163e+02
      vertex   -4.297701e+01 -8.143549e+01 3.531330e+02
    endloop
  endfacet
  facet normal -2.250633e-04 -6.660165e-01 7.459369e-01
    outer loop
      vertex   -2.570000e+02 -7.732190e+01 3.567413e+02
      vertex   -4.297701e+01 -8.143549e+01 3.531330e+02
      vertex   -4.171336e+01 -7.836073e+01 3.558787e+02
    endloop
  endfacet
  facet normal -4.063462e-04 -6.712892e-01 7.411954e-01
    outer loop
      vertex   -2.570000e+02 -7.732190e+01 3.567413e+02
      vertex   -2.570000e+02 -8.314054e+01 3.514714e+02
      vertex   -4.297701e+01 -8.143549e+01 3.531330e+02
    endloop
  endfacet
  facet normal -6.597090e-05 -6.468774e-01 7.625940e-01
    outer loop
      vertex   -2.570000e+02 -7.732190e+01 3.567413e+02
      vertex   -4.171336e+01 -7.836073e+01 3.558787e+02
      vertex   -4.068914e+01 -7.668941e+01 3.572965e+02
    endloop
  endfacet
  facet normal -1.507635e-04 -6.300989e-01 7.765149e-01
    outer loop
      vertex   -4.068914e+01 -7.668941e+01 3.572965e+02
      vertex   -3.954654e+01 -7.531286e+01 3.584137e+02
      vertex   -2.570000e+02 -7.732190e+01 3.567413e+02
    endloop
  endfacet
  facet normal -2.228060e-04 -6.048029e-01 7.963752e-01
    outer loop
      vertex   -3.740952e+01 -7.313692e+01 3.601357e+02
      vertex   -3.634933e+01 -7.231635e+01 3.607592e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -5.582061e-04 -6.202351e-01 7.844158e-01
    outer loop
      vertex   -3.954654e+01 -7.531286e+01 3.584137e+02
      vertex   -3.740952e+01 -7.313692e+01 3.601357e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -3.755618e-04 -6.154648e-01 7.881643e-01
    outer loop
      vertex   -2.570000e+02 -7.732190e+01 3.567413e+02
      vertex   -3.954654e+01 -7.531286e+01 3.584137e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -8.150329e-05 -5.957109e-01 8.031990e-01
    outer loop
      vertex   -3.634933e+01 -7.231635e+01 3.607592e+02
      vertex   -3.356405e+01 -7.054906e+01 3.620702e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -6.022358e-05 -5.795398e-01 8.149439e-01
    outer loop
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
      vertex   -3.222910e+01 -6.988381e+01 3.625509e+02
      vertex   -3.083888e+01 -6.931573e+01 3.629550e+02
    endloop
  endfacet
  facet normal -5.102596e-05 -5.855811e-01 8.106138e-01
    outer loop
      vertex   -3.356405e+01 -7.054906e+01 3.620702e+02
      vertex   -3.222910e+01 -6.988381e+01 3.625509e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal 9.717144e-04 -5.685242e-01 8.226660e-01
    outer loop
      vertex   -1.865433e+01 -6.813744e+01 3.637788e+02
      vertex   -1.594969e+01 -6.875972e+01 3.633455e+02
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
    endloop
  endfacet
  facet normal 5.831507e-04 -5.616355e-01 8.273846e-01
    outer loop
      vertex   -3.000000e+00 -6.752954e+01 3.641804e+02
      vertex   -2.310793e+01 -6.778645e+01 3.640202e+02
      vertex   -1.865433e+01 -6.813744e+01 3.637788e+02
    endloop
  endfacet
  facet normal -9.485232e-05 -5.662144e-01 8.242580e-01
    outer loop
      vertex   -2.686896e+01 -6.819242e+01 3.637408e+02
      vertex   -2.310793e+01 -6.778645e+01 3.640202e+02
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal -1.774952e-04 -5.728221e-01 8.196797e-01
    outer loop
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
      vertex   -3.083888e+01 -6.931573e+01 3.629550e+02
      vertex   -2.686896e+01 -6.819242e+01 3.637408e+02
    endloop
  endfacet
  facet normal -1.125975e-04 -5.695981e-01 8.219234e-01
    outer loop
      vertex   -2.570000e+02 -6.622992e+01 3.650625e+02
      vertex   -2.570000e+02 -7.010260e+01 3.623787e+02
      vertex   -3.083888e+01 -6.931573e+01 3.629550e+02
    endloop
  endfacet
  facet normal 6.974959e-03 8.172285e-01 5.762715e-01
    outer loop
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
      vertex   -1.406641e+01 9.786776e+01 3.344337e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
    endloop
  endfacet
  facet normal 4.397003e-03 8.120439e-01 5.835798e-01
    outer loop
      vertex   -1.406641e+01 9.786776e+01 3.344337e+02
      vertex   -8.900414e+00 9.591057e+01 3.371181e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
    endloop
  endfacet
  facet normal 4.917805e-03 7.988098e-01 6.015636e-01
    outer loop
      vertex   -8.900414e+00 9.591057e+01 3.371181e+02
      vertex   -7.793053e+00 9.525175e+01 3.379839e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
    endloop
  endfacet
  facet normal 1.414929e-02 7.886969e-01 6.146194e-01
    outer loop
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
      vertex   -5.807900e+00 9.378835e+01 3.398552e+02
      vertex   -3.000000e+00 9.259142e+01 3.413265e+02
    endloop
  endfacet
  facet normal 7.891071e-03 7.917425e-01 6.108040e-01
    outer loop
      vertex   -7.793053e+00 9.525175e+01 3.379839e+02
      vertex   -5.807900e+00 9.378835e+01 3.398552e+02
      vertex   -3.000000e+00 9.596999e+01 3.369910e+02
    endloop
  endfacet
  facet normal -7.918485e-03 7.682501e-01 6.401009e-01
    outer loop
      vertex   -5.807900e+00 9.378835e+01 3.398552e+02
      vertex   -3.722995e+00 9.148266e+01 3.426483e+02
      vertex   -3.000000e+00 9.259142e+01 3.413265e+02
    endloop
  endfacet
  facet normal 2.785094e-02 7.582955e-01 6.513158e-01
    outer loop
      vertex   -3.000000e+00 9.259142e+01 3.413265e+02
      vertex   -3.722995e+00 9.148266e+01 3.426483e+02
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
    endloop
  endfacet
  facet normal -3.660268e-05 7.487996e-01 6.627965e-01
    outer loop
      vertex   2.510000e+02 9.054470e+01 3.437506e+02
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
      vertex   2.510000e+02 8.917723e+01 3.452955e+02
    endloop
  endfacet
  facet normal -2.093571e-04 7.601847e-01 6.497070e-01
    outer loop
      vertex   2.510000e+02 9.054470e+01 3.437506e+02
      vertex   2.510000e+02 9.056495e+01 3.437269e+02
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
    endloop
  endfacet
  facet normal 2.531426e-04 7.192940e-01 6.947057e-01
    outer loop
      vertex   2.510000e+02 8.916174e+01 3.453130e+02
      vertex   -1.581109e+00 8.584428e+01 3.488400e+02
      vertex   2.510000e+02 8.336461e+01 3.513153e+02
    endloop
  endfacet
  facet normal 1.696398e-05 7.277985e-01 6.857911e-01
    outer loop
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
      vertex   -1.581109e+00 8.584428e+01 3.488400e+02
      vertex   2.510000e+02 8.916174e+01 3.453130e+02
    endloop
  endfacet
  facet normal -3.722676e-05 7.490277e-01 6.625387e-01
    outer loop
      vertex   2.510000e+02 8.917723e+01 3.452955e+02
      vertex   -2.282889e+00 8.886592e+01 3.456332e+02
      vertex   2.510000e+02 8.916174e+01 3.453130e+02
    endloop
  endfacet
  facet normal -9.562185e-05 6.891668e-01 7.246027e-01
    outer loop
      vertex   2.510000e+02 8.336461e+01 3.513153e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.335896e+01 3.513207e+02
    endloop
  endfacet
  facet normal 3.869733e-05 7.084683e-01 7.057426e-01
    outer loop
      vertex   -1.581109e+00 8.584428e+01 3.488400e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.336461e+01 3.513153e+02
    endloop
  endfacet
  facet normal -3.286383e-04 6.758355e-01 7.370524e-01
    outer loop
      vertex   2.510000e+02 8.185120e+01 3.527518e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.183971e+01 3.527623e+02
    endloop
  endfacet
  facet normal -1.007536e-04 6.884239e-01 7.253086e-01
    outer loop
      vertex   2.510000e+02 8.335896e+01 3.513207e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.185120e+01 3.527518e+02
    endloop
  endfacet
  facet normal -2.752192e-05 6.859498e-01 7.276489e-01
    outer loop
      vertex   2.510000e+02 8.029554e+01 3.541772e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   -2.455305e+00 8.008673e+01 3.543645e+02
    endloop
  endfacet
  facet normal -6.797676e-04 6.636549e-01 7.480386e-01
    outer loop
      vertex   2.510000e+02 8.031304e+01 3.541617e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.029554e+01 3.541772e+02
    endloop
  endfacet
  facet normal -3.309430e-04 6.757077e-01 7.371696e-01
    outer loop
      vertex   2.510000e+02 8.183971e+01 3.527623e+02
      vertex   -1.555947e+00 8.425868e+01 3.504317e+02
      vertex   2.510000e+02 8.031304e+01 3.541617e+02
    endloop
  endfacet
  facet normal -7.988221e-02 6.775912e-01 7.310875e-01
    outer loop
      vertex   -3.000000e+00 7.872708e+01 3.555651e+02
      vertex   -2.455305e+00 8.008673e+01 3.543645e+02
      vertex   -3.558048e+00 7.796090e+01 3.562143e+02
    endloop
  endfacet
  facet normal -1.137805e-04 6.498082e-01 7.600982e-01
    outer loop
      vertex   2.510000e+02 7.875061e+01 3.555447e+02
      vertex   -2.455305e+00 8.008673e+01 3.543645e+02
      vertex   2.510000e+02 7.872691e+01 3.555650e+02
    endloop
  endfacet
  facet normal 7.196774e-06 6.627855e-01 7.488093e-01
    outer loop
      vertex   2.510000e+02 7.875061e+01 3.555447e+02
      vertex   2.510000e+02 8.029554e+01 3.541772e+02
      vertex   -2.455305e+00 8.008673e+01 3.543645e+02
    endloop
  endfacet
  facet normal -7.128218e-03 6.494276e-01 7.603900e-01
    outer loop
      vertex   -3.000000e+00 7.872708e+01 3.555651e+02
      vertex   -3.558048e+00 7.796090e+01 3.562143e+02
      vertex   -3.000000e+00 7.713451e+01 3.569253e+02
    endloop
  endfacet
  facet normal 1.001199e-02 6.379231e-01 7.700350e-01
    outer loop
      vertex   -3.558048e+00 7.796090e+01 3.562143e+02
      vertex   -5.160134e+00 7.560135e+01 3.581898e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal -5.250448e-02 6.307055e-01 7.742440e-01
    outer loop
      vertex   -3.000000e+00 7.713451e+01 3.569253e+02
      vertex   -3.558048e+00 7.796090e+01 3.562143e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal 6.223977e-03 6.024393e-01 7.981405e-01
    outer loop
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
      vertex   -7.852061e+00 7.302101e+01 3.602137e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal 5.491143e-04 6.127416e-01 7.902831e-01
    outer loop
      vertex   -5.811024e+00 7.485208e+01 3.587926e+02
      vertex   -7.852061e+00 7.302101e+01 3.602137e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal 1.579650e-03 6.245577e-01 7.809771e-01
    outer loop
      vertex   -5.427251e+00 7.528414e+01 3.584463e+02
      vertex   -5.811024e+00 7.485208e+01 3.587926e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal 2.658616e-03 6.272931e-01 7.787787e-01
    outer loop
      vertex   -5.160134e+00 7.560135e+01 3.581898e+02
      vertex   -5.427251e+00 7.528414e+01 3.584463e+02
      vertex   -3.000000e+00 7.469891e+01 3.589094e+02
    endloop
  endfacet
  facet normal 1.350911e-03 5.885225e-01 8.084797e-01
    outer loop
      vertex   -9.802357e+00 7.148827e+01 3.613814e+02
      vertex   -1.205183e+01 7.022472e+01 3.623049e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal 5.438525e-03 6.016045e-01 7.987756e-01
    outer loop
      vertex   -7.852061e+00 7.302101e+01 3.602137e+02
      vertex   -9.802357e+00 7.148827e+01 3.613814e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal 2.351644e-03 5.695965e-01 8.219211e-01
    outer loop
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
      vertex   -1.614321e+01 6.867019e+01 3.634090e+02
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal 1.129229e-03 5.770589e-01 8.167018e-01
    outer loop
      vertex   -1.205183e+01 7.022472e+01 3.623049e+02
      vertex   -1.614321e+01 6.867019e+01 3.634090e+02
      vertex   -3.000000e+00 7.010260e+01 3.623787e+02
    endloop
  endfacet
  facet normal -3.287391e-04 5.667650e-01 8.238795e-01
    outer loop
      vertex   -2.597064e+01 6.807118e+01 3.638244e+02
      vertex   -2.778066e+01 6.836952e+01 3.636184e+02
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
    endloop
  endfacet
  facet normal -3.690587e-04 5.644211e-01 8.254870e-01
    outer loop
      vertex   -2.570000e+02 6.446368e+01 3.661877e+02
      vertex   -2.597064e+01 6.807118e+01 3.638244e+02
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
    endloop
  endfacet
  facet normal -1.065304e-04 5.554442e-01 8.315538e-01
    outer loop
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
      vertex   -2.236961e+01 6.777764e+01 3.640262e+02
      vertex   -2.334514e+01 6.778671e+01 3.640200e+02
    endloop
  endfacet
  facet normal 4.100915e-04 5.599036e-01 8.285577e-01
    outer loop
      vertex   -2.063038e+01 6.786409e+01 3.639669e+02
      vertex   -2.236961e+01 6.777764e+01 3.640262e+02
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
    endloop
  endfacet
  facet normal -3.665247e-04 5.643090e-01 8.255636e-01
    outer loop
      vertex   -2.334514e+01 6.778671e+01 3.640200e+02
      vertex   -2.597064e+01 6.807118e+01 3.638244e+02
      vertex   -2.570000e+02 6.446368e+01 3.661877e+02
    endloop
  endfacet
  facet normal 1.096413e-03 5.649885e-01 8.250980e-01
    outer loop
      vertex   -3.000000e+00 6.622992e+01 3.650625e+02
      vertex   -1.614321e+01 6.867019e+01 3.634090e+02
      vertex   -2.063038e+01 6.786409e+01 3.639669e+02
    endloop
  endfacet
  facet normal -1.199355e-04 5.854757e-01 8.106899e-01
    outer loop
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
      vertex   -3.145869e+01 6.956229e+01 3.627798e+02
      vertex   -3.435621e+01 7.098924e+01 3.617488e+02
    endloop
  endfacet
  facet normal -2.050373e-04 5.747512e-01 8.183282e-01
    outer loop
      vertex   -2.778066e+01 6.836952e+01 3.636184e+02
      vertex   -3.145869e+01 6.956229e+01 3.627798e+02
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
    endloop
  endfacet
  facet normal -1.471667e-04 6.016249e-01 7.987787e-01
    outer loop
      vertex   -3.435621e+01 7.098924e+01 3.617488e+02
      vertex   -3.717712e+01 7.300696e+01 3.602286e+02
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
    endloop
  endfacet
  facet normal -3.846609e-04 6.167020e-01 7.871966e-01
    outer loop
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
      vertex   -3.974378e+01 7.546915e+01 3.582964e+02
      vertex   -2.570000e+02 7.752678e+01 3.565783e+02
    endloop
  endfacet
  facet normal -9.555263e-05 6.349799e-01 7.725287e-01
    outer loop
      vertex   -2.570000e+02 7.752678e+01 3.565783e+02
      vertex   -3.974378e+01 7.546915e+01 3.582964e+02
      vertex   -4.083983e+01 7.691170e+01 3.571106e+02
    endloop
  endfacet
  facet normal -3.979792e-04 6.170843e-01 7.868969e-01
    outer loop
      vertex   -3.717712e+01 7.300696e+01 3.602286e+02
      vertex   -3.974378e+01 7.546915e+01 3.582964e+02
      vertex   -2.570000e+02 7.072844e+01 3.619042e+02
    endloop
  endfacet
  facet normal -2.119964e-05 6.501209e-01 7.598307e-01
    outer loop
      vertex   -4.083983e+01 7.691170e+01 3.571106e+02
      vertex   -4.181630e+01 7.860445e+01 3.556622e+02
      vertex   -2.570000e+02 7.752678e+01 3.565783e+02
    endloop
  endfacet
  facet normal -6.457253e-05 6.551207e-01 7.555242e-01
    outer loop
      vertex   -2.570000e+02 7.752678e+01 3.565783e+02
      vertex   -4.181630e+01 7.860445e+01 3.556622e+02
      vertex   -2.570000e+02 8.029568e+01 3.541774e+02
    endloop
  endfacet
  facet normal -8.749303e-05 6.855334e-01 7.280412e-01
    outer loop
      vertex   -2.570000e+02 8.029568e+01 3.541774e+02
      vertex   -4.334353e+01 8.315371e+01 3.515119e+02
      vertex   -2.570000e+02 8.372610e+01 3.509472e+02
    endloop
  endfacet
  facet normal 1.998562e-04 6.740119e-01 7.387205e-01
    outer loop
      vertex   -4.181630e+01 7.860445e+01 3.556622e+02
      vertex   -4.334353e+01 8.315371e+01 3.515119e+02
      vertex   -2.570000e+02 8.029568e+01 3.541774e+02
    endloop
  endfacet
  facet normal -1.068021e-04 7.092607e-01 7.049463e-01
    outer loop
      vertex   -4.348146e+01 8.438995e+01 3.503117e+02
      vertex   -4.345567e+01 8.566367e+01 3.490302e+02
      vertex   -2.570000e+02 8.372610e+01 3.509472e+02
    endloop
  endfacet
  facet normal -3.002507e-05 6.965724e-01 7.174865e-01
    outer loop
      vertex   -2.570000e+02 8.372610e+01 3.509472e+02
      vertex   -4.334353e+01 8.315371e+01 3.515119e+02
      vertex   -4.348146e+01 8.438995e+01 3.503117e+02
    endloop
  endfacet
  facet normal -2.390210e-04 7.204392e-01 6.935180e-01
    outer loop
      vertex   -4.345567e+01 8.566367e+01 3.490302e+02
      vertex   -4.325159e+01 8.710465e+01 3.475333e+02
      vertex   -2.570000e+02 8.845644e+01 3.460554e+02
    endloop
  endfacet
  facet normal -2.820721e-04 7.188747e-01 6.951396e-01
    outer loop
      vertex   -2.570000e+02 8.372610e+01 3.509472e+02
      vertex   -4.345567e+01 8.566367e+01 3.490302e+02
      vertex   -2.570000e+02 8.845644e+01 3.460554e+02
    endloop
  endfacet
  facet normal -3.870857e-05 7.351009e-01 6.779577e-01
    outer loop
      vertex   -2.570000e+02 8.845644e+01 3.460554e+02
      vertex   -4.325159e+01 8.710465e+01 3.475333e+02
      vertex   -4.269644e+01 8.884487e+01 3.456464e+02
    endloop
  endfacet
  facet normal -8.330471e-04 7.533461e-01 6.576237e-01
    outer loop
      vertex   -4.269644e+01 8.884487e+01 3.456464e+02
      vertex   -4.087908e+01 9.198120e+01 3.420559e+02
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
    endloop
  endfacet
  facet normal -1.526137e-04 7.637660e-01 6.454933e-01
    outer loop
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
      vertex   -2.570000e+02 8.845644e+01 3.460554e+02
      vertex   -4.269644e+01 8.884487e+01 3.456464e+02
    endloop
  endfacet
  facet normal -3.070025e-04 7.702669e-01 6.377216e-01
    outer loop
      vertex   -4.087908e+01 9.198120e+01 3.420559e+02
      vertex   -3.966828e+01 9.336018e+01 3.403909e+02
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
    endloop
  endfacet
  facet normal -5.744665e-05 7.876082e-01 6.161763e-01
    outer loop
      vertex   -3.935779e+01 9.364923e+01 3.400318e+02
      vertex   -3.780420e+01 9.480156e+01 3.385590e+02
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
    endloop
  endfacet
  facet normal -1.671743e-04 7.790345e-01 6.269811e-01
    outer loop
      vertex   -3.935779e+01 9.364923e+01 3.400318e+02
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
      vertex   -3.966828e+01 9.336018e+01 3.403909e+02
    endloop
  endfacet
  facet normal -6.630472e-05 7.961750e-01 6.050664e-01
    outer loop
      vertex   -2.570000e+02 9.470631e+01 3.386604e+02
      vertex   -3.780420e+01 9.480156e+01 3.385590e+02
      vertex   -2.570000e+02 9.708204e+01 3.355342e+02
    endloop
  endfacet
  facet normal 3.954080e-05 8.255353e-01 5.643505e-01
    outer loop
      vertex   -2.570000e+02 9.708204e+01 3.355342e+02
      vertex   -3.398312e+01 9.691716e+01 3.357598e+02
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
    endloop
  endfacet
  facet normal -1.760121e-06 8.064824e-01 5.912580e-01
    outer loop
      vertex   -3.481652e+01 9.656103e+01 3.362456e+02
      vertex   -3.398312e+01 9.691716e+01 3.357598e+02
      vertex   -2.570000e+02 9.708204e+01 3.355342e+02
    endloop
  endfacet
  facet normal -7.128092e-05 7.960008e-01 6.052956e-01
    outer loop
      vertex   -2.570000e+02 9.708204e+01 3.355342e+02
      vertex   -3.780420e+01 9.480156e+01 3.385590e+02
      vertex   -3.481652e+01 9.656103e+01 3.362456e+02
    endloop
  endfacet
  facet normal -4.275427e-04 8.152230e-01 5.791470e-01
    outer loop
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
      vertex   -3.398312e+01 9.691716e+01 3.357598e+02
      vertex   -2.784589e+01 9.855550e+01 3.334582e+02
    endloop
  endfacet
  facet normal 3.598410e-03 8.221008e-01 5.693306e-01
    outer loop
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
      vertex   -1.712017e+01 9.858395e+01 3.334188e+02
      vertex   -1.406641e+01 9.786776e+01 3.344337e+02
    endloop
  endfacet
  facet normal 9.700182e-04 8.287259e-01 5.596539e-01
    outer loop
      vertex   -2.386830e+01 9.894104e+01 3.329017e+02
      vertex   -1.712017e+01 9.858395e+01 3.334188e+02
      vertex   -3.000000e+00 1.003438e+02 3.307883e+02
    endloop
  endfacet
  facet normal -2.566600e-04 8.228375e-01 5.682766e-01
    outer loop
      vertex   -2.570000e+02 1.001840e+02 3.309966e+02
      vertex   -2.784589e+01 9.855550e+01 3.334582e+02
      vertex   -2.386830e+01 9.894104e+01 3.329017e+02
    endloop
  endfacet
  facet normal 1.217621e-03 5.861384e-01 -8.102100e-01
    outer loop
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   -1.300498e+01 6.978907e+01 1.673821e+02
      vertex   -1.040515e+01 7.113698e+01 1.683611e+02
    endloop
  endfacet
  facet normal 4.161507e-03 5.694539e-01 -8.220127e-01
    outer loop
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   -1.619626e+01 6.865359e+01 1.665793e+02
      vertex   -1.300498e+01 6.978907e+01 1.673821e+02
    endloop
  endfacet
  facet normal 2.044574e-03 5.775397e-01 -8.163600e-01
    outer loop
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
      vertex   -1.619626e+01 6.865359e+01 1.665793e+02
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
    endloop
  endfacet
  facet normal 1.736487e-03 5.999195e-01 -8.000585e-01
    outer loop
      vertex   -1.040515e+01 7.113698e+01 1.683611e+02
      vertex   -7.753542e+00 7.303802e+01 1.697924e+02
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
    endloop
  endfacet
  facet normal 7.814150e-03 6.087420e-01 -7.933298e-01
    outer loop
      vertex   -3.000000e+00 7.094975e+01 1.682368e+02
      vertex   -7.753542e+00 7.303802e+01 1.697924e+02
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
    endloop
  endfacet
  facet normal 1.328180e-03 6.180138e-01 -7.861662e-01
    outer loop
      vertex   -7.753542e+00 7.303802e+01 1.697924e+02
      vertex   -5.178729e+00 7.559156e+01 1.718041e+02
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
    endloop
  endfacet
  facet normal 7.381951e-03 6.346025e-01 -7.728034e-01
    outer loop
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
      vertex   -5.178729e+00 7.559156e+01 1.718041e+02
      vertex   -3.930240e+00 7.730719e+01 1.732248e+02
    endloop
  endfacet
  facet normal 3.823806e-02 6.419176e-01 -7.658196e-01
    outer loop
      vertex   -3.930240e+00 7.730719e+01 1.732248e+02
      vertex   -2.580776e+00 7.980245e+01 1.753838e+02
      vertex   -3.000000e+00 7.510293e+01 1.714236e+02
    endloop
  endfacet
  facet normal -1.386281e-04 6.824232e-01 -7.309573e-01
    outer loop
      vertex   -2.580776e+00 7.980245e+01 1.753838e+02
      vertex   -1.531095e+00 8.399028e+01 1.792933e+02
      vertex   2.510000e+02 8.183971e+01 1.772377e+02
    endloop
  endfacet
  facet normal 5.548323e-05 6.692526e-01 -7.430350e-01
    outer loop
      vertex   2.510000e+02 7.875061e+01 1.744553e+02
      vertex   -2.580776e+00 7.980245e+01 1.753838e+02
      vertex   2.510000e+02 8.183971e+01 1.772377e+02
    endloop
  endfacet
  facet normal -9.021984e-05 6.494545e-01 -7.604004e-01
    outer loop
      vertex   2.510000e+02 7.872691e+01 1.744351e+02
      vertex   -2.580776e+00 7.980245e+01 1.753838e+02
      vertex   2.510000e+02 7.875061e+01 1.744553e+02
    endloop
  endfacet
  facet normal 1.611811e-04 7.009321e-01 -7.132280e-01
    outer loop
      vertex   2.510000e+02 8.185120e+01 1.772482e+02
      vertex   -1.531095e+00 8.399028e+01 1.792933e+02
      vertex   2.510000e+02 8.631536e+01 1.816354e+02
    endloop
  endfacet
  facet normal -2.443449e-04 6.758355e-01 -7.370525e-01
    outer loop
      vertex   2.510000e+02 8.183971e+01 1.772377e+02
      vertex   -1.531095e+00 8.399028e+01 1.792933e+02
      vertex   2.510000e+02 8.185120e+01 1.772482e+02
    endloop
  endfacet
  facet normal 7.463916e-05 7.066563e-01 -7.075570e-01
    outer loop
      vertex   2.510000e+02 8.631536e+01 1.816354e+02
      vertex   -1.504616e+00 8.451730e+01 1.798130e+02
      vertex   -1.504450e+00 8.455477e+01 1.798504e+02
    endloop
  endfacet
  facet normal -1.897397e-04 7.252441e-01 -6.884918e-01
    outer loop
      vertex   2.510000e+02 8.632072e+01 1.816410e+02
      vertex   2.510000e+02 8.631536e+01 1.816354e+02
      vertex   -1.504450e+00 8.455477e+01 1.798504e+02
    endloop
  endfacet
  facet normal 1.395326e-04 7.021129e-01 -7.120656e-01
    outer loop
      vertex   2.510000e+02 8.631536e+01 1.816354e+02
      vertex   -1.522146e+00 8.416840e+01 1.794690e+02
      vertex   -1.504616e+00 8.451730e+01 1.798130e+02
    endloop
  endfacet
  facet normal 1.395751e-04 7.021104e-01 -7.120681e-01
    outer loop
      vertex   -1.531095e+00 8.399028e+01 1.792933e+02
      vertex   -1.522146e+00 8.416840e+01 1.794690e+02
      vertex   2.510000e+02 8.631536e+01 1.816354e+02
    endloop
  endfacet
  facet normal -1.906401e-04 7.253063e-01 -6.884263e-01
    outer loop
      vertex   2.510000e+02 8.775182e+01 1.831488e+02
      vertex   2.510000e+02 8.632072e+01 1.816410e+02
      vertex   -1.504450e+00 8.455477e+01 1.798504e+02
    endloop
  endfacet
  facet normal -5.038322e-04 7.370524e-01 -6.758354e-01
    outer loop
      vertex   2.510000e+02 8.776235e+01 1.831603e+02
      vertex   2.510000e+02 8.775182e+01 1.831488e+02
      vertex   -1.504450e+00 8.455477e+01 1.798504e+02
    endloop
  endfacet
  facet normal -2.977123e-05 7.371694e-01 -6.757079e-01
    outer loop
      vertex   2.510000e+02 8.916174e+01 1.846870e+02
      vertex   2.510000e+02 8.776235e+01 1.831603e+02
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
    endloop
  endfacet
  facet normal -1.620052e-04 7.486692e-01 -6.629437e-01
    outer loop
      vertex   2.510000e+02 8.917723e+01 1.847045e+02
      vertex   2.510000e+02 8.916174e+01 1.846870e+02
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
    endloop
  endfacet
  facet normal -4.222910e-05 7.197254e-01 -6.942588e-01
    outer loop
      vertex   -1.504450e+00 8.455477e+01 1.798504e+02
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
      vertex   2.510000e+02 8.776235e+01 1.831603e+02
    endloop
  endfacet
  facet normal -4.463855e-04 7.604428e-01 -6.494047e-01
    outer loop
      vertex   2.510000e+02 9.054470e+01 1.862494e+02
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
      vertex   2.510000e+02 9.056495e+01 1.862731e+02
    endloop
  endfacet
  facet normal -1.635848e-04 7.488037e-01 -6.627919e-01
    outer loop
      vertex   2.510000e+02 8.917723e+01 1.847045e+02
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
      vertex   2.510000e+02 9.054470e+01 1.862494e+02
    endloop
  endfacet
  facet normal -4.639278e-05 7.440405e-01 -6.681345e-01
    outer loop
      vertex   -1.964604e+00 8.784049e+01 1.832567e+02
      vertex   -3.297513e+00 9.089682e+01 1.866603e+02
      vertex   2.510000e+02 9.056495e+01 1.862731e+02
    endloop
  endfacet
  facet normal -2.273282e-03 7.652081e-01 -6.437789e-01
    outer loop
      vertex   -3.297513e+00 9.089682e+01 1.866603e+02
      vertex   -4.885621e+00 9.290666e+01 1.890549e+02
      vertex   -3.000000e+00 9.259142e+01 1.886735e+02
    endloop
  endfacet
  facet normal 3.409830e-03 7.789687e-01 -6.270535e-01
    outer loop
      vertex   -3.000000e+00 9.259142e+01 1.886735e+02
      vertex   -4.885621e+00 9.290666e+01 1.890549e+02
      vertex   -3.000000e+00 9.456129e+01 1.911206e+02
    endloop
  endfacet
  facet normal 1.478337e-03 7.993848e-01 -6.008176e-01
    outer loop
      vertex   -3.000000e+00 9.456129e+01 1.911206e+02
      vertex   -7.296565e+00 9.491563e+01 1.915815e+02
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
    endloop
  endfacet
  facet normal -2.454415e-03 7.815839e-01 -6.237953e-01
    outer loop
      vertex   -4.885621e+00 9.290666e+01 1.890549e+02
      vertex   -7.296565e+00 9.491563e+01 1.915815e+02
      vertex   -3.000000e+00 9.456129e+01 1.911206e+02
    endloop
  endfacet
  facet normal 2.247302e-03 7.987394e-01 -6.016729e-01
    outer loop
      vertex   -7.296565e+00 9.491563e+01 1.915815e+02
      vertex   -1.013446e+01 9.651190e+01 1.936900e+02
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
    endloop
  endfacet
  facet normal 1.120946e-03 8.096183e-01 -5.869556e-01
    outer loop
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
      vertex   -1.013446e+01 9.651190e+01 1.936900e+02
      vertex   -1.302021e+01 9.762377e+01 1.952181e+02
    endloop
  endfacet
  facet normal 2.845430e-03 8.196023e-01 -5.729257e-01
    outer loop
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
      vertex   -1.764671e+01 9.867125e+01 1.967071e+02
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
    endloop
  endfacet
  facet normal 5.028358e-03 8.251572e-01 -5.648808e-01
    outer loop
      vertex   -1.302021e+01 9.762377e+01 1.952181e+02
      vertex   -1.764671e+01 9.867125e+01 1.967071e+02
      vertex   -3.000000e+00 9.676320e+01 1.940503e+02
    endloop
  endfacet
  facet normal -5.089948e-05 8.173921e-01 -5.760817e-01
    outer loop
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
      vertex   -2.896144e+01 9.840080e+01 1.963176e+02
      vertex   -3.076971e+01 9.796425e+01 1.956984e+02
    endloop
  endfacet
  facet normal -4.978317e-05 8.174816e-01 -5.759547e-01
    outer loop
      vertex   -2.570000e+02 9.708204e+01 1.944658e+02
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
      vertex   -3.076971e+01 9.796425e+01 1.956984e+02
    endloop
  endfacet
  facet normal -2.364503e-05 8.214957e-01 -5.702147e-01
    outer loop
      vertex   -2.705324e+01 9.870218e+01 1.967517e+02
      vertex   -2.896144e+01 9.840080e+01 1.963176e+02
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
    endloop
  endfacet
  facet normal -1.840199e-05 8.235147e-01 -5.672950e-01
    outer loop
      vertex   -2.398978e+01 9.898121e+01 1.971567e+02
      vertex   -2.705324e+01 9.870218e+01 1.967517e+02
      vertex   -2.570000e+02 9.889137e+01 1.970338e+02
    endloop
  endfacet
  facet normal 3.951639e-05 8.304563e-01 -5.570838e-01
    outer loop
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
      vertex   -2.056506e+01 9.897296e+01 1.971446e+02
      vertex   -2.398978e+01 9.898121e+01 1.971567e+02
    endloop
  endfacet
  facet normal 7.012408e-04 8.271324e-01 -5.620067e-01
    outer loop
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
      vertex   -1.897055e+01 9.884793e+01 1.969626e+02
      vertex   -2.056506e+01 9.897296e+01 1.971446e+02
    endloop
  endfacet
  facet normal 1.144424e-03 8.253009e-01 -5.646920e-01
    outer loop
      vertex   -3.000000e+00 1.000625e+02 1.987701e+02
      vertex   -1.764671e+01 9.867125e+01 1.967071e+02
      vertex   -1.897055e+01 9.884793e+01 1.969626e+02
    endloop
  endfacet
  facet normal 1.917811e-05 8.115104e-01 -5.843380e-01
    outer loop
      vertex   -2.570000e+02 9.708204e+01 1.944658e+02
      vertex   -3.076971e+01 9.796425e+01 1.956984e+02
      vertex   -3.430059e+01 9.677942e+01 1.940528e+02
    endloop
  endfacet
  facet normal -1.722833e-04 7.933529e-01 -6.087620e-01
    outer loop
      vertex   -2.570000e+02 9.708204e+01 1.944658e+02
      vertex   -3.700586e+01 9.536608e+01 1.921672e+02
      vertex   -2.570000e+02 9.389509e+01 1.903125e+02
    endloop
  endfacet
  facet normal -2.478153e-05 8.001889e-01 -5.997480e-01
    outer loop
      vertex   -3.430059e+01 9.677942e+01 1.940528e+02
      vertex   -3.700586e+01 9.536608e+01 1.921672e+02
      vertex   -2.570000e+02 9.708204e+01 1.944658e+02
    endloop
  endfacet
  facet normal -1.369758e-04 7.913510e-01 -6.113621e-01
    outer loop
      vertex   -2.570000e+02 9.389509e+01 1.903125e+02
      vertex   -3.700586e+01 9.536608e+01 1.921672e+02
      vertex   -3.850182e+01 9.435259e+01 1.908557e+02
    endloop
  endfacet
  facet normal -8.645846e-05 7.817483e-01 -6.235940e-01
    outer loop
      vertex   -3.850182e+01 9.435259e+01 1.908557e+02
      vertex   -3.972382e+01 9.329141e+01 1.895255e+02
      vertex   -2.570000e+02 9.389509e+01 1.903125e+02
    endloop
  endfacet
  facet normal -2.303249e-04 7.679108e-01 -6.405567e-01
    outer loop
      vertex   -2.570000e+02 9.389509e+01 1.903125e+02
      vertex   -4.121696e+01 9.158949e+01 1.874709e+02
      vertex   -2.570000e+02 8.987126e+01 1.854886e+02
    endloop
  endfacet
  facet normal -1.701539e-04 7.701790e-01 -6.378278e-01
    outer loop
      vertex   -2.570000e+02 9.389509e+01 1.903125e+02
      vertex   -3.972382e+01 9.329141e+01 1.895255e+02
      vertex   -4.121696e+01 9.158949e+01 1.874709e+02
    endloop
  endfacet
  facet normal -3.680265e-05 7.576165e-01 -6.526999e-01
    outer loop
      vertex   -4.121696e+01 9.158949e+01 1.874709e+02
      vertex   -4.217855e+01 9.010396e+01 1.857466e+02
      vertex   -2.570000e+02 8.987126e+01 1.854886e+02
    endloop
  endfacet
  facet normal -8.439587e-05 7.372781e-01 -6.755893e-01
    outer loop
      vertex   -2.570000e+02 8.987126e+01 1.854886e+02
      vertex   -4.311689e+01 8.776243e+01 1.831605e+02
      vertex   -2.570000e+02 8.704161e+01 1.824006e+02
    endloop
  endfacet
  facet normal 3.083766e-06 7.412891e-01 -6.711858e-01
    outer loop
      vertex   -2.570000e+02 8.987126e+01 1.854886e+02
      vertex   -4.217855e+01 9.010396e+01 1.857466e+02
      vertex   -4.311689e+01 8.776243e+01 1.831605e+02
    endloop
  endfacet
  facet normal 2.777782e-04 6.853213e-01 -7.282407e-01
    outer loop
      vertex   -2.570000e+02 8.704161e+01 1.824006e+02
      vertex   -4.311689e+01 8.776243e+01 1.831605e+02
      vertex   -4.311694e+01 8.776227e+01 1.831604e+02
    endloop
  endfacet
  facet normal -6.377120e-05 7.175422e-01 -6.965151e-01
    outer loop
      vertex   -2.570000e+02 8.410591e+01 1.794132e+02
      vertex   -4.311694e+01 8.776227e+01 1.831604e+02
      vertex   -4.346597e+01 8.462021e+01 1.799235e+02
    endloop
  endfacet
  facet normal 8.658467e-05 7.132504e-01 -7.009094e-01
    outer loop
      vertex   -2.570000e+02 8.410591e+01 1.794132e+02
      vertex   -2.570000e+02 8.704161e+01 1.824006e+02
      vertex   -4.311694e+01 8.776227e+01 1.831604e+02
    endloop
  endfacet
  facet normal 4.905669e-05 6.939861e-01 -7.199884e-01
    outer loop
      vertex   -4.346597e+01 8.462021e+01 1.799235e+02
      vertex   -4.310448e+01 8.199843e+01 1.773964e+02
      vertex   -2.570000e+02 8.410591e+01 1.794132e+02
    endloop
  endfacet
  facet normal -1.101089e-04 6.855359e-01 -7.280388e-01
    outer loop
      vertex   -2.570000e+02 8.410591e+01 1.794132e+02
      vertex   -4.310448e+01 8.199843e+01 1.773964e+02
      vertex   -2.570000e+02 8.067559e+01 1.761831e+02
    endloop
  endfacet
  facet normal -1.255043e-04 6.597104e-01 -7.515199e-01
    outer loop
      vertex   -2.570000e+02 8.067559e+01 1.761831e+02
      vertex   -4.214236e+01 7.921201e+01 1.748625e+02
      vertex   -2.570000e+02 7.713451e+01 1.730747e+02
    endloop
  endfacet
  facet normal 3.552726e-05 6.727982e-01 -7.398260e-01
    outer loop
      vertex   -4.214236e+01 7.921201e+01 1.748625e+02
      vertex   -2.570000e+02 8.067559e+01 1.761831e+02
      vertex   -4.310448e+01 8.199843e+01 1.773964e+02
    endloop
  endfacet
  facet normal 1.494545e-05 6.513963e-01 -7.587377e-01
    outer loop
      vertex   -4.214236e+01 7.921201e+01 1.748625e+02
      vertex   -4.103326e+01 7.724531e+01 1.731740e+02
      vertex   -2.570000e+02 7.713451e+01 1.730747e+02
    endloop
  endfacet
  facet normal -1.071740e-04 6.346216e-01 -7.728230e-01
    outer loop
      vertex   -2.570000e+02 7.428352e+01 1.707718e+02
      vertex   -4.103326e+01 7.724531e+01 1.731740e+02
      vertex   -3.923679e+01 7.490987e+01 1.712560e+02
    endloop
  endfacet
  facet normal 3.559775e-05 6.283522e-01 -7.779290e-01
    outer loop
      vertex   -2.570000e+02 7.428352e+01 1.707718e+02
      vertex   -2.570000e+02 7.713451e+01 1.730747e+02
      vertex   -4.103326e+01 7.724531e+01 1.731740e+02
    endloop
  endfacet
  facet normal -2.925450e-05 6.179093e-01 -7.862494e-01
    outer loop
      vertex   -3.923679e+01 7.490987e+01 1.712560e+02
      vertex   -3.758312e+01 7.329203e+01 1.699845e+02
      vertex   -2.570000e+02 7.428352e+01 1.707718e+02
    endloop
  endfacet
  facet normal -1.883265e-04 6.011804e-01 -7.991133e-01
    outer loop
      vertex   -2.570000e+02 7.428352e+01 1.707718e+02
      vertex   -3.633034e+01 7.229414e+01 1.692232e+02
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal -1.129421e-04 6.064406e-01 -7.951288e-01
    outer loop
      vertex   -2.570000e+02 7.428352e+01 1.707718e+02
      vertex   -3.758312e+01 7.329203e+01 1.699845e+02
      vertex   -3.633034e+01 7.229414e+01 1.692232e+02
    endloop
  endfacet
  facet normal -8.276387e-05 5.954161e-01 -8.034175e-01
    outer loop
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
      vertex   -3.633034e+01 7.229414e+01 1.692232e+02
      vertex   -3.352523e+01 7.052185e+01 1.679095e+02
    endloop
  endfacet
  facet normal -3.832168e-05 5.734144e-01 -8.192655e-01
    outer loop
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
      vertex   -3.032895e+01 6.911089e+01 1.668997e+02
      vertex   -2.876449e+01 6.863835e+01 1.665689e+02
    endloop
  endfacet
  facet normal -1.744120e-05 5.789184e-01 -8.153855e-01
    outer loop
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
      vertex   -3.149862e+01 6.957776e+01 1.672312e+02
      vertex   -3.032895e+01 6.911089e+01 1.668997e+02
    endloop
  endfacet
  facet normal -1.306206e-04 5.598488e-01 -8.285948e-01
    outer loop
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
      vertex   -2.876449e+01 6.863835e+01 1.665689e+02
    endloop
  endfacet
  facet normal -1.423803e-05 5.834303e-01 -8.121632e-01
    outer loop
      vertex   -3.352523e+01 7.052185e+01 1.679095e+02
      vertex   -3.149862e+01 6.957776e+01 1.672312e+02
      vertex   -2.570000e+02 6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal 6.423563e-04 5.694788e-01 -8.220058e-01
    outer loop
      vertex   -1.744070e+01 6.833855e+01 1.663601e+02
      vertex   -1.619626e+01 6.865359e+01 1.665793e+02
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal 2.271697e-04 5.661811e-01 -8.242809e-01
    outer loop
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
      vertex   -2.139619e+01 6.781135e+01 1.659969e+02
      vertex   -1.744070e+01 6.833855e+01 1.663601e+02
    endloop
  endfacet
  facet normal -2.985361e-05 5.616211e-01 -8.273946e-01
    outer loop
      vertex   -2.400293e+01 6.782562e+01 1.660067e+02
      vertex   -2.139619e+01 6.781135e+01 1.659969e+02
      vertex   -3.000000e+00 6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal -2.579631e-04 5.644465e-01 -8.254696e-01
    outer loop
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -2.594354e+01 6.802619e+01 1.661444e+02
      vertex   -2.400293e+01 6.782562e+01 1.660067e+02
    endloop
  endfacet
  facet normal -3.430696e-04 5.687858e-01 -8.224856e-01
    outer loop
      vertex   -2.570000e+02 6.491012e+01 1.640859e+02
      vertex   -2.876449e+01 6.863835e+01 1.665689e+02
      vertex   -2.594354e+01 6.802619e+01 1.661444e+02
    endloop
  endfacet
  facet normal 5.981822e-03 -8.115581e-01 -5.842411e-01
    outer loop
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
      vertex   -1.163321e+01 -9.714678e+01 1.945572e+02
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
    endloop
  endfacet
  facet normal 2.545573e-03 -8.164205e-01 -5.774522e-01
    outer loop
      vertex   -1.458350e+01 -9.802414e+01 1.957847e+02
      vertex   -1.163321e+01 -9.714678e+01 1.945572e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal 8.338196e-04 -8.019524e-01 -5.973873e-01
    outer loop
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
      vertex   -1.163321e+01 -9.714678e+01 1.945572e+02
      vertex   -8.257816e+00 -9.548177e+01 1.923268e+02
    endloop
  endfacet
  facet normal 7.797553e-03 -7.849930e-01 -6.194556e-01
    outer loop
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
      vertex   -5.141311e+00 -9.314566e+01 1.893489e+02
      vertex   -3.000000e+00 -9.259142e+01 1.886735e+02
    endloop
  endfacet
  facet normal 1.078654e-03 -7.873330e-01 -6.165271e-01
    outer loop
      vertex   -8.257816e+00 -9.548177e+01 1.923268e+02
      vertex   -5.141311e+00 -9.314566e+01 1.893489e+02
      vertex   -3.000000e+00 -9.551022e+01 1.923723e+02
    endloop
  endfacet
  facet normal -4.532269e-03 -7.659322e-01 -6.429054e-01
    outer loop
      vertex   -5.141311e+00 -9.314566e+01 1.893489e+02
      vertex   -3.434055e+00 -9.108093e+01 1.868770e+02
      vertex   -3.000000e+00 -9.259142e+01 1.886735e+02
    endloop
  endfacet
  facet normal 1.941870e-05 -7.430356e-01 -6.692519e-01
    outer loop
      vertex   2.510000e+02 -9.054470e+01 1.862494e+02
      vertex   -2.100292e+00 -8.835109e+01 1.838066e+02
      vertex   2.510000e+02 -8.776235e+01 1.831603e+02
    endloop
  endfacet
  facet normal -3.230039e-04 -7.604429e-01 -6.494048e-01
    outer loop
      vertex   2.510000e+02 -9.056495e+01 1.862731e+02
      vertex   -2.100292e+00 -8.835109e+01 1.838066e+02
      vertex   2.510000e+02 -9.054470e+01 1.862494e+02
    endloop
  endfacet
  facet normal -6.509569e-05 -7.266267e-01 -6.870325e-01
    outer loop
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
      vertex   -2.100292e+00 -8.835109e+01 1.838066e+02
      vertex   -1.531095e+00 -8.570666e+01 1.810097e+02
    endloop
  endfacet
  facet normal -1.131046e-05 -7.370525e-01 -6.758355e-01
    outer loop
      vertex   2.510000e+02 -8.776235e+01 1.831603e+02
      vertex   -2.100292e+00 -8.835109e+01 1.838066e+02
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
    endloop
  endfacet
  facet normal 2.705791e-04 -7.077374e-01 -7.064756e-01
    outer loop
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
      vertex   -1.504616e+00 -8.518702e+01 1.804827e+02
      vertex   -1.504450e+00 -8.514960e+01 1.804452e+02
    endloop
  endfacet
  facet normal 2.838481e-04 -7.071062e-01 -7.071073e-01
    outer loop
      vertex   2.510000e+02 -8.185120e+01 1.772482e+02
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
      vertex   -1.504450e+00 -8.514960e+01 1.804452e+02
    endloop
  endfacet
  facet normal 1.807296e-04 -7.120571e-01 -7.021216e-01
    outer loop
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
      vertex   -1.522146e+00 -8.553104e+01 1.808316e+02
      vertex   -1.504616e+00 -8.518702e+01 1.804827e+02
    endloop
  endfacet
  facet normal 1.805065e-04 -7.120694e-01 -7.021091e-01
    outer loop
      vertex   -1.531095e+00 -8.570666e+01 1.810097e+02
      vertex   -1.522146e+00 -8.553104e+01 1.808316e+02
      vertex   2.510000e+02 -8.775182e+01 1.831488e+02
    endloop
  endfacet
  facet normal -3.420178e-05 -6.758355e-01 -7.370525e-01
    outer loop
      vertex   2.510000e+02 -8.185120e+01 1.772482e+02
      vertex   -1.813037e+00 -8.224298e+01 1.776192e+02
      vertex   2.510000e+02 -8.183971e+01 1.772377e+02
    endloop
  endfacet
  facet normal 2.821557e-05 -6.971000e-01 -7.169739e-01
    outer loop
      vertex   -1.504450e+00 -8.514960e+01 1.804452e+02
      vertex   -1.813037e+00 -8.224298e+01 1.776192e+02
      vertex   2.510000e+02 -8.185120e+01 1.772482e+02
    endloop
  endfacet
  facet normal -3.456744e-05 -6.757127e-01 -7.371650e-01
    outer loop
      vertex   2.510000e+02 -8.031304e+01 1.758383e+02
      vertex   2.510000e+02 -8.183971e+01 1.772377e+02
      vertex   -1.813037e+00 -8.224298e+01 1.776192e+02
    endloop
  endfacet
  facet normal -2.158938e-04 -6.627412e-01 -7.488485e-01
    outer loop
      vertex   2.510000e+02 -8.029554e+01 1.758228e+02
      vertex   2.510000e+02 -8.031304e+01 1.758383e+02
      vertex   -1.813037e+00 -8.224298e+01 1.776192e+02
    endloop
  endfacet
  facet normal 4.564740e-06 -6.627907e-01 -7.488047e-01
    outer loop
      vertex   2.510000e+02 -7.875061e+01 1.744553e+02
      vertex   2.510000e+02 -8.029554e+01 1.758228e+02
      vertex   -2.717147e+00 -7.952516e+01 1.751393e+02
    endloop
  endfacet
  facet normal -6.741086e-05 -6.494545e-01 -7.604004e-01
    outer loop
      vertex   2.510000e+02 -7.872691e+01 1.744351e+02
      vertex   2.510000e+02 -7.875061e+01 1.744553e+02
      vertex   -2.717147e+00 -7.952516e+01 1.751393e+02
    endloop
  endfacet
  facet normal -5.679314e-05 -6.740341e-01 -7.387002e-01
    outer loop
      vertex   -1.813037e+00 -8.224298e+01 1.776192e+02
      vertex   -2.717147e+00 -7.952516e+01 1.751393e+02
      vertex   2.510000e+02 -8.029554e+01 1.758228e+02
    endloop
  endfacet
  facet normal -1.982104e-05 -6.580380e-01 -7.529847e-01
    outer loop
      vertex   2.510000e+02 -7.872691e+01 1.744351e+02
      vertex   -2.717147e+00 -7.952516e+01 1.751393e+02
      vertex   -3.312455e+00 -7.831074e+01 1.740781e+02
    endloop
  endfacet
  facet normal 5.511013e-03 -6.406753e-01 -7.677922e-01
    outer loop
      vertex   -3.312455e+00 -7.831074e+01 1.740781e+02
      vertex   -4.811960e+00 -7.605411e+01 1.721843e+02
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
    endloop
  endfacet
  facet normal 6.112563e-03 -6.194630e-01 -7.850020e-01
    outer loop
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
      vertex   -7.015781e+00 -7.366091e+01 1.702713e+02
      vertex   -3.000000e+00 -7.221780e+01 1.691637e+02
    endloop
  endfacet
  facet normal 3.304637e-03 -6.225264e-01 -7.825918e-01
    outer loop
      vertex   -3.000000e+00 -7.591671e+01 1.720826e+02
      vertex   -4.811960e+00 -7.605411e+01 1.721843e+02
      vertex   -7.015781e+00 -7.366091e+01 1.702713e+02
    endloop
  endfacet
  facet normal 1.119728e-03 -5.901367e-01 -8.073025e-01
    outer loop
      vertex   -3.000000e+00 -7.221780e+01 1.691637e+02
      vertex   -9.548022e+00 -7.167409e+01 1.687572e+02
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal -1.058797e-03 -6.069689e-01 -7.947249e-01
    outer loop
      vertex   -7.015781e+00 -7.366091e+01 1.702713e+02
      vertex   -9.548022e+00 -7.167409e+01 1.687572e+02
      vertex   -3.000000e+00 -7.221780e+01 1.691637e+02
    endloop
  endfacet
  facet normal 8.335827e-04 -5.895225e-01 -8.077515e-01
    outer loop
      vertex   -9.548022e+00 -7.167409e+01 1.687572e+02
      vertex   -1.242885e+01 -7.006232e+01 1.675779e+02
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
    endloop
  endfacet
  facet normal 9.621609e-04 -5.711887e-01 -8.208182e-01
    outer loop
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
      vertex   -1.545603e+01 -6.885549e+01 1.667203e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal 1.899356e-04 -5.789423e-01 -8.153685e-01
    outer loop
      vertex   -3.000000e+00 -6.968170e+01 1.673099e+02
      vertex   -1.242885e+01 -7.006232e+01 1.675779e+02
      vertex   -1.545603e+01 -6.885549e+01 1.667203e+02
    endloop
  endfacet
  facet normal -1.824979e-04 -5.723275e-01 -8.200251e-01
    outer loop
      vertex   -2.717887e+01 -6.825595e+01 1.663030e+02
      vertex   -3.044616e+01 -6.916191e+01 1.669360e+02
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
    endloop
  endfacet
  facet normal -1.966881e-04 -5.661817e-01 -8.242804e-01
    outer loop
      vertex   -2.407604e+01 -6.784484e+01 1.660199e+02
      vertex   -2.717887e+01 -6.825595e+01 1.663030e+02
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
    endloop
  endfacet
  facet normal -2.460572e-04 -5.560013e-01 -8.311814e-01
    outer loop
      vertex   -2.143748e+01 -6.779947e+01 1.659887e+02
      vertex   -2.407604e+01 -6.784484e+01 1.660199e+02
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
    endloop
  endfacet
  facet normal 1.331521e-04 -5.645009e-01 -8.254324e-01
    outer loop
      vertex   -1.984529e+01 -6.792469e+01 1.660746e+02
      vertex   -2.143748e+01 -6.779947e+01 1.659887e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal 3.053931e-04 -5.669085e-01 -8.237807e-01
    outer loop
      vertex   -1.755833e+01 -6.831721e+01 1.663456e+02
      vertex   -1.984529e+01 -6.792469e+01 1.660746e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal 6.415401e-04 -5.696464e-01 -8.218896e-01
    outer loop
      vertex   -1.545603e+01 -6.885549e+01 1.667203e+02
      vertex   -1.755833e+01 -6.831721e+01 1.663456e+02
      vertex   -3.000000e+00 -6.710315e+01 1.655155e+02
    endloop
  endfacet
  facet normal -2.115282e-04 -5.803666e-01 -8.143553e-01
    outer loop
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.044616e+01 -6.916191e+01 1.669360e+02
      vertex   -3.239884e+01 -6.997499e+01 1.675160e+02
    endloop
  endfacet
  facet normal -4.866646e-04 -5.989719e-01 -8.007699e-01
    outer loop
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.488973e+01 -7.129164e+01 1.684731e+02
      vertex   -3.626605e+01 -7.225228e+01 1.691925e+02
    endloop
  endfacet
  facet normal -2.781519e-04 -5.876538e-01 -8.091124e-01
    outer loop
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.239884e+01 -6.997499e+01 1.675160e+02
      vertex   -3.488973e+01 -7.129164e+01 1.684731e+02
    endloop
  endfacet
  facet normal -3.045742e-04 -6.078943e-01 -7.940179e-01
    outer loop
      vertex   -2.570000e+02 -7.591671e+01 1.720826e+02
      vertex   -3.626605e+01 -7.225228e+01 1.691925e+02
      vertex   -3.822426e+01 -7.386908e+01 1.704311e+02
    endloop
  endfacet
  facet normal -5.133711e-04 -6.000220e-01 -7.999833e-01
    outer loop
      vertex   -2.570000e+02 -6.858984e+01 1.665872e+02
      vertex   -3.626605e+01 -7.225228e+01 1.691925e+02
      vertex   -2.570000e+02 -7.591671e+01 1.720826e+02
    endloop
  endfacet
  facet normal -1.082081e-04 -6.207814e-01 -7.839837e-01
    outer loop
      vertex   -3.822426e+01 -7.386908e+01 1.704311e+02
      vertex   -3.928437e+01 -7.497062e+01 1.713034e+02
      vertex   -2.570000e+02 -7.591671e+01 1.720826e+02
    endloop
  endfacet
  facet normal -2.006623e-04 -6.540513e-01 -7.564502e-01
    outer loop
      vertex   -2.570000e+02 -7.591671e+01 1.720826e+02
      vertex   -4.101865e+01 -7.724523e+01 1.731740e+02
      vertex   -2.570000e+02 -8.047022e+01 1.760197e+02
    endloop
  endfacet
  facet normal -4.100679e-06 -6.351735e-01 -7.723695e-01
    outer loop
      vertex   -2.570000e+02 -7.591671e+01 1.720826e+02
      vertex   -3.928437e+01 -7.497062e+01 1.713034e+02
      vertex   -4.101865e+01 -7.724523e+01 1.731740e+02
    endloop
  endfacet
  facet normal -7.999896e-05 -6.586229e-01 -7.524732e-01
    outer loop
      vertex   -2.570000e+02 -8.047022e+01 1.760197e+02
      vertex   -4.101865e+01 -7.724523e+01 1.731740e+02
      vertex   -4.270626e+01 -8.063944e+01 1.761451e+02
    endloop
  endfacet
  facet normal -1.144840e-04 -6.847464e-01 -7.287814e-01
    outer loop
      vertex   -4.270626e+01 -8.063944e+01 1.761451e+02
      vertex   -4.340461e+01 -8.370701e+01 1.790274e+02
      vertex   -2.570000e+02 -8.047022e+01 1.760197e+02
    endloop
  endfacet
  facet normal -4.249774e-04 -6.956086e-01 -7.184208e-01
    outer loop
      vertex   -2.570000e+02 -8.667461e+01 1.820271e+02
      vertex   -2.570000e+02 -8.047022e+01 1.760197e+02
      vertex   -4.340461e+01 -8.370701e+01 1.790274e+02
    endloop
  endfacet
  facet normal -9.903129e-05 -7.073681e-01 -7.068454e-01
    outer loop
      vertex   -4.340461e+01 -8.370701e+01 1.790274e+02
      vertex   -4.339208e+01 -8.627595e+01 1.815982e+02
      vertex   -2.570000e+02 -8.667461e+01 1.820271e+02
    endloop
  endfacet
  facet normal -1.244953e-05 -7.293423e-01 -6.841489e-01
    outer loop
      vertex   -2.570000e+02 -8.667461e+01 1.820271e+02
      vertex   -4.339208e+01 -8.627595e+01 1.815982e+02
      vertex   -4.277441e+01 -8.873849e+01 1.842234e+02
    endloop
  endfacet
  facet normal -1.221081e-04 -7.346528e-01 -6.784433e-01
    outer loop
      vertex   -2.570000e+02 -8.987126e+01 1.854886e+02
      vertex   -2.570000e+02 -8.667461e+01 1.820271e+02
      vertex   -4.277441e+01 -8.873849e+01 1.842234e+02
    endloop
  endfacet
  facet normal 5.687415e-05 -7.497829e-01 -6.616839e-01
    outer loop
      vertex   -4.277441e+01 -8.873849e+01 1.842234e+02
      vertex   -4.147919e+01 -9.125493e+01 1.870750e+02
      vertex   -2.570000e+02 -8.987126e+01 1.854886e+02
    endloop
  endfacet
  facet normal -5.969904e-05 -7.696002e-01 -6.385260e-01
    outer loop
      vertex   -4.147919e+01 -9.125493e+01 1.870750e+02
      vertex   -3.965901e+01 -9.336078e+01 1.896130e+02
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
    endloop
  endfacet
  facet normal -1.641698e-04 -7.645647e-01 -6.445469e-01
    outer loop
      vertex   -2.570000e+02 -8.987126e+01 1.854886e+02
      vertex   -4.147919e+01 -9.125493e+01 1.870750e+02
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
    endloop
  endfacet
  facet normal -2.383503e-04 -7.905982e-01 -6.123352e-01
    outer loop
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
      vertex   -3.839837e+01 -9.444588e+01 1.909736e+02
      vertex   -3.742730e+01 -9.511814e+01 1.918412e+02
    endloop
  endfacet
  facet normal -9.805549e-05 -7.818611e-01 -6.234526e-01
    outer loop
      vertex   -3.965901e+01 -9.336078e+01 1.896130e+02
      vertex   -3.839837e+01 -9.444588e+01 1.909736e+02
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
    endloop
  endfacet
  facet normal -2.862374e-04 -7.960880e-01 -6.051808e-01
    outer loop
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
      vertex   -3.417544e+01 -9.680870e+01 1.940948e+02
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
    endloop
  endfacet
  facet normal -4.757012e-04 -8.002692e-01 -5.996408e-01
    outer loop
      vertex   -3.742730e+01 -9.511814e+01 1.918412e+02
      vertex   -3.417544e+01 -9.680870e+01 1.940948e+02
      vertex   -2.570000e+02 -9.307674e+01 1.892910e+02
    endloop
  endfacet
  facet normal -4.153530e-05 -8.156298e-01 -5.785741e-01
    outer loop
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
      vertex   -3.250361e+01 -9.744547e+01 1.949707e+02
      vertex   -2.908362e+01 -9.838358e+01 1.962929e+02
    endloop
  endfacet
  facet normal -9.010488e-05 -8.089253e-01 -5.879115e-01
    outer loop
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
      vertex   -3.417544e+01 -9.680870e+01 1.940948e+02
      vertex   -3.250361e+01 -9.744547e+01 1.949707e+02
    endloop
  endfacet
  facet normal 1.228824e-03 -8.207087e-01 -5.713455e-01
    outer loop
      vertex   -1.779515e+01 -9.867027e+01 1.967059e+02
      vertex   -1.458350e+01 -9.802414e+01 1.957847e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal 6.308433e-04 -8.261716e-01 -5.634183e-01
    outer loop
      vertex   -2.265247e+01 -9.902140e+01 1.972153e+02
      vertex   -1.779515e+01 -9.867027e+01 1.967059e+02
      vertex   -3.000000e+00 -9.918036e+01 1.974704e+02
    endloop
  endfacet
  facet normal -1.360721e-04 -8.270356e-01 -5.621495e-01
    outer loop
      vertex   -2.417742e+01 -9.898851e+01 1.971673e+02
      vertex   -2.265247e+01 -9.902140e+01 1.972153e+02
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
    endloop
  endfacet
  facet normal -1.062206e-04 -8.247778e-01 -5.654569e-01
    outer loop
      vertex   -2.521370e+01 -9.891535e+01 1.970608e+02
      vertex   -2.417742e+01 -9.898851e+01 1.971673e+02
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
    endloop
  endfacet
  facet normal -8.233982e-05 -8.228245e-01 -5.682955e-01
    outer loop
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
      vertex   -2.838898e+01 -9.851420e+01 1.964804e+02
      vertex   -2.521370e+01 -9.891535e+01 1.970608e+02
    endloop
  endfacet
  facet normal -6.729058e-05 -8.206485e-01 -5.714333e-01
    outer loop
      vertex   -2.908362e+01 -9.838358e+01 1.962929e+02
      vertex   -2.838898e+01 -9.851420e+01 1.964804e+02
      vertex   -2.570000e+02 -9.798671e+01 1.957498e+02
    endloop
  endfacet
  facet normal -1.273144e-04 9.657167e-01 2.595981e-01
    outer loop
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
      vertex   -2.570000e+02 1.161556e+02 2.950399e+02
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
    endloop
  endfacet
  facet normal -1.019845e-03 9.786396e-01 2.055808e-01
    outer loop
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
      vertex   -2.570000e+02 1.161556e+02 2.950399e+02
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal -4.999479e-03 9.778972e-01 2.090266e-01
    outer loop
      vertex   -2.244379e+02 1.170346e+02 2.915124e+02
      vertex   -2.230574e+02 1.173561e+02 2.900411e+02
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
    endloop
  endfacet
  facet normal -1.206138e-02 9.713199e-01 2.374705e-01
    outer loop
      vertex   -2.244379e+02 1.162955e+02 2.945353e+02
      vertex   -2.244379e+02 1.170346e+02 2.915124e+02
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
    endloop
  endfacet
  facet normal -2.557696e-03 9.805410e-01 1.962977e-01
    outer loop
      vertex   -2.230574e+02 1.173561e+02 2.900411e+02
      vertex   -2.209847e+02 1.178682e+02 2.875105e+02
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
    endloop
  endfacet
  facet normal 2.663657e-04 9.854526e-01 1.699502e-01
    outer loop
      vertex   -2.209847e+02 1.178682e+02 2.875105e+02
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
    endloop
  endfacet
  facet normal -2.365210e-05 9.902073e-01 1.396047e-01
    outer loop
      vertex   -2.570000e+02 1.191055e+02 2.796243e+02
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
      vertex   -2.159969e+02 1.190939e+02 2.797139e+02
    endloop
  endfacet
  facet normal 1.984594e-05 9.902679e-01 1.391744e-01
    outer loop
      vertex   -2.570000e+02 1.185226e+02 2.837721e+02
      vertex   -2.182249e+02 1.185645e+02 2.834681e+02
      vertex   -2.570000e+02 1.191055e+02 2.796243e+02
    endloop
  endfacet
  facet normal 4.524433e-05 9.940874e-01 1.085831e-01
    outer loop
      vertex   -2.159969e+02 1.190939e+02 2.797139e+02
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
      vertex   -2.570000e+02 1.191055e+02 2.796243e+02
    endloop
  endfacet
  facet normal -1.417256e-03 9.963152e-01 8.575521e-02
    outer loop
      vertex   -2.570000e+02 1.191055e+02 2.796243e+02
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
      vertex   -2.570000e+02 1.198013e+02 2.715408e+02
    endloop
  endfacet
  facet normal -2.805016e-04 9.970416e-01 7.686359e-02
    outer loop
      vertex   -2.145584e+02 1.193975e+02 2.769337e+02
      vertex   -2.121507e+02 1.198201e+02 2.714612e+02
      vertex   -2.570000e+02 1.198013e+02 2.715408e+02
    endloop
  endfacet
  facet normal -1.165804e-03 9.996874e-01 2.497309e-02
    outer loop
      vertex   -2.570000e+02 1.198013e+02 2.715408e+02
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
      vertex   -2.570000e+02 1.199909e+02 2.639529e+02
    endloop
  endfacet
  facet normal -3.607152e-04 9.994811e-01 3.220753e-02
    outer loop
      vertex   -2.570000e+02 1.198013e+02 2.715408e+02
      vertex   -2.121507e+02 1.198201e+02 2.714612e+02
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
    endloop
  endfacet
  facet normal -3.386720e-04 9.995483e-01 -3.005284e-02
    outer loop
      vertex   -2.570000e+02 1.199909e+02 2.639529e+02
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
      vertex   -2.570000e+02 1.198493e+02 2.592432e+02
    endloop
  endfacet
  facet normal 7.360665e-04 9.999300e-01 -1.180886e-02
    outer loop
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
      vertex   -2.570000e+02 1.199909e+02 2.639529e+02
      vertex   -2.104139e+02 1.199850e+02 2.663616e+02
    endloop
  endfacet
  facet normal -9.834857e-04 9.972496e-01 -7.411009e-02
    outer loop
      vertex   -2.570000e+02 1.198493e+02 2.592432e+02
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
      vertex   -2.570000e+02 1.193837e+02 2.529787e+02
    endloop
  endfacet
  facet normal 5.492332e-04 9.986180e-01 -5.255322e-02
    outer loop
      vertex   -2.090786e+02 1.199223e+02 2.611313e+02
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
      vertex   -2.570000e+02 1.198493e+02 2.592432e+02
    endloop
  endfacet
  facet normal -5.242773e-05 9.959196e-01 -9.024486e-02
    outer loop
      vertex   -2.080975e+02 1.196401e+02 2.557795e+02
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
      vertex   -2.570000e+02 1.193837e+02 2.529787e+02
    endloop
  endfacet
  facet normal -2.191938e-03 9.999651e-01 -8.059533e-03
    outer loop
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
    endloop
  endfacet
  facet normal -5.847194e-04 9.998175e-01 -1.909382e-02
    outer loop
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
      vertex   -1.681614e+02 1.199184e+02 2.606984e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.991594e-01 -4.099386e-02
    outer loop
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
      vertex   -1.661501e+02 1.197988e+02 2.580540e+02
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
    endloop
  endfacet
  facet normal -3.050597e-04 9.989677e-01 -4.542512e-02
    outer loop
      vertex   -1.681614e+02 1.199184e+02 2.606984e+02
      vertex   -1.661501e+02 1.197988e+02 2.580540e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
    endloop
  endfacet
  facet normal 1.199399e-03 9.966429e-01 -8.186236e-02
    outer loop
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
    endloop
  endfacet
  facet normal -1.521991e-03 9.987188e-01 -5.058163e-02
    outer loop
      vertex   -1.851814e+02 1.197988e+02 2.580540e+02
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
      vertex   -1.997579e+02 1.198936e+02 2.603636e+02
    endloop
  endfacet
  facet normal -4.626862e-04 9.960572e-01 -8.871165e-02
    outer loop
      vertex   -1.985206e+02 1.192734e+02 2.518699e+02
      vertex   -1.990857e+02 1.197166e+02 2.568498e+02
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
    endloop
  endfacet
  facet normal -3.845682e-04 9.763780e-01 2.160692e-01
    outer loop
      vertex   -1.686295e+02 1.170623e+02 2.913898e+02
      vertex   -1.669383e+02 1.171277e+02 2.910972e+02
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 1.616126e-05 9.761694e-01 2.170101e-01
    outer loop
      vertex   -1.696300e+02 1.170633e+02 2.913853e+02
      vertex   -1.686295e+02 1.170623e+02 2.913898e+02
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 1.784538e-03 9.763140e-01 2.163511e-01
    outer loop
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
      vertex   -1.669383e+02 1.171277e+02 2.910972e+02
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
    endloop
  endfacet
  facet normal 1.433065e-04 9.760642e-01 2.174824e-01
    outer loop
      vertex   -1.709677e+02 1.171071e+02 2.911895e+02
      vertex   -1.696300e+02 1.170633e+02 2.913853e+02
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal -2.357510e-04 9.767057e-01 2.145829e-01
    outer loop
      vertex   -1.732960e+02 1.173322e+02 2.901626e+02
      vertex   -1.709677e+02 1.171071e+02 2.911895e+02
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 4.996425e-04 9.785780e-01 2.058759e-01
    outer loop
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
      vertex   -1.736923e+02 1.174104e+02 2.897917e+02
      vertex   -1.732960e+02 1.173322e+02 2.901626e+02
    endloop
  endfacet
  facet normal -1.110551e-04 9.776255e-01 2.103529e-01
    outer loop
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
      vertex   -1.736923e+02 1.174104e+02 2.897917e+02
      vertex   -1.668814e+02 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 2.910279e-04 9.821730e-01 1.879788e-01
    outer loop
      vertex   -1.635117e+02 1.176793e+02 2.884797e+02
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
    endloop
  endfacet
  facet normal -6.176431e-04 9.795140e-01 2.013750e-01
    outer loop
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
      vertex   -1.635117e+02 1.176793e+02 2.884797e+02
    endloop
  endfacet
  facet normal 3.208031e-04 9.803088e-01 1.974703e-01
    outer loop
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
      vertex   -1.736923e+02 1.174104e+02 2.897917e+02
      vertex   -1.648753e+02 1.173505e+02 2.900748e+02
    endloop
  endfacet
  facet normal -7.384885e-04 9.841973e-01 1.770737e-01
    outer loop
      vertex   -1.625170e+02 1.182053e+02 2.856751e+02
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
      vertex   -1.627440e+02 1.180381e+02 2.866037e+02
    endloop
  endfacet
  facet normal 5.574411e-04 9.827887e-01 1.847324e-01
    outer loop
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
      vertex   -1.750505e+02 1.178269e+02 2.877263e+02
      vertex   -1.625170e+02 1.182053e+02 2.856751e+02
    endloop
  endfacet
  facet normal -4.747996e-04 9.855282e-01 1.695108e-01
    outer loop
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
      vertex   -1.625170e+02 1.182053e+02 2.856751e+02
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
    endloop
  endfacet
  facet normal 3.934656e-05 9.847863e-01 1.737698e-01
    outer loop
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
      vertex   -1.755582e+02 1.183090e+02 2.850740e+02
      vertex   -1.754026e+02 1.180535e+02 2.865216e+02
    endloop
  endfacet
  facet normal -2.435439e-05 9.866453e-01 1.628838e-01
    outer loop
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
      vertex   -1.755582e+02 1.183090e+02 2.850740e+02
    endloop
  endfacet
  facet normal 8.365909e-05 9.869713e-01 1.608965e-01
    outer loop
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
      vertex   -1.624046e+02 1.183216e+02 2.849992e+02
    endloop
  endfacet
  facet normal -3.546733e-03 9.628776e-01 2.699151e-01
    outer loop
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
      vertex   -1.801562e+02 1.159113e+02 2.960571e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal -7.707724e-03 9.573843e-01 2.887143e-01
    outer loop
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
      vertex   -1.782423e+02 1.155716e+02 2.972940e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal -2.035611e-04 9.657167e-01 2.595981e-01
    outer loop
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal -9.797966e-04 9.705766e-01 2.407908e-01
    outer loop
      vertex   -1.815025e+02 1.162394e+02 2.948031e+02
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal -1.358702e-03 9.670888e-01 2.544356e-01
    outer loop
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
      vertex   -1.801562e+02 1.159113e+02 2.960571e+02
      vertex   -1.815025e+02 1.162394e+02 2.948031e+02
    endloop
  endfacet
  facet normal -3.421891e-03 9.736806e-01 2.278912e-01
    outer loop
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
      vertex   -1.840521e+02 1.171387e+02 2.910473e+02
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
    endloop
  endfacet
  facet normal -2.572015e-03 9.732708e-01 2.296462e-01
    outer loop
      vertex   -1.826833e+02 1.166043e+02 2.933272e+02
      vertex   -1.840521e+02 1.171387e+02 2.910473e+02
      vertex   -1.916617e+02 1.162955e+02 2.945353e+02
    endloop
  endfacet
  facet normal 1.428871e-03 9.789549e-01 2.040717e-01
    outer loop
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -1.840521e+02 1.171387e+02 2.910473e+02
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
    endloop
  endfacet
  facet normal -5.278606e-04 9.808859e-01 1.945831e-01
    outer loop
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.816513e-01 1.906849e-01
    outer loop
      vertex   -1.916617e+02 1.174581e+02 2.895683e+02
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
      vertex   -2.134557e+02 1.174581e+02 2.895683e+02
    endloop
  endfacet
  facet normal -5.760653e-05 9.827511e-01 1.849330e-01
    outer loop
      vertex   -1.849033e+02 1.177273e+02 2.882294e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
    endloop
  endfacet
  facet normal -6.645502e-04 9.855734e-01 1.692472e-01
    outer loop
      vertex   -2.115251e+02 1.179613e+02 2.869774e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
    endloop
  endfacet
  facet normal 2.452563e-04 9.869486e-01 1.610352e-01
    outer loop
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
      vertex   -1.851633e+02 1.181485e+02 2.859910e+02
      vertex   -1.849592e+02 1.186873e+02 2.826886e+02
    endloop
  endfacet
  facet normal -4.569625e-04 9.904266e-01 1.380398e-01
    outer loop
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
      vertex   -1.849592e+02 1.186873e+02 2.826886e+02
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
    endloop
  endfacet
  facet normal -1.404553e-03 9.914604e-01 1.304004e-01
    outer loop
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
      vertex   -2.091121e+02 1.185757e+02 2.834093e+02
    endloop
  endfacet
  facet normal -1.570547e-04 9.931417e-01 1.169171e-01
    outer loop
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
      vertex   -1.844456e+02 1.190054e+02 2.804078e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
    endloop
  endfacet
  facet normal -1.409938e-03 9.958804e-01 9.066543e-02
    outer loop
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
      vertex   -1.810216e+02 1.196693e+02 2.738388e+02
    endloop
  endfacet
  facet normal -5.965700e-04 9.961484e-01 8.768108e-02
    outer loop
      vertex   -1.835283e+02 1.192798e+02 2.780786e+02
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
      vertex   -2.062828e+02 1.192364e+02 2.784162e+02
    endloop
  endfacet
  facet normal -1.172263e-03 9.990924e-01 4.258030e-02
    outer loop
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
    endloop
  endfacet
  facet normal 1.592319e-04 9.981352e-01 6.104183e-02
    outer loop
      vertex   -1.810216e+02 1.196693e+02 2.738388e+02
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
      vertex   -2.036061e+02 1.197457e+02 2.726484e+02
    endloop
  endfacet
  facet normal 1.367367e-03 9.997464e-01 2.247833e-02
    outer loop
      vertex   -1.785521e+02 1.198519e+02 2.708469e+02
      vertex   -1.719038e+02 1.199873e+02 2.644195e+02
      vertex   -2.018383e+02 1.199498e+02 2.679082e+02
    endloop
  endfacet
  facet normal 1.297407e-04 9.654540e-01 2.605734e-01
    outer loop
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
    endloop
  endfacet
  facet normal -2.197711e-04 9.639043e-01 2.662488e-01
    outer loop
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal 9.043473e-05 9.671018e-01 2.543897e-01
    outer loop
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
      vertex   -1.582443e+02 1.159188e+02 2.960271e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
    endloop
  endfacet
  facet normal -2.700364e-04 9.702072e-01 2.422767e-01
    outer loop
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
    endloop
  endfacet
  facet normal 2.742466e-04 9.678210e-01 2.516393e-01
    outer loop
      vertex   -1.570167e+02 1.161933e+02 2.949829e+02
      vertex   -1.418745e+02 1.159429e+02 2.959295e+02
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
    endloop
  endfacet
  facet normal 1.038693e-05 9.730614e-01 2.305463e-01
    outer loop
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
      vertex   -1.448091e+02 1.169003e+02 2.920947e+02
    endloop
  endfacet
  facet normal 4.347297e-04 9.721550e-01 2.343386e-01
    outer loop
      vertex   -1.434835e+02 1.163815e+02 2.942442e+02
      vertex   -1.448091e+02 1.169003e+02 2.920947e+02
      vertex   -1.555598e+02 1.166307e+02 2.932332e+02
    endloop
  endfacet
  facet normal 1.191188e-04 9.767484e-01 2.143887e-01
    outer loop
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
    endloop
  endfacet
  facet normal 9.784511e-05 9.767633e-01 2.143210e-01
    outer loop
      vertex   -1.548318e+02 1.169125e+02 2.920434e+02
      vertex   -1.448091e+02 1.169003e+02 2.920947e+02
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
    endloop
  endfacet
  facet normal -6.166001e-04 9.808674e-01 1.946759e-01
    outer loop
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
      vertex   -1.531785e+02 1.178889e+02 2.873992e+02
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
    endloop
  endfacet
  facet normal -1.001130e-04 9.802732e-01 1.976471e-01
    outer loop
      vertex   -1.458720e+02 1.175007e+02 2.893587e+02
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
      vertex   -1.537350e+02 1.174800e+02 2.894573e+02
    endloop
  endfacet
  facet normal -7.202137e-05 9.761891e-01 2.169212e-01
    outer loop
      vertex   -1.296592e+02 1.172125e+02 2.907144e+02
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
      vertex   -1.304878e+02 1.170770e+02 2.913237e+02
    endloop
  endfacet
  facet normal -3.408784e-04 9.762664e-01 2.165728e-01
    outer loop
      vertex   -1.296592e+02 1.172125e+02 2.907144e+02
      vertex   -1.304878e+02 1.170770e+02 2.913237e+02
      vertex   -1.289850e+02 1.171816e+02 2.908545e+02
    endloop
  endfacet
  facet normal 8.191655e-05 9.766948e-01 2.146329e-01
    outer loop
      vertex   -1.296592e+02 1.172125e+02 2.907144e+02
      vertex   -1.289850e+02 1.171816e+02 2.908545e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
    endloop
  endfacet
  facet normal -1.029632e-04 9.774061e-01 2.113698e-01
    outer loop
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
      vertex   -1.296592e+02 1.172125e+02 2.907144e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
    endloop
  endfacet
  facet normal -1.713456e-04 9.781104e-01 2.080864e-01
    outer loop
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
    endloop
  endfacet
  facet normal -2.775540e-05 9.761533e-01 2.170822e-01
    outer loop
      vertex   -1.344566e+02 1.171990e+02 2.907746e+02
      vertex   -1.322642e+02 1.170606e+02 2.913969e+02
      vertex   -1.296592e+02 1.172125e+02 2.907144e+02
    endloop
  endfacet
  facet normal -1.600378e-04 9.781227e-01 2.080287e-01
    outer loop
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.286305e+02 1.172246e+02 2.906588e+02
      vertex   -1.270642e+02 1.175000e+02 2.893652e+02
    endloop
  endfacet
  facet normal -9.093969e-05 9.794708e-01 2.015859e-01
    outer loop
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.270642e+02 1.175000e+02 2.893652e+02
      vertex   -1.266291e+02 1.176020e+02 2.888696e+02
    endloop
  endfacet
  facet normal -4.126183e-04 9.809239e-01 1.943919e-01
    outer loop
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.266291e+02 1.176020e+02 2.888696e+02
    endloop
  endfacet
  facet normal -1.120146e-04 9.804882e-01 1.965781e-01
    outer loop
      vertex   -1.367704e+02 1.178245e+02 2.877448e+02
      vertex   -1.360262e+02 1.175182e+02 2.892729e+02
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
    endloop
  endfacet
  facet normal 1.811841e-04 9.833143e-01 1.819147e-01
    outer loop
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.367704e+02 1.178245e+02 2.877448e+02
    endloop
  endfacet
  facet normal -2.660226e-03 9.818465e-01 1.896587e-01
    outer loop
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
      vertex   -1.464366e+02 1.179832e+02 2.869081e+02
    endloop
  endfacet
  facet normal -1.108886e-04 9.841510e-01 1.773324e-01
    outer loop
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.464366e+02 1.179832e+02 2.869081e+02
    endloop
  endfacet
  facet normal 2.748714e-04 9.879586e-01 1.547179e-01
    outer loop
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
    endloop
  endfacet
  facet normal 1.507298e-04 9.838133e-01 1.791962e-01
    outer loop
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
      vertex   -1.257413e+02 1.178939e+02 2.873837e+02
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
    endloop
  endfacet
  facet normal -1.178521e-03 9.819790e-01 1.889864e-01
    outer loop
      vertex   -1.260324e+02 1.177842e+02 2.879517e+02
      vertex   -1.257413e+02 1.178939e+02 2.873837e+02
      vertex   -1.371596e+02 1.181686e+02 2.858849e+02
    endloop
  endfacet
  facet normal 1.076869e-03 9.836156e-01 1.802754e-01
    outer loop
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.531785e+02 1.178889e+02 2.873992e+02
      vertex   -1.461728e+02 1.177356e+02 2.881938e+02
    endloop
  endfacet
  facet normal 1.178549e-04 9.857957e-01 1.679487e-01
    outer loop
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
      vertex   -1.528495e+02 1.182137e+02 2.856248e+02
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
    endloop
  endfacet
  facet normal 5.200529e-04 9.870831e-01 1.602080e-01
    outer loop
      vertex   -1.252196e+02 1.181561e+02 2.859433e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
    endloop
  endfacet
  facet normal 1.083739e-04 9.890843e-01 1.473509e-01
    outer loop
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 2.796092e-04 9.877207e-01 1.562297e-01
    outer loop
      vertex   -1.605388e+02 1.185220e+02 2.837720e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 2.800402e-04 9.877180e-01 1.562469e-01
    outer loop
      vertex   -1.527121e+02 1.183960e+02 2.845547e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.605388e+02 1.185220e+02 2.837720e+02
    endloop
  endfacet
  facet normal 8.389897e-05 9.888514e-01 1.489058e-01
    outer loop
      vertex   -1.330877e+02 1.188455e+02 2.816055e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
    endloop
  endfacet
  facet normal -4.135318e-04 9.884172e-01 1.517605e-01
    outer loop
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
      vertex   -1.755372e+02 1.184361e+02 2.843042e+02
      vertex   -1.622619e+02 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 5.001157e-03 9.935463e-01 1.133171e-01
    outer loop
      vertex   -1.238922e+02 1.193584e+02 2.773924e+02
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
    endloop
  endfacet
  facet normal -1.293834e-03 9.912193e-01 1.322218e-01
    outer loop
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.240018e+02 1.192028e+02 2.787616e+02
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
    endloop
  endfacet
  facet normal 8.558238e-05 9.908482e-01 1.349812e-01
    outer loop
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
    endloop
  endfacet
  facet normal -5.743780e-05 9.905243e-01 1.373373e-01
    outer loop
      vertex   -1.330877e+02 1.188455e+02 2.816055e+02
      vertex   -1.355207e+02 1.188824e+02 2.813388e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
    endloop
  endfacet
  facet normal -1.018119e-04 9.908323e-01 1.350978e-01
    outer loop
      vertex   -1.355207e+02 1.188824e+02 2.813388e+02
      vertex   -1.370341e+02 1.189316e+02 2.809763e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
    endloop
  endfacet
  facet normal -2.829833e-04 9.916298e-01 1.291130e-01
    outer loop
      vertex   -1.370341e+02 1.189316e+02 2.809763e+02
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
      vertex   -1.751437e+02 1.187733e+02 2.821089e+02
    endloop
  endfacet
  facet normal 8.125217e-03 9.912988e-01 1.313796e-01
    outer loop
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.249221e+02 1.192418e+02 2.784599e+02
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
    endloop
  endfacet
  facet normal 2.805267e-03 9.904154e-01 1.380926e-01
    outer loop
      vertex   -1.276847e+02 1.190038e+02 2.804269e+02
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
    endloop
  endfacet
  facet normal 1.402531e-03 9.898387e-01 1.421878e-01
    outer loop
      vertex   -1.295873e+02 1.189031e+02 2.811877e+02
      vertex   -1.307346e+02 1.188688e+02 2.814380e+02
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
    endloop
  endfacet
  facet normal 5.907643e-04 9.893443e-01 1.455934e-01
    outer loop
      vertex   -1.245212e+02 1.186517e+02 2.828875e+02
      vertex   -1.307346e+02 1.188688e+02 2.814380e+02
      vertex   -1.330877e+02 1.188455e+02 2.816055e+02
    endloop
  endfacet
  facet normal 1.432068e-03 9.922896e-01 1.239327e-01
    outer loop
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
      vertex   -1.736218e+02 1.191845e+02 2.789627e+02
    endloop
  endfacet
  facet normal -6.229216e-05 9.944763e-01 1.049616e-01
    outer loop
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
      vertex   -1.736218e+02 1.191845e+02 2.789627e+02
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
    endloop
  endfacet
  facet normal -1.241940e-04 9.927668e-01 1.200586e-01
    outer loop
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
      vertex   -1.394744e+02 1.190700e+02 2.799081e+02
      vertex   -1.410653e+02 1.191958e+02 2.788660e+02
    endloop
  endfacet
  facet normal -7.724491e-04 9.943579e-01 1.060739e-01
    outer loop
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
      vertex   -1.744387e+02 1.190072e+02 2.803916e+02
      vertex   -1.410653e+02 1.191958e+02 2.788660e+02
    endloop
  endfacet
  facet normal -1.282483e-03 9.974390e-01 7.151033e-02
    outer loop
      vertex   -1.448983e+02 1.196092e+02 2.746668e+02
      vertex   -1.460302e+02 1.197676e+02 2.724373e+02
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
    endloop
  endfacet
  facet normal -1.707598e-03 9.976240e-01 6.887258e-02
    outer loop
      vertex   -1.460302e+02 1.197676e+02 2.724373e+02
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
    endloop
  endfacet
  facet normal -1.634591e-05 9.959869e-01 8.949948e-02
    outer loop
      vertex   -1.434307e+02 1.194343e+02 2.766133e+02
      vertex   -1.448983e+02 1.196092e+02 2.746668e+02
      vertex   -1.715692e+02 1.194413e+02 2.765304e+02
    endloop
  endfacet
  facet normal -1.117173e-03 9.983518e-01 5.737899e-02
    outer loop
      vertex   -1.460302e+02 1.197676e+02 2.724373e+02
      vertex   -1.464924e+02 1.198395e+02 2.711772e+02
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
    endloop
  endfacet
  facet normal -1.330614e-03 9.992734e-01 3.809069e-02
    outer loop
      vertex   -1.464924e+02 1.198395e+02 2.711772e+02
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
    endloop
  endfacet
  facet normal -2.115792e-03 9.994324e-01 3.362054e-02
    outer loop
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
      vertex   -1.663592e+02 1.198047e+02 2.713953e+02
    endloop
  endfacet
  facet normal 1.666819e-04 9.998863e-01 1.508001e-02
    outer loop
      vertex   -1.471671e+02 1.199586e+02 2.680297e+02
      vertex   -1.473336e+02 1.199941e+02 2.656736e+02
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
    endloop
  endfacet
  facet normal -3.856116e-04 9.999858e-01 -5.308074e-03
    outer loop
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.574234e+02 1.199666e+02 2.622346e+02
    endloop
  endfacet
  facet normal -8.553783e-04 9.999748e-01 -7.041681e-03
    outer loop
      vertex   -1.473336e+02 1.199941e+02 2.656736e+02
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.611331e+02 1.199868e+02 2.663111e+02
    endloop
  endfacet
  facet normal 4.185865e-04 9.994603e-01 -3.284589e-02
    outer loop
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
      vertex   -1.574234e+02 1.199666e+02 2.622346e+02
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
    endloop
  endfacet
  facet normal 1.000794e-03 9.993982e-01 -3.467370e-02
    outer loop
      vertex   -1.471912e+02 1.199721e+02 2.625333e+02
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
    endloop
  endfacet
  facet normal -5.093056e-04 9.985515e-01 -5.380200e-02
    outer loop
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
      vertex   -1.556136e+02 1.198876e+02 2.598534e+02
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
    endloop
  endfacet
  facet normal 1.571505e-04 9.983945e-01 -5.664282e-02
    outer loop
      vertex   -1.464769e+02 1.198535e+02 2.591336e+02
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
    endloop
  endfacet
  facet normal -4.834543e-04 9.968463e-01 -7.935524e-02
    outer loop
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
      vertex   -1.541070e+02 1.197534e+02 2.573491e+02
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
    endloop
  endfacet
  facet normal -1.549471e-03 9.970226e-01 -7.709388e-02
    outer loop
      vertex   -1.457077e+02 1.197387e+02 2.571132e+02
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
    endloop
  endfacet
  facet normal -3.513941e-03 9.953672e-01 -9.608182e-02
    outer loop
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.428166e+02 1.193885e+02 2.529041e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal 8.063444e-04 9.944652e-01 -1.050636e-01
    outer loop
      vertex   -1.526429e+02 1.194767e+02 2.538644e+02
      vertex   -1.442089e+02 1.195443e+02 2.545686e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal -4.646842e-04 9.939504e-01 -1.098290e-01
    outer loop
      vertex   -1.428166e+02 1.193885e+02 2.529041e+02
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal -4.740574e-05 9.648449e-01 2.628199e-01
    outer loop
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
    endloop
  endfacet
  facet normal -4.019867e-05 9.648690e-01 2.627313e-01
    outer loop
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
    endloop
  endfacet
  facet normal 5.036644e-05 9.619030e-01 2.733911e-01
    outer loop
      vertex   -1.242488e+02 1.155105e+02 2.975155e+02
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
    endloop
  endfacet
  facet normal -1.122959e-03 9.694387e-01 2.453313e-01
    outer loop
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
    endloop
  endfacet
  facet normal -3.363851e-06 9.679259e-01 2.512358e-01
    outer loop
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
      vertex   -1.201478e+02 1.163197e+02 2.944910e+02
      vertex   -1.214152e+02 1.160241e+02 2.956297e+02
    endloop
  endfacet
  facet normal 1.044160e-03 9.717775e-01 2.358972e-01
    outer loop
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
      vertex   -1.201478e+02 1.163197e+02 2.944910e+02
      vertex   -1.012708e+02 1.160056e+02 2.957013e+02
    endloop
  endfacet
  facet normal 1.036600e-03 9.754509e-01 2.202147e-01
    outer loop
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
      vertex   -1.172102e+02 1.172536e+02 2.905241e+02
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
    endloop
  endfacet
  facet normal 8.022132e-04 9.757114e-01 2.190586e-01
    outer loop
      vertex   -1.034522e+02 1.166691e+02 2.930693e+02
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
      vertex   -1.181394e+02 1.169099e+02 2.920506e+02
    endloop
  endfacet
  facet normal 8.331155e-04 9.790322e-01 2.037039e-01
    outer loop
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
      vertex   -1.172102e+02 1.172536e+02 2.905241e+02
      vertex   -1.058501e+02 1.178188e+02 2.877612e+02
    endloop
  endfacet
  facet normal -9.481651e-04 9.804685e-01 1.966740e-01
    outer loop
      vertex   -1.058501e+02 1.178188e+02 2.877612e+02
      vertex   -1.172102e+02 1.172536e+02 2.905241e+02
      vertex   -1.052374e+02 1.174588e+02 2.895584e+02
    endloop
  endfacet
  facet normal 7.647538e-04 9.835255e-01 1.807679e-01
    outer loop
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.150962e+02 1.183726e+02 2.846805e+02
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
    endloop
  endfacet
  facet normal -1.508385e-03 9.844794e-01 1.754937e-01
    outer loop
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.163313e+02 1.176533e+02 2.885992e+02
      vertex   -1.058501e+02 1.178188e+02 2.877612e+02
    endloop
  endfacet
  facet normal 3.012553e-04 9.885275e-01 1.510408e-01
    outer loop
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
      vertex   -1.150962e+02 1.183726e+02 2.846805e+02
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
    endloop
  endfacet
  facet normal 3.116593e-03 9.896125e-01 1.437269e-01
    outer loop
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
      vertex   -1.066307e+02 1.183897e+02 2.845514e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
    endloop
  endfacet
  facet normal -2.028111e-04 9.922935e-01 1.239093e-01
    outer loop
      vertex   -1.145115e+02 1.188453e+02 2.815855e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
      vertex   -1.141438e+02 1.192663e+02 2.782145e+02
    endloop
  endfacet
  facet normal 1.023898e-02 9.957919e-01 9.106913e-02
    outer loop
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
      vertex   -1.141438e+02 1.192663e+02 2.782145e+02
    endloop
  endfacet
  facet normal 4.061584e-03 9.952378e-01 9.739199e-02
    outer loop
      vertex   -1.075421e+02 1.197464e+02 2.727375e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
      vertex   -1.072253e+02 1.189966e+02 2.803855e+02
    endloop
  endfacet
  facet normal -1.977135e-03 9.988455e-01 4.799646e-02
    outer loop
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
    endloop
  endfacet
  facet normal -5.089897e-03 9.990072e-01 4.425640e-02
    outer loop
      vertex   -1.075421e+02 1.197464e+02 2.727375e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
      vertex   -1.139837e+02 1.196646e+02 2.738411e+02
    endloop
  endfacet
  facet normal 8.026751e-04 9.999650e-01 8.329762e-03
    outer loop
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.142460e+02 1.199532e+02 2.678263e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
    endloop
  endfacet
  facet normal 1.725715e-03 9.999752e-01 6.827966e-03
    outer loop
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.073247e+02 1.199436e+02 2.683111e+02
    endloop
  endfacet
  facet normal 1.314226e-04 9.996990e-01 -2.453481e-02
    outer loop
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
      vertex   -1.148127e+02 1.199879e+02 2.637090e+02
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
    endloop
  endfacet
  facet normal 1.632124e-03 9.995830e-01 -2.883049e-02
    outer loop
      vertex   -1.066507e+02 1.199767e+02 2.632942e+02
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
    endloop
  endfacet
  facet normal -2.037989e-04 9.988195e-01 -4.857433e-02
    outer loop
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.157163e+02 1.199002e+02 2.601292e+02
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
    endloop
  endfacet
  facet normal 7.477137e-04 9.982440e-01 -5.923085e-02
    outer loop
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.056219e+02 1.198567e+02 2.591922e+02
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
    endloop
  endfacet
  facet normal -1.113843e-03 9.974359e-01 -7.155636e-02
    outer loop
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
      vertex   -1.164344e+02 1.198076e+02 2.582294e+02
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
    endloop
  endfacet
  facet normal 1.827268e-04 9.967265e-01 -8.084709e-02
    outer loop
      vertex   -1.045058e+02 1.196922e+02 2.564354e+02
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
    endloop
  endfacet
  facet normal -4.775171e-04 9.954989e-01 -9.477156e-02
    outer loop
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
      vertex   -1.023758e+02 1.194081e+02 2.531035e+02
    endloop
  endfacet
  facet normal -2.183983e-04 9.952800e-01 -9.704472e-02
    outer loop
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
      vertex   -1.186354e+02 1.195359e+02 2.544755e+02
      vertex   -1.031313e+02 1.195021e+02 2.540945e+02
    endloop
  endfacet
  facet normal 1.772250e-04 9.943022e-01 -1.065981e-01
    outer loop
      vertex   -1.023758e+02 1.194081e+02 2.531035e+02
      vertex   -1.004897e+02 1.192095e+02 2.512542e+02
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
    endloop
  endfacet
  facet normal -3.893675e-04 9.927408e-01 -1.202726e-01
    outer loop
      vertex   -1.004897e+02 1.192095e+02 2.512542e+02
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
    endloop
  endfacet
  facet normal 6.766460e-05 9.933080e-01 -1.154959e-01
    outer loop
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
      vertex   -1.208941e+02 1.193027e+02 2.520890e+02
    endloop
  endfacet
  facet normal -5.709427e-05 9.983268e-01 -5.782413e-02
    outer loop
      vertex   -6.434243e+01 1.197716e+02 2.576744e+02
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.054979e+01 1.198604e+02 2.592136e+02
    endloop
  endfacet
  facet normal 2.152054e-03 9.966032e-01 -8.232566e-02
    outer loop
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -6.434243e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal -7.166276e-04 9.978215e-01 -6.596796e-02
    outer loop
      vertex   -6.434243e+01 1.197716e+02 2.576744e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
    endloop
  endfacet
  facet normal 6.425119e-04 9.957481e-01 -9.211522e-02
    outer loop
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
      vertex   -7.868640e+01 1.195959e+02 2.551720e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
    endloop
  endfacet
  facet normal 5.073344e-04 9.936064e-01 -1.128986e-01
    outer loop
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
    endloop
  endfacet
  facet normal 2.293586e-03 9.922412e-01 -1.243066e-01
    outer loop
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
    endloop
  endfacet
  facet normal 7.598698e-04 9.941987e-01 -1.075565e-01
    outer loop
      vertex   -8.048625e+01 1.193828e+02 2.528561e+02
      vertex   -6.434243e+01 1.193837e+02 2.529787e+02
      vertex   -8.207336e+01 1.192237e+02 2.513739e+02
    endloop
  endfacet
  facet normal 4.581206e-03 9.616588e-01 2.742105e-01
    outer loop
      vertex   -8.206047e+01 1.158527e+02 2.962636e+02
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal 1.290752e-03 9.656753e-01 2.597491e-01
    outer loop
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
    endloop
  endfacet
  facet normal 1.162124e-03 9.704480e-01 2.413076e-01
    outer loop
      vertex   -7.948557e+01 1.165605e+02 2.935162e+02
      vertex   -8.028574e+01 1.162942e+02 2.945912e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal 2.028760e-03 9.663453e-01 2.572403e-01
    outer loop
      vertex   -8.028574e+01 1.162942e+02 2.945912e+02
      vertex   -8.206047e+01 1.158527e+02 2.962636e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal 3.028991e-03 9.755575e-01 2.197235e-01
    outer loop
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
      vertex   -7.858759e+01 1.168965e+02 2.921137e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal 1.299321e-03 9.722927e-01 2.337630e-01
    outer loop
      vertex   -7.858759e+01 1.168965e+02 2.921137e+02
      vertex   -7.948557e+01 1.165605e+02 2.935162e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal 3.217316e-03 9.757486e-01 2.188706e-01
    outer loop
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
      vertex   -6.434243e+01 1.164789e+02 2.937716e+02
    endloop
  endfacet
  facet normal -6.118097e-05 9.794389e-01 2.017411e-01
    outer loop
      vertex   -7.799664e+01 1.171657e+02 2.909105e+02
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
      vertex   -7.671030e+01 1.178709e+02 2.874870e+02
    endloop
  endfacet
  facet normal 1.792380e-03 9.840632e-01 1.778101e-01
    outer loop
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
      vertex   -7.671030e+01 1.178709e+02 2.874870e+02
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal 7.781152e-03 9.874508e-01 1.577351e-01
    outer loop
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
      vertex   -6.434243e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal 6.895803e-03 9.877718e-01 1.557541e-01
    outer loop
      vertex   -7.564920e+01 1.187178e+02 2.824510e+02
      vertex   -7.608757e+01 1.183170e+02 2.850118e+02
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
    endloop
  endfacet
  facet normal 6.316555e-04 9.916835e-01 1.286987e-01
    outer loop
      vertex   -7.564920e+01 1.187178e+02 2.824510e+02
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
      vertex   -7.522482e+01 1.192562e+02 2.783002e+02
    endloop
  endfacet
  facet normal 5.283633e-03 9.952567e-01 9.714043e-02
    outer loop
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
      vertex   -7.522482e+01 1.192562e+02 2.783002e+02
    endloop
  endfacet
  facet normal -1.523690e-03 9.936863e-01 1.121837e-01
    outer loop
      vertex   -6.434243e+01 1.195434e+02 2.754587e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
      vertex   -6.434243e+01 1.190442e+02 2.798804e+02
    endloop
  endfacet
  facet normal 8.061085e-05 9.976500e-01 6.851613e-02
    outer loop
      vertex   -6.434243e+01 1.197945e+02 2.718024e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
      vertex   -6.434243e+01 1.195434e+02 2.754587e+02
    endloop
  endfacet
  facet normal -2.561252e-03 9.982045e-01 5.984271e-02
    outer loop
      vertex   -6.434243e+01 1.197945e+02 2.718024e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.510338e+01 1.195712e+02 2.750659e+02
    endloop
  endfacet
  facet normal -1.172901e-02 9.999260e-01 -3.221246e-03
    outer loop
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.054979e+01 1.199710e+02 2.676379e+02
    endloop
  endfacet
  facet normal 5.353597e-03 9.999359e-01 9.974658e-03
    outer loop
      vertex   -7.585338e+01 1.199791e+02 2.637552e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.995070e-01 3.139633e-02
    outer loop
      vertex   -6.434243e+01 1.199710e+02 2.676379e+02
      vertex   -7.054979e+01 1.199710e+02 2.676379e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
    endloop
  endfacet
  facet normal 1.596224e-03 9.991015e-01 4.235222e-02
    outer loop
      vertex   -6.434243e+01 1.197945e+02 2.718024e+02
      vertex   -6.434243e+01 1.199710e+02 2.676379e+02
      vertex   -7.522637e+01 1.199212e+02 2.692218e+02
    endloop
  endfacet
  facet normal -2.694499e-03 9.997006e-01 -2.431837e-02
    outer loop
      vertex   -7.054979e+01 1.198604e+02 2.592136e+02
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
    endloop
  endfacet
  facet normal 9.627273e-04 9.995598e-01 -2.965398e-02
    outer loop
      vertex   -7.707051e+01 1.198304e+02 2.587016e+02
      vertex   -7.585338e+01 1.199791e+02 2.637552e+02
      vertex   -7.054979e+01 1.199566e+02 2.631678e+02
    endloop
  endfacet
  facet normal -3.567802e-04 9.999956e-01 -2.957814e-03
    outer loop
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
    endloop
  endfacet
  facet normal 2.139975e-04 9.963283e-01 -8.561470e-02
    outer loop
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal -3.802619e-04 9.989305e-01 -4.623481e-02
    outer loop
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal 3.679698e-04 9.980468e-01 -6.246995e-02
    outer loop
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
    endloop
  endfacet
  facet normal -1.898327e-04 9.994187e-01 -3.409020e-02
    outer loop
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
    endloop
  endfacet
  facet normal -1.531242e-03 9.997031e-01 -2.431843e-02
    outer loop
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
      vertex   -1.401963e+01 1.199606e+02 2.620361e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.983401e-01 -5.759464e-02
    outer loop
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
      vertex   -5.506548e+01 1.198604e+02 2.592136e+02
      vertex   -3.460164e+01 1.198604e+02 2.592136e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.987255e-01 -5.047076e-02
    outer loop
      vertex   -1.186332e+01 1.198799e+02 2.596596e+02
      vertex   -1.076126e+01 1.197988e+02 2.580540e+02
      vertex   -2.979263e+01 1.197988e+02 2.580540e+02
    endloop
  endfacet
  facet normal 8.634821e-04 9.966051e-01 -8.232582e-02
    outer loop
      vertex   -5.506548e+01 1.193837e+02 2.529787e+02
      vertex   -5.506548e+01 1.197716e+02 2.576744e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
    endloop
  endfacet
  facet normal 5.452420e-04 9.936064e-01 -1.128986e-01
    outer loop
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -5.506548e+01 1.193837e+02 2.529787e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
    endloop
  endfacet
  facet normal -4.488338e-04 9.925969e-01 -1.214549e-01
    outer loop
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
      vertex   -1.518801e+02 1.191647e+02 2.509168e+02
    endloop
  endfacet
  facet normal -3.545445e-04 9.925255e-01 -1.220365e-01
    outer loop
      vertex   -1.404418e+02 1.191717e+02 2.509316e+02
      vertex   -1.384771e+02 1.190334e+02 2.498008e+02
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 9.814243e-05 9.913775e-01 -1.310372e-01
    outer loop
      vertex   -1.384771e+02 1.190334e+02 2.498008e+02
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -1.560727e-04 9.902099e-01 -1.395861e-01
    outer loop
      vertex   -1.349492e+02 1.188831e+02 2.486665e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.908019e-01 -1.353206e-01
    outer loop
      vertex   -1.851814e+02 1.189489e+02 2.491523e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.940654e-01 -1.087842e-01
    outer loop
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.851814e+02 1.189489e+02 2.491523e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 3.198872e-03 9.929640e-01 -1.183734e-01
    outer loop
      vertex   -1.985206e+02 1.192734e+02 2.518699e+02
      vertex   -1.851814e+02 1.194248e+02 2.535007e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.908019e-01 -1.353206e-01
    outer loop
      vertex   -1.515866e+02 1.189489e+02 2.491523e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
      vertex   -1.851814e+02 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.940327e-01 -1.090829e-01
    outer loop
      vertex   -2.979263e+01 1.189489e+02 2.491523e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -2.979263e+01 1.193404e+02 2.527199e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.885904e-01 -1.506284e-01
    outer loop
      vertex   -3.000000e+00 1.185433e+02 2.464897e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -2.979263e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.885904e-01 -1.506284e-01
    outer loop
      vertex   -3.000000e+00 1.189489e+02 2.491523e+02
      vertex   -3.000000e+00 1.185433e+02 2.464897e+02
      vertex   -2.979263e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.908086e-01 -1.352711e-01
    outer loop
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 1.069114e-04 9.901431e-01 -1.400591e-01
    outer loop
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal 2.474391e-04 9.892996e-01 -1.458979e-01
    outer loop
      vertex   -3.000000e+00 1.185433e+02 2.464897e+02
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -5.506548e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -1.873948e-04 9.884562e-01 -1.515070e-01
    outer loop
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.880366e-01 -1.542196e-01
    outer loop
      vertex   -2.076235e+02 1.189489e+02 2.491523e+02
      vertex   -1.983806e+02 1.189489e+02 2.491523e+02
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
    endloop
  endfacet
  facet normal -3.174400e-03 9.933135e-01 -1.154042e-01
    outer loop
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
      vertex   -2.076235e+02 1.189489e+02 2.491523e+02
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
    endloop
  endfacet
  facet normal -3.628431e-05 9.875348e-01 -1.574009e-01
    outer loop
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal -6.541254e-08 9.903300e-01 -1.387318e-01
    outer loop
      vertex   -9.252737e+01 1.188395e+02 2.483512e+02
      vertex   -9.110770e+01 1.188325e+02 2.483016e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal 4.712108e-07 9.904437e-01 -1.379173e-01
    outer loop
      vertex   -9.431639e+01 1.188662e+02 2.485436e+02
      vertex   -9.252737e+01 1.188395e+02 2.483512e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal -1.545065e-05 9.908991e-01 -1.346071e-01
    outer loop
      vertex   -9.657965e+01 1.189439e+02 2.491157e+02
      vertex   -9.431639e+01 1.188662e+02 2.485436e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal -1.459817e-04 9.916671e-01 -1.288271e-01
    outer loop
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -9.657965e+01 1.189439e+02 2.491157e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
    endloop
  endfacet
  facet normal 6.900770e-04 9.894507e-01 -1.448687e-01
    outer loop
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -1.299533e+02 1.188428e+02 2.483754e+02
      vertex   -1.284992e+02 1.188627e+02 2.485181e+02
    endloop
  endfacet
  facet normal 2.463421e-04 9.907141e-01 -1.359612e-01
    outer loop
      vertex   -1.284992e+02 1.188627e+02 2.485181e+02
      vertex   -1.263468e+02 1.189322e+02 2.490285e+02
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
    endloop
  endfacet
  facet normal -7.493363e-05 9.919338e-01 -1.267572e-01
    outer loop
      vertex   -9.856552e+01 1.190561e+02 2.499818e+02
      vertex   -1.263468e+02 1.189322e+02 2.490285e+02
      vertex   -1.234855e+02 1.190940e+02 2.502928e+02
    endloop
  endfacet
  facet normal 1.349945e-05 9.907481e-01 -1.357137e-01
    outer loop
      vertex   -8.967083e+01 1.188454e+02 2.483939e+02
      vertex   -8.676811e+01 1.189217e+02 2.489508e+02
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -5.146800e-05 9.916954e-01 -1.286086e-01
    outer loop
      vertex   -8.676811e+01 1.189217e+02 2.489508e+02
      vertex   -8.452818e+01 1.190396e+02 2.498592e+02
      vertex   -6.434243e+01 1.189489e+02 2.491523e+02
    endloop
  endfacet
  facet normal -2.875747e-04 9.909833e-01 -1.339855e-01
    outer loop
      vertex   -2.570000e+02 1.183317e+02 2.451981e+02
      vertex   -2.570000e+02 1.193837e+02 2.529787e+02
      vertex   -2.077750e+02 1.193625e+02 2.527165e+02
    endloop
  endfacet
  facet normal -5.364140e-03 9.999459e-01 8.915062e-03
    outer loop
      vertex   -3.713972e+00 1.199948e+02 2.641674e+02
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
      vertex   -3.000000e+00 1.199772e+02 2.665705e+02
    endloop
  endfacet
  facet normal -2.138877e-04 9.999727e-01 7.385123e-03
    outer loop
      vertex   3.500586e+01 1.199973e+02 2.649414e+02
      vertex   -3.713972e+00 1.199948e+02 2.641674e+02
      vertex   -3.000000e+00 1.199772e+02 2.665705e+02
    endloop
  endfacet
  facet normal 8.402846e-03 9.988546e-01 4.710469e-02
    outer loop
      vertex   -3.000000e+00 1.199772e+02 2.665705e+02
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
      vertex   -3.000000e+00 1.196073e+02 2.744133e+02
    endloop
  endfacet
  facet normal 3.091596e-03 9.986834e-01 5.120436e-02
    outer loop
      vertex   -7.744902e+00 1.199365e+02 2.682799e+02
      vertex   -1.339072e+01 1.196755e+02 2.737113e+02
      vertex   -3.000000e+00 1.196073e+02 2.744133e+02
    endloop
  endfacet
  facet normal 3.640097e-04 9.958234e-01 9.129934e-02
    outer loop
      vertex   -3.000000e+00 1.196073e+02 2.744133e+02
      vertex   -1.339072e+01 1.196755e+02 2.737113e+02
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
    endloop
  endfacet
  facet normal 5.441433e-03 9.939462e-01 1.097333e-01
    outer loop
      vertex   -3.000000e+00 1.196073e+02 2.744133e+02
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
      vertex   -3.000000e+00 1.187457e+02 2.822173e+02
    endloop
  endfacet
  facet normal 2.361424e-03 9.925526e-01 1.217939e-01
    outer loop
      vertex   -3.000000e+00 1.187457e+02 2.822173e+02
      vertex   -1.788001e+01 1.192404e+02 2.784745e+02
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
    endloop
  endfacet
  facet normal 7.279232e-04 9.908267e-01 1.351368e-01
    outer loop
      vertex   -3.000000e+00 1.187457e+02 2.822173e+02
      vertex   -1.900116e+01 1.190198e+02 2.802938e+02
      vertex   -1.979407e+01 1.187579e+02 2.822186e+02
    endloop
  endfacet
  facet normal 7.277237e-04 9.898173e-01 1.423420e-01
    outer loop
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
      vertex   -3.000000e+00 1.187457e+02 2.822173e+02
      vertex   -1.979407e+01 1.187579e+02 2.822186e+02
    endloop
  endfacet
  facet normal -2.079536e-04 9.763134e-01 2.163611e-01
    outer loop
      vertex   -1.385739e+01 1.170593e+02 2.914029e+02
      vertex   -1.245231e+01 1.170824e+02 2.913000e+02
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 1.553765e-03 9.757325e-01 2.189602e-01
    outer loop
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
      vertex   -1.245231e+01 1.170824e+02 2.913000e+02
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
    endloop
  endfacet
  facet normal 1.514351e-04 9.760644e-01 2.174817e-01
    outer loop
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
      vertex   -1.481712e+01 1.170728e+02 2.913431e+02
      vertex   -1.385739e+01 1.170593e+02 2.914029e+02
    endloop
  endfacet
  facet normal 2.108595e-04 9.760010e-01 2.177657e-01
    outer loop
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
      vertex   -1.481712e+01 1.170728e+02 2.913431e+02
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal -2.511988e-04 9.766468e-01 2.148509e-01
    outer loop
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
      vertex   -1.534414e+01 1.170910e+02 2.912622e+02
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal -1.017608e-05 9.773724e-01 2.115260e-01
    outer loop
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
      vertex   -1.149261e+01 1.172200e+02 2.906803e+02
    endloop
  endfacet
  facet normal 5.596381e-04 9.828523e-01 1.843937e-01
    outer loop
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal -1.125528e-03 9.793592e-01 2.021245e-01
    outer loop
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
      vertex   -8.092141e+00 1.176860e+02 2.884483e+02
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
    endloop
  endfacet
  facet normal 5.679740e-05 9.783853e-01 2.067902e-01
    outer loop
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
      vertex   -1.865998e+01 1.174848e+02 2.894377e+02
      vertex   -1.775243e+01 1.173137e+02 2.902469e+02
    endloop
  endfacet
  facet normal 1.465127e-03 9.810343e-01 1.938285e-01
    outer loop
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
      vertex   -1.865998e+01 1.174848e+02 2.894377e+02
      vertex   -9.865427e+00 1.172907e+02 2.903535e+02
    endloop
  endfacet
  facet normal 2.425480e-04 9.833776e-01 1.815722e-01
    outer loop
      vertex   -2.009433e+01 1.181431e+02 2.860262e+02
      vertex   -1.978383e+01 1.178934e+02 2.873779e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal 2.294870e-04 9.857471e-01 1.682335e-01
    outer loop
      vertex   -2.014447e+01 1.183714e+02 2.846882e+02
      vertex   -2.009433e+01 1.181431e+02 2.860262e+02
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
    endloop
  endfacet
  facet normal 4.092017e-04 9.860359e-01 1.665323e-01
    outer loop
      vertex   -7.231086e+00 1.181423e+02 2.860135e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
      vertex   -2.014447e+01 1.183714e+02 2.846882e+02
    endloop
  endfacet
  facet normal 1.488875e-03 9.895137e-01 1.444315e-01
    outer loop
      vertex   -3.000000e+00 1.187457e+02 2.822173e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
      vertex   -4.074998e+00 1.185047e+02 2.838795e+02
    endloop
  endfacet
  facet normal -1.356998e-04 9.871066e-01 1.600640e-01
    outer loop
      vertex   -2.005551e+01 1.185879e+02 2.833532e+02
      vertex   -2.014447e+01 1.183714e+02 2.846882e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 9.236445e-05 9.889638e-01 1.481572e-01
    outer loop
      vertex   -1.979407e+01 1.187579e+02 2.822186e+02
      vertex   -2.005551e+01 1.185879e+02 2.833532e+02
      vertex   -6.873124e+00 1.185498e+02 2.835997e+02
    endloop
  endfacet
  facet normal 1.172236e-02 9.592282e-01 2.823894e-01
    outer loop
      vertex   -2.472520e+00 1.159962e+02 2.957370e+02
      vertex   -5.640534e+00 1.154711e+02 2.976523e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal -2.015947e-03 9.610739e-01 2.762839e-01
    outer loop
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
      vertex   -2.570000e+02 1.161556e+02 2.950399e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.553090e-01 2.956091e-01
    outer loop
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -2.244379e+02 1.152378e+02 2.984701e+02
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
    endloop
  endfacet
  facet normal -3.114000e-04 9.571380e-01 2.896323e-01
    outer loop
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
    endloop
  endfacet
  facet normal -1.927658e-04 9.558648e-01 2.938068e-01
    outer loop
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
      vertex   -2.570000e+02 1.142755e+02 3.015799e+02
    endloop
  endfacet
  facet normal -1.567938e-08 9.582741e-01 2.858509e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -1.679633e+02 1.149894e+02 2.993138e+02
      vertex   -1.691356e+02 1.149770e+02 2.993553e+02
    endloop
  endfacet
  facet normal 1.738492e-07 9.583630e-01 2.855528e-01
    outer loop
      vertex   -1.668203e+02 1.150153e+02 2.992270e+02
      vertex   -1.679633e+02 1.149894e+02 2.993138e+02
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
    endloop
  endfacet
  facet normal 1.024599e-05 9.592467e-01 2.825698e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -1.635742e+02 1.151959e+02 2.986136e+02
      vertex   -1.668203e+02 1.150153e+02 2.992270e+02
    endloop
  endfacet
  facet normal -5.016863e-04 9.341015e-01 3.570072e-01
    outer loop
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 2.484463e-04 9.576748e-01 2.878524e-01
    outer loop
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal -3.387527e-05 9.475097e-01 3.197271e-01
    outer loop
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 1.076424e-06 9.586103e-01 2.847213e-01
    outer loop
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -8.795835e+01 1.150710e+02 2.990388e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
    endloop
  endfacet
  facet normal 7.132979e-05 9.600218e-01 2.799251e-01
    outer loop
      vertex   -8.518957e+01 1.153176e+02 2.981925e+02
      vertex   -8.795835e+01 1.150710e+02 2.990388e+02
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
    endloop
  endfacet
  facet normal 1.321694e-04 9.606767e-01 2.776695e-01
    outer loop
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
      vertex   -1.635742e+02 1.151959e+02 2.986136e+02
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
    endloop
  endfacet
  facet normal 1.425076e-04 9.618108e-01 2.737151e-01
    outer loop
      vertex   -1.367380e+02 1.151557e+02 2.987505e+02
      vertex   -1.404940e+02 1.156418e+02 2.970446e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal -1.758824e-04 9.589639e-01 2.835283e-01
    outer loop
      vertex   -1.344416e+02 1.150155e+02 2.992262e+02
      vertex   -1.367380e+02 1.151557e+02 2.987505e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal -8.510020e-03 8.928913e-01 4.501920e-01
    outer loop
      vertex   -1.320934e+02 1.149831e+02 2.993348e+02
      vertex   -1.344416e+02 1.150155e+02 2.992262e+02
      vertex   -1.617673e+02 1.153726e+02 2.980013e+02
    endloop
  endfacet
  facet normal 1.472075e-05 9.591684e-01 2.828356e-01
    outer loop
      vertex   -1.274703e+02 1.151530e+02 2.987608e+02
      vertex   -1.290720e+02 1.150542e+02 2.990958e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
    endloop
  endfacet
  facet normal 1.176625e-04 9.613009e-01 2.755007e-01
    outer loop
      vertex   -9.498749e+01 1.150972e+02 2.989498e+02
      vertex   -9.912625e+01 1.155655e+02 2.973174e+02
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
    endloop
  endfacet
  facet normal -1.811987e-04 9.584227e-01 2.853521e-01
    outer loop
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
      vertex   -9.498749e+01 1.150972e+02 2.989498e+02
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
    endloop
  endfacet
  facet normal 8.917950e-05 9.604217e-01 2.785502e-01
    outer loop
      vertex   -1.255125e+02 1.153455e+02 2.980963e+02
      vertex   -1.274703e+02 1.151530e+02 2.987608e+02
      vertex   -9.175506e+01 1.149840e+02 2.993319e+02
    endloop
  endfacet
  facet normal -1.078361e-04 9.671870e-01 2.540655e-01
    outer loop
      vertex   -5.541862e+01 1.153514e+02 2.980765e+02
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal 2.888041e-05 9.606907e-01 2.776211e-01
    outer loop
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
      vertex   -5.541862e+01 1.153514e+02 2.980765e+02
    endloop
  endfacet
  facet normal -4.114454e-05 9.597242e-01 2.809440e-01
    outer loop
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
      vertex   -5.541862e+01 1.153514e+02 2.980765e+02
    endloop
  endfacet
  facet normal -1.480522e-05 9.592589e-01 2.825287e-01
    outer loop
      vertex   -1.690145e+01 1.150292e+02 2.991803e+02
      vertex   -1.893107e+01 1.151454e+02 2.987855e+02
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
    endloop
  endfacet
  facet normal -2.903569e-09 9.582505e-01 2.859300e-01
    outer loop
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
      vertex   -1.582586e+01 1.149974e+02 2.992870e+02
    endloop
  endfacet
  facet normal 4.059360e-05 9.488281e-01 3.157929e-01
    outer loop
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
    endloop
  endfacet
  facet normal 1.336151e-03 9.526958e-01 3.039227e-01
    outer loop
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
      vertex   -1.073784e+01 1.150429e+02 2.991338e+02
      vertex   -1.338596e+01 1.149924e+02 2.993036e+02
    endloop
  endfacet
  facet normal -1.846653e-06 9.584727e-01 2.851843e-01
    outer loop
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -1.637812e+01 1.150115e+02 2.992396e+02
      vertex   -1.690145e+01 1.150292e+02 2.991803e+02
    endloop
  endfacet
  facet normal -7.761279e-07 9.583731e-01 2.855187e-01
    outer loop
      vertex   -1.582586e+01 1.149974e+02 2.992870e+02
      vertex   -1.637812e+01 1.150115e+02 2.992396e+02
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
    endloop
  endfacet
  facet normal 5.749200e-03 9.576701e-01 2.878106e-01
    outer loop
      vertex   -5.640534e+00 1.154711e+02 2.976523e+02
      vertex   -7.682700e+00 1.152392e+02 2.984647e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 3.001570e-03 9.557635e-01 2.941208e-01
    outer loop
      vertex   -7.682700e+00 1.152392e+02 2.984647e+02
      vertex   -1.073784e+01 1.150429e+02 2.991338e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 0.000000e+00 9.481577e-01 3.178003e-01
    outer loop
      vertex   -5.629332e+01 1.149765e+02 2.993569e+02
      vertex   -6.434243e+01 1.149765e+02 2.993569e+02
      vertex   -3.000000e+00 1.146517e+02 3.003261e+02
    endloop
  endfacet
  facet normal 7.279934e-05 9.597506e-01 2.808537e-01
    outer loop
      vertex   -1.702650e+02 1.149842e+02 2.993311e+02
      vertex   -1.760108e+02 1.152917e+02 2.982818e+02
      vertex   -1.916617e+02 1.152378e+02 2.984701e+02
    endloop
  endfacet
  facet normal -2.788264e-03 9.624131e-01 2.715753e-01
    outer loop
      vertex   -2.168113e+01 1.154029e+02 2.978949e+02
      vertex   -2.446381e+01 1.158529e+02 2.962714e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal -5.325959e-04 9.696966e-01 2.443116e-01
    outer loop
      vertex   -2.610656e+01 1.162351e+02 2.948219e+02
      vertex   -2.702544e+01 1.165023e+02 2.937590e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal -1.157753e-03 9.666368e-01 2.561484e-01
    outer loop
      vertex   -2.446381e+01 1.158529e+02 2.962714e+02
      vertex   -2.610656e+01 1.162351e+02 2.948219e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal -4.031614e-04 9.728870e-01 2.312806e-01
    outer loop
      vertex   -2.702544e+01 1.165023e+02 2.937590e+02
      vertex   -2.831703e+01 1.169801e+02 2.917471e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal -1.842604e-03 9.765597e-01 2.152388e-01
    outer loop
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal -1.296585e-03 9.755368e-01 2.198322e-01
    outer loop
      vertex   -2.831703e+01 1.169801e+02 2.917471e+02
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
      vertex   -5.230544e+01 1.165489e+02 2.935193e+02
    endloop
  endfacet
  facet normal -7.183290e-04 9.784642e-01 2.064155e-01
    outer loop
      vertex   -2.873742e+01 1.171798e+02 2.908582e+02
      vertex   -2.945709e+01 1.176557e+02 2.885998e+02
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal -5.328916e-04 9.821382e-01 1.881604e-01
    outer loop
      vertex   -2.945709e+01 1.176557e+02 2.885998e+02
      vertex   -2.973875e+01 1.180351e+02 2.866187e+02
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal -3.863588e-03 9.867044e-01 1.624792e-01
    outer loop
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
      vertex   -4.387629e+01 1.188915e+02 2.811804e+02
    endloop
  endfacet
  facet normal -2.458039e-03 9.857244e-01 1.683491e-01
    outer loop
      vertex   -2.973875e+01 1.180351e+02 2.866187e+02
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
      vertex   -4.881401e+01 1.176819e+02 2.884084e+02
    endloop
  endfacet
  facet normal -1.738934e-04 9.895563e-01 1.441469e-01
    outer loop
      vertex   -2.971076e+01 1.184878e+02 2.839686e+02
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
      vertex   -4.387629e+01 1.188915e+02 2.811804e+02
    endloop
  endfacet
  facet normal -2.365112e-03 9.933907e-01 1.147576e-01
    outer loop
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
    endloop
  endfacet
  facet normal -1.330074e-03 9.936728e-01 1.123060e-01
    outer loop
      vertex   -4.387629e+01 1.188915e+02 2.811804e+02
      vertex   -2.910918e+01 1.189707e+02 2.806544e+02
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal -4.211084e-04 9.959067e-01 9.038633e-02
    outer loop
      vertex   -2.762349e+01 1.193983e+02 2.769832e+02
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal -2.073787e-03 9.977134e-01 6.755516e-02
    outer loop
      vertex   -4.032308e+01 1.194841e+02 2.759791e+02
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
    endloop
  endfacet
  facet normal -1.369777e-03 9.978133e-01 6.608191e-02
    outer loop
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
      vertex   -2.642318e+01 1.195806e+02 2.749805e+02
      vertex   -2.342848e+01 1.198403e+02 2.711217e+02
    endloop
  endfacet
  facet normal 1.270169e-03 9.996185e-01 2.759014e-02
    outer loop
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
      vertex   -2.342848e+01 1.198403e+02 2.711217e+02
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
    endloop
  endfacet
  facet normal 3.327256e-03 9.993222e-01 3.666126e-02
    outer loop
      vertex   -3.460164e+01 1.199717e+02 2.676038e+02
      vertex   -3.639849e+01 1.198812e+02 2.702341e+02
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
    endloop
  endfacet
  facet normal -2.732840e-04 9.999941e-01 -3.413932e-03
    outer loop
      vertex   -3.460164e+01 1.199566e+02 2.631678e+02
      vertex   -3.460164e+01 1.199717e+02 2.676038e+02
      vertex   -1.834659e+01 1.199712e+02 2.661434e+02
    endloop
  endfacet
  facet normal -2.545858e-04 -8.888624e-01 -4.581742e-01
    outer loop
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
      vertex   8.757386e+01 -1.066949e+02 2.100802e+02
      vertex   8.270525e+01 -1.069076e+02 2.104955e+02
    endloop
  endfacet
  facet normal -3.121643e-04 -8.965494e-01 -4.429436e-01
    outer loop
      vertex   7.718331e+01 -1.074371e+02 2.115488e+02
      vertex   7.382110e+01 -1.078791e+02 2.124458e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
    endloop
  endfacet
  facet normal -7.032581e-04 -9.034855e-01 -4.286180e-01
    outer loop
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
      vertex   7.382110e+01 -1.078791e+02 2.124458e+02
    endloop
  endfacet
  facet normal -2.237842e-04 -8.929863e-01 -4.500837e-01
    outer loop
      vertex   8.270525e+01 -1.069076e+02 2.104955e+02
      vertex   7.718331e+01 -1.074371e+02 2.115488e+02
      vertex   -3.000000e+00 -1.070214e+02 2.107638e+02
    endloop
  endfacet
  facet normal -8.916068e-04 -9.021490e-01 -4.314237e-01
    outer loop
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
      vertex   7.382110e+01 -1.078791e+02 2.124458e+02
      vertex   6.816070e+01 -1.088681e+02 2.145256e+02
    endloop
  endfacet
  facet normal -1.265042e-04 -9.136123e-01 -4.065864e-01
    outer loop
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
      vertex   6.644155e+01 -1.092275e+02 2.153093e+02
      vertex   6.306252e+01 -1.100072e+02 2.170624e+02
    endloop
  endfacet
  facet normal -3.850011e-04 -9.086526e-01 -4.175527e-01
    outer loop
      vertex   6.816070e+01 -1.088681e+02 2.145256e+02
      vertex   6.644155e+01 -1.092275e+02 2.153093e+02
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
    endloop
  endfacet
  facet normal 2.556023e-04 -9.238558e-01 -3.827407e-01
    outer loop
      vertex   6.306252e+01 -1.100072e+02 2.170624e+02
      vertex   5.713868e+01 -1.116070e+02 2.209199e+02
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
    endloop
  endfacet
  facet normal -2.302218e-04 -9.200166e-01 -3.918794e-01
    outer loop
      vertex   -3.000000e+00 -1.098259e+02 2.166755e+02
      vertex   6.306252e+01 -1.100072e+02 2.170624e+02
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
    endloop
  endfacet
  facet normal -4.371492e-04 -9.353069e-01 -3.538373e-01
    outer loop
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
      vertex   5.451531e+01 -1.124062e+02 2.230006e+02
      vertex   -3.000000e+00 -1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal -1.473896e-04 -9.334322e-01 -3.587539e-01
    outer loop
      vertex   5.713868e+01 -1.116070e+02 2.209199e+02
      vertex   5.451531e+01 -1.124062e+02 2.230006e+02
      vertex   -3.000000e+00 -1.112621e+02 2.200472e+02
    endloop
  endfacet
  facet normal 1.682937e-04 -9.430443e-01 -3.326672e-01
    outer loop
      vertex   5.451531e+01 -1.124062e+02 2.230006e+02
      vertex   5.058303e+01 -1.137136e+02 2.267046e+02
      vertex   -3.000000e+00 -1.129293e+02 2.244543e+02
    endloop
  endfacet
  facet normal -7.481482e-04 -9.496986e-01 -3.131645e-01
    outer loop
      vertex   -3.000000e+00 -1.129293e+02 2.244543e+02
      vertex   5.058303e+01 -1.137136e+02 2.267046e+02
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
    endloop
  endfacet
  facet normal -5.579890e-04 -9.505138e-01 -3.106817e-01
    outer loop
      vertex   5.058303e+01 -1.137136e+02 2.267046e+02
      vertex   4.800076e+01 -1.146268e+02 2.295033e+02
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
    endloop
  endfacet
  facet normal 6.375905e-05 -9.602990e-01 -2.789727e-01
    outer loop
      vertex   4.800076e+01 -1.146268e+02 2.295033e+02
      vertex   4.496398e+01 -1.157742e+02 2.334522e+02
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
    endloop
  endfacet
  facet normal -1.531053e-03 -9.667762e-01 -2.556196e-01
    outer loop
      vertex   -3.000000e+00 -1.148965e+02 2.304201e+02
      vertex   4.496398e+01 -1.157742e+02 2.334522e+02
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
    endloop
  endfacet
  facet normal -1.217852e-03 -9.676371e-01 -2.523429e-01
    outer loop
      vertex   4.496398e+01 -1.157742e+02 2.334522e+02
      vertex   4.322144e+01 -1.164629e+02 2.361016e+02
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
    endloop
  endfacet
  facet normal 3.520645e-05 -9.756831e-01 -2.191858e-01
    outer loop
      vertex   4.322144e+01 -1.164629e+02 2.361016e+02
      vertex   4.052270e+01 -1.175733e+02 2.410441e+02
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
    endloop
  endfacet
  facet normal -1.433262e-03 -9.796657e-01 -2.006314e-01
    outer loop
      vertex   -3.000000e+00 -1.168366e+02 2.377576e+02
      vertex   4.052270e+01 -1.175733e+02 2.410441e+02
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
    endloop
  endfacet
  facet normal -1.118320e-04 -9.826675e-01 -1.853767e-01
    outer loop
      vertex   4.052270e+01 -1.175733e+02 2.410441e+02
      vertex   3.879548e+01 -1.183089e+02 2.449443e+02
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
    endloop
  endfacet
  facet normal -8.195980e-04 -9.890151e-01 -1.478123e-01
    outer loop
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
      vertex   3.760651e+01 -1.188282e+02 2.482948e+02
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
    endloop
  endfacet
  facet normal -3.218646e-04 -9.881840e-01 -1.532719e-01
    outer loop
      vertex   3.879548e+01 -1.183089e+02 2.449443e+02
      vertex   3.760651e+01 -1.188282e+02 2.482948e+02
      vertex   -3.000000e+00 -1.182543e+02 2.446802e+02
    endloop
  endfacet
  facet normal 2.618584e-04 -9.927506e-01 -1.201920e-01
    outer loop
      vertex   3.760651e+01 -1.188282e+02 2.482948e+02
      vertex   3.636882e+01 -1.193790e+02 2.528413e+02
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
    endloop
  endfacet
  facet normal -1.273946e-03 -9.949678e-01 -1.001875e-01
    outer loop
      vertex   -3.000000e+00 -1.190281e+02 2.498577e+02
      vertex   3.636882e+01 -1.193790e+02 2.528413e+02
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
    endloop
  endfacet
  facet normal 5.633180e-05 -9.964420e-01 -8.428083e-02
    outer loop
      vertex   3.636882e+01 -1.193790e+02 2.528413e+02
      vertex   3.558606e+01 -1.197326e+02 2.570212e+02
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
    endloop
  endfacet
  facet normal -1.486288e-03 -9.989535e-01 -4.571305e-02
    outer loop
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
      vertex   3.522228e+01 -1.198983e+02 2.601275e+02
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
    endloop
  endfacet
  facet normal -6.797572e-04 -9.985751e-01 -5.335955e-02
    outer loop
      vertex   3.558606e+01 -1.197326e+02 2.570212e+02
      vertex   3.522228e+01 -1.198983e+02 2.601275e+02
      vertex   -3.000000e+00 -1.196575e+02 2.561079e+02
    endloop
  endfacet
  facet normal -2.448953e-04 -9.996388e-01 -2.687539e-02
    outer loop
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
      vertex   3.522228e+01 -1.198983e+02 2.601275e+02
      vertex   3.505275e+01 -1.199758e+02 2.630130e+02
    endloop
  endfacet
  facet normal -4.431553e-04 -9.999792e-01 -6.432734e-03
    outer loop
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
      vertex   3.505275e+01 -1.199758e+02 2.630130e+02
      vertex   3.501195e+01 -1.199945e+02 2.659215e+02
    endloop
  endfacet
  facet normal -4.071016e-04 -9.997428e-01 2.267577e-02
    outer loop
      vertex   -3.000000e+00 -1.198812e+02 2.702341e+02
      vertex   3.501195e+01 -1.199945e+02 2.659215e+02
      vertex   3.518153e+01 -1.199169e+02 2.693454e+02
    endloop
  endfacet
  facet normal -1.853828e-03 -9.999490e-01 9.929657e-03
    outer loop
      vertex   -3.000000e+00 -1.199566e+02 2.626442e+02
      vertex   3.501195e+01 -1.199945e+02 2.659215e+02
      vertex   -3.000000e+00 -1.198812e+02 2.702341e+02
    endloop
  endfacet
  facet normal -1.221814e-03 -9.983095e-01 5.810819e-02
    outer loop
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
      vertex   3.518153e+01 -1.199169e+02 2.693454e+02
      vertex   3.577916e+01 -1.196452e+02 2.740257e+02
    endloop
  endfacet
  facet normal 6.723389e-04 -9.976188e-01 6.896531e-02
    outer loop
      vertex   -3.000000e+00 -1.198812e+02 2.702341e+02
      vertex   3.518153e+01 -1.199169e+02 2.693454e+02
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal 6.438035e-04 -9.954855e-01 9.491183e-02
    outer loop
      vertex   3.577916e+01 -1.196452e+02 2.740257e+02
      vertex   3.686427e+01 -1.191575e+02 2.791342e+02
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
    endloop
  endfacet
  facet normal -5.111715e-04 -9.940030e-01 1.093513e-01
    outer loop
      vertex   -3.000000e+00 -1.189688e+02 2.806625e+02
      vertex   -3.000000e+00 -1.194841e+02 2.759791e+02
      vertex   3.686427e+01 -1.191575e+02 2.791342e+02
    endloop
  endfacet
  facet normal -1.428838e-03 -9.907826e-01 1.354539e-01
    outer loop
      vertex   3.686427e+01 -1.191575e+02 2.791342e+02
      vertex   3.821116e+01 -1.185629e+02 2.834979e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
    endloop
  endfacet
  facet normal 1.136674e-03 -9.884344e-01 1.516449e-01
    outer loop
      vertex   3.686427e+01 -1.191575e+02 2.791342e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
      vertex   -3.000000e+00 -1.189688e+02 2.806625e+02
    endloop
  endfacet
  facet normal 1.393740e-04 -9.854403e-01 1.700216e-01
    outer loop
      vertex   3.821116e+01 -1.185629e+02 2.834979e+02
      vertex   3.972464e+01 -1.179099e+02 2.872813e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
    endloop
  endfacet
  facet normal -3.977803e-05 -9.795998e-01 2.009582e-01
    outer loop
      vertex   3.972464e+01 -1.179099e+02 2.872813e+02
      vertex   4.145130e+01 -1.171847e+02 2.908165e+02
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
    endloop
  endfacet
  facet normal -8.826277e-04 -9.814754e-01 1.915862e-01
    outer loop
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
      vertex   -3.000000e+00 -1.182543e+02 2.853198e+02
      vertex   3.972464e+01 -1.179099e+02 2.872813e+02
    endloop
  endfacet
  facet normal -9.977561e-04 -9.730879e-01 2.304320e-01
    outer loop
      vertex   4.145130e+01 -1.171847e+02 2.908165e+02
      vertex   4.314328e+01 -1.164970e+02 2.937278e+02
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
    endloop
  endfacet
  facet normal 1.021356e-04 -9.708490e-01 2.396920e-01
    outer loop
      vertex   -3.000000e+00 -1.171511e+02 2.909718e+02
      vertex   4.145130e+01 -1.171847e+02 2.908165e+02
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
    endloop
  endfacet
  facet normal 6.175814e-04 -9.644770e-01 2.641659e-01
    outer loop
      vertex   4.754618e+01 -1.147969e+02 2.999246e+02
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
      vertex   4.314328e+01 -1.164970e+02 2.937278e+02
    endloop
  endfacet
  facet normal -7.771645e-04 -9.600005e-01 2.799971e-01
    outer loop
      vertex   -3.000000e+00 -1.159590e+02 2.958000e+02
      vertex   4.754618e+01 -1.147969e+02 2.999246e+02
      vertex   -3.000000e+00 -1.141268e+02 3.020820e+02
    endloop
  endfacet
  facet normal -2.484266e-05 -9.419933e-01 3.356318e-01
    outer loop
      vertex   -3.000000e+00 -1.141268e+02 3.020820e+02
      vertex   4.944255e+01 -1.141052e+02 3.021464e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal 2.755246e-05 -9.548039e-01 2.972364e-01
    outer loop
      vertex   4.754618e+01 -1.147969e+02 2.999246e+02
      vertex   4.944255e+01 -1.141052e+02 3.021464e+02
      vertex   -3.000000e+00 -1.141268e+02 3.020820e+02
    endloop
  endfacet
  facet normal -9.833290e-04 -9.455709e-01 3.254146e-01
    outer loop
      vertex   4.944255e+01 -1.141052e+02 3.021464e+02
      vertex   5.286401e+01 -1.129372e+02 3.055507e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal -6.279411e-04 -9.392151e-01 3.433286e-01
    outer loop
      vertex   5.286401e+01 -1.129372e+02 3.055507e+02
      vertex   5.437768e+01 -1.124479e+02 3.068920e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal -7.770541e-04 -9.313020e-01 3.642471e-01
    outer loop
      vertex   5.437768e+01 -1.124479e+02 3.068920e+02
      vertex   5.869795e+01 -1.111589e+02 3.101968e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal -1.776676e-03 -9.232083e-01 3.842958e-01
    outer loop
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
      vertex   6.353984e+01 -1.098899e+02 3.132072e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -2.693457e-03 -9.199144e-01 3.921099e-01
    outer loop
      vertex   5.869795e+01 -1.111589e+02 3.101968e+02
      vertex   6.353984e+01 -1.098899e+02 3.132072e+02
      vertex   -3.000000e+00 -1.125455e+02 3.065200e+02
    endloop
  endfacet
  facet normal -6.375822e-04 -9.140647e-01 4.055678e-01
    outer loop
      vertex   6.353984e+01 -1.098899e+02 3.132072e+02
      vertex   6.524728e+01 -1.094885e+02 3.141146e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -1.416384e-04 -9.036596e-01 4.282514e-01
    outer loop
      vertex   6.865246e+01 -1.087696e+02 3.156867e+02
      vertex   7.143751e+01 -1.082674e+02 3.167475e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -1.988475e-04 -9.068042e-01 4.215520e-01
    outer loop
      vertex   6.795508e+01 -1.089086e+02 3.153875e+02
      vertex   6.865246e+01 -1.087696e+02 3.156867e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -2.557587e-04 -9.087211e-01 4.174038e-01
    outer loop
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
      vertex   6.658969e+01 -1.091941e+02 3.147650e+02
      vertex   6.795508e+01 -1.089086e+02 3.153875e+02
    endloop
  endfacet
  facet normal -3.641943e-04 -9.107186e-01 4.130272e-01
    outer loop
      vertex   6.524728e+01 -1.094885e+02 3.141146e+02
      vertex   6.658969e+01 -1.091941e+02 3.147650e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -2.033857e-04 -8.993402e-01 4.372496e-01
    outer loop
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
      vertex   7.676004e+01 -1.074815e+02 3.183627e+02
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
    endloop
  endfacet
  facet normal -2.328213e-04 -8.989148e-01 4.381234e-01
    outer loop
      vertex   7.143751e+01 -1.082674e+02 3.167475e+02
      vertex   7.676004e+01 -1.074815e+02 3.183627e+02
      vertex   -3.000000e+00 -1.085232e+02 3.161829e+02
    endloop
  endfacet
  facet normal -2.262383e-06 -8.870116e-01 4.617471e-01
    outer loop
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
      vertex   8.264628e+01 -1.069085e+02 3.195029e+02
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal 6.300088e-07 -8.912051e-01 4.536006e-01
    outer loop
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
      vertex   8.148208e+01 -1.069939e+02 3.193351e+02
      vertex   8.264628e+01 -1.069085e+02 3.195029e+02
    endloop
  endfacet
  facet normal -2.296504e-06 -8.918995e-01 4.522337e-01
    outer loop
      vertex   8.068946e+01 -1.070599e+02 3.192049e+02
      vertex   8.148208e+01 -1.069939e+02 3.193351e+02
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
    endloop
  endfacet
  facet normal -2.094927e-05 -8.941855e-01 4.476967e-01
    outer loop
      vertex   7.676004e+01 -1.074815e+02 3.183627e+02
      vertex   8.068946e+01 -1.070599e+02 3.192049e+02
      vertex   -3.000000e+00 -1.069208e+02 3.194789e+02
    endloop
  endfacet
  facet normal -4.353126e-06 -8.856257e-01 4.643997e-01
    outer loop
      vertex   8.762442e+01 -1.066968e+02 3.199162e+02
      vertex   2.510000e+02 -1.066536e+02 3.200000e+02
      vertex   2.510000e+02 -1.058915e+02 3.214535e+02
    endloop
  endfacet
  facet normal -9.992864e-05 -8.886785e-01 4.585308e-01
    outer loop
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
      vertex   8.499439e+01 -1.067717e+02 3.197705e+02
      vertex   8.762442e+01 -1.066968e+02 3.199162e+02
    endloop
  endfacet
  facet normal -1.328994e-04 -8.894302e-01 4.570709e-01
    outer loop
      vertex   8.399071e+01 -1.068240e+02 3.196684e+02
      vertex   8.499439e+01 -1.067717e+02 3.197705e+02
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal -1.635826e-04 -8.900777e-01 4.558087e-01
    outer loop
      vertex   8.264628e+01 -1.069085e+02 3.195029e+02
      vertex   8.399071e+01 -1.068240e+02 3.196684e+02
      vertex   -3.000000e+00 -1.059537e+02 3.213366e+02
    endloop
  endfacet
  facet normal -4.285399e-03 -5.008823e-01 8.655048e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   8.769390e+01 -5.492844e+01 3.716905e+02
      vertex   8.382111e+01 -5.463816e+01 3.718393e+02
    endloop
  endfacet
  facet normal -1.250870e-03 -4.519925e-01 8.920209e-01
    outer loop
      vertex   -3.000000e+00 -5.862844e+01 3.696885e+02
      vertex   8.769390e+01 -5.492844e+01 3.716905e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal -1.300617e-03 -4.502719e-01 8.928906e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   7.998316e+01 -5.407034e+01 3.721278e+02
      vertex   7.574357e+01 -5.308999e+01 3.726160e+02
    endloop
  endfacet
  facet normal -2.029212e-03 -4.637506e-01 8.859635e-01
    outer loop
      vertex   8.382111e+01 -5.463816e+01 3.718393e+02
      vertex   7.998316e+01 -5.407034e+01 3.721278e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal -5.676247e-04 -4.276152e-01 9.039607e-01
    outer loop
      vertex   7.245065e+01 -5.211520e+01 3.730920e+02
      vertex   6.803415e+01 -5.039814e+01 3.739014e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal -9.334566e-04 -4.413354e-01 8.973418e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   7.574357e+01 -5.308999e+01 3.726160e+02
      vertex   7.245065e+01 -5.211520e+01 3.730920e+02
    endloop
  endfacet
  facet normal -5.881342e-04 -4.121704e-01 9.111066e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   6.803415e+01 -5.039814e+01 3.739014e+02
      vertex   6.352677e+01 -4.819826e+01 3.748937e+02
    endloop
  endfacet
  facet normal -2.774589e-04 -4.043055e-01 9.146239e-01
    outer loop
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
      vertex   5.968971e+01 -4.582694e+01 3.758947e+02
      vertex   -3.000000e+00 -4.495279e+01 3.762621e+02
    endloop
  endfacet
  facet normal -1.460904e-03 -3.908822e-01 9.204396e-01
    outer loop
      vertex   6.352677e+01 -4.819826e+01 3.748937e+02
      vertex   5.968971e+01 -4.582694e+01 3.758947e+02
      vertex   -3.000000e+00 -5.045859e+01 3.738282e+02
    endloop
  endfacet
  facet normal 1.381227e-04 -3.652579e-01 9.309064e-01
    outer loop
      vertex   5.968971e+01 -4.582694e+01 3.758947e+02
      vertex   5.392573e+01 -4.150379e+01 3.775918e+02
      vertex   -3.000000e+00 -4.202329e+01 3.773964e+02
    endloop
  endfacet
  facet normal 4.301278e-04 -3.610871e-01 9.325320e-01
    outer loop
      vertex   -3.000000e+00 -4.495279e+01 3.762621e+02
      vertex   5.968971e+01 -4.582694e+01 3.758947e+02
      vertex   -3.000000e+00 -4.202329e+01 3.773964e+02
    endloop
  endfacet
  facet normal -3.403890e-04 -3.191424e-01 9.477067e-01
    outer loop
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
      vertex   -3.000000e+00 -4.202329e+01 3.773964e+02
      vertex   5.392573e+01 -4.150379e+01 3.775918e+02
    endloop
  endfacet
  facet normal -1.940539e-03 -3.323673e-01 9.431480e-01
    outer loop
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
      vertex   5.392573e+01 -4.150379e+01 3.775918e+02
      vertex   5.078054e+01 -3.849571e+01 3.786454e+02
    endloop
  endfacet
  facet normal 1.263596e-04 -3.006573e-01 9.537322e-01
    outer loop
      vertex   5.078054e+01 -3.849571e+01 3.786454e+02
      vertex   4.638255e+01 -3.346616e+01 3.802315e+02
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
    endloop
  endfacet
  facet normal -1.009273e-03 -2.730802e-01 9.619907e-01
    outer loop
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
      vertex   -3.000000e+00 -3.532612e+01 3.796517e+02
      vertex   4.638255e+01 -3.346616e+01 3.802315e+02
    endloop
  endfacet
  facet normal 1.103410e-04 -2.599944e-01 9.656101e-01
    outer loop
      vertex   4.311119e+01 -2.873187e+01 3.815066e+02
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
      vertex   4.638255e+01 -3.346616e+01 3.802315e+02
    endloop
  endfacet
  facet normal -6.485997e-04 -2.190902e-01 9.757044e-01
    outer loop
      vertex   4.311119e+01 -2.873187e+01 3.815066e+02
      vertex   4.048438e+01 -2.390496e+01 3.825887e+02
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
    endloop
  endfacet
  facet normal 7.940719e-04 -2.296725e-01 9.732676e-01
    outer loop
      vertex   -3.000000e+00 -2.953533e+01 3.812955e+02
      vertex   4.048438e+01 -2.390496e+01 3.825887e+02
      vertex   -3.000000e+00 -2.494940e+01 3.823777e+02
    endloop
  endfacet
  facet normal -1.946987e-03 -1.981367e-01 9.801725e-01
    outer loop
      vertex   4.048438e+01 -2.390496e+01 3.825887e+02
      vertex   3.980561e+01 -2.248258e+01 3.828749e+02
      vertex   -3.000000e+00 -1.980194e+01 3.833317e+02
    endloop
  endfacet
  facet normal -3.938062e
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
