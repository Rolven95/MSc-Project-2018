# Kinect Tracker
This project implements a hand tracking library that uses the Kinect for sensing and OpenCV for processing

# Prerequisites
To build the project, you need the following dependencies:

- OpenCV: To work with OpenCV, you should download the latest release of version 2.4 from the [Github page](https://github.com/opencv/opencv/releases). Download the Windows executable and then install it into the `./dependencies` folder.
- Boost: Again, best to install the [pre-built binary](https://dl.bintray.com/boostorg/release/1.67.0/binaries/) for Windows. Install it to the default address. 
- Kinect SDK 2.0: [Download it from Microsoft](https://www.microsoft.com/en-us/download/details.aspx?id=44561). The build system will query the Windows environment for the SDK location, so you should be able to install it anywhere. However, I always recommend installing *to the default installation location* anyway.
