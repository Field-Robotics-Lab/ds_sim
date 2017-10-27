# ds_sim

This package includes a set of common gazebo tools for building a 
simulation.  Includes Xacro and Gazebo binary plugins for a number of 
useful sensors, some utilities, that stuff.

Vehicle-specific stuff should either live elsewhere or in the
vehicle config repo.

## Contents of this Package

Simulation tools for:
* Phins
* 300kHz RDI DVL
* Paroscientific Digiquartz depth sensor
* Sonardyne Avtrak 6 USBL / SMS device
* Xeos GPS

## Getting Started

This is a DS ROS package intended to be run as part of the larger DSL ROS echosystem.
See the [Sentry Wiki](http://sentry-wiki.whoi.edu/ROS_Upgrade)

### Prerequisites

This package requires ds_msgs and gazeboros.  These (and any other) dependencies should be managed correctly in the package.xml.

### Installing

You should generally be installing this as part of a rosinstall file.  
If handled seperately though, simply add to a catkin workspace and 
build with:

```
catkin_make
```

## Deployment

We use [SemVer](http://semver.org/) for versioning.

## Authors

* **Ian Vaughn** - *Initial work* - [WHOI email](mailto:ivaughn@whoi.edu)
* **Stefano Suman** - *Initial work* - [WHOI email](mailto:ssuman@whoi.edu)

## Acknowledgments

* IFREMER for their architectural support
* Louis Whitcomb et. al. for his message definitions to look over


