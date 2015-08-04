# p3dURDFcodec

This codec is intended to convert P3D and URDF files, from one to another.

P3D file is LAAS homemade 3D model format. The basic mesh description is in the form of a wavefront file. The full model description is built on Move3D (LAAS open-source software, see https://www.openrobots.org/wiki/move3d) P3D format ('.macro' extension) and has its own syntax.

URDF is the well known Universal Robot Description Format initiated in ROS (see http://wiki.ros.org/urdf/).

The converter is a simple binary with a list of option, allowing to choose almost every parameter of the conversion.
For usage, simply enter 'p3dURDFcodec -h'.

The sources are written in an object oriented C++ fashion.

## Installation
p3dURDFcodec is provide with a CMakeList and relies on classic CMake build system.
It has 3 dependencies :
* Eigen
* tinyxml2
* assimp3
Problems can occur on some system with official library of these packages. Therefore, tinyxml2 sources are embedded in the project.
Concerning Assimp, p3dURDFcodec relies on Assimp(3.1.1) library for creating collada files used by URDF. As for current status, Assimp library provided by Ubuntu official repository seems to have problems. You may have to install it by hand (see http://assimp.sourceforge.net/).

## Note and limitations:
p3dURDFcodec is a work-in-progress software. Regarding this status, here is a small list of current limitations:
* Current implementation allow only conversion from P3D to URDF.
* A "collada" folder should already exist in the destination folder (current folder if option '-o' not used).
* As some p3d exotic primitives are not supported in URDF (which only supports BOX, CYLINDRE and SPHERE), they are reduced to the encompassing shape. For instance the primitive "oval" is encompassed in a sphere with the largest radius.

Improvements and TODO:
* Change logger in order to use a C++ stream
* Change pointers on basic data (double, int) to Eigen::Vector
