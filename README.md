# p3dURDFcodec

This codec is intended to convert P3D and URDF files, from one to another.

P3D file is LAAS homemade 3D model format. The basic meshe description is in the form of a wavefront file. The full model description is built on Move3D (LAAS open-source software, see https://www.openrobots.org/wiki/move3d) P3D format ('.macro' extension) and has its own syntax.

URDF is the well known Universal Robot Description Format initiated in ROS (see http://wiki.ros.org/urdf/).

The converter is a simple binary with a list of option, allowing to choose almost every parameter of the conversion.
For usage, simply enter 'p3dURDFcodec -h'
The sources are written in C++.

Current implementation allow only conversion from P3D to URDF. Again, see the help message to get the list of relevant options.