all: p3d2urdf

p3d2urdf: p3d2urdf.o
	g++ -L/opt/ros/hydro/lib p3d2urdf.o -o p3dURDFcodec -ltinyxml -Wall -g
p3d2urdf.o: p3d2urdf.cpp
	g++ -I/opt/ros/hydro/include -I/usr/include/eigen3 -c -g -std=c++11 p3d2urdf.cpp


clean:
	rm -rf *.o p3d2urdf