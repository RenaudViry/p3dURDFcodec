#ifndef P3D2URDF_HPP
#define P3D2URDF_HPP

#include <stdio.h>
// #include <stdlib.h>
#include <string.h>
#include <vector>
// #include <math.h>

#include <iostream>
#include <fstream>

// #include <urdf/model.h>
// #include <tinyxml.h>
#include <Eigen/Dense>

// #include <assimp/Importer.hpp>      // C++ importer interface
// #include <assimp/scene.h>           // Output data structure
// #include <assimp/postprocess.h>     // Post processing flags

// #include <boost/filesystem.hpp>

typedef enum {
	FATAL_LVL = 0,
	ERROR_LVL,
	WARNING_LVL,
	INFO_LVL,
	DEBUG_LVL,
	NB_LVL
} LOG_LEVELS;
const std::vector<std::string> LOG_LEVELS_NAME = {"FATAL", "ERROR", "WARNING", "INFO", "DEBUG"};

typedef enum {
	P3D_ROTATE = 0,
	P3D_TRANSLATE, 
	P3D_BASE,
	P3D_PLAN,
	P3D_FREEFLYER,
	P3D_QUADROTOR,
	P3D_FIXED,
	P3D_KNEE,
	NB_JNT_TYPE
} jntType;
const std::vector<std::string> jntTypeName = {"P3D_ROTATE",	"P3D_TRANSLATE", "P3D_BASE",
											  "P3D_PLAN", "P3D_FREEFLYER", "P3D_QUADROTOR",
											  "P3D_FIXED", "P3D_KNEE"};

typedef enum {
	BOX = 0,
	CYLINDER,
	SPHERE,
	MESH,
	NB_GEOM
} geomType;
const std::vector<std::string> geomTypeName = {"BOX", "CYLINDER" , "SPHERE", "MESH"};

class Joint
{
public:
	void setName(std::string n) { _name = n; }
	std::string getName() const { return _name; }
	void setType(std::string t);
	std::string getType() const { return jntTypeName[_type]; }
	void setParentJnt(Joint *jnt) { this->_parentJnt = jnt; }
	Joint* getParentJoint() const { return _parentJnt; }
	void setMin(std::string m);
	float getMin() const { return _min; }
	void setMax(std::string m);
	float getMax() const { return _max; }
	void setRank(int r) { _rank = r; }
	int getRank() const { return _rank; }
	void setPosAxe(std::string ax);
	void setPosMat(std::string mat);
	const double* getPosition() const { return _position; }
	const double* getAxis() const { return _axis; }
	void setRelative(bool rel) { _relative = rel; }
	bool isRelative() const { return _relative; }
	friend std::ostream& operator<< (std::ostream& out, const Joint& jnt);

private:
	std::string _name;
	double _axis[3];
	double _position[3];
	bool _relative;
	float _min;
	float _max;
	jntType _type;
	Joint *_parentJnt;
	Joint **_children;
	int _rank;
};

class BodyElement
{
public:
	void setType(int t) { _type = (geomType)t; }
	void setColor(double *c);
	void setPosition(double *p);
	void setAxis(double *a);
	geomType getType() { return _type; }
	double *getPosition() { return _position; }
	double *getAxis() { return _axis; }
	double *getColor() { return _color; }

private:
	geomType _type;
	double _position[3];
	double _axis[3];
	double _color[4];
};

class Body
{
public:
	void setName(std::string n) { _name = n; }
	std::string getName() const { return _name; }
	void setPathIn(std::string p);
	std::string getPathIn() const { return _pathIn; }
	void setPathOut(std::string p) { _pathOut = p; }
	std::string getPathOut() const { return _pathOut; }
	void setPosAxe(std::string m, std::string type);
	void setColor(std::string c, std::string type);
	friend std::ostream& operator<< (std::ostream& out, const Body& bdy);

private:
	std::string _name;
	BodyElement *_visual;
	BodyElement *_collision;
	std::string _pathIn;
	std::string _pathOut;

};

class KinematicChain {
public:
	void setRootJoint(Joint *jnt) { _rootJoint = jnt; }
	Joint* getRootJoint() const { return _rootJoint; }
	void setJoint(std::string type);
	Joint* getJoint(int i) const { return _joints[i]; }
	void setBody(std::string name, std::string pathIn);
	Body* getBody(int i) const { return _bodies[i]; }
	std::vector< Joint* > getJointsVector() const { return _joints; }
	std::vector< Body* > getBodiesVector() const { return _bodies; }
	friend std::ostream& operator<< (std::ostream& out, const KinematicChain& chain);

private:
	Joint *_rootJoint;
	std::vector< Joint* > _joints;
	std::vector< Body* > _bodies;
};


#endif