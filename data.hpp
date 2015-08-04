#ifndef DATA_STRUCT_HPP
#define DATA_STRUCT_HPP

#include <stdio.h>
#include <string.h>
#include <vector>
#include <Eigen/Dense>
#include "logger.hpp"

typedef enum {
	ROTATION = 0,
	TRANSLATION,
	FIXED,
	FREEFLYER,
	PLAN,
	JNT_TYPE_UNKNOWN,
	NB_JNT_TYPE
} jntType;
const std::string jntTypeName[NB_JNT_TYPE] = {"rotation", "translation", "fixed", "freeflyer", "plan", "UNKNOWN"};

typedef enum {
	GRAPHIC = 0,
	COLLISION,
	BOTH,
	BODY_TYPE_UNKNOWN,
	NB_BODY_TYPE
} bodyType;
const std::string bodyTypeName[NB_BODY_TYPE] = {"graphic", "collision", "both", "UNKNOWN"};

typedef enum {
	BOX = 0,
	CYLINDER,
	SPHERE,
	POLY,
	GEOM_TYPE_UNKNOWN,
	NB_GEOM_TYPE
} geomType;
const std::string geomTypeName[NB_GEOM_TYPE] = {"BOX", "CYLINDER", "SPHERE", "POLY", "UNKNOWN"};

/*********
 * Links *
 *********/
class Geometry
{
public:
	void setName(std::string n) { _name = n; }
	void setBodyType(int b) { _bType = (bodyType)b; }
	void setGeomType(int g) { _gType = (geomType)g; }
	void setColor(double *c);
	void setPosition(double *p);
	void setAngle(double *a);
	std::string getName() const { return _name; }
	int getBodyType() const { return _bType; }
	int getGeomType() const { return _gType; }
	const double* getColor() const { return _color; }
	const double* getPosition() const { return _position; }
	const double* getAngle() const { return _angle; }

	virtual void setParams(double *p) = 0;
	virtual double* getParams() = 0;

	virtual void addVertex(Eigen::Vector3d v) = 0;
	virtual void addFace(Eigen::Vector3i f) = 0;
	virtual Eigen::Vector3d getVertex(int i) = 0;
	virtual std::vector< Eigen::Vector3d > getVertices() = 0;
	virtual Eigen::Vector3i getFace(int i) = 0;
	virtual std::vector< Eigen::Vector3i > getFaces() = 0;

	friend std::ostream& operator<< (std::ostream& ost, const Geometry& g);

protected:
	double _color[4];
	double _position[3];
	double _angle[3];
	std::string _name;
	geomType _gType;
	bodyType _bType;
};

class BasicPrimitive : public Geometry
{
public:
	virtual void setParams(double *p);
	virtual double* getParams() { return _params; }

	virtual void addVertex(Eigen::Vector3d v) {}
	virtual void addFace(Eigen::Vector3i f) {}
	virtual Eigen::Vector3d getVertex(int i);
	virtual std::vector< Eigen::Vector3d > getVertices();
	virtual Eigen::Vector3i getFace(int i);
	virtual std::vector< Eigen::Vector3i > getFaces();

protected:
	double _params[3];
};

class Polyhedre : public Geometry
{
public:
	virtual void addVertex(Eigen::Vector3d v) { _vertices.push_back(v); }
	virtual void addFace(Eigen::Vector3i f) { _faces.push_back(f); }
	virtual Eigen::Vector3d getVertex(int i) { return _vertices[i]; }
	virtual std::vector< Eigen::Vector3d > getVertices() { return _vertices; }
	virtual Eigen::Vector3i getFace(int i) { return _faces[i]; }
	virtual std::vector< Eigen::Vector3i > getFaces() { return _faces; }

	virtual void setParams(double *p) {}
	virtual double* getParams();

protected:
	std::vector< Eigen::Vector3d > _vertices;
	std::vector< Eigen::Vector3i > _faces;
};

class Body
{
public:
	void setName(std::string n) { _name = n; }
	// void setWavefrontPath(std::string p) { _wavefrontPath = p; }
	void setGeom(std::string name, int g, int b, double *p);
	void setPosition(double *p);
	void setAngle(double *a);
	std::string getName() const { return _name; }
	// std::string getWavefrontPath() { return _wavefrontPath; }
	Geometry* getGeom(int i) const { return _geoms[i]; }
	Geometry* getGeom(std::string name);
	Geometry* getLastGeom() {return _geoms[_geoms.size() - 1]; }
	std::vector< Geometry* > getGeomOfBodyType(int b);
	std::vector< Geometry* > getGeomOfGeomType(int g);
	std::vector< Geometry* > getGeomsVector() const { return _geoms; }
	const double* getPosition() const { return _position; }
	const double* getAngle() const { return _angle; }
	// void convertWavefrontToPolys();
	// void convertPolysToWavefront();
	friend std::ostream& operator<< (std::ostream& ost, const Body& b);

protected:
	std::string _name;
	// std::string _wavefrontPath;
	double _position[3];
	double _angle[3];
	std::vector< Geometry* > _geoms;
};

class Link
{
public:
	Link();
	Link(Link *l);
	void setName(std::string n) { _name = n; }
	void setBody(std::string name);
	std::string getName() const { return _name; }
	Body* getBody(int i) const { return _bodies[i]; }
	Body* getBody(std::string n);
	Body* getLastBody() {return _bodies[_bodies.size() - 1]; }
	std::vector< Body* > getBodiesVector() const { return _bodies; }
	friend std::ostream& operator<< (std::ostream& ost, const Link& l);

protected:
	std::string _name;
	std::vector< Body* > _bodies;
};

/**********
 * Joints *
 **********/
class Joint
{
public:
	void setName(std::string n) { _name = n; }
	void setType(int t) { _type = (jntType)t; };
	void setRank(int r) { _rank = r; }
	void setMin(double *m);
	void setMax(double *m);
	void setPosAxe(double *ax);
	void setPosMat(double *mat);
	void setParentJoint(Joint *jnt) { _parentJnt = jnt; }
	void setParentLink(Link *l) { _parentLnk = l; }
	void setChildLink(Link *l) { _childLnk = l; }
	void setRelative(bool rel) { _relative = rel; }
	std::string getName() const { return _name; }
	jntType getType() const { return _type; }
	int getRank() const { return _rank; }
	const double* getMin() const { return _min; }
	const double* getMax() const { return _max; }
	const double* getPosition() const { return _position; }
	const double* getAxis() const { return _axis; }
	Joint* getParentJoint() const { return _parentJnt; }
	Link* getParentLink() const { return _parentLnk; }
	Link* getChildLink() const { return _childLnk; }
	bool isRelative() const { return _relative; }
	friend std::ostream& operator<< (std::ostream& ost, const Joint& jnt);

protected:
	std::string _name;
	double _axis[3];
	double _position[3];
	bool _relative;
	double _min[6];
	double _max[6];
	jntType _type;
	Joint *_parentJnt;
	Link *_parentLnk;
	Link *_childLnk;
	int _rank;
};

/****************************
 * Kinematic Chain handling *
 ****************************/
class KinematicChain {
public:
	// void setRootJoint(Joint *jnt) { _rootJoint = jnt; }
	void setJoint(int t);
	void setLink(std::string n);
	void setLink(Link *l, std::string n);
	// Joint* getRootJoint() const { return _rootJoint; }
	Joint* getJoint(int i) const { return _joints[i]; }
	Joint* getJoint(std::string n);
	Joint* getLastJoint() { return _joints[_joints.size() - 1]; }
	Link* getLink(int i) const { return _links[i]; }
	Link* getLink(std::string n);
	Link* getLastLink() { return _links[_links.size() - 1]; }
	std::vector< Joint* > getJointsVector() const { return _joints; }
	std::vector< Link* > getLinksVector() const { return _links; }
	friend std::ostream& operator<< (std::ostream& ost, const KinematicChain& chain);

protected:
	// Joint *_rootJoint;
	std::vector< Joint* > _joints;
	std::vector< Link* > _links;
};

#endif /* DATA_STRUCT_HPP */