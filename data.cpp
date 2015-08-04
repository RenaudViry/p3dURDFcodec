#include <iostream>
#include <fstream>
#include "data.hpp"

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void Geometry::setColor(double *c)
{
	for (int i = 0; i < 4; i++)
		_color[i] = c[i];
}

void Geometry::setPosition(double *p)
{
	for (int i = 0; i < 3; i++)
		_position[i] = p[i];
}

void Geometry::setAngle(double *a)
{
	for (int i = 0; i < 3; i++)
		_angle[i] = a[i];
}

std::ostream& operator<< (std::ostream& ost, const Geometry& g)
{
	ost << "           ** Name : " << g.getName() << std::endl
		<< "            * Geometry type : " << geomTypeName[g.getGeomType()] << std::endl
		<< "            * Body type : " << bodyTypeName[g.getBodyType()] << std::endl
		<< "            * Position / Angle : " << g.getPosition()[0] << " "
											   << g.getPosition()[1] << " "
											   << g.getPosition()[2] << " / "
											   << g.getAngle()[0] << " "
											   << g.getAngle()[1] << " "
											   << g.getAngle()[2] << std::endl
		<< "            * Color : (" << g.getColor()[0] << ", "
									 << g.getColor()[1] << ", "
									 << g.getColor()[2] << ", "
									 << g.getColor()[3] << ")" << std::endl;
	return ost;
}

/*************************************************************************************************/

void BasicPrimitive::setParams(double *p)
{
	for (int i = 0; i < 3; i++)
		_params[i] = p[i];
}

Eigen::Vector3d BasicPrimitive::getVertex(int i)
{
	logger(WARNING_LVL, "getVertex(" + std::to_string(i) + ") called for BasicPrimitive !");
	return Eigen::Vector3d();
}

std::vector< Eigen::Vector3d > BasicPrimitive::getVertices()
{
	logger(WARNING_LVL, "getVertices called for BasicPrimitive !");
	return std::vector< Eigen::Vector3d >();	
}

Eigen::Vector3i BasicPrimitive::getFace(int i)
{
	logger(WARNING_LVL, "getFace(" + std::to_string(i) + ") called for BasicPrimitive !");
	return Eigen::Vector3i();	
}

std::vector< Eigen::Vector3i > BasicPrimitive::getFaces()
{
	logger(WARNING_LVL, "getFaces called for BasicPrimitive !");
	return std::vector< Eigen::Vector3i >();
}

/*************************************************************************************************/

double* Polyhedre::getParams()
{
	logger(WARNING_LVL, "getParams called for Polhyedre !");
	return NULL;
}

/*************************************************************************************************/

void Body::setGeom(std::string n, int g, int b, double *p)
{
	Geometry *newGeom(0);
	switch (g) {
		case BOX:
		case CYLINDER:
		case SPHERE:
		{
			BasicPrimitive *newPrim = new BasicPrimitive();
			newPrim->setParams(p);
			newGeom = newPrim;
			break;
		}
		case POLY:
		{
			Polyhedre *newPoly = new Polyhedre();
			newGeom = newPoly;
			break;
		}
		case GEOM_TYPE_UNKNOWN:
		default:
		{
			logger(WARNING_LVL, "Geometry of undefined type.");
			break;
		}
	}
	newGeom->setName(n);
	newGeom->setGeomType(g);
	newGeom->setBodyType(b);
	_geoms.push_back(newGeom);
	logger(DEBUG_LVL, "Set a new geometry with name : " + n + " (in body : " + _name +
					  "), of type : " + geomTypeName[g] + ", and : " + bodyTypeName[b]);
}

void Body::setPosition(double *p)
{
	for (int i = 0; i < 3; i++)
		_position[i] = p[i];
}

void Body::setAngle(double *a)
{
	for (int i = 0; i < 3; i++)
		_angle[i] = a[i];
}

Geometry* Body::getGeom(std::string n)
{
	for (std::vector< Geometry* >::iterator it = _geoms.begin(); it != _geoms.end(); ++it) {
		if (n.compare((*it)->getName()) == 0)
			return *it;
	}
	return NULL;
}

std::vector< Geometry* > Body::getGeomOfBodyType(int b)
{
	if (b == BODY_TYPE_UNKNOWN)
		logger(WARNING_LVL, "List of body type UNKNOWN requested ...");

	std::vector< Geometry* > geomOfBodyType;
	for (std::vector< Geometry* >::iterator it = _geoms.begin(); it != _geoms.end(); ++it) {
		if ((*it)->getBodyType() == b || (*it)->getBodyType() == BOTH)
			geomOfBodyType.push_back(*it);
	}
	if (geomOfBodyType.size() == 0)
		logger(WARNING_LVL, "No geometry of type " + bodyTypeName[b] + " found in " + getName() + ".");
	return geomOfBodyType;
}

std::vector< Geometry* > Body::getGeomOfGeomType(int g)
{
	if (g == GEOM_TYPE_UNKNOWN)
		logger(WARNING_LVL, "List of geom type UNKNOWN requested ...");

	std::vector< Geometry* > geomOfGeomType;
	for (std::vector< Geometry* >::iterator it = _geoms.begin(); it != _geoms.end(); ++it) {
		if ((*it)->getGeomType() == g)
			geomOfGeomType.push_back(*it);
	}
	if (geomOfGeomType.size() == 0)
		logger(WARNING_LVL, "No geometry of type " + geomTypeName[g] + " found in " + getName() + ".");
	return geomOfGeomType;
}

// void Body::convertPolysToWavefront()
// {
// 	logger(INFO_LVL, "Convert polyhedras of body " + getName() + "into Wavefront.");
// 	std::vector< Geometry* > graphicGeoms = getGeomOfBodyType(GRAPHIC);
// 	std::vector< Geometry* > graphicPolys;
// 	for (int i = 0; i < graphicGeoms.size(); i++) {
// 		if (graphicGeoms[i]->getGeomType() == POLY)
// 			graphicPolys.push_back(graphicGeoms[i]);
// 	}
// 	if (graphicPolys.size() == 0) {
// 		logger(WARNING_LVL, "In p3d::Codec::convertPolysToWavefront, no poly to convert !");
// 		return;
// 	}

// 	std::string wfPath = "./tmp/" + getName() + ".obj";
// 	setWavefrontPath(wfPath);

// 	std::ifstream ifs(wfPath.c_str());
// 	if(ifs && ifs.good()) {
// 		logger(INFO_LVL, "Wavefront file " + wfPath + " already exists, skipping!");
// 		return;
// 	}

// 	std::ofstream *ofs = new std::ofstream();
// 	logger(INFO_LVL, "Open '" + wfPath + "' Wavefront file in write mode ...");
// 	ofs->open(wfPath.c_str(), std::ofstream::out);
// 	if (!ofs) {
// 		logger(ERROR_LVL, "Unable to open input file: " + wfPath);
// 		return;
// 	}
// 	if(ofs->good()) {
// 		for (int p = 0; p < graphicPolys.size(); p++) {
// 			*ofs << "o " << graphicPolys[p]->getName() << std::endl;
// 			for (int v = 0; v < graphicPolys[p]->getVertices().size(); v++) {
// 				*ofs << "    v " << graphicPolys[p]->getVertex(v)[0] << " "
// 								 << graphicPolys[p]->getVertex(v)[1] << " "
// 								 << graphicPolys[p]->getVertex(v)[2] << std::endl;
// 			}
// 			*ofs << std::endl;
// 			for (int f = 0; f < graphicPolys[p]->getFaces().size(); f++) {
// 				*ofs << "    f " << graphicPolys[p]->getFace(f)[0] << " "
// 								 << graphicPolys[p]->getFace(f)[1] << " "
// 								 << graphicPolys[p]->getFace(f)[2] << std::endl;	
// 			}
// 			*ofs << std::endl;
// 		}
// 	}	
// 	ofs->close();
// 	logger(INFO_LVL, "Close input file (" + wfPath + ") !");
// }

// void Body::convertWavefrontToPolys()
// {
// 	// TODO
// }

std::ostream& operator<< (std::ostream& ost, const Body& b)
{
	ost << "        * Name : " << b.getName() << std::endl;
	ost << "        * Position / Angles : " << b.getPosition()[0] << " "
											<< b.getPosition()[1] << " "
											<< b.getPosition()[2] << " / "
											<< b.getAngle()[0] << " "
											<< b.getAngle()[1] << " "
											<< b.getAngle()[2] << std::endl;
	ost << "        * Geometries : " << b.getGeomsVector().size() << std::endl;
	for (int i = 0; i < b.getGeomsVector().size(); i++) {
		ost << *(b.getGeom(i));
	}
	ost << std::endl;
	return ost;
}

/*************************************************************************************************/

Link::Link() : _name(""), _bodies()
{}

Link::Link(Link *l)
{
	_name = l->_name;
	for(std::vector< Body* >::iterator it = l->_bodies.begin(); it != l->_bodies.end(); ++it)
		_bodies.push_back(*it);
}
void Link::setBody(std::string n)
{
	Body *newBody = new Body();
	newBody->setName(n);
	_bodies.push_back(newBody);
	logger(DEBUG_LVL, "Set a new body with name : " + n + ", in link : " + _name);
}

Body* Link::getBody(std::string n)
{
	for (std::vector< Body* >::iterator it = _bodies.begin(); it != _bodies.end(); ++it) {
		if (n.compare((*it)->getName()) == 0)
			return *it;
	}
	return NULL;
}

std::ostream& operator<< (std::ostream& ost, const Link& l)
{
	ost << std::endl
	   << "    * Name : " << l.getName() << std::endl;
	ost << "    * Bodies : " << l.getBodiesVector().size() << std::endl;
	for (int i = 0; i < l.getBodiesVector().size(); i++) {
		ost << *(l.getBody(i));
	}
	ost << std::endl;
	return ost;
}


/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void Joint::setMin(double *m)
{
	for (int i = 0; i < 6; i++) {
		_min[i] = m[i];
	}
}

void Joint::setMax(double *m)
{
	for (int i = 0; i < 6; i++) {
		_max[i] = m[i];
	}
}

void Joint::setPosAxe(double *ax)
{
	for (int i = 0; i < 3; i++) {
		_position[i] = ax[i];
		_axis[i] = ax[i + 3];
	}
}

void Joint::setPosMat(double *mat)
{
	Eigen::Matrix4d input, prevJnt, output;
	for (int i = 0; i < 4; i++)
		for (int j = 0; j < 4; j++)
			input(i, j) = mat[i * 4 + j];
	output = input;

	if (isRelative()) {
		prevJnt.setIdentity();
		prevJnt(0, 3) = _parentJnt->getPosition()[0];
		prevJnt(1, 3) = _parentJnt->getPosition()[1];
		prevJnt(2, 3) = _parentJnt->getPosition()[2];
		
		Eigen::Vector3d axe(_parentJnt->getAxis()[0],
							_parentJnt->getAxis()[1],
							_parentJnt->getAxis()[2]);
		axe.normalize();
		Eigen::Matrix3d m;
		m = Eigen::AngleAxisd(0., axe);

		prevJnt.block<3, 3>(0,0) = m;
		output = output * prevJnt;
	}
	double ax[6];
	ax[0] = output(0, 3);
	ax[1] = output(1, 3);
	ax[2] = output(2, 3);

	Eigen::Matrix3d matrix (output.block<3, 3>(0,0));
	double epsilon = 0.01;

	if ((fabs(matrix(0, 1) - matrix(1, 0)) < epsilon)
		&& (fabs(matrix(0, 2) - matrix(2, 0)) < epsilon)
		&& (fabs(matrix(1, 2) - matrix(2, 1)) < epsilon)) {
		// singularity found
		if ((fabs(matrix(0, 1) + matrix(1, 0)) < 0.1)
			&& (fabs(matrix(0, 2) + matrix(2, 0)) < 0.1)
			&& (fabs(matrix(1, 2) + matrix(2, 1)) < 0.1)
		 	&& (fabs(matrix(0, 0) + matrix(1, 1) + matrix(2, 2) - 3) < 0.1)) {
			ax[3]= 1;
			ax[4]= 0;
			ax[5]= 0;
			setPosAxe(ax);
			return;
		}
		ax[3] = (matrix(0, 0) + 1) / 2;
		if (ax[3] > 0) {
			ax[3] = sqrt(ax[3]);
		} else {
			ax[3] = 0;
		}

		ax[4] = (matrix(1, 1) + 1) / 2;
		if (ax[4] > 0) {
			ax[4] = sqrt(ax[4]);
		} else {
			ax[4] = 0;
		}

		ax[5] = (matrix(2, 2) + 1) / 2;
		if (ax[5] > 0) {
			ax[5] = sqrt(ax[5]);
		} else {
			ax[5] = 0;
		}


		int xZero = (fabs(ax[3]) < epsilon);
		int yZero = (fabs(ax[4]) < epsilon);
		int zZero = (fabs(ax[5]) < epsilon);
		int xyPositive = (matrix(0, 1) > 0);
		int xzPositive = (matrix(0, 2) > 0);
		int yzPositive = (matrix(1, 2) > 0);
		if (xZero && !yZero && !zZero) {
			if (!yzPositive)
				ax[4] = -ax[4];
		} else {
			if (yZero && !zZero) {
				if (!xzPositive)
		    		ax[5] = -ax[5];
			} else {
		   		if (zZero) {
					if (!xyPositive)
		    			ax[3] = -ax[3];
		    	}
			}
		}
		setPosAxe(ax);
		return;
	} else {
		double s = sqrt( pow(matrix(2, 1) - matrix(1, 2), 2) +
			 			 pow(matrix(0, 2) - matrix(2, 0), 2) +
			 			 pow(matrix(1, 0) - matrix(0, 1), 2) );
		if (fabs(s) < 0.001)
			s=1;
		ax[3] = (matrix(2, 1) - matrix(1, 2)) / s;
		ax[4] = (matrix(0, 2) - matrix(2, 0)) / s;
		ax[5] = (matrix(1, 0) - matrix(0, 1)) / s;
		setPosAxe(ax);
		return;
	}
}

std::ostream& operator<< (std::ostream& ost, const Joint& jnt)
{
	ost << std::endl << "    * Name : " << jnt.getName() << std::endl
	   << "    * Type : " << jntTypeName[jnt.getType()] << std::endl;
	if (jnt.getRank() >= 0 && jnt.getParentJoint() != NULL)
		ost << "    * Previous joint index : " << jnt.getParentJoint()->getRank() << std::endl;
	ost << "    * Min / Max : ";
	switch (jnt.getType()) {
		case ROTATION:
		case TRANSLATION:
		case FIXED:
			ost << jnt.getMin()[0] << " / " << jnt.getMax()[0] << std::endl;
			break;
		case FREEFLYER:
			ost << jnt.getMin()[0] << " / " << jnt.getMax()[0] << std::endl
			   << "                  " << jnt.getMin()[1] << " / " << jnt.getMax()[1] << std::endl
			   << "                  " << jnt.getMin()[2] << " / " << jnt.getMax()[2] << std::endl
			   << "                  " << jnt.getMin()[3] << " / " << jnt.getMax()[3] << std::endl
			   << "                  " << jnt.getMin()[4] << " / " << jnt.getMax()[4] << std::endl
			   << "                  " << jnt.getMin()[5] << " / " << jnt.getMax()[5] << std::endl;
			break;
		case PLAN:
		case JNT_TYPE_UNKNOWN:
		default:
			ost << "0 / 0" << std::endl;
			break;
	}
	ost << "    * Position / Axis : " << jnt.getPosition()[0] << " "
									 << jnt.getPosition()[1] << " "
									 << jnt.getPosition()[2] << " / "
									 << jnt.getAxis()[0] << " "
									 << jnt.getAxis()[1] << " "
									 << jnt.getAxis()[2] << std::endl;
	ost << "    * Parent / Child Link : ";
	if (jnt.getParentLink() != NULL)
		ost << jnt.getParentLink()->getName() << " / ";
	else
		ost << "None / ";
	if (jnt.getChildLink() != NULL)
		ost << jnt.getChildLink()->getName();
	else
		ost << "None";
	ost << std::endl;
	return ost;
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void KinematicChain::setJoint(int t)
{
	Joint *newJoint = new Joint();
	newJoint->setType(t);
	newJoint->setRank(_joints.size());
	_joints.push_back(newJoint);
	logger(DEBUG_LVL, "Set a new joint of type : " + jntTypeName[t]);
}

void KinematicChain::setLink(Link *l, std::string n)
{
	Link *newLink = new Link(l);
	newLink->setName(n);
	_links.push_back(newLink);
	getLastJoint()->getParentJoint()->setChildLink(newLink);
	logger(DEBUG_LVL, "Set a new link with name : " + n);
}

void KinematicChain::setLink(std::string n)
{
	Link *newLink = new Link();
	newLink->setName(n);
	_links.push_back(newLink);
	getLastJoint()->setChildLink(newLink);
	logger(DEBUG_LVL, "Set a new link with name : " + n);
}

Joint* KinematicChain::getJoint(std::string n)
{
	for (std::vector< Joint* >::iterator it = _joints.begin(); it != _joints.end(); ++it) {
		if(n.compare((*it)->getName()) == 0)
			return *it;
	}
	return NULL;
}

Link* KinematicChain::getLink(std::string n)
{
	for (std::vector< Link* >::iterator it = _links.begin(); it != _links.end(); ++it) {
		if(n.compare((*it)->getName()) == 0)
			return *it;
	}
	return NULL;
}

std::ostream& operator<< (std::ostream& ost, const KinematicChain& kc)
{
	ost << "***********************************************" << std::endl
	   << "     Print Kinematic chain    " << std::endl
	   << std::endl << "== Joints ==" << std::endl;
	   //<< "Root Joint : " << *(kc.getRootJoint()) << std::endl;
	for (int j = 0; j < kc.getJointsVector().size(); j++)
		ost << "    Joint " << j << ": " << *(kc.getJoint(j)) << std::endl;
	ost << "== Links ==" << std::endl;
	for (int b = 0; b < kc.getLinksVector().size(); b++)
		ost << "    Link " << b << ": " << *(kc.getLink(b)) << std::endl;
	ost << "Kinematic chain has " << kc.getJointsVector().size() << " joints." << std::endl;
	ost << "Kinematic chain has " << kc.getLinksVector().size() << " links." << std::endl;
	return ost;
}
