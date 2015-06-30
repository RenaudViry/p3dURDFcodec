/**
  p3d2Urdf: conversion of a P3D file (.macro) to a URDF file (.urdf).
  Author: Renaud Viry, renaud.viry@laas.fr
  Copyright LAAS-CNRS 2015
*/

#include "p3d2urdf.hpp"

int LOG_LEVEL = 4;


/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void log (int level, std::string message)
{
	if (level <= LOG_LEVEL)
		std::cout << LOG_LEVELS_NAME[level] << ": " << message << std::endl;

}
void help()
{
	std::cout << "Usage : p3d2urdf [options]" << std::endl
			  << std::endl
			  << "p3d2urdf is intended to convert a robot P3D (macro) file to a URDF one" << std::endl
			  << "For details on URDF model, see http://wiki.ros.org/urdf" << std::endl
			  << std::endl
			  << "Options :" << std::endl
			  << "    -h : print this help message" << std::endl
			  << "    -d : Select log level in ['FATAL', 'ERROR', 'WARNING', 'INFO', ('DEBUG')]" << std::endl
			  << "    -o : Define output path (relative to current folder)" << std::endl
			  << "    -i : Define input file (REQUIRED)" << std::endl
			  << std::endl
			  << "WARNING : the current implementation does not handle FREEFLYER joints properly."
			  << "          They will simply be squized off."
			  << std::endl;
			  exit(0);
}

int parseInput(std::ifstream *ifs)
{
	KinematicChain *robot = new KinematicChain();

	log(INFO_LVL, "Parse input file ...");
	std::string line;
	int nbLignes = 0;
	while (!ifs->eof()) {
		getline(*ifs, line);
		// Remove comments
		line = line.substr(0, line.find("#"));
		// Remove empty line
		if (line.size() == 0)
			continue;
		nbLignes ++;


		// TODO : add these tags :
		// * p3d_set_pos_xyz + p3d_set_dof (P3D_FREEFLYER)
		// * p3d_add_desc_(sphere / oval / cone) (GRAPHIC)
		// * p3d_set_body_abs_pos
		// * p3d_set_body_poly_color
		// * p3d_make_body_deformable ?


		/* Joints data */
		if (line.find("p3d_beg_desc_jnt ") != std::string::npos) {
			robot->setJoint(line.substr(line.find("_jnt ") + 5));
			continue;
		}
		if (line.find("p3d_set_name ") != std::string::npos) {
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			curJnt->setName(line.substr((line.find("name ") + 5)));
			continue;
		}
		if (line.find("p3d_set_prev_jnt ") != std::string::npos) {
			int prevJntIdx = atoi(line.substr((line.find("jnt ") + 4)).c_str());
			if (prevJntIdx > robot->getJointsVector().size())
				log(ERROR_LVL, "Previous joint index is out of bounds !");
			if (prevJntIdx == 0) {
				log(DEBUG_LVL, "Previous joint is world frame, skipping");
				continue;
			} else {
				Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
				curJnt->setParentJnt(robot->getJointsVector()[prevJntIdx]);
				continue;
			}
		}
		if (line.find("p3d_set_pos_relative ") != std::string::npos) {
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			curJnt->setRelative(true);
		}
		if (line.find("p3d_set_pos_mat ") != std::string::npos) {
			std::string value = line.substr(line.find("mat ") + 4);
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			curJnt->setPosMat(value);
		}
		if (line.find("p3d_set_pos_axe ") != std::string::npos) {
			std::string value = line.substr(line.find("axe ") + 4);
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			curJnt->setPosAxe(value);
			continue;
		}
		if (line.find("p3d_set_dof_vmin ") != std::string::npos) {
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			std::string vMin = line.substr((line.find("vmin ") + 5));
			curJnt->setMin(vMin);
			continue;
		}
		if (line.find("p3d_set_dof_vmax ") != std::string::npos) {
			Joint *curJnt = robot->getJointsVector()[robot->getJointsVector().size() - 1];
			std::string vMax = line.substr((line.find("vmax ") + 5));
			curJnt->setMax(vMax);
			continue;
		}
		/* Links data */
		if (line.find("p3d_read_macro ") != std::string::npos) {
			std::string tmp = line.substr(line.find("./"), line.find(".macro") + 6 - line.find("./"));
			robot->setBody(tmp, line.substr(line.find(".macro") + 7));
			continue;
		}
		if (line.find("p3d_set_body_abs_pos ") != std::string::npos) {
			Body *curBdy = robot->getBodiesVector()[robot->getBodiesVector().size() - 1];
			std::string absPos = line.substr((line.find("pos ") + 4));
			curBdy->setPosAxe(absPos, "visual");
		}
		if (line.find("p3d_set_body_poly_color ") != std::string::npos) {
			Body *curBdy = robot->getBodiesVector()[robot->getBodiesVector().size() - 1];
			std::string color = line.substr((line.find("pos ") + 4));
			curBdy->setColor(color, "visual");
		}
		if (line.find("p3d_beg_desc ") != std::string::npos)
		{

		}

	}

	robot->setRootJoint(robot->getJointsVector()[0]);
	log(DEBUG_LVL, std::to_string(nbLignes) + " lignes lues.");
	log(DEBUG_LVL, std::to_string(robot->getJointsVector().size()) + " joints in kinematic chain");
	log(DEBUG_LVL, std::to_string(robot->getBodiesVector().size()) + " bodies in kinematic chain");

	if (LOG_LEVEL > 2)
		std::cout << *robot << std::endl;
	return 0; 
}

int writeOutput(std::ofstream *ofs)
{
	return 0;
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/

int main(int argc,char** argv)
{
	if (argc < 2){
		std::cout << argv[0] << ": ERROR : too few arguments" << std::endl;
		std::cout << std::endl << "Try '" << argv[0] << " -h' for more information." << std::endl << std::endl;
		return -1;
	}

	int c;
	char *dbg = NULL;
	char *op = NULL;
	char *ip = NULL;
	while ((c = getopt(argc, argv, "hd:o:i:")) != -1) {
		switch (c) {
			case 'h':
				help();
				break;
			case 'd':
				dbg = optarg;
				break;
			case 'o':
				op = optarg;
				break;
			case 'i':
				ip = optarg;
				break;
			default:
				break;
		} 
	}

	if (ip == NULL) {
		log(FATAL_LVL, "Need to specify at least input file (.macro)");
		exit(-1);
	}

	if (op == NULL) {
		log(WARNING_LVL, "No output path specified, output files will be stored in current folder");
		op = (char*)"./";
	}

	if (dbg != NULL)
		for (int i = 0; i < NB_LVL; i++)
			if (std::string(dbg).compare(LOG_LEVELS_NAME[i]) == 0)
				LOG_LEVEL = (LOG_LEVELS)i;

	std::string inputFile = std::string(ip);
	if (inputFile.find(".macro") == std::string::npos) {
		log(ERROR_LVL, "Input file is not a '.macro' file (P3D)");
		help();
		return -1;
	}

	std::ifstream *ifs = new std::ifstream();
	log(INFO_LVL, "Open '" + inputFile + "' MACRO file in read mode ...");
	ifs->open(inputFile.c_str(), std::ifstream::in);
	if (!ifs) {
		log(ERROR_LVL, "Unable to open input file: " + inputFile);
		return -1;
	}
	parseInput(ifs);
	ifs->close();
	log(INFO_LVL, "Close input file (" + inputFile + ") !");

	std::string outputFile = inputFile.substr(0, inputFile.find(".macro"));
	outputFile += ".urdf";
	outputFile = op + outputFile;
	log(INFO_LVL, "Open '" + outputFile + "' URDF file in write mode ...");
	// std::ofstream *ofs = new std::ofstream();
	// WARNING: output file opened WITHOUT append, will be erased.
	// Add std::ofstream::app for append operation
	// ofs->open(outputFile.c_str(), std::ofstream::out);
	// if (!ofs) {
	// 	log(ERROR_LVL, "Unable to open output file: " + outputFile);
	// 	return -1;
	// }
	// writeOutput(ofs);
	// ofs->close();
	// log(INFO_LVL, "Close output file (" + outputFile + ") !");

	return 0;
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void Joint::setMin(std::string m)
{
	switch (_type) {
		case P3D_ROTATE:
			_min = atof(m.c_str()) * 3.14 / 180.0;
			break;
		case P3D_FREEFLYER:
		case P3D_TRANSLATE:
		case P3D_PLAN:
		case P3D_FIXED:
			_min = atof(m.c_str());
			break;
		default:
			_min = 0;
			break;
	}
}

void Joint::setMax(std::string m)
{
	switch (_type) {
		case P3D_ROTATE:
			_max = atof(m.c_str()) * 3.14 / 180.0;
			break;
		case P3D_FREEFLYER:
		case P3D_TRANSLATE:
		case P3D_PLAN:
		case P3D_FIXED:
			_max = atof(m.c_str());
			break;
		default:
			_max = 0;
			break;
	}
}

void Joint::setType(std::string t)
{
	size_t l = t.find(" ");
	std::string tmp = t.substr(0, l);
	for (int i = 0; i < jntTypeName.size(); i++)
		if (tmp.compare(jntTypeName[i]) == 0)
			_type = (jntType)i;
}

void Joint::setPosAxe(std::string ax)
{
	std::string tmp = ax;
	// log(DEBUG_LVL, "ax : " + ax);
	// log(DEBUG_LVL, "\t\tpositions :: " + tmp);
	for (int i = 0; i < 3; i++) {
		_position[i] = atof(tmp.substr(0, tmp.find(" ")).c_str());
		tmp = tmp.substr(tmp.find(" ") + 1);
		// log(DEBUG_LVL, "tmp : " + tmp);
	}
	// log(DEBUG_LVL, "\t\taxis :: " + tmp);
	for (int i = 0; i < 3; i++) {
		_axis[i] = atof(tmp.substr(0, tmp.find(" ")).c_str());
		tmp = tmp.substr(tmp.find(" ") + 1);
		// log(DEBUG_LVL, "tmp : " + tmp);
	}
}

void Joint::setPosMat(std::string mat)
{
	if (!isRelative()) {
		setPosAxe(mat);
		return;
	}

	// TODO :
	// * create matrix M1 from input value
	// * create matrix M2 from previous joint
	// * create absolute matrix M = M2*M1 and extract axis and position from M

}

std::ostream& operator<< (std::ostream& st, const Joint& jnt)
{
	st << std::endl << "    * Name : " << jnt.getName() << std::endl
	   << "    * Type : " << jnt.getType() << std::endl;
	if (jnt.getRank() > 0 && jnt.getParentJoint() != NULL)
		st << "    * Previous joint index : " << jnt.getParentJoint()->getRank() << std::endl;
	st << "    * Min / Max : " << jnt.getMin() << " / " << jnt.getMax() << std::endl
	   << "    * Position / Axis : " << jnt.getPosition()[0] << " "
	   								 << jnt.getPosition()[1] << " "
	   								 << jnt.getPosition()[2] << " / "
	   								 << jnt.getAxis()[0] << " "
	   								 << jnt.getAxis()[1] << " "
	   								 << jnt.getAxis()[2] << std::endl;
	return st;
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void Body::setPathIn(std::string p)
{
	std::string dataPath = std::string(std::getenv("ROBOTPKG_BASE")) + "/share/move3d/assets/ADREAM/MACROS";
	std::string macroPath = dataPath + p.substr(1);
	_pathIn = macroPath;
}

void Body::setPosAxe(std::string m, std::string type)
{
	std::string tmp = m;
	double pos[3], axis[3];
	// log(DEBUG_LVL, "ax : " + ax);
	// log(DEBUG_LVL, "\t\tpositions :: " + tmp);
	for (int i = 0; i < 3; i++) {
		pos[i] = atof(tmp.substr(0, tmp.find(" ")).c_str());
		tmp = tmp.substr(tmp.find(" ") + 1);
		// log(DEBUG_LVL, "tmp : " + tmp);
	}
	// log(DEBUG_LVL, "\t\taxis :: " + tmp);
	for (int i = 0; i < 3; i++) {
		axis[i] = atof(tmp.substr(0, tmp.find(" ")).c_str());
		tmp = tmp.substr(tmp.find(" ") + 1);
		// log(DEBUG_LVL, "tmp : " + tmp);
	}
	if (type.compare("visual") == 0) {
		_visual->setPosition(pos);
		_visual->setAxis(axis);
	} else if (type.compare("collision") == 0) {
		_collision->setPosition(pos);
		_collision->setAxis(axis);
	} else {
		log(ERROR_LVL, "Wrong type for Body::setposAxe");
	}
}

void Body::setColor(std::string c, std::string type)
{
	std::string tmp = c;
	double color[4];
	int i = 0;
	// log(DEBUG_LVL, "ax : " + ax);
	// log(DEBUG_LVL, "\t\tpositions :: " + tmp);
	while (tmp.find(" ") != std::string::npos) {
		color[i] = atof(tmp.substr(0, tmp.find(" ")).c_str());
		tmp = tmp.substr(tmp.find(" ") + 1);
		i++;
		log(DEBUG_LVL, "tmp : " + tmp);
	}
	if (i == 2)
		color [3] = 1;

	if (type.compare("visual") == 0)
		_visual->setColor(color);
	else if (type.compare("collision") == 0)
		_collision->setColor(color);
	else
		log(ERROR_LVL, "Wrong type for Body::setposAxe");
	log(DEBUG_LVL, "Color set : " + std::to_string(color[0]) + ", " + std::to_string(color[1]) +
							 ", " + std::to_string(color[2]) + ", " + std::to_string(color[3]));
}

std::ostream& operator<< (std::ostream& st, const Body& bdy)
{
	st << std::endl << "    * Name : " << bdy.getName() << std::endl
	   << "    * Input Path : " << bdy.getPathIn() << std::endl;
	return st;
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void BodyElement::setColor(double *c)
{
	for (int i =0; i < 4; i++)
		_color[i] = c[i];
}

void BodyElement::setPosition(double *p)
{
	for (int i =0; i < 3; i++)
		_position[i] = p[i];
}

void BodyElement::setAxis(double *a)
{
	for (int i =0; i < 3; i++)
		_axis[i] = a[i];
}

/*************************************************************************************************
 *																								 *
 *************************************************************************************************/
void KinematicChain::setJoint(std::string type)
{
	Joint *newJoint = new Joint();
	newJoint->setType(type);
	newJoint->setRank(_joints.size());
	_joints.push_back(newJoint);
	log(DEBUG_LVL, "Set a new joint of type : " + type);
}

void KinematicChain::setBody(std::string pathIn, std::string name)
{
	Body *newBody = new Body();
	newBody->setPathIn(pathIn);
	newBody->setName(name);
	_bodies.push_back(newBody);
	// TODO : Open the file pointed by _pathIn and parse it with the parser
	log(DEBUG_LVL, "Set a new body with name : " + name + ", and input path : " + pathIn);
}

std::ostream& operator<< (std::ostream& st, const KinematicChain& kc)
{
	st << "***********************************************" << std::endl
	   << "     Print Kinematic chain    " << std::endl
	   << std::endl << "== Joints ==" << std::endl;
	   //<< "Root Joint : " << *(kc.getRootJoint()) << std::endl;
	for (int j = 0; j < kc.getJointsVector().size(); j++)
		st << "    Joint " << j << ": " << *(kc.getJoint(j)) << std::endl;
	st << std::endl
	   << "== Bodies ==" << std::endl;
	for (int b = 0; b < kc.getBodiesVector().size(); b++)
		st << "    Body " << b << ": " << *(kc.getBody(b)) << std::endl;
	return st;
}

// void KinematicChain::print()
// {
// 	std::cout << "Root Joint : " << _rootJoint << std::endl;
// 	std::cout << std::endl;
// 	for (int i = 0; i < _joints.size(); i++)
// 		std::cout << "Joint " << i << ": " << _joints[i] << std::endl;
// }