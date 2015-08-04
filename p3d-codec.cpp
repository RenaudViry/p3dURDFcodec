#include <cctype> // isspace
#include <stdlib.h> // atof, atoi
#include "p3d-codec.hpp"

namespace p3d {
	std::string Codec::removeWhiteSpace(const std::string& str, const int w)
	{
		int s = 0;
		std::string result = str;
		if ((w == FRONT) || (w == BOTH_WAY)){
			s = 0;
			while(std::isspace(result[s]))
				s++;
			result = result.substr(s);
		}
		if ((w == BACK) || (w == BOTH_WAY)) {
			s = result.size();
			while(std::isspace(result[s - 1]))
				s--;
			result = result.substr(0, s);
		}
		return result;
	}

	void Codec::triangulate(int *in, Eigen::Vector3i *out)
	{
		Eigen::Vector3i triangle;
		triangle[0] = in[0]; triangle[1] = in[1]; triangle[2] = in[2];
		out[0] = triangle;
		triangle[0] = in[1]; triangle[1] = in[2]; triangle[2] = in[3];
		out[1] = triangle;
		triangle[0] = in[2]; triangle[1] = in[3]; triangle[2] = in[0];
		out[2] = triangle;
	}

	void Codec::parseJointTypeInfo(double *tmp, jntType t, std::string& in)
	{
		in = removeWhiteSpace(in);
		switch (t) {
			case ROTATION:
				tmp[0] = atof(in.c_str()) * 3.14 / 180; // Convert to radian
				break;
			case TRANSLATION:
			case FIXED:
				tmp[0] = atof(in.c_str());
				break;
			case FREEFLYER:
				for (int i = 0; i < 6; i++) {
					tmp[i] = atof(removeWhiteSpace(in.substr(0, in.find(" "))).c_str());
					if (in.find(" ") != std::string::npos)
						in = removeWhiteSpace(in.substr(in.find(" ")));
				}
				for (int i = 0; i < 3; i++)
					tmp[i + 3] = tmp[i + 3] * 3.14 / 180; // Convert to radian
				break;
			case PLAN:
			case JNT_TYPE_UNKNOWN:
			default:
				tmp[0] = 0;
				break;
		}
	}

	void Codec::parseBodyInfo(double *params, int *b, geomType g, std::string& n, std::string& in)
	{
		in = removeWhiteSpace(in);
		n = in.substr(0, in.find(" "));
		int i, nbParams = 3;
		switch (g) {
			case BOX:
				nbParams = 3;
				break;
			case CYLINDER:
				nbParams = 2;
				break;
			case SPHERE:
				nbParams = 1;
				break;
			case POLY:
			case GEOM_TYPE_UNKNOWN:
			default:
				nbParams = 0;
				break;
		}
		for (i = 0; i < 3; i++)
			params[i] = 0;
		std::string tmp = removeWhiteSpace(in.substr(in.find(" ")));
		tmp = removeWhiteSpace(tmp);
		for (i = 0; i < nbParams; i++) {
			params[i] = atof(removeWhiteSpace(tmp.substr(0, tmp.find(" "))).c_str());
			if (tmp.find(" ") != std::string::npos)
				tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
		}
		if (tmp.find("P3D") != std::string::npos) {
			for (i = 0; i < NB_BODY_TYPE - 1; i++)
				if (tmp.compare(p3d::bodyTypeName[i]) == 0)
					break;
			if (i >= BODY_TYPE_UNKNOWN)
				i = BODY_TYPE_UNKNOWN;
			*b = i;
		} else {
			*b = BOTH;
		}
	}

	void Codec::parsePositionAndAngles(double *position, double *angles, std::string& in)
	{
		in = removeWhiteSpace(in);
		for (int i = 0; i < 3; i++) {
			position[i] = atof(removeWhiteSpace(in.substr(0, in.find(" "))).c_str());
			if (in.find(" ") != std::string::npos) // Limit race condition
				in = removeWhiteSpace(in.substr(in.find(" ")));
		}
		for (int i = 0; i < 3; i++) {
			angles[i] = atof(removeWhiteSpace(in.substr(0, in.find(" "))).c_str());
			if (in.find(" ") != std::string::npos) // Limit race condition
				in = removeWhiteSpace(in.substr(in.find(" ")));
		}
	}

	bool Codec::parse(std::ifstream *ifs, KinematicChain *kc)
	{
		logger(INFO_LVL, "In p3d::Codec::Parse ...");

		std::string line;
		int nbLignes = 0;
		while (!ifs->eof()) {
			if (ifs->good()) {
				getline(*ifs, line);
			} else {
				logger(ERROR_LVL, "Input file stream has errors !");
				return false;
			}
			// Remove comments
			line = line.substr(0, line.find("#"));
			// Remove empty line
			if (line.size() == 0)
				continue;
			nbLignes ++;


			// TODO : add these tags :
			// * p3d_set_pos_xyz + p3d_set_dof (P3D_FREEFLYER)
			// * p3d_add_desc_(sphere / oval / cone) (GRAPHIC)
			// * p3d_make_body_deformable ?


			/* Joints data */
			if (line.find("p3d_beg_desc_jnt ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr(line.find("_jnt ") + 5));
				int i;
				for (i = 0; i < NB_JNT_TYPE - 1; i++)
					if (tmp.compare(p3d::jntTypeName[i]) == 0)
						break;
				if (i >= JNT_TYPE_UNKNOWN)
					i = JNT_TYPE_UNKNOWN;
				kc->setJoint(i);
				continue;
			}
			if (line.find("p3d_set_name ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				curJnt->setName(removeWhiteSpace(line.substr((line.find("_name ") + 6))));
				continue;
			}
			if (line.find("p3d_set_prev_jnt ") != std::string::npos) {
				int prevJntIdx = atoi(removeWhiteSpace(line.substr((line.find("_jnt ") + 5))).c_str());
				if (prevJntIdx == 0) {
					logger(DEBUG_LVL, "Previous joint is world frame, skipping");
					continue;
				} else if (prevJntIdx > kc->getJointsVector().size()) {
					logger(ERROR_LVL, "Previous joint index is out of bounds !");
				} else {
					Joint *curJnt = kc->getLastJoint();
					curJnt->setParentJoint(kc->getJoint(prevJntIdx - 1));
					if (curJnt->getParentJoint()->getChildLink() == NULL)
						kc->setLink(curJnt->getParentJoint()->getParentLink(),
									curJnt->getParentJoint()->getParentLink()->getName() + "+" + curJnt->getName());
					curJnt->setParentLink(curJnt->getParentJoint()->getChildLink());
					continue;
				}
			}
			if (line.find("p3d_set_pos_relative ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				curJnt->setRelative(true);
			}
			if (line.find("p3d_set_pos_mat ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				std::string value = removeWhiteSpace(line.substr(line.find("mat ") + 4));
				double tmp[16];
				for (int i = 0; i < 16; i++) {
					tmp[i] = atof(removeWhiteSpace(value.substr(0, value.find(" "))).c_str());
					if (value.find(" ") != std::string::npos) {
						value = removeWhiteSpace(value.substr(value.find(" ")));
					}
				}
				curJnt->setPosMat(tmp);
			}
			if (line.find("p3d_set_pos_axe ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				std::string value = removeWhiteSpace(line.substr(line.find("axe ") + 4));
				double tmp[16];
				for (int i = 0; i < 6; i++) {
					tmp[i] = atof(removeWhiteSpace(value.substr(0, value.find(" "))).c_str());
					if (value.find(" ") != std::string::npos) {
						value = removeWhiteSpace(value.substr(value.find(" ")));
					}
				}
				curJnt->setPosAxe(tmp);
				continue;
			}
			if (line.find("p3d_set_dof_vmin ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				std::string vMin = removeWhiteSpace(line.substr((line.find("vmin ") + 5)));
				double tmp[6] = {0, 0, 0, 0, 0, 0};
				parseJointTypeInfo(tmp, curJnt->getType(), vMin);
				curJnt->setMin(tmp);
				continue;
			}
			if (line.find("p3d_set_dof_vmax ") != std::string::npos) {
				Joint *curJnt = kc->getLastJoint();
				std::string vMax = removeWhiteSpace(line.substr((line.find("vmax ") + 5)));
				double tmp[6] = {0, 0, 0, 0, 0, 0};
				parseJointTypeInfo(tmp, curJnt->getType(), vMax);
				curJnt->setMax(tmp);
				continue;
			}

			/* Links data */
			if (line.find("p3d_read_macro ") != std::string::npos) {
				std::string path = removeWhiteSpace(line.substr(line.find("./") + 1, line.find(".macro") + 5 - line.find("./")));
				std::string name = removeWhiteSpace(line.substr(line.find (".macro ") + 7));
				kc->setLink(name);

				if (_dataDir.back() != '/')
					_dataDir += "/";
				if (_dataDir.find(p3d::macroPath) == std::string::npos)
					_dataDir += p3d::macroPath;
				std::string full_path = _dataDir + path;
				logger(DEBUG_LVL, "Input macro file has : name : " + name + 
								"\n                       path : " + path +
								"\n                       full path : " + full_path );
				std::ifstream *mfs = new std::ifstream();
				mfs->open(full_path.c_str(), std::ifstream::in);
				if (!mfs) {
					logger(ERROR_LVL, "Unable to open input macro file: " + full_path);
					return false;
				}
				if (!mfs->is_open())
					logger(ERROR_LVL, "Input macro file not opened !");
				if (!mfs->good())
					logger(ERROR_LVL, "Input macro file stream has some error(s) !");
				_isLink = true;
				this->parse(mfs, kc);
				_isLink = false;
				mfs->close();

				continue;
			}

			/* Bodies specific data */
			if (line.find("p3d_beg_desc ") != std::string::npos) {
				if (line.find("P3D_BODY") != std::string::npos) {
					std::string name = removeWhiteSpace(line.substr(line.find("P3D_BODY ") + 9));
					if (!_isLink)
						kc->setLink(name);
					Link *curLnk = kc->getLastLink();
					curLnk->setBody(name);
				}
			}
			if (line.find("p3d_set_body_abs_pos ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr((line.find("abs_pos ") + 8)));
				int p = tmp.find(".");
				std::string linkName = removeWhiteSpace(tmp.substr(0, p));
				std::string bodyName = removeWhiteSpace(tmp.substr(p + 1, tmp.find(" ") - p));
				std::string posAng = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				double position[3] = {0, 0, 0};
				double angles[3] = {0, 0, 0};
				parsePositionAndAngles(position, angles, posAng);
				Link *curLnk = kc->getLink(linkName);
				if (!curLnk) {
					logger(ERROR_LVL, "Unable to find link with name : " + linkName + " in KinematicChain !");
					continue;
				}
				Body *curBdy = curLnk->getBody(bodyName);
				if(!curBdy) {
					logger(ERROR_LVL, "Unable to find body with name : " + bodyName +
									  " in link " + linkName + " !");
					continue;
				}
				curBdy->setPosition(position);
				curBdy->setAngle(angles);
			}

			/* Basic Geometry Primitives */
			if (line.find("p3d_add_desc_box ") != std::string::npos) {
				Body *curBdy = kc->getLastLink()->getLastBody();
				std::string tmp = removeWhiteSpace(line.substr(line.find("box ") + 4));
				int bt = BODY_TYPE_UNKNOWN;
				double p[3] = {0, 0, 0};
				std::string name = "";
				parseBodyInfo(p, &bt, BOX, name, tmp);
				curBdy->setGeom(name, BOX, bt, p);
			}
			if (line.find("p3d_add_desc_cylindre ") != std::string::npos) {
				Body *curBdy = kc->getLastLink()->getLastBody();
				std::string tmp = removeWhiteSpace(line.substr(line.find("cylindre ") + 9));
				int bt = BODY_TYPE_UNKNOWN;
				double p[3] = {0, 0, 0};
				std::string name = "";
				parseBodyInfo(p, &bt, CYLINDER, name, tmp);
				curBdy->setGeom(name, CYLINDER, bt, p);
			}
			if (line.find("p3d_add_desc_sphere ") != std::string::npos) {
				Body *curBdy = kc->getLastLink()->getLastBody();
				std::string tmp = removeWhiteSpace(line.substr(line.find("sphere ") + 7));
				int bt = BODY_TYPE_UNKNOWN;
				double p[3] = {0, 0, 0};
				std::string name = "";
				parseBodyInfo(p, &bt, SPHERE, name, tmp);
				curBdy->setGeom(name, SPHERE, bt, p);
			}
			if (line.find("p3d_add_desc_cylindre_oval ") != std::string::npos) {
				Body *curBdy = kc->getLastLink()->getLastBody();
				std::string tmp = removeWhiteSpace(line.substr(line.find("oval ") + 5));
				int bt = BODY_TYPE_UNKNOWN;
				double p[3] = {0, 0, 0};
				std::string name = "";
				// Use BOX primitive as oval cylinder has 3 params
				// Then keep the largest radius and keep only 1 radius with the length of the cylinder
				parseBodyInfo(p, &bt, BOX, name, tmp);
				if (p[0] < p[1])
					p[0] = p[1];
				p[1] = p[2];
				p[2] = 0;
				curBdy->setGeom(name, CYLINDER, bt, p);
			}
			if (line.find("p3d_add_desc_oval ") != std::string::npos) {
				Body *curBdy = kc->getLastLink()->getLastBody();
				std::string tmp = removeWhiteSpace(line.substr(line.find("oval ") + 5));
				int bt = BODY_TYPE_UNKNOWN;
				double p[3] = {0, 0, 0};
				std::string name = "";
				// Use BOX primitive as oval has 3 params
				// Then keep the largest radius and create a sphere
				parseBodyInfo(p, &bt, BOX, name, tmp);
				if (p[1] < p[2])
					p[1] = p[2];
				if (p[0] < p[1])
					p[0] = p[1];
				p[1] = p[2] = 0;
				curBdy->setGeom(name, SPHERE, bt, p);
			}
			if (line.find("p3d_set_prim_pos ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr(line.find("prim_pos ") + 9));
				std::string name = removeWhiteSpace(tmp.substr(0, tmp.find(" ")));
				tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				double position[3] = {0, 0, 0};
				double angles[3] = {0, 0, 0};
				parsePositionAndAngles(position, angles, tmp);
				Geometry *curGeom = kc->getLastLink()->getLastBody()->getLastGeom();
				curGeom->setPosition(position);
				curGeom->setAngle(angles);
			}

			/* Polyhedra */
			if (line.find("p3d_add_desc_poly ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr(line.find("poly ") + 5));
				std::string name = removeWhiteSpace(tmp.substr(0, tmp.find(" ")));
				int bt = BODY_TYPE_UNKNOWN;
				if (tmp.find(" ") != std::string::npos) {
					tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
					int i = 0;
					for (i = 0; i < NB_BODY_TYPE; i++) {
						if (tmp.compare(p3d::bodyTypeName[i]) == 0)
							break;
					}
					if (i >= BODY_TYPE_UNKNOWN)
						i = BODY_TYPE_UNKNOWN;
					bt = i;
				} else {
					bt = BOTH;
				}
				double p[3] = {0, 0, 0};
				Body *curBdy = kc->getLastLink()->getLastBody();
				curBdy->setGeom(name, POLY, bt, p);
				
			}
			if (line.find("p3d_add_desc_vert ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr(line.find("vert ") + 5));
				Eigen::Vector3d vertex;
				for (int i = 0; i < 3; i++) {
					vertex[i] = atof(removeWhiteSpace(tmp.substr(0, tmp.find(" "))).c_str());
					if(tmp.find(" ") != std::string::npos)
						tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				}
				Geometry *curPly = kc->getLastLink()->getLastBody()->getLastGeom();
				if(!curPly) {
					logger(ERROR_LVL, "Unable to find last polyhedre of last body in last link !");
					continue;
				}
				curPly->addVertex(vertex);
			}
			if (line.find("p3d_add_desc_face") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr(line.find("face ") + 5));
				Eigen::Vector3i face[3];
				int faceI[4] = {0, 0, 0, 0};
				int i = 0;
				while(tmp.find(" ") != std::string::npos) {
					faceI[i] = atoi(removeWhiteSpace(tmp.substr(0, tmp.find(" "))).c_str());
					tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
					i++;
				}
				faceI[i] = atoi(removeWhiteSpace(tmp).c_str());
				Geometry *curPly = kc->getLastLink()->getLastBody()->getLastGeom();
				if(!curPly) {
					logger(ERROR_LVL, "Unable to find last polyhedre of last body in last link !");
					continue;
				}
				if (i == 3) {
					triangulate(faceI, face);
					curPly->addFace(face[0]);
					curPly->addFace(face[1]);
					curPly->addFace(face[2]);
				} else {
					face[0][0] = faceI[0]; face[0][1] = faceI[1]; face[0][2] = faceI[2];
					curPly->addFace(face[0]);
				}
			}
			if (line.find("p3d_set_body_poly_color ") != std::string::npos) {
				std::string tmp = removeWhiteSpace(line.substr((line.find("color ") + 6)));
				int p = tmp.find(".");
				std::string linkName = removeWhiteSpace(tmp.substr(0, p));
				std::string bodyName = removeWhiteSpace(tmp.substr(p + 1, tmp.find(" ") - p));
				tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				int polyIndex = atoi(removeWhiteSpace(tmp.substr(0, tmp.find(" "))).c_str());
				tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				std::string type = removeWhiteSpace(tmp.substr(0, tmp.find(" ")));
				tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				double color[4] = {0, 0, 0, 1}; // Set alpha to 1 as it is not handled in p3d
				for (int i = 0; i < 3; i++) {
					color[i] = atof(removeWhiteSpace(tmp.substr(0, tmp.find(" "))).c_str());
					if(tmp.find(" ") != std::string::npos)
						tmp = removeWhiteSpace(tmp.substr(tmp.find(" ")));
				}

				Link *curLnk = kc->getLink(linkName);
				if (!curLnk) {
					logger(ERROR_LVL, "Unable to find link with name : " + linkName + " in KinematicChain !");
					continue;
				}
				Body *curBdy = curLnk->getBody(bodyName);
				if(!curBdy) {
					logger(ERROR_LVL, "Unable to find body with name : " + bodyName +
									  " in link " + linkName + " !");
					continue;
				}
				Geometry *curGeom = curBdy->getGeom(polyIndex - 1);
				if(!curGeom) {
					logger(ERROR_LVL, "Unable to find polyhedre : " + std::to_string(polyIndex) +
									  " in body " + bodyName + ", in link " + linkName + " !");
					continue;
				}
				curGeom->setColor(color);
			}
		}

		//kc->setRootJoint(kc->getJointsVector()[0]);
		logger(DEBUG_LVL, std::to_string(nbLignes) + " lignes lues.");
		logger(DEBUG_LVL, std::to_string(kc->getJointsVector().size()) + " joints in kinematic chain");
		logger(DEBUG_LVL, std::to_string(kc->getLinksVector().size()) + " links in kinematic chain");

		return true;
	}

	bool Codec::write(std::ofstream *ofs, KinematicChain *kc, std::string robotName)
	{
		return false;
	}
}