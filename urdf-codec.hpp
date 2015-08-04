#ifndef URDF_CODEC_HPP
#define URDF_CODEC_HPP

#include <iostream>
#include <fstream>
#include <tinyxml2.h>
#include "data.hpp"
#include "logger.hpp"

namespace urdf {
	// Should add continuous (revolute without limit) for completion
	const std::string jntTypeName[NB_JNT_TYPE] = {"revolute", "prismatic", "fixed", "floating", "planar", "UNKNONW"};
	const std::string bodyTypeName[NB_BODY_TYPE] = {"visual", "collision", "both", "UNKNOWN"};
	const std::string geomTypeName[NB_GEOM_TYPE] = {"box", "cylinder", "sphere", "mesh", "UNKNOWN"};

	class Codec
	{
	public:
		bool parse(std::ifstream *ifs, KinematicChain *kc);
		bool write(std::string outputPath, KinematicChain *kc, std::string robotName);
	private:
		void insertLink(tinyxml2::XMLDocument *d, Link *l);
		void insertJoint(tinyxml2::XMLDocument *d, Joint *j);
		void convertToCollada(Geometry *g, std::string path);
		void insertBodyElement(tinyxml2::XMLDocument *d, tinyxml2::XMLElement *e, Geometry *g,
							   std::string prefix);

		std::string _colladaDir;
		bool _hasPoly;

	};
}

#endif /* URDF_CODEC_HPP */