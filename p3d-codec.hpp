#ifndef P3D_CODEC_HPP
#define P3D_CODEC_HPP

#include <iostream>
#include <fstream>
#include "data.hpp"
#include "logger.hpp"

namespace p3d {
	typedef enum {
		FRONT = 0,
		BACK,
		BOTH_WAY
	} whiteSpaceRemovalWay;

	const std::string jntTypeName[NB_JNT_TYPE] = {"P3D_ROTATE", "P3D_TRANSLATE", "P3D_FIXED", "P3D_FREEFLYER", "P3D_PLAN", "UNKNONW"};
	const std::string bodyTypeName[NB_BODY_TYPE] = {"P3D_GRAPHIC", "P3D_GHOST", "P3D_BOTH", "UNKNOWN"};
	const std::string geomTypeName[NB_GEOM_TYPE] = {"BOX", "CYLINDER", "SPHERE", "POLY", "UNKNOWN"};
	const std::string macroPath = "share/move3d/assets/ADREAM/MACROS";

	class Codec
	{
	public:
		bool parse(std::ifstream *ifs, KinematicChain *kc);
		bool write(std::ofstream *ofs, KinematicChain *kc, std::string robotName);
		void setDataDir(std::string path) { _dataDir = path; }
		std::string getDataDir() { return _dataDir; }

	private:
		void parseJointTypeInfo(double *tmp, jntType t, std::string& in);
		void parseBodyInfo(double *params, int *b, geomType g, std::string& n, std::string& in);
		void parsePositionAndAngles(double *position, double *angles, std::string& in);
		void triangulate(int *in, Eigen::Vector3i *out);
		std::string removeWhiteSpace(const std::string& str, const int w = BOTH_WAY);

		bool _isLink;
		std::string _dataDir;
	};
}

#endif /* P3D_CODEC_HPP */