#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstdio>
#include "data.hpp"
#include "p3d-codec.hpp"
#include "urdf-codec.hpp"
#include "logger.hpp"

using namespace std;


bool PRINT_KC = false;


void help()
{
	string call("p3dURDFcodec");
	cout << "Usage : " << call << " [options]" << endl
			  << endl
			  << call << " is intended to convert P3D and URDF files, from one to another" << endl
			  << "The current implementation converts from P3D to URDF." << endl
			  << "For details on URDF model, see http://wiki.ros.org/urdf" << endl
			  << endl
			  << "Options :" << endl
			  << "    -h : print this help message" << endl
			  << "    -d : Select log level in ['FATAL', 'ERROR', 'WARNING', 'INFO', ('DEBUG')]" << endl
			  << "    -w : Define the conversion way [('P3D2URDF'), 'URDF2P3D'] -- Warning: currently only P3D2URDF (default) is working" << endl
			  << "    -o : Define output path (relative to current folder)" << endl
			  << "    -i : Define input file (REQUESTED)" << endl
			  << "    -k : Special output to print the kinematic chain data struct (no argument)" << endl
			  << endl
			  << "WARNING : the current implementation does not handle FREEFLYER joints properly."
			  << "          They will simply be squized off."
			  << endl;
			  exit(0);
}

void p3d2urdf(char *ip, char *op)
{
	p3d::Codec *p3dCodec = new p3d::Codec();
	KinematicChain *kc = new KinematicChain();
	string inputFile = string(ip);
	if (inputFile.find(".macro") == string::npos) {
		logger(ERROR_LVL, "Input file is not a '.macro' file (P3D)");
		help();
		exit(-1);
	}

	string robName = "robot";
	if (inputFile.find("/") != string::npos) {
		robName = inputFile.substr(inputFile.rfind("/") + 1);
		robName = robName.substr(0, robName.find(".macro"));
	}

	string env = getenv("ROBOTPKG_BASE");
	if (env.c_str() == NULL) {
		logger(WARNING_LVL, "Your ROBOTPKG_BASE env varibale is not defined, data_dir is set to ./");
		env = "./";
	}
	p3dCodec->setDataDir(env);

	ifstream *ifs = new ifstream();
	logger(INFO_LVL, "Open '" + inputFile + "' MACRO file in read mode for robot " + robName + " ...");
	ifs->open(inputFile.c_str(), ifstream::in);
	if (!ifs) {
		logger(ERROR_LVL, "Unable to open input file: " + inputFile);
		exit(-1);
	}
	p3dCodec->parse(ifs, kc);
	ifs->close();
	logger(INFO_LVL, "Close input file (" + inputFile + ") !");

	if (PRINT_KC)
		cout << *kc << endl;

	urdf::Codec *urdfCodec = new urdf::Codec();
	string outputFile = string(op);
	logger(INFO_LVL, "Write output URDF in '" + outputFile + "' for robot " + robName + " ...");
	urdfCodec->write(outputFile, kc, robName);



	/* Create wavefront file for each body which has polyhedron */
	// struct stat st = {0};
	// if (stat("./tmp", &st) == -1)
	// 	mkdir("./tmp", 0777);
	// for (int l = 0; l < kc->getLinksVector().size(); l++) {
	// 	for (int b = 0; b < kc->getLink(l)->getBodiesVector().size(); b++) {
	// 		kc->getLink(l)->getBody(b)->convertPolysToWavefront();
	// 	}
	// }

	// urdf::Writer *writer = new urdf::Writer();
	// string outputFile = inputFile.substr(0, inputFile.find(".macro"));
	// outputFile += ".urdf";
	// outputFile = op + outputFile;
	// logger(INFO_LVL, "Open '" + outputFile + "' URDF file in write mode ...");
	// ofstream *ofs = new ofstream();
	// WARNING: output file opened WITHOUT append, will be erased.
	// Add ofstream::app for append operation
	// ofs->open(outputFile.c_str(), ofstream::out);
	// if (!ofs) {
	// 	logger(ERROR_LVL, "Unable to open output file: " + outputFile);
	// 	return -1;
	// }
	// writeOutput(ofs);
	// ofs->close();
	// logger(INFO_LVL, "Close output file (" + outputFile + ") !");
	
	// for (int l = 0; l < kc->getLinksVector().size(); l++) {
	// 	for (int b = 0; b < kc->getLink(l)->getBodiesVector().size(); b++) {
	// 		remove((kc->getLink(l)->getBody(b)->getWavefrontPath()).c_str());
	// 	}
	// }
	// if (stat("./tmp", &st) == 0)
	// 	remove("./tmp");
	logger(INFO_LVL, "Finish, everything went fine !");

}

void urdf2p3d(char *ip, char *op)
{
	// TODO : similar to p3d2urdf
}

int main(int argc,char** argv)
{
	if (argc < 2){
		cout << argv[0] << ": ERROR : too few arguments" << endl;
		cout << endl << "Try '" << argv[0] << " -h' for more information." << endl << endl;
		return -1;
	}

	int c;
	char *dbg = NULL;
	char *way = NULL;
	char *op = NULL;
	char *ip = NULL;
	while ((c = getopt(argc, argv, "hd:o:i:w:k")) != -1) {
		switch (c) {
			case 'h':
				help();
				break;
			case 'd':
				dbg = optarg;
				break;
			case 'w':
				way = optarg;
				break;
			case 'o':
				op = optarg;
				break;
			case 'i':
				ip = optarg;
				break;
			case 'k':
				PRINT_KC = true;
				break;
			default:
				break;
		} 
	}

	if (ip == NULL) {
		logger(FATAL_LVL, "Need to specify at least input file (.macro)");
		exit(-1);
	}

	if (op == NULL) {
		logger(WARNING_LVL, "No output path specified, output files will be stored in current folder");
		op = (char*)"./";
	}

	if (dbg != NULL)
		for (int i = 0; i < NB_LVL; i++)
			if (string(dbg).compare(LOGGER_LEVELS_NAME[i]) == 0)
				LOGGER_LEVEL = (LOGGER_LEVELS)i;

	if (way != NULL) {
		string codec(way);
		if (codec.compare("P3D2URDF") == 0) {
			p3d2urdf(ip, op);
		} else if (codec.compare("URDF2P3D") == 0) {
			urdf2p3d(ip, op);
		} else {
			logger(ERROR_LVL, "Wrong way selected !");
			help();
			exit(-1);
		}
	} else {
		p3d2urdf(ip, op);
	}

	logger(INFO_LVL, "Successfully parse and convert file !");
	return 0;
}