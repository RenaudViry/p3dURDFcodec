#include <scene.h>
#include <mesh.h>
#include <vector3.h>
#include <IOSystem.hpp>
#include <Exporter.hpp>
#include <postprocess.h>
#include "urdf-codec.hpp"

using namespace std;
using namespace tinyxml2;

namespace urdf {
	void Codec::convertToCollada(Geometry *g, string path)
	{
		aiScene *scene = new aiScene();
		scene->mRootNode = new aiNode();

		scene->mMaterials = new aiMaterial*[1];
	    scene->mMaterials[0] = new aiMaterial();
	    scene->mNumMaterials = 1;

		aiMesh *mesh = new aiMesh();
		mesh->mVertices = new aiVector3D[g->getVertices().size()];
		mesh->mNumVertices = 0;
		for (int v = 0; v < g->getVertices().size(); v++) {
			aiVector3D vertex(g->getVertex(v)[0], g->getVertex(v)[1], g->getVertex(v)[2]);
			mesh->mVertices[v] = vertex;
			mesh->mNumVertices++;
		}
		mesh->mFaces = new aiFace[g->getFaces().size()];
		mesh->mNumFaces = 0;
		for (int f = 0; f < g->getFaces().size(); f++) {
			aiFace& face = mesh->mFaces[f];

            face.mIndices = new unsigned int[3];
            face.mNumIndices = 3;

            face.mIndices[0] = g->getFace(f)[0];
            face.mIndices[1] = g->getFace(f)[1];
            face.mIndices[2] = g->getFace(f)[2];
			mesh->mNumFaces++;
		}
		mesh->mMaterialIndex = 0;
		scene->mMeshes = new aiMesh*[1];
        scene->mMeshes[0] = mesh;
        scene->mNumMeshes = 1;

        scene->mRootNode->mMeshes = new unsigned int[1];
        scene->mRootNode->mMeshes[0] = 0;
        scene->mRootNode->mNumMeshes = 1;

        Assimp::Exporter *exporter = new Assimp::Exporter();
        int i = 0;
        string collada = "collada";
        ifstream ifs(path.c_str());
        if(ifs && ifs.good()) {
			logger(INFO_LVL, "Collada file " + path + " already exists, skipping!");
			return;
		}
        for (i = 0; i < exporter->GetExportFormatCount(); i++) {
        	if (collada.compare(exporter->GetExportFormatDescription(i)->id) == 0) {
        		logger(INFO_LVL, "Exporting " + g->getName() + " to collada in " + path + "!");
        		break;
        	}
        }

        aiReturn ret = exporter->Export(scene, exporter->GetExportFormatDescription(i)->id, path);
        if (ret != AI_SUCCESS) {
        	logger(ERROR_LVL, "Export of " + g->getName() + " to collada in " + path + " FAILED !");
        	logger(ERROR_LVL, "    " + string(exporter->GetErrorString()));
        }
        delete scene;
        delete exporter;
	}

	void Codec::insertBodyElement(XMLDocument *d, XMLElement *e, Geometry *g, string prefix) {
		if (g == NULL)
			return;

		XMLElement *eOrigin = d->NewElement("origin");
		string pos = to_string(g->getPosition()[0]) + " " +
					 to_string(g->getPosition()[1]) + " " +
					 to_string(g->getPosition()[2]);
		eOrigin->SetAttribute("xyz", pos.c_str());
		string angle = to_string(g->getAngle()[0]) + " " +
					   to_string(g->getAngle()[1]) + " " +
					   to_string(g->getAngle()[2]);
		eOrigin->SetAttribute("rpy", angle.c_str());
		e->InsertEndChild(eOrigin);

		XMLElement *eGeom = d->NewElement("geometry");
		XMLElement *eGeomMesh = d->NewElement("");
		switch(g->getGeomType()) {
			case BOX: {
				eGeomMesh->SetName("box");
				string size = to_string(g->getParams()[0]) + " " +
							  to_string(g->getParams()[1]) + " " +
							  to_string(g->getParams()[2]);
				eGeomMesh->SetAttribute("size", size.c_str());
				break;
			} case CYLINDER: {
				eGeomMesh->SetName("cylinder");
				eGeomMesh->SetAttribute("radius", to_string(g->getParams()[0]).c_str());
				eGeomMesh->SetAttribute("length", to_string(g->getParams()[1]).c_str());
				break;
			} case SPHERE: {
				eGeomMesh->SetName("sphere");
				eGeomMesh->SetAttribute("radius", to_string(g->getParams()[0]).c_str());
				break;
			} case POLY: {
				eGeomMesh->SetName("mesh");
				string colladaPath = _colladaDir + prefix + "-" + g->getName() + ".dae";
				convertToCollada(g, colladaPath);
				eGeomMesh->SetAttribute("filename", colladaPath.c_str());
				break;
			} case GEOM_TYPE_UNKNOWN: {
			default:
				logger(WARNING_LVL, "Unable to handle undefined geometry type in URDF !");
				break;
			}
		}
		eGeom->InsertEndChild(eGeomMesh);
		e->InsertEndChild(eGeom);
	}

	void Codec::insertLink(XMLDocument *d, Link *l)
	{
		XMLElement *rob = d->RootElement();
		XMLElement *link = d->NewElement("link");
		link->SetAttribute("name", (l->getName()).c_str());

		for (int b = 0; b < l->getBodiesVector().size(); b++) {
			Body *curBdy = l->getBody(b);
			vector<Geometry*> visualGeom = curBdy->getGeomOfBodyType(GRAPHIC);
			for (int v = 0; v < visualGeom.size(); v++) {
				XMLElement *lVis = d->NewElement("visual");
				insertBodyElement(d, lVis, visualGeom[v], curBdy->getName());

				XMLElement *lVisMat = d->NewElement("material");
				lVisMat->SetAttribute("name", (curBdy->getName() + "_" + visualGeom[v]->getName()).c_str());
				XMLElement *lVisMatCol = d->NewElement("color");
				string color = to_string(visualGeom[v]->getColor()[0]) + " " +
							   to_string(visualGeom[v]->getColor()[1]) + " " +
							   to_string(visualGeom[v]->getColor()[2]) + " " +
							   to_string(visualGeom[v]->getColor()[3]);
				lVisMatCol->SetAttribute("rgba", color.c_str());
				lVisMat->InsertEndChild(lVisMatCol);
				lVis->InsertEndChild(lVisMat);
				link->InsertEndChild(lVis);
			}

			vector<Geometry*> colGeom = curBdy->getGeomOfBodyType(COLLISION);
			for (int c = 0; c < colGeom.size(); c++) {
				XMLElement *lCol = d->NewElement("collision");
					insertBodyElement(d, lCol, colGeom[c], curBdy->getName());
				link->InsertEndChild(lCol);
			}
		}
		rob->InsertEndChild(link);
	}

	void Codec::insertJoint(XMLDocument *d, Joint *j)
	{
		XMLElement *rob = d->RootElement();
		XMLElement *joint = d->NewElement("joint");
		joint->SetAttribute("name", (j->getName()).c_str());
		joint->SetAttribute("type", (jntTypeName[j->getType()]).c_str());

		XMLElement *jOrigin = d->NewElement("origin");
		string pos = to_string(j->getPosition()[0]) + " " +
					 to_string(j->getPosition()[1]) + " " +
					 to_string(j->getPosition()[2]);
		jOrigin->SetAttribute("xyz", pos.c_str());
		jOrigin->SetAttribute("rpy", "0 0 0");
		joint->InsertEndChild(jOrigin);

		XMLElement *jAxis = d->NewElement("axis");
		string axis = to_string(j->getAxis()[0]) + " " +
					  to_string(j->getAxis()[1]) + " " +
					  to_string(j->getAxis()[2]);
		jAxis->SetAttribute("xyz", axis.c_str());
		joint->InsertEndChild(jAxis);

		if (j->getParentLink() != NULL) {
			XMLElement *jParent = d->NewElement("parent");
			jParent->SetAttribute("link", (j->getParentLink()->getName()).c_str());
			joint->InsertEndChild(jParent);
		}

		if (j->getChildLink() != NULL) {
			XMLElement *jChild = d->NewElement("child");
			jChild->SetAttribute("link", (j->getChildLink()->getName()).c_str());
			joint->InsertEndChild(jChild);
		}

		XMLElement *jLimit = d->NewElement("limit");
		jLimit->SetAttribute("effort", "0");
		string min;
		switch (j->getType()) {
			case ROTATION:
			case TRANSLATION:
				min = to_string(j->getMin()[0]);
				break;
			case FIXED:
				min = "0";
				break;
			case FREEFLYER:
				min = to_string(j->getMin()[0]) + " " +
					  to_string(j->getMin()[1]) + " " +
					  to_string(j->getMin()[2]) + " " +
					  to_string(j->getMin()[3]) + " " +
					  to_string(j->getMin()[4]) + " " +
					  to_string(j->getMin()[5]);
			case PLAN:
			case JNT_TYPE_UNKNOWN:
			default:
				min = "0";
				break;
		}
		jLimit->SetAttribute("lower", min.c_str());
		string max;
		switch (j->getType()) {
			case ROTATION:
			case TRANSLATION:
				max = to_string(j->getMax()[0]);
				break;
			case FIXED:
				max = "0";
				break;
			case FREEFLYER:
				max = to_string(j->getMax()[0]) + " " +
					  to_string(j->getMax()[1]) + " " +
					  to_string(j->getMax()[2]) + " " +
					  to_string(j->getMax()[3]) + " " +
					  to_string(j->getMax()[4]) + " " +
					  to_string(j->getMax()[5]);
			case PLAN:
			case JNT_TYPE_UNKNOWN:
			default:
				max = "0";
				break;
		}
		jLimit->SetAttribute("upper", max.c_str());
		jLimit->SetAttribute("velocity", "0");
		joint->InsertEndChild(jLimit);

		rob->InsertEndChild(joint);
	}

	bool Codec::parse(ifstream *ifs, KinematicChain *kc)
	{
		return false;
	}

	bool Codec::write(string outputPath, KinematicChain *kc, string robotName)
	{
		_hasPoly = false;
		XMLDocument *doc = new XMLDocument();
		XMLDeclaration *decl = doc->NewDeclaration();
		doc->InsertEndChild(decl);
		XMLElement *rob = doc->NewElement("robot");
		rob->SetAttribute("xmlns:xacro", "http://ros.org/wiki/xacro");
		rob->SetAttribute("name", robotName.c_str());
		doc->InsertEndChild(rob);

		_colladaDir = outputPath;
		if (outputPath.back() != '/')
			_colladaDir += "/";
		_colladaDir += "collada/";

		/* Insert all links */
		for (int l = 0; l < kc->getLinksVector().size(); l++) {
			insertLink(doc, kc->getLink(l));
		}

		/* Insert all joints */
		for (int j = 0; j < kc->getJointsVector().size(); j++) {
			insertJoint(doc, kc->getJoint(j));
		}

		if (outputPath.back() != '/')
			outputPath += "/";
		outputPath += robotName + ".urdf";
		XMLError err = doc->SaveFile(outputPath.c_str());

		return true;
	}
}