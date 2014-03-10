//---------------------------------------------------------------------
//
// Copyright © 2011, Jason Gedge <gedge -at- ualberta -dot- ca>
//
// This file is part of StereoReconstruction.
//
// StereoReconstruction is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// StereoReconstruction is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with StereoReconstruction. If not, see <http:www.gnu.org/licenses/>.
//
//---------------------------------------------------------------------
#include "project.hpp"

#include <QtXml>
#include <QtXmlPatterns>

#include "features/surf.hpp"
#include "features/checkerboard.hpp"

#include "camera.hpp"
#include "imageset.hpp"
#include "projectimage.hpp"

//---------------------------------------------------------------------

using namespace Eigen;
using std::fabs;

//---------------------------------------------------------------------
// XML schemas and validators
namespace {
	const QString PROJECT_NS = QString();
	const QString SCHEMA_PATH = QString(":/project_schema");

    QXmlSchema & getSchema() {
        static QXmlSchema schema;
        return schema;
    }

    QXmlSchemaValidator & getValidator() {
        static QXmlSchemaValidator validator;
        return validator;
    }
}
//---------------------------------------------------------------------
// Utility functions
bool hasAttribute(const QDomNode &node, const QString &attr) {
	return (node.hasAttributes() && node.attributes().contains(attr));
}

QString getAttribute(const QDomNode &node, const QString &attr, const QString &defaultValue = QString()) {
	return (node.attributes().contains(attr)
			? node.attributes().namedItem(attr).nodeValue()
			: defaultValue);
}

QDomElement createSimpleElement(QDomDocument *doc, QString elementName, double val) {
	QDomElement valueNode = doc->createElement(elementName);
	valueNode.appendChild(doc->createTextNode(QString::number(val, 'f', 4)));
	return valueNode;
}

//---------------------------------------------------------------------

Project::Project(QString projectPath)
	: projectPath_(projectPath)
{
	if(projectPath.isNull())
		return;

    auto &schema = getSchema();
    auto &validator = getValidator();

    if(!schema.isValid()) {
		QFile schemaFile(SCHEMA_PATH);
		if(schemaFile.open(QFile::ReadOnly) && schema.load(&schemaFile))
			validator.setSchema(schema);
	}

	// First make sure the XML is valid w.r.t. our schema
	if(!validator.validate(QUrl::fromLocalFile(projectPath)))
		throw std::runtime_error("Failed to validate");

	// Load the file into a DOM model
	QDomDocument doc;
	QFile xmlFile(projectPath);
	if(!xmlFile.open(QFile::ReadOnly))
		throw std::runtime_error("Failed to open file");

	if(!doc.setContent(&xmlFile)) {
		xmlFile.close();
		throw std::runtime_error("Failed to set XML content");
	}
	xmlFile.close();

	cameras_.clear();
	imageSets_.clear();

	//
	// Cameras
	//
	qDebug() << "Loading cameras...";

	QDomNode camerasNode = doc.documentElement().firstChildElement("cameras");
	QDomNode cameraNode = camerasNode.firstChildElement("camera");
	while(!cameraNode.isNull()) {
		CameraPtr cam(new Camera(getAttribute(cameraNode, "id")));
		cam->setName(getAttribute(cameraNode, "name", cam->id()));

		// The projection matrix node is required, so no need to check for null
		QDomNode projectionMatrixNode = cameraNode.firstChildElement("projectionMatrix");
		if(!projectionMatrixNode.isNull()) {
			ProjMat P;
			P << getAttribute(projectionMatrixNode, "m11").toDouble(),
			     getAttribute(projectionMatrixNode, "m12").toDouble(),
			     getAttribute(projectionMatrixNode, "m13").toDouble(),
			     getAttribute(projectionMatrixNode, "m14").toDouble(),
			     getAttribute(projectionMatrixNode, "m21").toDouble(),
			     getAttribute(projectionMatrixNode, "m22").toDouble(),
			     getAttribute(projectionMatrixNode, "m23").toDouble(),
			     getAttribute(projectionMatrixNode, "m24").toDouble(),
			     getAttribute(projectionMatrixNode, "m31").toDouble(),
			     getAttribute(projectionMatrixNode, "m32").toDouble(),
			     getAttribute(projectionMatrixNode, "m33").toDouble(),
			     getAttribute(projectionMatrixNode, "m34").toDouble();

			cam->setP(P);
		}

		//
		QDomNode lensDistortionNode = cameraNode.firstChildElement("lensDistortion");
		if(!lensDistortionNode.isNull()) {
			LensDistortions dist;
			dist[0] = getAttribute(lensDistortionNode, "k1", "0").toDouble();
			dist[1] = getAttribute(lensDistortionNode, "k2", "0").toDouble();
			dist[2] = getAttribute(lensDistortionNode, "p1", "0").toDouble();
			dist[3] = getAttribute(lensDistortionNode, "p2", "0").toDouble();
			dist[4] = getAttribute(lensDistortionNode, "k3", "0").toDouble();
			cam->setLensDistortion(dist);
		}

		//
		QDomNode responseNode = cameraNode.firstChildElement("response");
        Responses responses(256, Response::Zero());
		while(!responseNode.isNull()) {
			QString channelName = getAttribute(responseNode, "channel", QString());
			if(!channelName.isNull()) {
				int channel = -1;
				if(channelName == "red") channel = 0;
				else if(channelName == "green") channel = 1;
				else if(channelName == "blue") channel = 2;

				if(channel >= 0) {
					QDomNode valueNode = responseNode.firstChild();
					for(int index = 0; index < 256; ++index, valueNode = valueNode.nextSibling())
						responses[index][channel] = valueNode.firstChild().toText().data().toDouble();
				}
			}
			responseNode = responseNode.nextSiblingElement("response");
		}
		cam->setResponse(responses);

		//
		QDomNode refractiveNode = cameraNode.firstChildElement("refractiveInterface");
		if(!refractiveNode.isNull()) {
			double px = getAttribute(refractiveNode, "px", "0.0").toDouble();
			double py = getAttribute(refractiveNode, "py", "0.0").toDouble();
			double dist = getAttribute(refractiveNode, "dist", "0.0").toDouble();

			cam->setRefractiveIndex(getAttribute(refractiveNode, "refractiveRatio", "1.0").toDouble());
			cam->setPlane(Plane3d(cam->K().inverse() * Vector3d(px, py, 1), dist));
		}

		//
		cameras_[cam->id()] = cam;
		cameraNode = cameraNode.nextSiblingElement("camera");
	}

	//
	// Image sets
	//
	qDebug() << "Loading image sets...";

	QDomNode imageSetsNode = doc.documentElement().firstChildElement("imageSets");
	QDomNode imageSetNode = imageSetsNode.firstChildElement("imageSet");
	while(!imageSetNode.isNull()) {
		ImageSetPtr imageSet(new ImageSet(getAttribute(imageSetNode, "id")));
		imageSet->setName(getAttribute(imageSetNode, "name", imageSet->id()));

		QDir root(projectPath);
		root.cdUp();
		if(hasAttribute(imageSetNode, "root")) {
			QString rootStr = getAttribute(imageSetNode, "root");
			if(QDir::isRelativePath(rootStr))
				root.cd(rootStr);
			else
				root.setPath(rootStr);
		}
		imageSet->setRoot(root);

		// Images
		QDomNode imageNode = imageSetNode.firstChild();
		while(!imageNode.isNull()) {
			ProjectImagePtr image(new ProjectImage(imageSet->root().absoluteFilePath(getAttribute(imageNode, "file"))));
			image->setExposure(getAttribute(imageNode, "exposure", "-1.0").toDouble());

			if(cameras_.contains(getAttribute(imageNode, "for")))
				imageSet->addImageForCamera(cameras_[getAttribute(imageNode, "for")], image);

			imageNode = imageNode.nextSibling();
		}

		//
		if(imageSet->images().size() > 0)
			imageSets_[imageSet->id()] = imageSet;
		imageSetNode = imageSetNode.nextSiblingElement("imageSet");
	}

	//
	// Features
	//
	qDebug() << "Loading features...";

	std::map<ProjectImagePtr, std::map<int, FeaturePtr> > featureMap;

	QDomNode featureSetsNode = doc.documentElement().firstChildElement("featureSets");
	QDomNode featureSetNode = featureSetsNode.firstChildElement("featureSet");
	while(!featureSetNode.isNull()) {
		//
		CameraPtr cam = cameras_[getAttribute(featureSetNode, "camera")];
		ImageSetPtr imageSet = imageSets_[getAttribute(featureSetNode, "imageSet")];
		ProjectImagePtr image;
		if(cam && imageSet)
			image = imageSet->defaultImageForCamera(cam);

		//
		if(image) {
			Features features;

			QDomElement featureNode = featureSetNode.firstChildElement("feature");
			while(!featureNode.isNull()) {
				QString type = getAttribute(featureNode, "type");

				FeaturePtr feature;
				if(type.compare("surf", Qt::CaseInsensitive) == 0) {
					feature = SurfDetector::load(featureNode);
				} else if(type.compare("checkerboard", Qt::CaseInsensitive) == 0) {
					feature = CheckerboardDetector::load(featureNode);
				}

				// Index mapping is necessary for establishing correspondence
				// relationships later
				int index = getAttribute(featureNode, "index").toInt();
				featureMap[image][index] = feature;

				//
				features.push_back(feature);
				featureNode = featureNode.nextSiblingElement("feature");
			}

			CameraPtr cam = cameras_[getAttribute(featureSetNode, "camera")];
			ImageSetPtr imageSet = imageSets_[getAttribute(featureSetNode, "imageSet")];
			ProjectImagePtr image = imageSet->defaultImageForCamera(cam);

			//
			if(features.size() > 0)
				featuresDB_.features(image).swap(features);
		}

		//
		featureSetNode = featureSetNode.nextSiblingElement("featureSet");
	}

	//
	// Correspondences
	//
	qDebug() << "Loading correspondences...";

	QDomNode correspondenceSetsNode = doc.documentElement().firstChildElement("correspondenceSets");
	QDomNode correspondenceSetNode = correspondenceSetsNode.firstChildElement("correspondenceSet");

	Correspondences correspondences;
	while(!correspondenceSetNode.isNull()) {
		//
		CameraPtr cam1 = cameras_[getAttribute(correspondenceSetNode, "camera1")];
		CameraPtr cam2 = cameras_[getAttribute(correspondenceSetNode, "camera2")];
		ImageSetPtr imageSet1 = imageSets_[getAttribute(correspondenceSetNode, "imageSet1")];
		ImageSetPtr imageSet2 = imageSets_[getAttribute(correspondenceSetNode, "imageSet2")];

		ProjectImagePtr image1, image2;
		if(cam1 && imageSet1) image1 = imageSet1->defaultImageForCamera(cam1);
		if(cam2 && imageSet2) image2 = imageSet2->defaultImageForCamera(cam2);

		//
		if(image1 && image2) {
			correspondences.clear();
			correspondences.reserve(correspondenceSetNode.childNodes().size());

			QDomElement correspondenceNode = correspondenceSetNode.firstChildElement("correspondence");
			while(!correspondenceNode.isNull()) {
				int index1 = getAttribute(correspondenceNode, "index1", "-1").toInt();
				int index2 = getAttribute(correspondenceNode, "index2", "-1").toInt();
				if(index1 >= 0 && index2 >= 0) {
					FeaturePtr feature1 = featureMap[image1][index1];
					FeaturePtr feature2 = featureMap[image2][index2];
					if(feature1 && feature2)
						correspondences.push_back( Correspondence(feature1, feature2) );
				}
				correspondenceNode = correspondenceNode.nextSiblingElement("correspondence");
			}

			if(correspondences.size() > 0)
				featuresDB_.correspondences(image1, image2).swap(correspondences);
		}

		correspondenceSetNode = correspondenceSetNode.nextSiblingElement("correspondenceSet");
	}
}

//---------------------------------------------------------------------

QDomDocument *Project::toXML() {
	QDomDocument *doc = new QDomDocument;

	QDomElement projectNode = doc->createElement("project");

	//
	// Cameras
	//
	QDomElement camerasNode = doc->createElement("cameras");
	foreach(CameraPtr cam, cameras_) {
		QDomElement cameraNode = doc->createElement("camera");

		cameraNode.setAttribute("id", cam->id());
		if(!cam->name().isEmpty() && cam->name() != cam->id())
			cameraNode.setAttribute("name", cam->name());

		// The projection matrix node is required, so no need to check for null
		QDomElement projectionMatrixNode = doc->createElement("projectionMatrix");
		projectionMatrixNode.setAttribute("m11", cam->P()(0, 0));
		projectionMatrixNode.setAttribute("m12", cam->P()(0, 1));
		projectionMatrixNode.setAttribute("m13", cam->P()(0, 2));
		projectionMatrixNode.setAttribute("m14", cam->P()(0, 3));
		projectionMatrixNode.setAttribute("m21", cam->P()(1, 0));
		projectionMatrixNode.setAttribute("m22", cam->P()(1, 1));
		projectionMatrixNode.setAttribute("m23", cam->P()(1, 2));
		projectionMatrixNode.setAttribute("m24", cam->P()(1, 3));
		projectionMatrixNode.setAttribute("m31", cam->P()(2, 0));
		projectionMatrixNode.setAttribute("m32", cam->P()(2, 1));
		projectionMatrixNode.setAttribute("m33", cam->P()(2, 2));
		projectionMatrixNode.setAttribute("m34", cam->P()(2, 3));
		cameraNode.appendChild(projectionMatrixNode);

		//
		QDomElement lensDistortionNode = doc->createElement("lensDistortion");
		if(qAbs(cam->lensDistortion()[0]) > 1e-10)
			lensDistortionNode.setAttribute("k1", cam->lensDistortion()[0]);

		if(qAbs(cam->lensDistortion()[1]) > 1e-10)
			lensDistortionNode.setAttribute("k2", cam->lensDistortion()[1]);

		if(qAbs(cam->lensDistortion()[2]) > 1e-10)
			lensDistortionNode.setAttribute("p1", cam->lensDistortion()[2]);

		if(qAbs(cam->lensDistortion()[3]) > 1e-10)
			lensDistortionNode.setAttribute("p2", cam->lensDistortion()[3]);

		if(qAbs(cam->lensDistortion()[4]) > 1e-10)
			lensDistortionNode.setAttribute("k3", cam->lensDistortion()[4]);

		if(lensDistortionNode.attributes().size() > 0)
			cameraNode.appendChild(lensDistortionNode);

		//
		if(cam->response().size() == 256) {
			QDomElement redResponseCurveNode = doc->createElement("response");
			QDomElement greenResponseCurveNode = doc->createElement("response");
			QDomElement blueResponseCurveNode = doc->createElement("response");

			redResponseCurveNode.setAttribute("channel", "red");
			greenResponseCurveNode.setAttribute("channel", "green");
			blueResponseCurveNode.setAttribute("channel", "blue");

			foreach(const Response &response, cam->response()) {
				if(fabs(response[0]) > 1e-10)
					redResponseCurveNode.appendChild( createSimpleElement(doc, "value", response[0]) );

				if(fabs(response[1]) > 1e-10)
					greenResponseCurveNode.appendChild( createSimpleElement(doc, "value", response[1]) );

				if(fabs(response[2]) > 1e-10)
					blueResponseCurveNode.appendChild( createSimpleElement(doc, "value", response[2]) );
			}

			if(redResponseCurveNode.childNodes().size() > 0) cameraNode.appendChild(redResponseCurveNode);
			if(greenResponseCurveNode.childNodes().size() > 0) cameraNode.appendChild(greenResponseCurveNode);
			if(blueResponseCurveNode.childNodes().size() > 0) cameraNode.appendChild(blueResponseCurveNode);
		}

		if(fabs(cam->refractiveIndex() - 1) > 1e-10 && fabs(cam->plane().distance()) > 1e-10) {
			Vector3d p = cam->K() * cam->plane().normal();
			p /= p[2];

			QDomElement interfaceNode = doc->createElement("refractiveInterface");
			interfaceNode.setAttribute("px", QString("%1").arg(p[0]));
			interfaceNode.setAttribute("py", QString("%1").arg(p[1]));
			interfaceNode.setAttribute("dist", QString("%1").arg(cam->plane().distance()));
			interfaceNode.setAttribute("refractiveRatio", QString("%1").arg(cam->refractiveIndex()));
			cameraNode.appendChild(interfaceNode);
		}

		camerasNode.appendChild(cameraNode);
	}

	//
	// Image sets
	//
	QDir projectDir(projectPath_);
	projectDir.cdUp();

	QDomElement imageSetsNode = doc->createElement("imageSets");
	foreach(ImageSetPtr imageSet, imageSets_) {
		QDomElement imageSetNode = doc->createElement("imageSet");

		// Image set attributes
		if(!imageSet->id().isNull())
			imageSetNode.setAttribute("id", imageSet->id());

		// XXX allow the user to choose whether or not the root is stored as
		//     an absolute/relative path?
		if(projectDir != imageSet->root())
			imageSetNode.setAttribute("root", projectDir.relativeFilePath(imageSet->root().absolutePath()));

		if(!imageSet->name().isEmpty() && imageSet->name() != imageSet->id())
			imageSetNode.setAttribute("name", imageSet->name());

		// Images
		foreach(ProjectImagePtr image, imageSet->images()) {
			QDomElement imageNode = doc->createElement("image");

			imageNode.setAttribute("file", imageSet->root().relativeFilePath(image->file()));
			if(image->exposure() > 0)
				imageNode.setAttribute("exposure", image->exposure());

			if(image->camera()) {
				imageNode.setAttribute("for", image->camera()->id());
				if(imageSet->defaultImageForCamera(image->camera()) == image)
					imageNode.setAttribute("default", "yes");
			}

			imageSetNode.appendChild(imageNode);
		}

		imageSetsNode.appendChild(imageSetNode);
	}

	//
	// Features
	//
	std::map<FeaturePtr, int> feature2index;

	int numFeatures = 0;
	const FeaturesMap &featuresMap = featuresDB_.features();
	QDomElement featuresNode = doc->createElement("featureSets");
	for(auto iter = featuresMap.begin(); iter != featuresMap.end(); ++iter) {
		if(iter->second.size() > 0) {
			numFeatures += iter->second.size();

			QDomElement featureSetNode = doc->createElement("featureSet");
			featureSetNode.setAttribute("imageSet", iter->first->imageSet()->id());
			featureSetNode.setAttribute("camera", iter->first->camera()->id());

			int index = 0;
			foreach(FeaturePtr feature, iter->second) {
				feature2index[feature] = index;

				QDomElement featureNode = doc->createElement("feature");
				featureNode.setAttribute("x", feature->x());
				featureNode.setAttribute("y", feature->y());
				featureNode.setAttribute("index", index++);
				feature->save(featureNode);
				featureSetNode.appendChild(featureNode);
			}

			featuresNode.appendChild(featureSetNode);
		}
	}

	//
	// Feature correspondencs
	//
	int numCorrespondences = 0;
	const CorrespondencesMap &correspondencesMap = featuresDB_.correspondences();
	QDomElement correspondencesNode = doc->createElement("correspondenceSets");
	for(auto iter = correspondencesMap.begin(); iter != correspondencesMap.end(); ++iter) {
		if(iter->second.size() > 0) {
			numCorrespondences += iter->second.size();

			QDomElement correspondenceSetNode = doc->createElement("correspondenceSet");
			correspondenceSetNode.setAttribute("imageSet1", iter->first.first->imageSet()->id());
			correspondenceSetNode.setAttribute("camera1", iter->first.first->camera()->id());
			correspondenceSetNode.setAttribute("imageSet2", iter->first.second->imageSet()->id());
			correspondenceSetNode.setAttribute("camera2", iter->first.second->camera()->id());

			foreach(const Correspondence &corr, iter->second) {
				QDomElement correspondenceNode = doc->createElement("correspondence");
				correspondenceNode.setAttribute("index1", feature2index[corr.first]);
				correspondenceNode.setAttribute("index2", feature2index[corr.second]);
				correspondenceSetNode.appendChild(correspondenceNode);
			}

			correspondencesNode.appendChild(correspondenceSetNode);
		}
	}

	//
	//
	//
	if(cameras_.size() > 0) projectNode.appendChild(camerasNode);
	if(imageSets_.size() > 0) projectNode.appendChild(imageSetsNode);
	if(numFeatures > 0) projectNode.appendChild(featuresNode);
	if(numCorrespondences > 0) projectNode.appendChild(correspondencesNode);

	doc->appendChild(projectNode);
	return doc;
}

//---------------------------------------------------------------------

void Project::addCamera(CameraPtr cam) {
	if(cam) {
		cameras_[cam->id()] = cam;
		emit cameraAdded(cam);
	}
}

//---------------------------------------------------------------------

void Project::removeCamera(CameraPtr cam, bool removeImages) {
	if(cameras_.contains(cam->id())) {
		cameras_.remove(cam->id());
		foreach(ImageSetPtr imageSet, imageSets_) {
			if(removeImages) {
				imageSet->removeImagesForCamera(cam);
			} else {
				foreach(ProjectImagePtr image, imageSet->images()) {
					if(image->camera() == cam)
						image->setCamera(CameraPtr());
				}
			}
		}
		emit cameraRemoved(cam);
	}
}

//---------------------------------------------------------------------

CameraPtr Project::cameraFromName(QString name) {
	foreach(CameraPtr cam, cameras_)
		if(cam->name() == name)
			return cam;
	return CameraPtr();
}

//---------------------------------------------------------------------

void Project::addImageSet(ImageSetPtr imageSet) {
	if(imageSet) {
		imageSets_[imageSet->id()] = imageSet;
		emit imageSetAdded(imageSet);
	}
}

//---------------------------------------------------------------------

void Project::removeImageSet(ImageSetPtr imageSet) {
	if(imageSets_.contains(imageSet->id())) {
		imageSets_.remove(imageSet->id());
		emit imageSetRemoved(imageSet);
	}
}

//---------------------------------------------------------------------
