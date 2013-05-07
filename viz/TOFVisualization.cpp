#include <iostream>
#include "TOFVisualization.hpp"
#include <vizkit/Vizkit3DHelper.hpp>

using namespace vizkit;

TOFVisualization::TOFVisualization()
{
}

TOFVisualization::~TOFVisualization()
{
}

osg::ref_ptr<osg::Node> TOFVisualization::createMainNode()
{
    transformNode = new osg::PositionAttitudeTransform();
    scanNode = new osg::Geode();
    transformNode->addChild(scanNode);

    scanGeom = new osg::Geometry();

    //setup normals
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,-1.0f,0.0f));
    scanGeom->setNormalArray(normals);
    scanGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    //set size
    osg::ref_ptr<osg::Point> point = new osg::Point();
    point->setSize(5.0);
    point->setDistanceAttenuation( osg::Vec3(1.0, 1.0, 1.0 ) );
    point->setMinSize( 3.0 );
    point->setMaxSize( 5.0 );
    scanGeom->getOrCreateStateSet()->setAttribute( point, osg::StateAttribute::ON );

    //turn on transparacny
    scanNode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    scanNode->addDrawable(scanGeom);

    return transformNode;
}

void TOFVisualization::updateMainNode ( osg::Node* node )
{
    osg::Vec3Array *scanVertices = new osg::Vec3Array();
    std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign> > points = data.points;

    scanVertices->reserve(points.size()+1);
    scanVertices->push_back(osg::Vec3(0,0,0));

    for(std::vector<Eigen::Matrix<double, 3, 1, Eigen::DontAlign> >::const_iterator it = points.begin(); it != points.end(); it++)
    {
       Eigen::Vector3d vec = *it;
       // convertion just for visualisation
       scanVertices->push_back(eigenVectorToOsgVec3(Eigen::Vector3d(vec.x()*1000, -vec.z()*1000, vec.y()*1000)));
    }
    scanGeom->setVertexArray(scanVertices);

    while(!scanGeom->getPrimitiveSetList().empty())
        scanGeom->removePrimitiveSet(0);

    scanGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::POINTS,0,scanVertices->size()));
}

void TOFVisualization::updateDataIntern(base::samples::Pointcloud const& value)
{
    data = value;
}

void TOFVisualization::updateConfidenceData_(base::samples::Pointcloud const& pointcloud)
{
    confidence_data_ = pointcloud;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(TOFVisualization);

