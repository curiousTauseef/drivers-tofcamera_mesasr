#include <iostream>
#include "TOFVisualization.hpp"

using namespace vizkit;

struct TOFVisualization::Data {
    // Copy of the value given to updateDataIntern.
    //
    // Making a copy is required because of how OSG works
    base::samples::Pointcloud data;
};


TOFVisualization::TOFVisualization()
    : p(new Data)
{
}

TOFVisualization::~TOFVisualization()
{
    delete p;
}

osg::ref_ptr<osg::Node> TOFVisualization::createMainNode()
{
    // Geode is a common node used for vizkit plugins. It allows to display
    // "arbitrary" geometries
    return new osg::Geode();
}

void TOFVisualization::updateMainNode ( osg::Node* node )
{
    osg::Geode* geode = static_cast<osg::Geode*>(node);
    // Update the main node using the data in p->data
}

void TOFVisualization::updateDataIntern(base::samples::Pointcloud const& value)
{
    p->data = value;
    std::cout << "got new sample data" << std::endl;
}

//Macro that makes this plugin loadable in ruby, this is optional.
VizkitQtPlugin(TOFVisualization)

