#ifndef tofcamera_mesa_swissranger_TOFVisualization_H
#define tofcamera_mesa_swissranger_TOFVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <base/samples/pointcloud.h>

#include <osg/PositionAttitudeTransform>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>

namespace vizkit
{
    class TOFVisualization
        : public vizkit::Vizkit3DPlugin<base::samples::Pointcloud>
        , boost::noncopyable
    {
    Q_OBJECT
    public:
        TOFVisualization();
        ~TOFVisualization();

    Q_INVOKABLE void updateData(base::samples::Pointcloud const &sample)
    {vizkit::Vizkit3DPlugin<base::samples::Pointcloud>::updateData(sample);}

    Q_INVOKABLE void updateConfidenceData(base::samples::Pointcloud const& pointcloud)
    {vizkit::TOFVisualization::updateConfidenceData_(pointcloud);}

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::samples::Pointcloud const& plan);
        void updateConfidenceData_(base::samples::Pointcloud const& pointcloud);
        
    private:
        base::samples::Pointcloud data;
        base::samples::Pointcloud confidence_data_;

        osg::ref_ptr< osg::PositionAttitudeTransform > transformNode;
        osg::ref_ptr<osg::Geode> scanNode;
        osg::ref_ptr<osg::Geometry> scanGeom;
    };
}
#endif
