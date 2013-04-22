#ifndef tofcamera_mesa_swissranger_TOFVisualization_H
#define tofcamera_mesa_swissranger_TOFVisualization_H

#include <boost/noncopyable.hpp>
#include <vizkit/Vizkit3DPlugin.hpp>
#include <osg/Geode>
#include <base/samples/pointcloud.h>

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

    protected:
        virtual osg::ref_ptr<osg::Node> createMainNode();
        virtual void updateMainNode(osg::Node* node);
        virtual void updateDataIntern(base::samples::Pointcloud const& plan);
        
    private:
        struct Data;
        Data* p;
    };
}
#endif
