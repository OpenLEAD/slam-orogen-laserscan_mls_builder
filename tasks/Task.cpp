/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace accumulated_pointcloud;

Task::Task(std::string const& name)
    : TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine)
    : TaskBase(name, engine)
{
}

Task::~Task()
{
}

void Task::laserscanTransformerCallback(base::Time const& timestamp, base::samples::LaserScan const& sample)
{
    Eigen::Affine3d laser2body;
    if (!_laser2body.get(timestamp, laser2body, false))
        return;
    Eigen::Affine3d body2world;
    if (!_body2world.get(timestamp, body2world, false))
        return;
    
    std::vector<Eigen::Vector3d> points;
    sample.convertScanToPointCloud(points, body2world * laser2body, true);
    std::copy( points.begin(), points.end(), std::back_inserter( envire_pointcloud->vertices ) );
    env->itemModified(envire_pointcloud.get());
    
    // write out point cloud area of interest
    if(_point_cloud.connected())
    {
        //TODO
        base::samples::Pointcloud point_cloud;
        point_cloud.time = base::Time::now();
        std::copy( envire_pointcloud->vertices.begin(), envire_pointcloud->vertices.end(), std::back_inserter( point_cloud.points ) );
        _point_cloud.write(point_cloud);
    }
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

//bool Task::configureHook()
//{
//    if (! TaskBase::configureHook())
//        return false;
//    return true;
//}

bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;
    
    env = boost::shared_ptr<envire::Environment>(new envire::Environment());
    envire_pointcloud = boost::shared_ptr<envire::Pointcloud>(new envire::Pointcloud());
    env->attachItem(envire_pointcloud.get());
    
    orocosEmitter = NULL;
    
    return true;
}

void Task::updateHook()
{
    if( !orocosEmitter && _envire_data.connected() )
    {
        // register the binary event dispatcher, 
        // which will write envire data to a port
        orocosEmitter = new envire::OrocosEmitter( _envire_data );
        orocosEmitter->useContextUpdates( env.get() );
        orocosEmitter->useEventQueue( true );
        orocosEmitter->attach( env.get() );
    }
    TaskBase::updateHook();

    if( orocosEmitter )
    {
        if( _envire_data.connected() )
        {
            if( (lastEnvireDataUpdate + base::Time::fromSeconds(_envire_period.value())) < base::Time::now() ) 
            {
                orocosEmitter->flush();
                lastEnvireDataUpdate = base::Time::now();
            }
        }
        else
        {
            delete orocosEmitter;
            orocosEmitter = NULL;
        }
    }
}

// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
void Task::stopHook()
{
    TaskBase::stopHook();
    
    delete orocosEmitter;
    orocosEmitter = NULL;
}
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

