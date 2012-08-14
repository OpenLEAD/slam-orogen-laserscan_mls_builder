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
    Eigen::Affine3d laser2world;
    if (!_laser2world.get(timestamp, laser2world, false))
    {
        std::cerr << "skip, have no transformation sample!" << std::endl;
        return;
    }
    
    std::vector<Eigen::Vector3d> points;
    sample.convertScanToPointCloud(points, laser2world, true);
    std::copy( points.begin(), points.end(), std::back_inserter( envire_pointcloud->vertices ) );
    if(_show_mls_grid)
    {
        if(!mls_grid)
            setupMLSGrid();
        projection->updateAll();
    }
    else
        env.itemModified(envire_pointcloud.get());
    
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

void Task::setupMLSGrid()
{
    double grid_count_x = _grid_size_x / _cell_resolution_x;
    double grid_count_y = _grid_size_y / _cell_resolution_y;
    
    if(mls_grid)
        mls_grid->detach();
    if(projection)
        projection->detach();
    projection = boost::shared_ptr<envire::MLSProjection>(new envire::MLSProjection());
    mls_grid = boost::shared_ptr<envire::MultiLevelSurfaceGrid>(new envire::MultiLevelSurfaceGrid(grid_count_y, grid_count_x, _cell_resolution_x, _cell_resolution_y, -0.5 * _grid_size_x, -0.5 * _grid_size_y));
    env.attachItem(mls_grid.get());
    env.addInput(projection.get(), envire_pointcloud.get());
    env.addOutput(projection.get(), mls_grid.get());
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
    
    envire_pointcloud = boost::shared_ptr<envire::Pointcloud>(new envire::Pointcloud());
    env.attachItem(envire_pointcloud.get());
    
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
        orocosEmitter->useContextUpdates( &env );
        orocosEmitter->useEventQueue( true );
        orocosEmitter->attach( &env );
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

