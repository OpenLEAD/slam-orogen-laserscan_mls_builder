/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSProjection.hpp>

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

void Task::filterLaserScan(unsigned int window_size, base::samples::LaserScan& scan, double sensor_sigma, double min_density)
{
    // force window size to be odd
    if(window_size % 2 == 0 && window_size != 0)
        window_size -= 1;
    // skip if window or laser scan size is to small
    if(window_size < 3 || window_size > scan.ranges.size())
        return;
    
    if(scan.minRange < base::samples::MAX_RANGE_ERROR)
        scan.minRange = base::samples::MAX_RANGE_ERROR;
    
    double sum = 0.0;
    unsigned valid_sample_count = 0;
    unsigned window_start = 0;
    unsigned window_end = window_size - 1;
    
    // fill first window
    for(unsigned i = window_start; i <= window_end; i++)
    {
        if(scan.ranges[i] > scan.minRange)
        {
            sum += scan.ranges[i];
            valid_sample_count++;
        }
    }
    
    std::vector<unsigned> delete_index;
    for(unsigned i = window_start; i < scan.ranges.size(); i++)
    {        
        // filter valid ranges
        if(scan.ranges[i] > scan.minRange && valid_sample_count >= window_size * min_density)
        {
            double sigma = 0.0;
            double min_sigma = sensor_sigma * 1000.0;
            double mu = sum / valid_sample_count;
            
            for(unsigned j = window_start; j <= window_end; j++)
            {
                if(scan.ranges[j] > scan.minRange)
                    sigma += pow(scan.ranges[j] - mu, 2);
            }
            sigma = sqrt(sigma / (double)(valid_sample_count - 1));
            if(sigma < min_sigma)
                sigma = min_sigma;
            
            if(scan.ranges[i] < mu - sigma || scan.ranges[i] > mu + sigma)
            {
                delete_index.push_back(i);
            }
        }
        
        // move window
        if(window_end < scan.ranges.size() - 1 && !(window_start == 0 && std::ceil(window_size / 2.0) > i))
        {
            if(scan.ranges[window_start] > scan.minRange)
            {
                sum -= scan.ranges[window_start];
                valid_sample_count--;
            }
            window_start++;
            window_end++;
            if(scan.ranges[window_end] > scan.minRange)
            {
                sum += scan.ranges[window_end];
                valid_sample_count++;
            }
        }
    }
    
    // mark ranges as measurement error
    for(int i = (int)delete_index.size() - 1; i >= 0; i--)
        scan.ranges[delete_index[i]] = base::samples::MEASUREMENT_ERROR;
}

void Task::laserscanTransformerCallback(base::Time const& timestamp, base::samples::LaserScan const& sample)
{
    Eigen::Affine3d laser2world;
    if (!_laser2world.get(timestamp, laser2world, false))
    {
        std::cerr << "skip, have no transformation sample!" << std::endl;
        return;
    }
    
    // filter laserscan
    base::samples::LaserScan filtered_scan = sample;
    filtered_scan.minRange = _sensor_bounding_box * 1000;
    filterLaserScan(sample.ranges.size() / 10, filtered_scan, 0.05, 0.7);
    
    // compute pointcloud
    std::vector<Eigen::Vector3d> points;
    filtered_scan.convertScanToPointCloud(points, laser2world, true);
    
    // copy new points to the envire pointcloud
    std::copy( points.begin(), points.end(), std::back_inserter( envire_pointcloud->vertices ) );
    
    // create uncertainty information
    std::vector<double>& uncertainty(envire_pointcloud->getVertexData<double>(envire::Pointcloud::VERTEX_VARIANCE));
    for(unsigned i = 0; i < points.size(); i++)
        uncertainty.push_back(_sensor_uncertainty.get());
    
    // update visualization of mls grid or pointcloud
    if(_show_mls_grid)
    {
        if(projections.empty())
        {
            double grid_count_x = _grid_size_x / _cell_resolution_x;
            double grid_count_y = _grid_size_y / _cell_resolution_y;

            EnvireProjection projection(envire_pointcloud.get(), 
                        new envire::MultiLevelSurfaceGrid(grid_count_y, grid_count_x, _cell_resolution_x, _cell_resolution_y, -0.5 * _grid_size_x, -0.5 * _grid_size_y), 
                        new envire::MLSProjection());
            projections.push_back(projection);
            setupProjection(laser2world * Eigen::Affine3d(Eigen::AngleAxisd(-0.5*M_PI,Eigen::Vector3d::UnitY())), projection);

        }
        for(std::vector<EnvireProjection>::iterator it = projections.begin(); it != projections.end(); it++)
        {
            it->op->updateAll();
            env.itemModified(it->target_map);
        }
            
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

void Task::setupProjection(const Eigen::Affine3d& transform, EnvireProjection &projection)
{
    env.attachItem(projection.target_map);
    envire::FrameNode *fn = new envire::FrameNode(transform);
    env.getRootNode()->addChild(fn);
    projection.target_map->setFrameNode(fn);

    env.addInput(projection.op, projection.source_pointcloud);
    env.addOutput(projection.op, projection.target_map);
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
    
    if(_envire_path.get() != "")
        env.serialize(_envire_path);
    
    delete orocosEmitter;
    orocosEmitter = NULL;
}
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

