/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

#include <envire/maps/MLSGrid.hpp>
#include <envire/operators/MLSProjection.hpp>
#include <envire/maps/Grids.hpp>
#include <envire/operators/Projection.hpp>
#include <laserscan_mls_builder/laserscan_mls_builderTypes.hpp>

using namespace laserscan_mls_builder;

Task::Task(std::string const& name)
    : TaskBase(name)
{
    // transformation if using one surface
    std::vector<Eigen::Affine3d> vec;
    vec.push_back(Eigen::Affine3d(Eigen::AngleAxisd(-0.5*M_PI,Eigen::Vector3d::UnitY())));
    surface_transformations.push_back(vec);
    // transformations if using two surfaces
    std::vector<Eigen::Affine3d> vec2;
    vec2.push_back(Eigen::Affine3d(Eigen::AngleAxisd(-0.5*M_PI,Eigen::Vector3d::UnitX())));
    vec2.push_back(Eigen::Affine3d(Eigen::AngleAxisd(0.5*M_PI,Eigen::Vector3d::UnitX())));
    surface_transformations.push_back(vec2);
    
    stop_log_depth = 1.007;
    stop_log_height = 0.674;
    stop_log_length = 8.0;
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

void Task::dividePointcloudByPlane(const std::vector<Eigen::Vector3d> &source_pointcloud, const Eigen::Hyperplane<double, 3> &plane, 
                                      std::vector<Eigen::Vector3d> &sub_pointcloud_1, std::vector<Eigen::Vector3d> &sub_pointcloud_2)
{
    sub_pointcloud_1.clear();
    sub_pointcloud_2.clear();
    for(std::vector<Eigen::Vector3d>::const_iterator it = source_pointcloud.begin(); it != source_pointcloud.end(); it++)
    {
        if(plane.signedDistance(*it) < 0.0)
            sub_pointcloud_1.push_back(*it);
        else
            sub_pointcloud_2.push_back(*it);
    }
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
    for(unsigned i = 0; i <= filtered_scan.ranges.size(); ++i)
    {
        if(filtered_scan.ranges[i] < filtered_scan.minRange)
            filtered_scan.ranges[i] = 0;
    }
    if (_filter_laserscan.get())
        filterLaserScan(sample.ranges.size() / 10, filtered_scan, 0.05, 0.7);
    
    // compute pointcloud
    std::vector<Eigen::Vector3d> raw_points;
    filtered_scan.convertScanToPointCloud(raw_points, laser2world, true);

    // Filter out the provided bounding box
    std::vector<Box> boxes = _excluded_bounding_boxes.get();
    std::vector<Eigen::Vector3d> points;
    if (boxes.empty())
        points = raw_points;
    else
    {
        for (unsigned int j = 0; j < raw_points.size(); ++j)
        {
            unsigned int i;
            for (i = 0; i < boxes.size(); ++i)
            {
                if (boxes[i].include(raw_points[j]))
                    break;
            }
            if (i == boxes.size())
                points.push_back(raw_points[j]);
        }
    }

    // remove points outside of the stop log boundaries
    if(_use_stop_log && _surface_distance != 0.0)
    {
        double stop_log_size_variance = 0.1;
        Eigen::Vector3d varance(stop_log_size_variance,stop_log_size_variance,stop_log_size_variance);
        Eigen::AlignedBox<double,3> box;
        Eigen::Vector3d first_corner(_surface_distance - stop_log_depth, -stop_log_height * 0.5, -stop_log_length * 0.5);
        Eigen::Vector3d second_corner(_surface_distance, stop_log_height * 0.5, stop_log_length * 0.5);
        box.extend(laser2world * (first_corner - varance));
        box.extend(laser2world * (second_corner + varance));
        for(std::vector<Eigen::Vector3d>::iterator it = points.begin(); it != points.end();)
        {
            if(box.contains(*it))
                it++;
            else
                it = points.erase(it);
        }
    }
    
    unsigned surface_count = _surface_count;
    if(surface_count > MAX_COUNT_SURFACES)
        surface_count = MAX_COUNT_SURFACES;
    else if(surface_count <= 0)
        surface_count = 1;
    if(surface_count == 1)
    {
        // copy new points to the envire pointcloud
        std::copy( points.begin(), points.end(), std::back_inserter( envire_pointcloud[0].vertices ) );
    }
    else
    {
        // divide the point cloud at the current height of the sensor
        Eigen::Vector3d sensor_pos(0,0,0);
        sensor_pos = laser2world * sensor_pos;
        Eigen::Hyperplane<double, 3> plane(Eigen::Vector3d(0,0,1), sensor_pos);
        std::vector<Eigen::Vector3d> sub_points_1, sub_points_2;
        
        dividePointcloudByPlane(points, plane, sub_points_1, sub_points_2);
        std::copy( sub_points_1.begin(), sub_points_1.end(), std::back_inserter( envire_pointcloud[0].vertices ) );
        std::copy( sub_points_2.begin(), sub_points_2.end(), std::back_inserter( envire_pointcloud[1].vertices ) );
    }
    
    // create uncertainty information
    for(unsigned i = 0; i < surface_count; i++)
    {
        std::vector<double>& uncertainty(envire_pointcloud[i].getVertexData<double>(envire::Pointcloud::VERTEX_VARIANCE));
        while(uncertainty.size() < envire_pointcloud[i].vertices.size())
            uncertainty.push_back(_sensor_uncertainty.get());
    }
    
    // update visualization of mls grid or pointcloud
    if(_show_mls_grid || _show_elevation_grid)
    {
        if(projections.empty())
        {
            double grid_count_x = _grid_size_x / _cell_resolution_x;
            double grid_count_y = _grid_size_y / _cell_resolution_y;
            if(_show_mls_grid)
            {
                for(unsigned i = 0; i < surface_count; i++)
                {
                    EnvireProjection projection(&envire_pointcloud[i],
                                new envire::MultiLevelSurfaceGrid(grid_count_y, grid_count_x, _cell_resolution_x, _cell_resolution_y, -0.5 * _grid_size_x, -0.5 * _grid_size_y),
                                new envire::MLSProjection());
                    projections.push_back(projection);
                    setupProjection(laser2world, projection);
                }
            }
            else
            {
                for(unsigned i = 0; i < surface_count; i++)
                {
                    EnvireProjection projection(&envire_pointcloud[i],
                                                new envire::ElevationGrid(grid_count_y, grid_count_x, _cell_resolution_x, _cell_resolution_y, -0.5 * _grid_size_x, -0.5 * _grid_size_y),
                                                new envire::Projection());
                    projections.push_back(projection);
                    
                    setupProjection(laser2world, projection);
                }
            }
        }
        if (_envire_data.connected())
        {
            for(std::vector<EnvireProjection>::iterator it = projections.begin(); it != projections.end(); it++)
            {
                it->op->updateAll();
                env.itemModified(it->target_map);
            }
        }
            
    }
    else
        for(unsigned i = 0; i < MAX_COUNT_SURFACES; i++)
            env.itemModified(&envire_pointcloud[i]);
    
    // write out point cloud area of interest
    if(_point_cloud.connected())
    {
        //TODO
        base::samples::Pointcloud point_cloud;
        point_cloud.time = base::Time::now();
        for(unsigned i = 0; i < surface_count; i++)
            std::copy( envire_pointcloud[i].vertices.begin(), envire_pointcloud[i].vertices.end(), std::back_inserter( point_cloud.points ) );
        _point_cloud.write(point_cloud);
    }
}

void Task::setupProjection(const Eigen::Affine3d& transform, EnvireProjection &projection)
{
    env.attachItem(projection.target_map);
    projection.target_map->setFrameNode(env.getRootNode());

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
    
    for(unsigned i = 0; i < MAX_COUNT_SURFACES; i++)
        env.attachItem(&envire_pointcloud[i]);
    
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
    {
        if(_show_mls_grid || _show_elevation_grid)
        {
            for(std::vector<EnvireProjection>::iterator it = projections.begin(); it != projections.end(); it++)
            {
                it->op->updateAll();
                env.itemModified(it->target_map);
            }
        }
        else
        {
            for(unsigned i = 0; i < MAX_COUNT_SURFACES; i++)
                env.itemModified(&envire_pointcloud[i]);
        }

        env.serialize(_envire_path);
    }
    
    delete orocosEmitter;
    orocosEmitter = NULL;
}
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

