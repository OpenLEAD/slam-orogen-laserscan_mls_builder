/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef laserscan_mls_builder_TASK_TASK_HPP
#define laserscan_mls_builder_TASK_TASK_HPP

#include "laserscan_mls_builder/TaskBase.hpp"
#include <boost/shared_ptr.hpp>
#include <envire/Core.hpp>
#include <envire/maps/Pointcloud.hpp>
#include <envire/Orocos.hpp>

#define MAX_COUNT_SURFACES 2

namespace laserscan_mls_builder {

    struct EnvireProjection
    {
        envire::Pointcloud* source_pointcloud;
        envire::CartesianMap* target_map;
        envire::Operator* op;
        EnvireProjection() : source_pointcloud(NULL), target_map(NULL), op(NULL) {};
        EnvireProjection(envire::Pointcloud* pointcloud, envire::CartesianMap* map, envire::Operator* op) 
            : source_pointcloud(pointcloud), target_map(map), op(op) {};
    };
    
    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * 
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','laserscan_mls_builder::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;
    protected:
        envire::Environment env;
        envire::Pointcloud envire_pointcloud[MAX_COUNT_SURFACES];
        std::vector<EnvireProjection> projections;
        envire::OrocosEmitter* orocosEmitter;
        base::Time lastEnvireDataUpdate;
        
        std::vector< std::vector<Eigen::Affine3d> > surface_transformations;

        double stop_log_height;
        double stop_log_length;
        double stop_log_depth;
        
    protected:
        void setupProjection(const Eigen::Affine3d &transform, EnvireProjection &projection);
        void dividePointcloudByPlane(const std::vector<Eigen::Vector3d> &source_pointcloud, const Eigen::Hyperplane<double, 3> &plane, 
                                      std::vector<Eigen::Vector3d> &sub_pointcloud_1, std::vector<Eigen::Vector3d> &sub_pointcloud_2);
        
        /**
         * Removes outliers in a laser scan.
         * \param window_size Size of the window used to compute the mean and sigma.
         * \param scan The laser scan, its parameter minRange is used as bounding box of the sensor.
         * \param sensor_sigma Minimum sigma in meter given by the uncertainty of the sensor.
         * \param min_density Minimum density of range samples a window should have.
         */
        void filterLaserScan(unsigned int window_size, base::samples::LaserScan& scan, double sensor_sigma = 0.0, double min_density = 0.8);
        

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "laserscan_mls_builder::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
       //bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        // void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        // void cleanupHook();
        
        void laserscanTransformerCallback(base::Time const& timestamp, base::samples::LaserScan const& sample);
    };
}

#endif

