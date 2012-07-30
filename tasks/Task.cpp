/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "Task.hpp"

using namespace accumulated_pointcloud;

Task::Task(std::string const& name, TaskCore::TaskState initial_state)
    : TaskBase(name, initial_state)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine, TaskCore::TaskState initial_state)
    : TaskBase(name, engine, initial_state)
{
}

Task::~Task()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

// bool Task::configureHook()
// {
//     if (! TaskBase::configureHook())
//         return false;
//     return true;
// }
// bool Task::startHook()
// {
//     if (! TaskBase::startHook())
//         return false;
//     return true;
// }
void Task::updateHook()
{
    TaskBase::updateHook();

    base::samples::LaserScan laserscan;
    base::samples::RigidBodyState rad_pos_hb1;
    base::samples::RigidBodyState rad_pos_hb2;

    while (_laserscan.read(laserscan) == RTT::NewData)
    {
        // Got a new laserscan sample, do something with it.

	// Get motor angles.
        _rad_pos_hb1.read(rad_pos_hb1);
        _rad_pos_hb2.read(rad_pos_hb2);
    }
}
// void Task::errorHook()
// {
//     TaskBase::errorHook();
// }
// void Task::stopHook()
// {
//     TaskBase::stopHook();
// }
// void Task::cleanupHook()
// {
//     TaskBase::cleanupHook();
// }

