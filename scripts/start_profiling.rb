require 'vizkit'
require 'ping'
require "transformer/runtime"
require 'orocos'

include Orocos
Orocos::CORBA.max_message_size = 80000000

log = if ARGV[0]
          require 'orocos/log'
          log = Orocos::Log::Replay.open(ARGV)
          log.uw_portal.track(true)
          log.profiling.track(true)
          log
      else
          nil
      end

Orocos.initialize

## load transformer config
Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"transforms.rb"))

if log
    Vizkit.control log

    Orocos.run "accumulated_pointcloud_test" do

        ## setup accumulated pointcloud task
        accumulated_pointcloud = TaskContext.get 'accumulated_pointcloud'
        # show mls grid
        accumulated_pointcloud.show_mls_grid = false
        # show elevation grid
        accumulated_pointcloud.show_elevation_grid = true
        # x size of the grid in meter
        accumulated_pointcloud.grid_size_x = 10
        # y size of the grid in meter
        accumulated_pointcloud.grid_size_y = 10
        # x cell resolution in meter
        accumulated_pointcloud.cell_resolution_x = 0.04
        # y cell resolution in meter
        accumulated_pointcloud.cell_resolution_y = 0.04
        # true if laser scan should be filtered
        accumulated_pointcloud.filter_laserscan = true
        # filter: bounding box of the sensor in meter
        accumulated_pointcloud.sensor_bounding_box = 0.2
        # surfaces that are used to display the data
        accumulated_pointcloud.surface_count = 2

        ## connect ports
        log.profiling.profiling_scan.connect_to accumulated_pointcloud.laserscan, :type => :buffer, :size => 100

        Orocos.transformer.setup(accumulated_pointcloud)

        ## start tasks
        accumulated_pointcloud.configure
        accumulated_pointcloud.start

        ## setup vizkit visualizations
        view3d = Vizkit.vizkit3d_widget
        view3d.show()
        envireviz = Vizkit.default_loader.EnvireVisualization
        bodystateviz = Vizkit.default_loader.RigidBodyStateVisualization
        Vizkit.connect_port_to 'accumulated_pointcloud', 'envire_data', :pull => false, :update_frequency => 33 do |sample, name|
            envireviz.updateBinaryEvents(sample)
        end

        log.uw_portal.rigid_body_state.filter do |sample|
            sample.sourceFrame = "world"
            sample.targetFrame = "mounting"
            sample2 = sample.clone
            sample2.position = -sample.position
            bodystateviz.updateRigidBodyState(sample2)
            sample
        end

        Vizkit.display accumulated_pointcloud

        Vizkit.exec 
    end
elsif Ping.pingecho("192.168.128.12",0.5)
        Orocos.run "accumulated_pointcloud_test", "uw_portal::Task" => "uw_portal", "sonar_tritech::Profiling" => "profiling", "transformer::Task" => "transformer_broadcaster" do 
            Orocos.log_all_ports
            
            ## setup profiling driver
            profiling = TaskContext.get 'profiling'
            profiling.configure_timeout = 20.0
            # device name
            profiling.port = '/dev/ttyUSB0'
            config = profiling.config
            # left limit of the scan area
            config.left_limit.rad = 0.7 #3.14
            # right limit of the scan area
            config.right_limit.rad = -0.7 #-3.14
            # channel [1,2]
            config.select_channel = 2
            # operation frequency of channel 1 [600000-1200000]
            config.frequency_chan1 = 600000
            # operation frequency of channel 2 [600000-1200000]
            config.frequency_chan2 = 1200000
            # angular resolution [min. 0.225°]
            config.angular_resolution.rad = 0.00393 #0.0157 #0.007854
            # max. scan distance
            config.max_distance = 2.5
            # min. scan distance
            config.min_distance = 0.1
            # speed of sound in water
            config.speed_of_sound = 1500
            # gain [0..1]
            config.gain = 0.2
            # filt gain [0..1]
            config.filt_gain = 0.01
            profiling.config = config

            profiling.profiling_scan.frame = "laser"
            
            ## setup uw portal driver
            portal = Orocos::TaskContext.get "uw_portal"
            portal.server_name = "192.168.128.12"
            protal.source_frame_name = "world"
            protal.target_frame_name = "mounting"

            ## setup accumulated pointcloud task
            accumulated_pointcloud = TaskContext.get 'accumulated_pointcloud'
            # show mls grid
            accumulated_pointcloud.show_mls_grid = false
            # a path were the envirement can be saved
            accumulated_pointcloud.envire_path = "#{ENV['AUTOPROJ_PROJECT_BASE']}/csurvey/orogen/accumulated_pointcloud/env/"

            ## setup transformer broadcaster
            broadcaster = Orocos::TaskContext.get "transformer_broadcaster"
            Transformer.broadcaster = broadcaster 

            ## connect ports
            profiling.profiling_scan.connect_to accumulated_pointcloud.laserscan, :type => :buffer, :size => 100

            Orocos.transformer.setup(accumulated_pointcloud)

            ## start tasks
            profiling.configure
            profiling.start

            accumulated_pointcloud.configure
            accumulated_pointcloud.start

            portal.configure
            portal.start

            broadcaster.start
            
            ## setup vizkit visualizations
            view3d = Vizkit.vizkit3d_widget
            view3d.show()
            envireviz = Vizkit.default_loader.EnvireVisualization
            Vizkit.connect_port_to 'accumulated_pointcloud', 'envire_data', :pull => false, :update_frequency => 33 do |sample, name|
                envireviz.updateBinaryEvents(sample)
            end 

            Vizkit.display profiling
            Vizkit.display portal
            Vizkit.exec 
        end
end
