require 'vizkit'
require 'ping'
require "transformer/runtime"
require 'orocos'

log = if ARGV[0]
          require 'orocos/log'
          log = Orocos::Log::Replay.open(ARGV)
          log.uw_portal.track(true)
          log.profiling.track(true)
          log
      else
          nil
      end

include Orocos
Orocos.initialize

if log
    Vizkit.control log

    Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"transforms.rb"))
    Orocos.run "accumulated_pointcloud_test", "transformer::Task" => "transformer_broadcaster" do
        accumulated_pointcloud = TaskContext.get 'accumulated_pointcloud'
        # show mls grid
        accumulated_pointcloud.show_mls_grid = true

        log.profiling.profiling_scan.connect_to accumulated_pointcloud.laserscan

        accumulated_pointcloud.configure
        accumulated_pointcloud.start

        view3d = Vizkit.vizkit3d_widget
        view3d.show()
        envireviz = Vizkit.default_loader.EnvireVisualization
        Vizkit.connect_port_to 'accumulated_pointcloud', 'envire_data', :pull => false, :update_frequency => 33 do |sample, name|
            envireviz.updateBinaryEvents(sample)
        end 

        Vizkit.display accumulated_pointcloud
        
        broadcaster = Orocos::TaskContext.get "transformer_broadcaster"
        broadcaster.start
        Transformer.broadcaster = broadcaster

        Orocos.transformer.setup(accumulated_pointcloud)

        Vizkit.exec 
    end
elsif Ping.pingecho("192.168.128.12",0.5)
        Orocos.transformer.load_conf(File.join(File.dirname(__FILE__),"transforms.rb"))
        Orocos.run "uw_portal::Task" => "uw_portal", "sonar_tritech::Profiling" => "profiling", "transformer::Task" => "transformer_broadcaster" do 
            Orocos.log_all_ports

            profiling = TaskContext.get 'profiling'
            profiling.configure_timeout = 20.0
            # device name
            profiling.port = '/dev/ttyUSB0'
            config = profiling.config
            # left limit of the scan area
            config.left_limit.rad = 1.0 #3.14
            # right limit of the scan area
            config.right_limit.rad = -1.0 #-3.14
            # channel [1,2]
            config.select_channel = 2
            # operation frequency of channel 1 [600000-1200000]
            config.frequency_chan1 = 600000
            # operation frequency of channel 2 [600000-1200000]
            config.frequency_chan2 = 1200000
            # angular resolution [min. 0.225°]
            config.angular_resolution.rad = 0.00393 #0.0157 #0.007854
            # max. scan distance
            config.max_distance = 3.0
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

            profiling.configure
            profiling.start

            portal = Orocos::TaskContext.get "uw_portal"
            portal.server_name = "192.168.128.12"
            portal.configure
            portal.start

            broadcaster = Orocos::TaskContext.get "transformer_broadcaster"
            broadcaster.start
            Transformer.broadcaster = broadcaster   
            
            Vizkit.display profiling
            Vizkit.display portal
            Vizkit.exec 
        end
end
