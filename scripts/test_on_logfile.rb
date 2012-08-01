# -*- coding: utf-8 -*-

require 'orocos'
require "transformer/runtime"

def configure_structured_light(task)
    task.apply_conf_file File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}","csurvey", "run_scripts","resources","structured_light_settings.yml")

    settings = task.calibration_settings
    settings.path = File.expand_path(File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}","csurvey", "run_scripts","resources","csurvey_calibration.tiff"))
    settings.start_angle = 0.48715
    settings.angular_resolution = -0.000974
    task.calibration_settings = settings

    task.laser_scan.frame = "head"
end



log = if ARGV[0] 
          require 'orocos/log'
          Orocos::Log::Replay.open(ARGV[0])
      end

Orocos::CORBA::max_message_size = 8000000
Orocos::MQueue::auto = true
Orocos::MQueue::auto_sizes = true

include Orocos
Orocos.initialize

Orocos.transformer.load_conf(File.join("#{ENV['AUTOPROJ_PROJECT_BASE']}","csurvey","run_scripts","resources","transforms.rb"))
Orocos.run "accumulated_pointcloud_test", "structured_light", "transformer::Task" => "transformer_broadcaster" do

    #camera.frame.connect_to structured_light.frame, :type => :buffer,:size => 10
    #log....
    
    camera = log.camera
    hbridge = log.hbridge
    pointcloud = TaskContext.get 'accumulated_pointcloud'
    structured_light = TaskContext.get 'structured_light'

    configure_structured_light(structured_light)
    
    camera.frame.connect_to structured_light.frame, :type => :buffer,:size => 10
    structured_light.laserscan.connect_to pointcloud.laserscan
    hbridge.rad_pos_hb1.connect_to pointcloud.rad_pos_hb1
    hbridge.rad_pos_hb2.connect_to pointcloud.rad_pos_hb2
    
    structured_light.configure
    structured_light.start
    
    
    pointcloud.start

    hbridge1 = TaskContext.get 'hbridge'
                   
    broadcaster = Orocos::TaskContext.get "transformer_broadcaster"
    broadcaster.start
    Transformer.broadcaster = broadcaster
    Orocos.transformer.setup(hbridge,structured_light,pointcloud)

    require 'vizkit'
    Vizkit.control log
    Vizkit.exec
end

