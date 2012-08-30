dynamic_transform "uw_portal.rigid_body_state","world" => "mounting"

# sonar mounted on the front looking to the ground
#static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
#                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,-0.5*Math::PI,0),2,1,0),"mounting" => "body")

# sonar mounted on the right looking to the front
static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,-0.5*Math::PI),2,1,0),"mounting" => "body")

# unused at the moment
static_transform(Eigen::Vector3.new(0.0,0.0,0.0), #(0.42,0.0,0.15),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"body" => "laser")