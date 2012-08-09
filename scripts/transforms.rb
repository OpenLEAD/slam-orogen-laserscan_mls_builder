dynamic_transform "uw_portal.rigid_body_state","world" => "body"

static_transform(Eigen::Vector3.new(0.0,0.0,0.0),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0.5*Math::PI,0,0),2,1,0),"body" => "mounting")

static_transform(Eigen::Vector3.new(0.42,0.0,0.15),
                 Eigen::Quaternion.from_euler(Eigen::Vector3.new(0,0,0),2,1,0),"mounting" => "laser")
