#include "triton_gazebo/bounding_box_plugin.hpp"
#include <gazebo/rendering/rendering.hh>
#include <vector>
using namespace gazebo;

namespace triton_gazebo
{
    BoundingBoxPlugin::BoundingBoxPlugin() : SensorPlugin()
    {
        //Create publisher
        node_ = new rclcpp::Node(this->handleName+"bounding_box");
    }

    BoundingBoxPlugin::~BoundingBoxPlugin ()
    {
        delete node_;
    }


    void BoundingBoxPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
    {
        // Get the parent sensor.
        this->parentSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);

        // Make sure the parent sensor is valid.
        if (!this->parentSensor)
        {
            gzerr << "BoundingBoxPlugin requires a CameraSensor.\n";
            return;
        }

        // Load parameters for size/location of 3D box
        if (_sdf->HasElement("model_name"))
            model_name_ = _sdf->GetElement("model_name")->Get<std::string>();
        else 
            gzwarn << "[bounding_box] model_name not set." << std::endl;
            
        // Create ROS publisher
        std::string ros_namespace;
        if (_sdf->HasElement("ros") && _sdf->GetElement("ros")->HasElement("namespace"))
            ros_namespace = _sdf->GetElement("ros")->GetElement("namespace")->Get<std::string>();
        else
            gzwarn << "[bounding_box] ros/namespace not set." << std::endl;

        std::string camera_name;
        if (_sdf->HasElement("camera_name"))
            camera_name = _sdf->GetElement("camera_name")->Get<std::string>();
        else
            gzwarn << "[bounding_box] camera_name not set." << std::endl;

        publisher_ = node_->create_publisher<triton_interfaces::msg::DetectionBox>(
            ros_namespace+"/"+camera_name+"/bounding_box", 
            10
        );
        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&BoundingBoxPlugin::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);
    }

    void BoundingBoxPlugin::OnUpdate()
    {
        std::vector<ignition::math::Vector3d> corners;

        rendering::ScenePtr scene = rendering::get_scene();
        
        rendering::VisualPtr visual = scene->GetVisual(model_name_);
        ignition::math::AxisAlignedBox visual_box = visual->BoundingBox();

        auto min_corner = visual_box.Min() + visual->Position();
        auto max_corner = visual_box.Max() + visual->Position();

        if (!model_name_.empty()){
            corners.push_back(ignition::math::Vector3d(min_corner.X(),min_corner.Y(),min_corner.Z()));
            corners.push_back(ignition::math::Vector3d(min_corner.X(),min_corner.Y(),max_corner.Z()));
            corners.push_back(ignition::math::Vector3d(min_corner.X(),max_corner.Y(),min_corner.Z()));
            corners.push_back(ignition::math::Vector3d(min_corner.X(),max_corner.Y(),max_corner.Z()));
            corners.push_back(ignition::math::Vector3d(max_corner.X(),min_corner.Y(),min_corner.Z()));
            corners.push_back(ignition::math::Vector3d(max_corner.X(),min_corner.Y(),max_corner.Z()));
            corners.push_back(ignition::math::Vector3d(max_corner.X(),max_corner.Y(),min_corner.Z()));
            corners.push_back(ignition::math::Vector3d(max_corner.X(),max_corner.Y(),max_corner.Z()));
        } else {
            //default to finding the bounds of a unit box centred at the origin
            corners.push_back(ignition::math::Vector3d(-0.5,-0.5,-0.5));
            corners.push_back(ignition::math::Vector3d(-0.5,-0.5,0.5));
            corners.push_back(ignition::math::Vector3d(-0.5,0.5,-0.5));
            corners.push_back(ignition::math::Vector3d(-0.5,0.5,0.5));
            corners.push_back(ignition::math::Vector3d(0.5,-0.5,-0.5));
            corners.push_back(ignition::math::Vector3d(0.5,-0.5,0.5));
            corners.push_back(ignition::math::Vector3d(0.5,0.5,-0.5));
            corners.push_back(ignition::math::Vector3d(0.5,0.5,0.5));
        }
        
        //get max xy
        int x_min = this->parentSensor->Camera()->ViewportWidth() - 1;
        int x_max = 0;
        int y_min = this->parentSensor->Camera()->ViewportHeight() - 1;
        int y_max = 0;

        for (auto & corner : corners){
            auto corner_pixel = this->parentSensor->Camera()->Project(corner);
            //gzmsg << "[pos]" << corner << std::endl;
            //gzmsg << "[pixel]" << corner_pixel << std::endl;

            if (corner_pixel.X() < x_min){
                if (corner_pixel.X() < 0)
                    x_min = 0;
                else
                    x_min = corner_pixel.X();
            }
            if (corner_pixel.X()> x_max){
                if (corner_pixel.X() > this->parentSensor->Camera()->ViewportWidth() - 1)
                    x_max = this->parentSensor->Camera()->ViewportWidth() - 1;
                else
                    x_max = corner_pixel.X();
            }
            if (corner_pixel.Y() < y_min){
                if (corner_pixel.Y() < 0)
                    y_min = 0;
                else
                    y_min = corner_pixel.Y();
            }
            if (corner_pixel.Y() > y_max){
                if (corner_pixel.Y() > this->parentSensor->Camera()->ViewportHeight() - 1)
                    y_max = this->parentSensor->Camera()->ViewportHeight() - 1;
                else
                    y_max = corner_pixel.Y();
            }
        }

        triton_interfaces::msg::DetectionBox bbox;
        bbox.x = x_min;
        bbox.y = y_min;
        bbox.width = x_max-x_min;
        bbox.height = y_max-y_min;

        publisher_->publish(bbox);
    }
    
} // namespace triton_gazebo