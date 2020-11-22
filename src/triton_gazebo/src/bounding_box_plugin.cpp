#include "triton_gazebo/bounding_box_plugin.hpp"
#include "triton_interfaces/msg/detection_box.hpp"
using namespace gazebo;

namespace triton_gazebo
{
    BoundingBoxPlugin::BoundingBoxPlugin() : SensorPlugin()
    {
        //TODO: Parametrize this
        corners_.push_back(ignition::math::Vector3d(0,-1,-1));
        corners_.push_back(ignition::math::Vector3d(0,-1,1));
        corners_.push_back(ignition::math::Vector3d(0,1,1));
        corners_.push_back(ignition::math::Vector3d(0,1,-1));
    };

    BoundingBoxPlugin::~BoundingBoxPlugin (){}


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

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&BoundingBoxPlugin::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);
    };

    void BoundingBoxPlugin::OnUpdate()
    {
        //get max xy
        int x_min = 0;
        int x_max = 0;
        int y_min = 0;
        int y_max = 0;
        for (auto corner : corners_){
            auto corner_pixel = this->parentSensor->Camera()->Project(corner); 

            if (corner_pixel[0] < x_min){
                if (corner_pixel[0] < 0)
                    x_min = 0;
                else
                    x_min = corner_pixel[0];
            }
            if (corner_pixel[0] > x_max){
                if (corner_pixel[0] > this->parentSensor->Camera()->ViewportWidth() - 1)
                    x_max = this->parentSensor->Camera()->ViewportWidth() - 1;
                else
                    x_max = corner_pixel[0];
            }
            if (corner_pixel[1] < y_min){
                if (corner_pixel[0] < 0)
                    y_min = 0;
                else
                    y_min = corner_pixel[0];
            }
            if (corner_pixel[1] > y_max){
                if (corner_pixel[0] > this->parentSensor->Camera()->ViewportHeight() - 1)
                    y_max = this->parentSensor->Camera()->ViewportHeight() - 1;
                else
                    y_max = corner_pixel[0];
            }
        }

        triton_interfaces::msg::DetectionBox bbox;
        bbox.x = x_min;
        bbox.y = y_min;
        bbox.width = x_max-x_min;
        bbox.height = y_max-y_min;

        //TODO: Publish message
    };
    
} // namespace triton_gazebo