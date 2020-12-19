#include "triton_gazebo/bounding_box_plugin.hpp"
#include "triton_interfaces/msg/detection_box.hpp"
using namespace gazebo;

namespace triton_gazebo
{
    BoundingBoxPlugin::BoundingBoxPlugin() : SensorPlugin()
    {
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

        // Load parameters for size/location of 3D box
        float size_x = 2;//Object's width
        if (_sdf->HasElement("size_x"))
            size_x = _sdf->GetElement("size_x")->Get<float>();
        else
            gzwarn << "[bounding_box] size_x not set. Default: 2.0." << std::endl;

        float size_y = 2;//Object's depth
        if (_sdf->HasElement("size_y"))
            size_y = _sdf->GetElement("size_y")->Get<float>();
        else
            gzwarn << "[bounding_box] size_y not set. Default: 2.0." << std::endl;

        float size_z = 2;//Object's height
        if (_sdf->HasElement("size_z"))
            size_z = _sdf->GetElement("size_z")->Get<float>();
        else
            gzwarn << "[bounding_box] size_z not set. Default: 2.0." << std::endl;
        
        float origin_x = 0;
        if (_sdf->HasElement("origin_x"))
            origin_x = _sdf->GetElement("origin_x")->Get<float>();
        else
            gzwarn << "[bounding_box] origin_x not set. Default: 0.0." << std::endl;

        float origin_y = 0;
        if (_sdf->HasElement("origin_y"))
            origin_y = _sdf->GetElement("origin_y")->Get<float>();
        else
            gzwarn << "[bounding_box] origin_y in y not set. Default: 0.0." << std::endl;

        float origin_z = 0;
        if (_sdf->HasElement("origin_z"))
            origin_z = _sdf->GetElement("origin_z")->Get<float>();
        else
            gzwarn << "[bounding_box] origin_z in z not set. Default: 0.0." << std::endl;

        corners_.clear();
        //Set up corners of bounding box
        double coords[3][2] = {  {origin_x-size_x/2, origin_x+size_x/2},
                                {origin_y-size_y/2, origin_y+size_y/2},
                                {origin_z-size_z/2, origin_z+size_z/2}};
        for (int i = 0; i<2; i++){
            for (int j = 0; j<2; j++){
                for (int k = 0; k<2; k++){
                    corners_.push_back(ignition::math::Vector3d(coords[0][i],coords[1][j],coords[2][k]));
                }
            }
        }

        // Connect to the sensor update event.
        this->updateConnection = this->parentSensor->ConnectUpdated(
            std::bind(&BoundingBoxPlugin::OnUpdate, this));

        // Make sure the parent sensor is active.
        this->parentSensor->SetActive(true);
    }

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
    }
    
} // namespace triton_gazebo