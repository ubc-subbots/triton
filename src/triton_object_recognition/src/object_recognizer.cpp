#include "triton_object_recognition/object_recognizer.hpp"
#include <opencv2/dnn.hpp>
#include <boost/filesystem.hpp>
#include <rcl_yaml_param_parser/parser.h>
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using namespace std;
using namespace cv;
using namespace dnn;

namespace object_recognition
{
    ObjectRecognizer::ObjectRecognizer(const rclcpp::NodeOptions & options)
    : Node("object_recognizer", options) 
    {
        //Create publisher, subscriber, and service
        publisher_ = this->create_publisher<triton_interfaces::msg::DetectionBoxArray>("object_recognizer/out", 10);
        subscription_ = image_transport::create_subscription(this,"object_recognizer/in",bind(&ObjectRecognizer::subscriberCallback, this, _1),"raw");
        service_ = create_service<triton_interfaces::srv::ObjectDetection>("object_recognizer/recognize",bind(&ObjectRecognizer::serviceCallback, this, _1,_2));

        #if DEBUG_VISUALIZE
            debug_publisher_ = image_transport::create_publisher(this, "object_recognizer/debug");
        #endif

        //Populate parameters (values are not modified if parameters have not been declared)
        this->declare_parameter("model_folder", model_folder_);
        this->declare_parameter("weights_filename", weights_filename_);
        this->declare_parameter("cfg_filename", cfg_filename_);
        this->declare_parameter("weights_url", weights_url_);
        this->declare_parameter("cfg_url", cfg_url_);
        this->declare_parameter("classes", classes_);
        this->declare_parameter("conf_threshold", conf_threshold_);
        this->declare_parameter("nms_threshold", nms_threshold_);
        this->declare_parameter("scale", scale_);
        this->declare_parameter("inp_width", inp_width_);
        this->declare_parameter("inp_height", inp_height_);
        this->declare_parameter("swap_rb", swap_rb_);
        this->declare_parameter("mean", mean_);
        this->declare_parameter("backend", (int) backend_);
        this->declare_parameter("target", (int) target_);

        this->get_parameter("model_folder", model_folder_);
        this->get_parameter("weights_filename", weights_filename_);
        this->get_parameter("cfg_filename", cfg_filename_);
        this->get_parameter("weights_url", weights_url_);
        this->get_parameter("cfg_url", cfg_url_);
        this->get_parameter("classes", classes_);
        this->get_parameter("conf_threshold", conf_threshold_);
        this->get_parameter("nms_threshold", nms_threshold_);
        this->get_parameter("scale", scale_);
        this->get_parameter("inp_width", inp_width_);
        this->get_parameter("inp_height", inp_height_);
        this->get_parameter("swap_rb", swap_rb_);
        this->get_parameter("mean", mean_);
        int backend, target;
        if (this->get_parameter("backend", backend))
            backend_ = (Backend) backend;
        if (this->get_parameter("target", target))
            target_ = (Target) target;

        //Check model folder exists
        boost::filesystem::path model_folder = boost::filesystem::path(model_folder_);
        if (!boost::filesystem::is_directory(model_folder)){
            RCLCPP_ERROR(get_logger(),"Model folder not found");
        }

        //Check cfg file exists and downloads if it doesn't
        boost::filesystem::path model_config = model_folder / cfg_filename_;
        if (!boost::filesystem::exists(model_config)){
            RCLCPP_WARN(get_logger(),"Model weights not found. Downloading from " + cfg_url_);
            //Warning: This command is not portable
            string command = "wget -O " + model_config.string() + " " + cfg_url_;
            if (system(command.c_str())){
                RCLCPP_ERROR(get_logger(),"Model weights failed to download");
            };
        }
        
        //Check weights file exists and download if it doesn't
        boost::filesystem::path model_weights = model_folder / weights_filename_;
        if (!boost::filesystem::exists(model_weights)){
            RCLCPP_WARN(get_logger(),"Model config not found. Downloading from " + weights_url_);
            //Warning: This command is not portable
            string command = "wget -O " + model_config.string() + " " + weights_url_;
            if (system(command.c_str())){
                RCLCPP_ERROR(get_logger(),"Model config failed to download");
            };
        }
        
        //Load network
        net_ = std::make_shared<Net>();
        try {
        *net_ = readNet(model_weights.string(),model_config.string());
        } catch(Exception &e){
            RCLCPP_ERROR(get_logger(),e.what());
        }
        net_->setPreferableBackend(backend_);
        net_->setPreferableTarget(target_);
        RCLCPP_INFO(get_logger(),"ObjectRecognizer loaded successfully");
    }

    void ObjectRecognizer::subscriberCallback(const sensor_msgs::msg::Image::ConstSharedPtr & msg) const
    {
        publisher_->publish(process(*msg));
    }

    void ObjectRecognizer::serviceCallback(const triton_interfaces::srv::ObjectDetection::Request::SharedPtr request, 
            const triton_interfaces::srv::ObjectDetection::Response::SharedPtr response) const
    {
        response->boxes_out = process(request->image_in);
    }

    triton_interfaces::msg::DetectionBoxArray ObjectRecognizer::process(const sensor_msgs::msg::Image & msg) const
    {
        //Get image from message
        RCLCPP_INFO(this->get_logger(), "In object_recognizer");
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(),"cv_bridge exception: %s", e.what());
            return triton_interfaces::msg::DetectionBoxArray();
        }

        //Detect objects in image
        preprocess(cv_ptr->image);
        std::vector<cv::String> outNames = net_->getLayerNames();;
        std::vector<cv::Mat> outs;
        net_->forward(outs,outNames);

        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<Rect> boxes;
        postprocess(classIds,confidences,boxes,cv_ptr->image, outs);

        // Visualize when debugging
        #if DEBUG_VISUALIZE
            //Draw prediction boxes on input image
            auto drawPred = [&](int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
            {
                rectangle(frame, Point(left, top), Point(right, bottom), Scalar(0, 255, 0));

                std::string label = format("%.2f", conf);
                if (!classes_.empty())
                {
                    CV_Assert(classId < (int)classes_.size());
                    label = classes_[classId] + ": " + label;
                }

                int baseLine;
                Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                top = max(top, labelSize.height);
                rectangle(frame, Point(left, top - labelSize.height),
                        Point(left + labelSize.width, top + baseLine), Scalar::all(255), FILLED);
                putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.5, Scalar());
            };
            cv::Mat frame = cv_ptr->image;
            for (size_t idx = 0; idx < boxes.size(); ++idx)
            {
                Rect box = boxes[idx];
                drawPred(classIds[idx], confidences[idx], box.x, box.y,
                         box.x + box.width, box.y + box.height, frame);
            }
            sensor_msgs::msg::Image debug_msg;
            cv_bridge::CvImage debug_image;
            debug_image.image = frame;
            debug_image.toImageMsg(debug_msg);
            debug_publisher_.publish(debug_msg);
        #endif

        //Publish message with detected boxes
        auto message = triton_interfaces::msg::DetectionBoxArray();
        for (size_t i = 0; i<boxes.size(); i++){
            triton_interfaces::msg::DetectionBox out;
            out.class_id = classIds[i];
            out.confidence = confidences[i];
            out.x = boxes[i].x;
            out.y = boxes[i].y;
            out.height = boxes[i].height;
            out.width = boxes[i].width;
            message.boxes.push_back(out);
        }
        message.header.stamp = rclcpp::Time();
        return message;
    }

    void ObjectRecognizer::preprocess(const Mat& frame) const{

        static Mat blob;
        // Create a 4D blob from a frame.
        //if (inpSize.width <= 0) inpSize.width = frame.cols;
        //if (inpSize.height <= 0) inpSize.height = frame.rows;
        cv::Size inp_size;
        inp_size.width = inp_width_;
        inp_size.height = inp_height_;
        blob = blobFromImage(frame,(1.0/255.0),inp_size,mean_,swap_rb_,false);

        // Run model.
        net_->setInput(blob,"");
        if (net_->getLayer(0)->outputNameToIndex("im_info") != -1)
        {
            resize(frame, frame, inp_size);
            Mat imInfo = (Mat_<float>(1, 3) << inp_size.height, inp_size.width, 1.6f);
            net_->setInput(imInfo, "im_info");
        }
    }

    void ObjectRecognizer::postprocess(std::vector<int> & classIds, std::vector<float> & confidences, std::vector<Rect> & boxes, 
            cv::Mat & frame, const std::vector<Mat>& outs) const{
        static std::vector<int> outLayers = net_->getUnconnectedOutLayers();
        static std::string outLayerType = net_->getLayer(outLayers[0])->type;

        if (outLayerType == "DetectionOutput")
        {
            // Network produces output blob with a shape 1x1xNx7 where N is a number of
            // detections and an every detection is a vector of values
            // [batchId, classId, confidence, left, top, right, bottom]
            CV_Assert(outs.size() > 0);
            for (size_t k = 0; k < outs.size(); k++)
            {
                float* data = (float*)outs[k].data;
                for (size_t i = 0; i < outs[k].total(); i += 7)
                {
                    float confidence = data[i + 2];
                    if (confidence > conf_threshold_)
                    {
                        int left   = (int)data[i + 3];
                        int top    = (int)data[i + 4];
                        int right  = (int)data[i + 5];
                        int bottom = (int)data[i + 6];
                        int width  = right - left + 1;
                        int height = bottom - top + 1;
                        if (width <= 2 || height <= 2)
                        {
                            left   = (int)(data[i + 3] * frame.cols);
                            top    = (int)(data[i + 4] * frame.rows);
                            right  = (int)(data[i + 5] * frame.cols);
                            bottom = (int)(data[i + 6] * frame.rows);
                            width  = right - left + 1;
                            height = bottom - top + 1;
                        }
                        classIds.push_back((int)(data[i + 1]) - 1);  // Skip 0th background class id.
                        boxes.push_back(Rect(left, top, width, height));
                        confidences.push_back(confidence);
                    }
                }
            }
        }
        else if (outLayerType == "Region")
        {
            for (size_t i = 0; i < outs.size(); ++i)
            {
                // Network produces output blob with a shape NxC where N is a number of
                // detected objects and C is a number of classes + 4 where the first 4
                // numbers are [center_x, center_y, width, height]
                float* data = (float*)outs[i].data;
                for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
                {
                    Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                    Point classIdPoint;
                    double confidence;
                    minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                    if (confidence > conf_threshold_)
                    {
                        int centerX = (int)(data[0] * frame.cols);
                        int centerY = (int)(data[1] * frame.rows);
                        int width = (int)(data[2] * frame.cols);
                        int height = (int)(data[3] * frame.rows);
                        int left = centerX - width / 2;
                        int top = centerY - height / 2;

                        classIds.push_back(classIdPoint.x);
                        confidences.push_back((float)confidence);
                        boxes.push_back(Rect(left, top, width, height));
                    }
                }
            }
        }
        else
            CV_Error(Error::StsNotImplemented, "Unknown output layer type: " + outLayerType);

        // NMS is used inside Region layer only on DNN_BACKEND_OPENCV for another backends we need NMS in sample
        // or NMS is required if number of outputs > 1
        if (outLayers.size() > 1 || (outLayerType == "Region" ))
        {
            std::map<int, std::vector<size_t> > class2indices;
            for (size_t i = 0; i < classIds.size(); i++)
            {
                if (confidences[i] >= conf_threshold_)
                {
                    class2indices[classIds[i]].push_back(i);
                }
            }
            std::vector<Rect> nmsBoxes;
            std::vector<float> nmsConfidences;
            std::vector<int> nmsClassIds;
            for (std::map<int, std::vector<size_t> >::iterator it = class2indices.begin(); it != class2indices.end(); ++it)
            {
                std::vector<Rect> localBoxes;
                std::vector<float> localConfidences;
                std::vector<size_t> classIndices = it->second;
                for (size_t i = 0; i < classIndices.size(); i++)
                {
                    localBoxes.push_back(boxes[classIndices[i]]);
                    localConfidences.push_back(confidences[classIndices[i]]);
                }
                std::vector<int> nmsIndices;
                NMSBoxes(localBoxes, localConfidences, conf_threshold_, nms_threshold_, nmsIndices);
                for (size_t i = 0; i < nmsIndices.size(); i++)
                {
                    size_t idx = nmsIndices[i];
                    nmsBoxes.push_back(localBoxes[idx]);
                    nmsConfidences.push_back(localConfidences[idx]);
                    nmsClassIds.push_back(it->first);
                }
            }
            boxes = nmsBoxes;
            classIds = nmsClassIds;
            confidences = nmsConfidences;
        }
    }
} // namespace object_recognition