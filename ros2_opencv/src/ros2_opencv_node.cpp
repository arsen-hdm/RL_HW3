#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

using namespace cv;
using namespace std;
using namespace std::chrono_literals;
 
class MinimalImagePublisher : public rclcpp::Node {
public:
  MinimalImagePublisher() : Node("opencv_image_publisher"), count_(0) {
    subscription_ =
        this->create_subscription<sensor_msgs::msg::Image>("/videocamera", 10, std::bind(&MinimalImagePublisher::camera_callback, this, std::placeholders::_1));
    SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 5;
    params.maxThreshold = 220;
 
    // Filter by Area.
    params.filterByArea = false;
    /*params.minArea = 1500;*/
  
    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = 0.8;
 
    // Filter by Convexity
    params.filterByConvexity = false;
    /*params.minConvexity = 0.87;*/
 
    // Filter by Inertia
    params.filterByInertia = false;
    /*params.minInertiaRatio = 0.01;*/
 
    // Set up detector with params
    detector_ = SimpleBlobDetector::create(params);
    publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("blob_image", 10);
  }
 
private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received an image from /camera topic with width: %d and height: %d",
                msg->width, msg->height);
    // You can add further processing of the received image here if needed
    
    try
        {
            // Convertiamo l'immagine ROS in un'immagine OpenCV in scala di grigi
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            
            // Rilevamento dei blob
            vector<KeyPoint> keypoints;
            detector_->detect(cv_ptr->image, keypoints);

            // Disegnamo i keypoint come cerchi rossi sull'immagine
            Mat im_with_keypoints;
            drawKeypoints(cv_ptr->image, keypoints, im_with_keypoints, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            // Mostriamo l'immagine con i blob rilevati
            imshow("Blob Detection", im_with_keypoints);
            waitKey(1);  // Necessario per aggiornare la finestra OpenCV
            
            // Converto l'immagine modificata (con i blob rilevati) di nuovo in un messaggio ROS
            sensor_msgs::msg::Image::SharedPtr output_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", im_with_keypoints).toImageMsg();

            // Pubblico l'immagine con i blob rilevati
            publisher_->publish(*output_msg);
            RCLCPP_INFO(this->get_logger(), "Image with blobs published");
        }
        catch (const cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "Errore nella conversione dell'immagine: %s", e.what());
        }
  }
  sensor_msgs::msg::Image::SharedPtr msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  Ptr<SimpleBlobDetector> detector_;
  size_t count_;
};
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisher>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
