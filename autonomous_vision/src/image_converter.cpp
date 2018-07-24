

/*
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

class OpenCVWebCam {
	public:
		OpenCVWebCam() {
			cv::VideoCapture cap(CV_CAP_ANY); // open any camera
			if (!cap.isOpened()) {
				std::cout << "Could not open camera\n";
			}

			cv::namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

			while (1) {
				cv::Mat frame;
				if (cap.read(frame)) {
					cv::imshow("Webcam", frame);
				}
				if (cv::waitKey(30) == 27) {
					// if "esc" is pressed end the program
					std::cout << "Closing the program because esc pressed";
					break;
				}
			}
		}
		~OpenCVWebCam() {
			cv::destroyWindow("Webcam");
		}
};

int main(int argc, char** argv) {
	// set up ros
	ros::init(argc, argv, "opencv_test");
	OpenCVWebCam webcam;
	ROS_INFO("Webcam Tested");
	return 0;
}



*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("raspicam_node/image", 1,
      &ImageConverter::imageCb, this,image_transport::TransportHints("compressed"));
    image_pub_ = it_.advertise("raspicam_node/image_inv", 1);


    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    cv::Mat flip_image;
    cv::flip(cv_ptr->image,flip_image,0);
    cv_bridge::CvImage out_msg;
    out_msg.header   = cv_ptr->header; // Same timestamp and tf frame as input image
    out_msg.encoding = cv_ptr->encoding; // Or whatever
    out_msg.image    = flip_image; // Your cv::Mat
    

    // Update GUI Window
    //cv::imshow(OPENCV_WINDOW, flip_image);
    //cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}

