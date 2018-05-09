#include <ros/ros.h>
#include <ros/spinner.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>

#include <boost/format.hpp>

using namespace message_filters;
typedef image_transport::SubscriberFilter ImageSubscriber;
typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;

size_t counter=0;

boost::format left_format("left_%04d.bmp");
boost::format right_format("right_%04d.bmp");
std::string image_path = ".";

void callback(const sensor_msgs::Image::ConstPtr& left, const sensor_msgs::Image::ConstPtr& right) {
    cv_bridge::CvImageConstPtr cv_left, cv_right;
    try
    {
      cv_left = cv_bridge::toCvShare(left, sensor_msgs::image_encodings::BGR8 ); // left->encoding);
      cv_right = cv_bridge::toCvShare(right, sensor_msgs::image_encodings::BGR8 ); //left->encoding);

      cv::imwrite( image_path + "/" + boost::str( left_format % counter ), cv_left->image );
      cv::imwrite( image_path + "/" + boost::str( right_format % counter ), cv_right->image );
      ROS_INFO_STREAM("saved images to " << image_path);

      counter++;

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

}

int main(int argc, char ** argv)
{
	ros::init(argc, argv,"stereo_snapshot");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);
    
    ros::NodeHandle nhp("~");
    std::string camera_topic_left = "/camera/left/image_raw";
    std::string camera_topic_right = "/camera/right/image_raw";
    nhp.getParam("camera_topic_left", camera_topic_left);
    nhp.getParam("camera_topic_right", camera_topic_right);
    nhp.getParam("image_path", image_path);

	ImageSubscriber left(it, camera_topic_left, 1);
	ImageSubscriber right(it, camera_topic_right, 1);
	Synchronizer< MySyncPolicy > sync(MySyncPolicy(10), left, right);
	sync.registerCallback( boost::bind(&callback, _1, _2) );

    ROS_INFO_STREAM("Press ENTER to save stereo images, Ctrl+C when you're done!");

	while(ros::ok()) {
        std::cin.clear();
        std::cin.ignore();
        ros::spinOnce();
	}

	return 0;
}
