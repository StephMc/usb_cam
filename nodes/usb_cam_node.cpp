/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Robert Bosch nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;

  // shared image message
  sensor_msgs::Image img_; // Holder for full image
  sensor_msgs::Image left_img_;
  sensor_msgs::Image right_img_;
  image_transport::CameraPublisher image_pub_;
  image_transport::CameraPublisher right_image_pub_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_,
      camera_info_url_, right_camera_info_url_, camera_type_;
  //std::string start_service_name_, start_service_name_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  int sample_rate_; // Set if you want grab frames slower than the set camera framerate, helps reduce cpu usage
  bool autofocus_, autoexposure_, auto_white_balance_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;



  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {
    // Controls mono or stereo image output
    node_.param("camera_type", camera_type_, std::string("mono"));
    if (camera_type_ != "mono" && camera_type_ != "stereo")
    {
      ROS_ERROR("Invalid camera type set, defaulting to mono camera");
      camera_type_ = "mono";
    }

    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    if (camera_type_ == "mono")
    {
      image_pub_ = it.advertiseCamera("image_raw", 1);
    }
    else
    {
      image_pub_ = it.advertiseCamera("left/image_raw", 1);
      right_image_pub_ = it.advertiseCamera("right/image_raw", 1);
    }

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    node_.param("sample_rate", sample_rate_, framerate_);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));

    // load the camera info
    if (camera_type_ == "mono")
    {
      cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, "camera", camera_info_url_));
    }
    else
    {
      // For stereo, the left camera uses the 'camera' namespace whilst the right camera use the 'right_camera" namespace
      ros::NodeHandle nhcl("~/left");
      ros::NodeHandle nhcr("~/right");
      cinfo_.reset(new camera_info_manager::CameraInfoManager(nhcl, "left_camera", camera_info_url_));
      node_.param("right_camera_info_url", right_camera_info_url_, std::string(""));
      right_cinfo_.reset(new camera_info_manager::CameraInfoManager(nhcr, "right_camera", right_camera_info_url_));
    }

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      ROS_WARN("Camera not calibrated");
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);

      if (camera_type_ == "stereo")
      {
        cinfo_->setCameraName("left_" + video_device_name_);
        right_cinfo_->setCameraName("right_" + video_device_name_);
        right_cinfo_->setCameraInfo(camera_info);
      }
    }


    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);

    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }

    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }

    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }

    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }

    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }

    if (camera_type_ == "stereo")
    {
      // Prepare the image buffers to hold half the raw image received 
      left_img_.data.resize((image_width_ / 2) * image_height_ * 3);
      right_img_.data.resize((image_width_ / 2) * image_height_ * 3);
    }
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);

    // grab the camera info for single or left image
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    ci->header.frame_id = img_.header.frame_id;
    ci->header.stamp = img_.header.stamp;

    if (camera_type_ == "mono")
    {
      image_pub_.publish(img_, *ci);
      return true;
    }

    /// Handle copying the data to split the stereo frames ///

    left_img_.header = img_.header;
    left_img_.height = img_.height;
    left_img_.width = img_.width / 2;
    left_img_.step = img_.step / 2;
    left_img_.encoding = img_.encoding;

    // Copy the data from the img to the left img
    for (int i = 0; i < img_.height; ++i)
    {
      memcpy(&(left_img_.data[i * left_img_.step]), &(img_.data[i * img_.step]), left_img_.step);
    }

    // publish the image
    image_pub_.publish(left_img_, *ci);

    /// RIGHT IMAGE ///
    sensor_msgs::CameraInfoPtr cir(new sensor_msgs::CameraInfo(right_cinfo_->getCameraInfo()));
    cir->header.frame_id = img_.header.frame_id;
    cir->header.stamp = img_.header.stamp;

    right_img_.header = img_.header;
    right_img_.height = img_.height;
    right_img_.width = img_.width / 2;
    right_img_.step = img_.step / 2;
    right_img_.encoding = img_.encoding;

    // Copy the data from the img to the left img
    for (int i = 0; i < img_.height; ++i)
    {
      memcpy(&(right_img_.data[i * right_img_.step]),
          &(img_.data[(i * img_.step) + (img_.step / 2)]),
          right_img_.step);
    }

    // publish the image
    right_image_pub_.publish(right_img_, *cir);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->sample_rate_);
    while (node_.ok())
    {
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }






};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "usb_cam");
  usb_cam::UsbCamNode a;
  a.spin();
  return EXIT_SUCCESS;
}
