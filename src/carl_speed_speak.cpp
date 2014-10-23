#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <sstream>
#include <espeak/speak_lib.h>

int buff_len = 500;
int options = 0;
unsigned int *unique_identifier;
void* user_data;
bool beep = false;

void speak()
{
  if (!beep) {
    espeak_Synth("carl", 4, 0, POS_CHARACTER, 0, espeakCHARS_AUTO, unique_identifier, user_data);
  } else {
    espeak_Synth("beep", 4, 0, POS_CHARACTER, 0, espeakCHARS_AUTO, unique_identifier, user_data);
  }
  espeak_Synchronize();
}

void cmd_vel_cback(const geometry_msgs::Twist::ConstPtr& msg)
{
  float linear = msg->linear.x;
  int speed, pitch;

  if (linear < 0)
  {
    beep = true;
    speed = 1;
    pitch = 100;
  } else {
    beep = false;
    speed = (int) ((linear * 5000) + 500);
    pitch = (int) ((linear * 100) + 1);
  }
  espeak_SetParameter(espeakRATE, speed, 0);
  espeak_SetParameter(espeakPITCH, pitch, 0);
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_speed_espeak");

  // subscribe to cmd_vel
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_vel_cback);

  if (espeak_Initialize(AUDIO_OUTPUT_PLAYBACK, buff_len, NULL, options) == -1)
  {
    ROS_ERROR("espeak could not initialize!");
    return EXIT_FAILURE;
  }

  espeak_SetParameter(espeakRATE, 500, 0);
  espeak_SetParameter(espeakPITCH, 1, 0);

  ros::AsyncSpinner spinner(4);
  spinner.start();
  while (ros::ok())
  {
    speak();
  }

  return EXIT_SUCCESS;
}
