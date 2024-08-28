/*
* This file is to be tested and not yet confirmed to be used
* Used to control both turtles and send boolean values on is_attacking channels for both turtles
* make sure to write a logic in game node that check if is_attacking is ture and within range of the turtle to decrement attacker's attacks number
* and health of the other turtle
*
* NOTE!!!! YOU MUST REMOVE (V2) FROM FILE NAME TO USE IT
*
* modifications on main file:
* ---------------------------
* 1- removed unnecessary keys
* 2- added attack key for wasd turtle (q key)
* 3- added attack key for arrows turtle (l key)
* 4- quit from (e key)
* 5 - added two functions to publish on is_attacking topics when pressing attacking keys
*
* Edited by: Hazem Essam 
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <stdio.h>
#ifndef _WIN32
# include <termios.h>
# include <unistd.h>
#else
# include <windows.h>
#endif

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_E 0x65  // Exit key
#define KEYCODE_L 0x6C  // Turtle 1 attack key
#define KEYCODE_Q 0x71  // Turtle 2 attack key


class KeyboardReader
{
public:
  KeyboardReader()
#ifndef _WIN32
    : kfd(0)
#endif
  {
#ifndef _WIN32
    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    struct termios raw;
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
#endif
  }
  void readOne(char * c)
  {
#ifndef _WIN32
    int rc = read(kfd, c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#else
    for(;;)
    {
      HANDLE handle = GetStdHandle(STD_INPUT_HANDLE);
      INPUT_RECORD buffer;
      DWORD events;
      PeekConsoleInput(handle, &buffer, 1, &events);
      if(events > 0)
      {
        ReadConsoleInput(handle, &buffer, 1, &events);
        if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
        {
          *c = KEYCODE_LEFT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_UP)
        {
          *c = KEYCODE_UP;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
        {
          *c = KEYCODE_RIGHT;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
        {
          *c = KEYCODE_DOWN;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x71)
        {
          *c = KEYCODE_Q;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x6C)
        {
          *c = KEYCODE_L;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x77)
        {
          *c = KEYCODE_W;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x61)
        {
          *c = KEYCODE_A;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x73)
        {
          *c = KEYCODE_S;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x64)
        {
          *c = KEYCODE_D;
          return;
        }
        else if (buffer.Event.KeyEvent.wVirtualKeyCode == 0x65)
        {
	 *c = KEYCODE_E;
	  return;
       }
      }
    }
#endif
  }
  void shutdown()
  {
#ifndef _WIN32
    tcsetattr(kfd, TCSANOW, &cooked);
#endif
  }
private:
#ifndef _WIN32
  int kfd;
  struct termios cooked;
#endif
};

KeyboardReader input;

class TeleopTurtle
{
public:
  TeleopTurtle();
  void keyLoop();

private:

  
  ros::NodeHandle nh_;
  ros::NodeHandle nh2;
  double linear_, angular_, l_scale_, a_scale_;
  ros::Publisher twist_pub_;
  ros::Publisher twist2pub_;
  ros::Publisher attack_pub_;
  ros::Publisher attack2_pub_;

  void turtle1_attack();
  void turtle2_attack();
  
};

TeleopTurtle::TeleopTurtle():
  linear_(0),
  angular_(0),
  l_scale_(2.0),
  a_scale_(2.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh2.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  nh2.param("scale_linear", l_scale_, l_scale_);

  twist_pub_ = nh_.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1);
  twist2pub_ = nh2.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1);

  //----------------------------------------------------------------
  // Attack publishers
  attack_pub_ = nh_.advertise<std_msgs::Bool>("turtle1/is_attacking", 10);
  attack2_pub_ = nh2.advertise<std_msgs::Bool>("turtle2/is_attacking", 10);

  // Initialize attacking state to false
  std_msgs::Bool msg;
  msg.data = false;
  ros::Duration(1).sleep();
  attack_pub_.publish(msg);
  attack2_pub_.publish(msg);
  //----------------------------------------------------------------
}

void quit(int sig)
{
  (void)sig;
  input.shutdown();
  ros::shutdown();
  exit(0);
}

//--------------------Attacking Publishers------------------------
void TeleopTurtle::turtle1_attack()
{
  std_msgs::Bool msg;
  msg.data = true;
  attack_pub_.publish(msg);
}

void TeleopTurtle::turtle2_attack()
{
  std_msgs::Bool msg;
  msg.data = true;
  attack2_pub_.publish(msg);
}
//----------------------------------------------------------------

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  
  TeleopTurtle teleop_turtle;

  signal(SIGINT,quit);

  teleop_turtle.keyLoop();
  quit(0);
  
  return(0);
}

void TeleopTurtle::keyLoop()
{
  char c;
  bool dirty=false;
  bool dirty2=false;

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the 1st Turtle, 'l' for attack");
  puts("Use WASD keys to move the 2nd Turtle, 'q' for attack");
  puts("Use 'e' to exit.");

  for(;;)
  {
    // get the next event from the keyboard  
    try
    {
      input.readOne(&c);
    }
    catch (const std::runtime_error &)
    {
      perror("read():");
      return;
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", c);
  
    switch(c)
    {
      case KEYCODE_LEFT:
        ROS_DEBUG("LEFT");
        angular_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_RIGHT:
        ROS_DEBUG("RIGHT");
        angular_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_UP:
        ROS_DEBUG("UP");
        linear_ = 1.0;
        dirty = true;
        break;
      case KEYCODE_DOWN:
        ROS_DEBUG("DOWN");
        linear_ = -1.0;
        dirty = true;
        break;
      case KEYCODE_W:
        ROS_DEBUG("UP2");
        linear_ = 1.0;
        dirty2 = true;
        break;
      case KEYCODE_A:
        ROS_DEBUG("LEFT2");
        angular_ = 1.0;
        dirty2 = true;
        break;
      case KEYCODE_S:
        ROS_DEBUG("DOWN2");
        linear_ = -1.0;
        dirty2 = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("RIGHT2");
        angular_ = -1.0;
        dirty2 = true;
        break;
      case KEYCODE_Q:
        ROS_DEBUG("2nd Turtle  attacking..");
        turtle2_attack();
        break;
      case KEYCODE_L:
        ROS_DEBUG("1st Turtle attacking..");
        turtle1_attack();
        break;
      case KEYCODE_E:
        ROS_DEBUG("quit");
        return;
    }
   

    geometry_msgs::Twist twist;
    geometry_msgs::Twist twist2;
    twist.angular.z = a_scale_*angular_;
    twist2.angular.z = a_scale_*angular_;
    twist.linear.x = l_scale_*linear_;
    twist2.linear.x = l_scale_*linear_;
    if(dirty ==true)
    {
      twist_pub_.publish(twist);    
      dirty=false;
    }
    else if(dirty2 == true)
    {
      twist2pub_.publish(twist2);    
      dirty2=false;
    }
  }


  return;
}


