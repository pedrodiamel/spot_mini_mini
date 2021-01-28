#include "ros/ros.h"
#include "ball_chaser/WalkToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the /command service to drive the robot in the specified direction
void guide_robot(char com)
{
    ROS_INFO_STREAM(com);

    // Request given direction command character
    ball_chaser::WalkToTarget srv;
    srv.request.dir_command = com;

    // Call the /command service and pass the requested direction command
    if (!client.call(srv))
        ROS_ERROR("Failed to call service /command");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    /***
    Receives camera image and processes for the ball
    chaser application. Based on the position of a
    white ball in the image, sends the direction
    command for the robot to chase the ball, via the
    /command service. The direction is defined by a
    character following this convention:
        F : move forward
        L : turn left
        R : turn right
        S : stop

    Input:
        Image img: image sensor message
    
    Output:
        None
    ***/

    //-- Defining Parameters --//

    // White color value:
    int white_pixel = 255;

    // Obtain the width of each decision block
    int treshold = img.step/3;

    // Direction command:
    char com = 'S'; // Stop by default

    // Flag to indicate if the ball is in the image:
    bool no_ball = true;


    //-- Analising Image --//

    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {

            //Calculate horizontal pixel position:
            int j = i % img.step;

            //Check pixel position
            if (j < treshold) { //pixel on the left side
                //Go left:
                com = 'L';
                no_ball = false; //ball was found
            }
            else if (j >= treshold & j <= 2*treshold) { //pixel is in the middle
                //Go foward:
                com = 'F';
                no_ball = false; //ball was found
            }
            else if (j > 2*treshold) { //pixel on the right side
                //Go right:
                com = 'R';
                no_ball = false; //ball was found
            }

            break; //end loop
        }
    //finished loop:    
    //no_ball will still be true if white ball is not present in the image!
    
    }

    if (no_ball) { //no white ball in sight
        //Stop:
        com = 'S';
    }

    //Send desired velocity to drive_robot:
    guide_robot(com);             
    
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from /command
    client = n.serviceClient<ball_chaser::WalkToTarget>("/command");

    // Subscribe to /spot_camera/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub = n.subscribe("/spot_camera/image_raw", 10, process_image_callback);

    // Keep this node alive:
    ros::spin();

    return 0;
}
