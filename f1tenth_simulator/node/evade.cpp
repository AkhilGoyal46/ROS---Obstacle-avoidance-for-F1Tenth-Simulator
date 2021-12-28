#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <std_msgs/Int32MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class Evader {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;

    // Listen for odom messages
    ros::Subscriber odom_sub;
    ros::Subscriber laser_sub;
    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;

    double steering_angle;
public:
    Evader() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("evade_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
	n.getParam("scan_topic", scan_topic);
        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
        odom_sub = n.subscribe(odom_topic, 1, &Evader::odom_callback, this);
	laser_sub = n.subscribe(scan_topic,1,&Evader::laser_callback,this);

    }
    
    float average(std::vector<float> data , int idx1 , int idx2){
    	float sum =0;
    	for (int i=idx1 ; i <= idx2 ; i++){
    		sum = sum + data[i];
    	}
    	return sum/(idx2 -idx1 +1);	
    }
    
    int obsAhead( std::vector<float> data) {
    	if (average( data , 528 , 550 )<= 1.7){
    		return 1;
    		}
    	else{
    		return 0;
    	}
    }
    
    int obsOnRight( std::vector<float> data) {
    	if (average( data , 500 , 528)<= .3){
    		return 1;
    		}
    	else{
    		return 0;
    	}
    }
    
    int obsOnLeft( std::vector<float> data) {
    	if (average( data , 550, 578)<= .3){
    		return 1;
    		}
    	else{
    		return 0;
    	}
    }
    
    float distToRight( std::vector<float> data) {
    	return average( data , 202 , 337);
    		
    }
  
   float distToLeft( std::vector<float> data) {
    	return average( data , 741 , 876);
    		
    }
  int lookForPath( std::vector<float> data){
  	float  max_val = 0 ;
  	int idx=0 ; 
  	for(int i = 270 ; i < 810 ; i++){
  		if ( max_val <= data[i]){
  			max_val = data[i];
  			idx = i;
  		}
  	}
  		
  		if (idx < 539){
  			return -1;
  		}
  		else {
  			return 1;
  		}
  	
  	
  }
    

    void laser_callback( const sensor_msgs::LaserScan & msg) {
    if ( obsAhead(msg.ranges) == 1){
    	steering_angle = lookForPath(msg.ranges)*0.6;
    
    	
    
    }
    else if ( distToRight(msg.ranges) != distToLeft(msg.ranges)){
    	steering_angle= (distToLeft(msg.ranges) -  distToRight(msg.ranges))*0.6/10;
    }
    
    else{
    	steering_angle = 0.0;
    
    }  
    }
    
    void odom_callback(const nav_msgs::Odometry & msg) {
        // publishing is done in odom callback just so it's at the same rate as the sim

        // initialize message to be published
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        /// SPEED CALCULATION:
        // set constant speed to be half of max speed
        drive_msg.speed = 2.0;


        /// STEERING ANGLE CALCULATION
        // random number between 0 and 1
        //double random = ((double) rand() / RAND_MAX); - old prog
        // good range to cause lots of turning
        //double range = max_steering_angle / 2.0; -- old prog
        // compute random amount to change desired angle by (between -range and range)
        //double rand_ang = range * random - range / 2.0; -- old prog

        // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
        /*random = ((double) rand() / RAND_MAX);
        if ((random > .8) && (prev_angle != 0)) {
            double sign_rand = rand_ang / std::abs(rand_ang);
            double sign_prev = prev_angle / std::abs(prev_angle);
            rand_ang *= sign_rand * sign_prev;
        } -- old prog */

        // set angle (add random change to previous angle)
        /*drive_msg.steering_angle = std::min(std::max(prev_angle + rand_ang, -max_steering_angle), max_steering_angle); -- 	 old code */
	drive_msg.steering_angle = steering_angle; // remove this one 

        // reset previous desired angle
        //prev_angle = drive_msg.steering_angle; -- old code

        // set drive message in drive stamped message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        drive_pub.publish(drive_st_msg);


    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "evader");
    Evader ev;
    ros::spin();
    return 0;
}
