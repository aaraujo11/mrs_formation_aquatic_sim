#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseActionResult.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <stdlib.h>     /* srand, rand */
#include <math.h>



// calculate number of robots in each swarm (single swarm for now)
#define N_Init 4    // initial number of robots in a swarm

float pose_r0 [3] = {0.0};      // X,Y,W
float pose_r1 [3] = {0.0};
float pose_r2 [3] = {0.0};
float pose_r3 [3] = {0.0};

bool once_r0 = 0, once_r1 = 0, once_r2 = 0, once_r3 = 0;
bool goal_r0 = 1, goal_r1 = 1, goal_r2 = 1, goal_r3 = 1;
bool inserted_desired_pose = false;

float XYRob[N_Init][2] = {0.0};                                             // robots position
float vXYRob[N_Init][2] = {0.0};                                            // robots velocity
float vx_t3 = 0, vy_t3 = 0, vx_t2 = 0, vy_t2 = 0, vx_t1 = 0, vy_t1 = 0;
float vmin = -0.1, vmax = 0.1;


// create desired shape
// XYDes = [-1 -1 1 1; 1 -1 -1 1]';
// polyDes = polyshape(XYDes(:,1),XYDes(:,2));
float XYDes[N_Init][2] = {{-2.0,2.0},{-2.0,-2.0},{2.0,-2.0},{2.0,2.0}};
float vXYDes = 0.0;

// Desired centroid pose goal
float centerDes[]={0.0, 0.0};
geometry_msgs::Quaternion robot_orientation;


void CB_robot_0_pose(const nav_msgs::Odometry& msg)
{
    pose_r0[0] = msg.pose.pose.position.x;
    pose_r0[1] = msg.pose.pose.position.y;
    pose_r0[2] = tf::getYaw (msg.pose.pose.orientation);
    once_r0 = true;
}


void CB_robot_1_pose(const nav_msgs::Odometry& msg)
{
    pose_r1[0] = msg.pose.pose.position.x;
    pose_r1[1] = msg.pose.pose.position.y;
    pose_r1[2] = tf::getYaw (msg.pose.pose.orientation);
    once_r1 = true;
}


void CB_robot_2_pose(const nav_msgs::Odometry& msg)
{
    pose_r2[0] = msg.pose.pose.position.x;
    pose_r2[1] = msg.pose.pose.position.y;
    pose_r2[2] = tf::getYaw (msg.pose.pose.orientation);
    once_r2 = true;
}


void CB_robot_3_pose(const nav_msgs::Odometry& msg)
{
    pose_r3[0] = msg.pose.pose.position.x;
    pose_r3[1] = msg.pose.pose.position.y;
    pose_r3[2] = tf::getYaw (msg.pose.pose.orientation);
    once_r3 = true;
}



void CB_robot_0_mb_result (const move_base_msgs::MoveBaseActionResult& msg)
{
    // status codes : http://docs.ros.org/kinetic/api/actionlib_msgs/html/msg/GoalStatus.html
    
    if(msg.status.status == 3){
    
        ROS_INFO("Code: %d | Robot_0 reach goal at pose: %f, %f, %f", msg.status.status, pose_r0[0], pose_r0[1], pose_r0[2]);
    
        
    } else {
       
      ROS_WARN("Code: %d | Robot_0 Aborted goal!", msg.status.status);

    }
    
    goal_r0 = true;
}

void CB_robot_1_mb_result (const move_base_msgs::MoveBaseActionResult& msg)
{
    if(msg.status.status == 3){
    
        ROS_INFO("Code: %d | Robot_1 reach goal at pose: %f, %f, %f", msg.status.status, pose_r1[0], pose_r1[1], pose_r1[2]);
    
    } else {
       
      ROS_WARN("Code: %d | Robot_1 Aborted goal!", msg.status.status);

    }    
    goal_r1 = true;
}

void CB_robot_2_mb_result (const move_base_msgs::MoveBaseActionResult& msg)
{
    if(msg.status.status == 3){
    
        ROS_INFO("Code: %d | Robot_2 reach goal at pose: %f, %f, %f", msg.status.status, pose_r2[0], pose_r2[1], pose_r2[2]);
    
        
    } else {
       
      ROS_WARN("Code: %d | Robot_2 Aborted goal!", msg.status.status);

    } 
    
    goal_r2 = true;
}

void CB_robot_3_mb_result (const move_base_msgs::MoveBaseActionResult& msg)
{
    if(msg.status.status == 3){
    
        ROS_INFO("Code: %d | Robot_3 reach goal at pose: %f, %f, %f", msg.status.status, pose_r3[0], pose_r3[1], pose_r3[2]);
    
    
    } else {
       
      ROS_WARN("Code: %d | Robot_3 Aborted goal!", msg.status.status);

    }    
    goal_r3 = true;
}


void CB_centroid_goal (const geometry_msgs::PoseStamped& msg)
{
    
    centerDes[0] = msg.pose.position.x;
    centerDes[1] = msg.pose.position.y;
    robot_orientation = msg.pose.orientation;
    
    double yaw_degrees = tf::getYaw (robot_orientation) * 180.0 / M_PI; // conversion to degrees
    if( yaw_degrees < 0 ) yaw_degrees += 360.0; // convert negative to positive angles
    
    inserted_desired_pose = true;
  
    ROS_INFO("Received new centroid pose goal: %f, %f and yaw_degrees: %f", centerDes[0], centerDes[1], yaw_degrees);
}



float RandomFloat(float min, float max)
{
    assert(max > min); 
    float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;  
    return (random*range) + min;
}

/*
void compute_centroid(float *xy_pose[][], float xy_centroid[], size_t nr_poses){
    
    for(int i=0; i<nr_poses; i++){
        
        xy_centroid[0] = xy_centroid[0] + xy_pose[i][0];
        xy_centroid[1] = xy_centroid[1] + xy_pose[i][1];
    }
    
    xy_centroid[0] = xy_centroid[0] / (float) nr_poses;
    xy_centroid[1] = xy_centroid[1] / (float) nr_poses;
    
} */


/*
void define_desiredShape(float min_dist_robots, char form_shape, size_t num_robots, float *final_pos)
{
    
   switch(form_shape) {
      case 's' :            // Square shape

        for(int i=0; i <num_robots; i++ ){
            final_pos[i][0]= min_dist_robots;
            final_pos[i][0]= min_dist_robots;
        }
          
          
         break;

      default :

   }

} */





int main(int argc, char **argv)
{

  ros::init(argc, argv, "mrs_robot_formation");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Publisher pub_goal_robot_0 = n.advertise<geometry_msgs::PoseStamped>("/robot_0/move_base_simple/goal", 1);
  ros::Publisher pub_goal_robot_1 = n.advertise<geometry_msgs::PoseStamped>("/robot_1/move_base_simple/goal", 1);
  ros::Publisher pub_goal_robot_2 = n.advertise<geometry_msgs::PoseStamped>("/robot_2/move_base_simple/goal", 1);
  ros::Publisher pub_goal_robot_3 = n.advertise<geometry_msgs::PoseStamped>("/robot_3/move_base_simple/goal", 1);

  
  
  ros::Subscriber sub_odom_robot_0 = n.subscribe("/robot_0/p3d_odom", 1, CB_robot_0_pose);
  ros::Subscriber sub_odom_robot_1 = n.subscribe("/robot_1/p3d_odom", 1, CB_robot_1_pose);
  ros::Subscriber sub_odom_robot_2 = n.subscribe("/robot_2/p3d_odom", 1, CB_robot_2_pose);
  ros::Subscriber sub_odom_robot_3 = n.subscribe("/robot_3/p3d_odom", 1, CB_robot_3_pose);
  
  
  ros::Subscriber sub_move_base_result_0 = n.subscribe("/robot_0/move_base/result", 1, CB_robot_0_mb_result);
  ros::Subscriber sub_move_base_result_1 = n.subscribe("/robot_1/move_base/result", 1, CB_robot_1_mb_result);
  ros::Subscriber sub_move_base_result_2 = n.subscribe("/robot_2/move_base/result", 1, CB_robot_2_mb_result);
  ros::Subscriber sub_move_base_result_3 = n.subscribe("/robot_3/move_base/result", 1, CB_robot_3_mb_result);
  
  ros::Subscriber sub_centroid_goal = n.subscribe("/mrs_centroid/goal", 1, CB_centroid_goal);
  
  
//  ROS_INFO("Robot_0 pose: %f, %f, %f", pose_r0[0], pose_r0[1], pose_r0[2]);
//  ROS_INFO("Robot_1 pose: %f, %f, %f", pose_r1[0], pose_r1[1], pose_r1[2]);
//  ROS_INFO("Robot_2 pose: %f, %f, %f", pose_r2[0], pose_r2[1], pose_r2[2]);
//  ROS_INFO("Robot_3 pose: %f, %f, %f", pose_r3[0], pose_r3[1], pose_r3[2]);
  
  ROS_WARN("Waiting for desire centroid pose at /mrs_centroid/goal topic | geometry_msgs::PoseStamped msg");
  
  
  /* initialize random seed: */
  srand (time(NULL));
  
  // % define random initial position of robots between [-1,1] 
  // XYRob = -1+2*rand(N_Init,2);
  // % define random initial velocity of robots between [-1,1]  (just for seeding)
  // vXYRob = -1+2*rand(N_Init,2);
  for(int i=0; i <N_Init; i++ ){
      
      XYRob[i][0] = RandomFloat(-1.0, 1.0);
      XYRob[i][1] = RandomFloat(-1.0, 1.0);
   
      vXYRob[i][0] = RandomFloat(-1.0, 1.0);
      vXYRob[i][1] = RandomFloat(-1.0, 1.0);
  }
  
  
  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
      
    if(once_r0 && once_r1 && once_r2 && once_r3 && inserted_desired_pose) {  // Make sure get the robot initial pose
      
        
        
        if(goal_r0 && goal_r1 && goal_r2 && goal_r3){
            
            // reset goal flags
            goal_r0 = false;
            goal_r1 = false;
            goal_r2 = false;
            goal_r3 = false;
            inserted_desired_pose = false;
        
        
        
            ///     % 1. Compute the centroid of the group
            ///     centerRob = mean(XYRob); % desired center of mass
            
                float centerRob[2]={0.0};
                
                for(int i=0; i<N_Init; i++){
                
                    centerRob[0] = centerRob[0] + XYRob[i][0];
                    centerRob[1] = centerRob[1] + XYRob[i][1];
                }
            
                centerRob[0] = centerRob[0] / (float) N_Init;
                centerRob[1] = centerRob[1] / (float) N_Init;
                
                ROS_INFO("centerRob: %f, %f", centerRob[0], centerRob[1]);
                
                
                /// % 2. Compute angle of vectors between an horizontal line in the 
                /// %    centroid and robots (atan2)
                //  angRob = atan2(centerRob(1,2) - XYRob(:,2),centerRob(1,1) - XYRob(:,1));
                
                float angRob[N_Init] = {0.0};
                
                for(int i=0; i<N_Init; i++){
                
                    angRob[i] = atan2(centerRob[0] - XYRob[i][0], centerRob[1] - XYRob[i][1]);
                }
                
                ROS_INFO("angRob: %f, %f, %f, %f", angRob[0], angRob[1],angRob[2],angRob[3]);
                
                
                /// % 3. Sort the vertices according to this angle
                /// [~,isortRob] = sort(angRob);
                /// XYRobPol = XYRob(isortRob,:);
                
                
                
                /// %% compute individual fitness (cost) function
                /// centerDes = mean(XYDes); % desired center of mass
                /// for iN=1:N_Init
                ///  for iP=1:N_Init
                ///      A(iN,iP) = sqrt((XYRob(iN,1) - XYDes(iP,1)).^2 + (XYRob(iN,2) - XYDes(iP,2)).^2);
                ///   end
                /// end
                /// vx = vXYRob(:,1);
                /// vy = vXYRob(:,2);
                
                // Set goal pose
                //float centerDes[]={5.0, 5.0};
                
                // Udpate desired robot poses, using the centroid value or desired goal position
                for(int i=0; i < N_Init; i++){
                        
                        XYDes[i][0] = XYDes[i][0] + centerDes[0];
                        XYDes[i][1] = XYDes[i][1] + centerDes[1];
                }
                
                ROS_INFO("XDes: %f, %f, %f, %f", XYDes[0][0], XYDes[1][0],XYDes[2][0],XYDes[3][0]);
                ROS_INFO("YDes: %f, %f, %f, %f", XYDes[0][1], XYDes[1][1],XYDes[2][1],XYDes[3][1]);
                
                
                float a_fit[N_Init][N_Init]={0.0};
                
                for(int i=0; i < N_Init; i++){
                    
                    for(int j=0; j<N_Init; j++){
                        
                        a_fit[i][j] =  sqrt( pow( XYRob[i][0] - XYDes[j][0] ,2) +  pow( XYRob[i][1] - XYDes[j][1] ,2));
                        
                    }  
                }
                
                float vx[N_Init]={0.0};
                float vy[N_Init]={0.0};
                
                for(int i=0; i < N_Init; i++){
                        
                    vx[i] = vXYRob[i][0];
                    vy[i] = vXYRob[i][1];
                }
                
                
                ROS_INFO("vx: %f, %f, %f, %f", vx[0], vx[1],vx[2],vx[3]);
                ROS_INFO("vy: %f, %f, %f, %f", vy[0], vy[1],vy[2],vy[3]);
                
                
                
            /// for iP=1:N_Init
            ///  [~,iNmin] = nanmin(A(:,iP));
            ///  vx(iNmin(1),1) = -0.05*(XYRob(iNmin(1),1) - XYDes(iP,1));
            ///  vy(iNmin(1),1) = -0.05*(XYRob(iNmin(1),2) - XYDes(iP,2));
            ///  A(iNmin(1),:) = NaN;
            /// end
                
                
                // Get N min
                float n_min[N_Init] = {0.0};
                int idx_min[N_Init] = {0};
                
                for(int i=0; i < N_Init; i++){
                    
                    for(int j=0; j < N_Init; j++){
                        
                        if(a_fit[i][j] < n_min[i] || j == 0){   // feed with 1st value and check the min
                            n_min[i] = a_fit[i][j];
                            idx_min[i] = j;
                        }  
                    }
                }
                
                
                // Compute vx and vy
                for(int i=0; i < N_Init; i++){
                    
                    vx[idx_min[i]] = -0.05*(XYRob[idx_min[i]][0] - XYDes[i][0]);
                    vy[idx_min[i]] = -0.05*(XYRob[idx_min[i]][1] - XYDes[i][1]); 

                } 
                
                ROS_INFO("vx_: %f, %f, %f, %f", vx[0], vx[1],vx[2],vx[3]);
                ROS_INFO("vy_: %f, %f, %f, %f", vy[0], vy[1],vy[2],vy[3]);
                
                // Velocities protetion
                /// vx = (vx >= vmax).*vmax + (vx <= vmin).*vmin + (vx > vmin & vx < vmax).*vx;
                /// vy = (vy >= vmax).*vmax + (vy <= vmin).*vmin + (vy > vmin & vy < vmax).*vy;      
                
                for(int i=0; i < N_Init; i++){
                    
                    // vx
                    if(vx[i] >= vmax){
                        vx[i] = vmax;
                    } else if(vx[i] <= vmin){
                        vx[i] = vmin;
                    }
                    
                    //vy
                    if(vy[i] >= vmax){
                        vy[i] = vmax;
                    } else if(vy[i] <= vmin){
                        vy[i] = vmin;
                    }            
                }
                
                /// % update variable
                /// vXYRob = [vx vy];
                /// % vXYRob
                /// %% new desired position - to feed navigation stack
                /// XYRob_ = XYRob + vXYRob;
                /// %% let us here, for simulation purposes, the robot is already there
                /// XYRob = XYRob_;  
                
                // update variable
                for(int i=0; i < N_Init; i++){
                    vXYRob[i][0] = vx[i];
                    vXYRob[i][1] = vy[i];
                }
                
                float XYRob_[N_Init][2]={0.0};
                
                // new position to to feed navigation stack 
                for(int i=0; i < N_Init; i++){
                    XYRob_[i][0] = XYRob[i][0] + vXYRob[i][0];
                    XYRob_[i][1] = XYRob[i][1] + vXYRob[i][1];
                    
                    XYRob_[i][0] = XYRob[i][0] * 10.0;
                    XYRob_[i][1] = XYRob[i][1] * 10.0;
                }
                
                
                ROS_INFO("goal_x: %f, %f, %f, %f", XYRob_[0][0], XYRob_[1][0],XYRob_[2][0],XYRob_[3][0]);
                ROS_INFO("goal_y: %f, %f, %f, %f", XYRob_[0][1], XYRob_[1][1],XYRob_[2][1],XYRob_[3][1]);
                
                geometry_msgs::PoseStamped goalXY_0, goalXY_1, goalXY_2, goalXY_3; 
                
                goalXY_0.pose.position.x = XYRob_[0][0];
                goalXY_0.pose.position.y = XYRob_[0][1];
                goalXY_0.pose.orientation = robot_orientation;
                goalXY_0.header.frame_id = "robot_0/odom";
                
                goalXY_1.pose.position.x = XYRob_[1][0];
                goalXY_1.pose.position.y = XYRob_[1][1];
                goalXY_1.pose.orientation = robot_orientation;
                goalXY_1.header.frame_id = "robot_1/odom";
                
                goalXY_2.pose.position.x = XYRob_[2][0];
                goalXY_2.pose.position.y = XYRob_[2][1];
                goalXY_2.pose.orientation = robot_orientation;
                goalXY_2.header.frame_id = "robot_2/odom";
                
                goalXY_3.pose.position.x = XYRob_[3][0];
                goalXY_3.pose.position.y = XYRob_[3][1];
                goalXY_3.pose.orientation = robot_orientation;
                goalXY_3.header.frame_id = "robot_3/odom";
                
                pub_goal_robot_0.publish(goalXY_0);
                pub_goal_robot_1.publish(goalXY_1);
                pub_goal_robot_2.publish(goalXY_2);
                pub_goal_robot_3.publish(goalXY_3); 
                
                ROS_INFO("Waiting for robots reach their goal...");                
            }
    }
        
     
     ros::spinOnce();
     loop_rate.sleep();

   
  }


  return 0;
}
