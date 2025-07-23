#include "astarPlanner/bezier_curve_path.h"


SubscribeAndPublishPath::SubscribeAndPublishPath()
{
    //发布优化后的曲线
    // pub_ = n_.advertise<nav_msgs::Path>("/smooth_path", 1);
    // pub1_ = n_.advertise<nav_msgs::Path>("/blank_smooth_path", 1);
    //接收A*搜索到的路径
    // sub_ = n_.subscribe("/nav_path", 1, &SubscribeAndPublishPath::callback, this);
    // sub1_ = n_.subscribe("/nav_path", 1, &SubscribeAndPublishPath::callback_blank, this);
}
 
  // void callback(const nav_msgs::Path& input)
  // {
  //   nav_msgs::Path output;
  //   createCurve(input,output);//对A*路径进行优化
  //   pub_.publish(output);
  // }

  // void callback_blank(const nav_msgs::Path& input)
  // {
  //   nav_msgs::Path intermidiate;//中间路径
  //   nav_msgs::Path output;
  //   deal_path(input,intermidiate,5);//将输入路径按照给定的距离间隔进行采样，然后将采样点存储到输出路径中
  //   createCurve(intermidiate,output);
  //   pub1_.publish(output);
  // }


  //贝塞尔曲线优化
void SubscribeAndPublishPath::createCurve(const nav_msgs::Path& originPoint,nav_msgs::Path& output){     
    float scale = 0.6;
    int originCount=originPoint.poses.size();//原路径位姿点数  
    float midpoints_x[originCount];
    float midpoints_y[originCount];          
    for(int i = 0 ;i < originCount ; i++){      
        int nexti = (i + 1) % originCount;  
        midpoints_x[i] = (originPoint.poses[i].pose.position.x + originPoint.poses[nexti].pose.position.x)/2.0;  
        midpoints_y[i] = (originPoint.poses[i].pose.position.y + originPoint.poses[nexti].pose.position.y)/2.0;  
    }      
         
    float extrapoints_x[2 * originCount];
    float extrapoints_y[2 * originCount];
    for(int i = 0 ;i < originCount ; i++){  
         int nexti = (i + 1) % originCount;  
         int backi = (i + originCount - 1) % originCount;    
         float midinmid_x = (midpoints_x[i] + midpoints_x[backi])/2.0;  
         float midinmid_y = (midpoints_y[i] + midpoints_y[backi])/2.0;  
         float offsetx = originPoint.poses[i].pose.position.x - midinmid_x;  
         float offsety = originPoint.poses[i].pose.position.y - midinmid_y;  
         int extraindex = 2 * i;  
         extrapoints_x[extraindex] = midpoints_x[backi] + offsetx;  
         extrapoints_y[extraindex] = midpoints_y[backi] + offsety;      
         float addx = (extrapoints_x[extraindex] - originPoint.poses[i].pose.position.x) * scale;  
         float addy = (extrapoints_y[extraindex] - originPoint.poses[i].pose.position.y) * scale;  
         extrapoints_x[extraindex] = originPoint.poses[i].pose.position.x + addx;  
         extrapoints_y[extraindex] = originPoint.poses[i].pose.position.y + addy;  
           
         int extranexti = (extraindex + 1)%(2 * originCount);  
         extrapoints_x[extranexti] = midpoints_x[i] + offsetx;  
         extrapoints_y[extranexti] = midpoints_y[i] + offsety;     
         addx = (extrapoints_x[extranexti] - originPoint.poses[i].pose.position.x) * scale;  
         addy = (extrapoints_y[extranexti] - originPoint.poses[i].pose.position.y) * scale;  
         extrapoints_x[extranexti] = originPoint.poses[i].pose.position.x + addx;  
         extrapoints_y[extranexti] = originPoint.poses[i].pose.position.y + addy;  
           
    }      
      
    float controlPoint_x[4];
    float controlPoint_y[4];
    float controlPoint_bn_x[3];
    float controlPoint_bn_y[4];
    output.header.stamp = ros::Time::now();
    output.header.frame_id = "map";
    output.poses.clear();
    geometry_msgs::PoseStamped pathVehicle;   
                    pathVehicle.header.frame_id = "map";
                pathVehicle.header.stamp = ros::Time::now();
    for(int i = 0 ;i < originCount-1; i++){
           if (i==0)
             {
              controlPoint_bn_x[0] = originPoint.poses[i].pose.position.x;
              controlPoint_bn_y[0] = originPoint.poses[i].pose.position.y;  
              int extraindex = 2 * i;  
              // controlPoint_bn_x[1] = extrapoints_x[extraindex + 1];
              // controlPoint_bn_y[1] = extrapoints_y[extraindex + 1];
              int extranexti = (extraindex + 2) % (2 * originCount);  
              controlPoint_bn_x[1] = extrapoints_x[extranexti];
              controlPoint_bn_y[1] = extrapoints_y[extranexti];    
              int nexti = (i + 1) % originCount;  
              controlPoint_bn_x[2] = originPoint.poses[nexti].pose.position.x;
              controlPoint_bn_y[2] = originPoint.poses[nexti].pose.position.y;      
              float u = 0;  
              while(u <= 1){  
                float px = bezier2func(u,controlPoint_bn_x);//二阶贝塞尔曲线  
                float py = bezier2func(u,controlPoint_bn_y);    

                pathVehicle.pose.position.x = px;
                pathVehicle.pose.position.y = py;
                pathVehicle.pose.position.z = 0;
                output.poses.push_back(pathVehicle);
                u += 0.005;  
              }      
            }else if(i==originCount-2){
              controlPoint_bn_x[0] = originPoint.poses[i].pose.position.x;
              controlPoint_bn_y[0] = originPoint.poses[i].pose.position.y;  
              int extraindex = 2 * i;  
              controlPoint_bn_x[1] = extrapoints_x[extraindex + 1];
              controlPoint_bn_y[1] = extrapoints_y[extraindex + 1];
              //int extranexti = (extraindex + 2) % (2 * originCount);  
              //controlPoint_bn_x[1] = extrapoints_x[extranexti];
              //controlPoint_bn_y[1] = extrapoints_y[extranexti];    
              int nexti = (i + 1) % originCount;  
              controlPoint_bn_x[2] = originPoint.poses[nexti].pose.position.x;
              controlPoint_bn_y[2] = originPoint.poses[nexti].pose.position.y;      
              float u = 0;  
              while(u <= 1){  
                float px = bezier2func(u,controlPoint_bn_x);  
                float py = bezier2func(u,controlPoint_bn_y);    
                // pathVehicle.header.frame_id = "map";
                // pathVehicle.header.stamp = ros::Time(0);
                pathVehicle.pose.position.x = px;
                pathVehicle.pose.position.y = py;
                pathVehicle.pose.position.z = 0;
                output.poses.push_back(pathVehicle);
                u += 0.005;
              } 
            }else{           
              controlPoint_x[0] = originPoint.poses[i].pose.position.x;
              controlPoint_y[0] = originPoint.poses[i].pose.position.y;  
              int extraindex = 2 * i;  
              controlPoint_x[1] = extrapoints_x[extraindex + 1];
              controlPoint_y[1] = extrapoints_y[extraindex + 1];  
              int extranexti = (extraindex + 2) % (2 * originCount);  
              controlPoint_x[2] = extrapoints_x[extranexti];
              controlPoint_y[2] = extrapoints_y[extranexti];  
              int nexti = (i + 1) % originCount;  
              controlPoint_x[3] = originPoint.poses[nexti].pose.position.x;
              controlPoint_y[3] = originPoint.poses[nexti].pose.position.y;      
              float u = 0;  
              while(u <= 1){  
                float px = bezier3func(u,controlPoint_x);//三阶贝塞尔曲线  
                float py = bezier3func(u,controlPoint_y);    
                // pathVehicle.header.frame_id = "map";
                // pathVehicle.header.stamp = ros::Time(0);
                pathVehicle.pose.position.x = px;
                pathVehicle.pose.position.y = py;
                pathVehicle.pose.position.z = 0;
                output.poses.push_back(pathVehicle);
                u += 0.005;  
              }
            }  
      }
}   


//三阶贝塞尔曲线  
float SubscribeAndPublishPath::bezier3func(float uu,float* controlPoint){  
   float part0 = controlPoint[0] * (1-uu) * (1-uu) * (1-uu);  
   float part1 = 3 * controlPoint[1] * uu * (1-uu) * (1 - uu);  
   float part2 = 3 * controlPoint[2] * uu * uu * (1 - uu);  
   float part3 = controlPoint[3] * uu * uu * uu;     
   return part0 + part1 + part2 + part3;   
}   


//二阶贝塞尔曲线  
float SubscribeAndPublishPath::bezier2func(float uu,float* controlPoint){  
   float part0 = controlPoint[0] * (1-uu) * (1-uu);  
   float part1 = 2 * controlPoint[1] * uu * (1-uu);  
   float part2 = controlPoint[2] * uu * uu ;     
   return part0 + part1 + part2 ;   
}
  

//将输入路径按照给定的距离间隔进行采样，然后将采样点存储到输出路径中
void SubscribeAndPublishPath::deal_path(const nav_msgs::Path& input,nav_msgs::Path& output,int distance){
    output.header.stamp = ros::Time::now();//设置输出路径的时间戳为0
    output.header.frame_id = "map";
    output.poses.clear();
    geometry_msgs::PoseStamped pathVehicle;
    pathVehicle.header.frame_id = "map";
    pathVehicle.header.stamp = ros::Time::now();
    int length = input.poses.size();
    for (int i = 0; i < (length / distance); i++) {
    pathVehicle.pose.position.x = input.poses[i*distance].pose.position.x;
    pathVehicle.pose.position.y = input.poses[i*distance].pose.position.y;
    pathVehicle.pose.position.z = 0;
    output.poses.push_back(pathVehicle);
  }
  //The last point
  pathVehicle.header.frame_id = "map";
  pathVehicle.header.stamp = ros::Time::now();
  pathVehicle.pose.position.x = input.poses[length-1].pose.position.x;
  pathVehicle.pose.position.y = input.poses[length-1].pose.position.y;
  pathVehicle.pose.position.z = 0;
  output.poses.push_back(pathVehicle);
}



 
// int main(int argc, char **argv)
// {

//   ros::init(argc, argv, "subscribe_and_publish_smooth_path");
 
//   SubscribeAndPublishPath SAPObject;
 
//   ros::spin();
 
//   return 0;
// }