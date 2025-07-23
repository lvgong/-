#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <set>
#include <tf/transform_listener.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;
// sensor_msgs/LaserScan


// tool function
int is_in_map(const int x, const int y, const nav_msgs::OccupancyGrid &map_in);
int is_in_map(const float x, const float y, const nav_msgs::OccupancyGrid &map_in);
int xy_to_index(const float &x, const float &y, int &xi, int &yi, const nav_msgs::OccupancyGrid &map_in);
int index_to_float(const int &xi, const int &yi, float &x, float &y, const nav_msgs::OccupancyGrid &map_in);
void copy_map(const nav_msgs::OccupancyGrid &input, nav_msgs::OccupancyGrid &output);
void find_gezi(int hang,int counts,int* a,int floor,set<int>& s);

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //发布地图数据
	  pub_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_data", 1);
	  // inflate, pub_inflate_
    pub2_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_inflationdata_one", 1);
    //发布膨胀后的占用栅格
    pub3_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_inflationdata_two", 1);
    pub4_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_inflationdata_three", 1);

    //不膨胀接收原地图数据
    sub_  = n_.subscribe("/map", 1, &SubscribeAndPublish::callback, this);
    //获取原地图后的回调，进行膨胀（障碍物各膨胀1格）
    sub1_ = n_.subscribe("/map", 1, &SubscribeAndPublish::callback_inflation_one, this);
    //获取原地图后的回调，进行膨胀（障碍物各膨胀2格）
    sub2_ = n_.subscribe("/map", 1, &SubscribeAndPublish::callback_inflation_two, this);
    sub3_ = n_.subscribe("/map", 1, &SubscribeAndPublish::callback_inflation_three, this);

    //发布雷达扫描更新后的地图
    pub1_ = n_.advertise<nav_msgs::OccupancyGrid>("/map_with_scan", 1);
    //获取雷达数据
    sub_can_ = n_.subscribe("/scan", 1, &SubscribeAndPublish::SCAN_callback, this);
  }
  
  ros::Publisher get_pub2(){return pub2_;}

  nav_msgs::OccupancyGrid get_carto_map_inflation(){return carto_map_inflation;}
  

  void callback(const nav_msgs::OccupancyGrid& input)
  {
	  //nav_msgs::OccupancyGrid &output = carto_map_grid;
    nav_msgs::OccupancyGrid output;
    carto_map_ = input;
    int counts=input.info.width * input.info.height;
	  int a[counts] = { };
    output.header.seq = input.header.seq; 
    output.header.frame_id = input.header.frame_id;
    output.header.stamp = input.header.stamp;
    output.info.map_load_time = input.info.map_load_time;
    output.info.resolution = input.info.resolution;
    //printf("**********callback res: %f\n",carto_map_grid.info.resolution);
    output.info.width = input.info.width;
    output.info.height = input.info.height;
    output.info.origin = input.info.origin;
    //(sizeof(input.data)/sizeof(input.data[0]))
    for(int i=0; i<counts;i++)
    {
        //设置为占用
        if ((53<(int)input.data[i]) && ((int)input.data[i]<101))
          a[i]=100;
    }
	   std::vector<signed char> p(a, a+input.info.width*input.info.height);
	   output.data=p;
     //carto_map_grid=output;
     copy_map(output,carto_map_grid);//备份
    //  ros::Rate loop_rate(15);
    //  while (ros::ok())
    //  {
    //   pub_.publish(output);
    //   ros::spinOnce();
    //   loop_rate.sleep();
    // }
	  pub_.publish(output);
    is_has_map = true;//有地图了
  }

  void callback_inflation_one(const nav_msgs::OccupancyGrid& input)
  {
    nav_msgs::OccupancyGrid output;
    int counts=input.info.width * input.info.height;
    int a[counts] = {-1};
    int hang = input.info.width;
    int lie = input.info.height;
    output.header.seq = input.header.seq; 
    output.header.frame_id = input.header.frame_id;
    output.header.stamp = input.header.stamp;
    output.info.map_load_time = input.info.map_load_time;
    output.info.resolution = input.info.resolution;
    output.info.width = hang;
    output.info.height = lie;
    output.info.origin = input.info.origin;
    for(int i=0; i<counts;i++)
       {
        if((-1<(int)input.data[i]) && ((int)input.data[i]<54)){
          a[i]=0;
        }else if ((53<(int)input.data[i]) && ((int)input.data[i]<101))
        {
          a[i]=100;
        }
        else{
          a[i]=0;
        }
    }
    //set<int> o=find_gezi(hang,counts,a);
    //set<int>::iterator iter;
    set<int> o;
    find_gezi(hang,counts,a,1,o);
    // for (iter = o.begin() ; iter != o.end() ; ++iter)
    // {
    //   if (-1<*iter && *iter<counts)
    //   {
    //     a[*iter]=100;
    //   }
    // }
    std::vector<signed char> p(a, a+counts);
    output.data=p;
    copy_map(output,carto_map_inflation);
    pub2_.publish(output);
  }


  //对接收到的占用栅格数据进行膨胀
  void callback_inflation_two(const nav_msgs::OccupancyGrid& input)
  {
    nav_msgs::OccupancyGrid output;
    int counts=input.info.width * input.info.height;//地图尺寸
    int a[counts] = {-1};
    int hang = input.info.width;//行
    int lie = input.info.height;//列
    output.header.seq = input.header.seq; 
    output.header.frame_id = input.header.frame_id;
    output.header.stamp = input.header.stamp;
    output.info.map_load_time = input.info.map_load_time;
    output.info.resolution = input.info.resolution;
    //output.info.resolution = input.info.resolution*2;
    output.info.width = hang;
    output.info.height = lie;
    output.info.origin = input.info.origin;//地图坐标系原点
    for(int i=0; i<counts;i++)
       {
        //代价值在0-53之间的为空闲
        if((-1<(int)input.data[i]) && ((int)input.data[i]<54)){
          a[i]=0;
          //代价值在54-100之间的为占用，赋100代价值
        }else if ((53<(int)input.data[i]) && ((int)input.data[i]<101))
        {
          a[i]=100;
        }
        //代价值为-1的未知区域标记为空闲
        else{
          a[i]=0;
        }
    }
    set<int> o;
    //a为膨胀前/后，o为标记膨胀的栅格，这里的2表示现有的障碍物每个膨胀2格
    find_gezi(hang,counts,a,2,o);
    // set<int>::iterator iter;
    // for (iter = o.begin() ; iter != o.end() ; ++iter)
    // {
    //   if (-1<*iter && *iter<counts)
    //   {
    //     a[*iter]=100;
    //   }
    // }
/*创建一个名为p的std::vector<signed char> 类型的对象，其中a是一个指向 signed char 类型数组的指针，counts 是该数组中元素的数量。
构造函数使用指针 a 和 counts 来初始化 p 中的元素。具体来说，它使用从 a 开始、长度为 counts 的一段连续内存空间中的值来初始化 p 。这段内存空间包含 counts 个 signed char 类型的值，它们被复制到了 p 中。
因此，p 中的元素将与 a 数组中的元素具有相同的值和顺序。*/
    std::vector<signed char> p(a, a+counts);
    output.data=p;
    pub3_.publish(output);
  }

  void callback_inflation_three(const nav_msgs::OccupancyGrid& input)
  {
    nav_msgs::OccupancyGrid output;
    int counts=input.info.width * input.info.height;
    int a[counts] = {-1};
    int hang = input.info.width;
    int lie = input.info.height;
    output.header.seq = input.header.seq; 
    output.header.frame_id = input.header.frame_id;
    output.header.stamp = input.header.stamp;
    output.info.map_load_time = input.info.map_load_time;
    output.info.resolution = input.info.resolution;
    output.info.width = hang;
    output.info.height = lie;
    output.info.origin = input.info.origin;
    for(int i=0; i<counts;i++)
       {
        if((-1<(int)input.data[i]) && ((int)input.data[i]<54)){
          a[i]=0;
        }else if ((53<(int)input.data[i]) && ((int)input.data[i]<101))
        {
          a[i]=100;
        }
        else{
          a[i]=0;
        }
    }
    set<int> o;
    find_gezi(hang,counts,a,3,o);
    // set<int>::iterator iter;
    // for (iter = o.begin() ; iter != o.end() ; ++iter)
    // {
    //   if (-1<*iter && *iter<counts)
    //   {
    //     a[*iter]=100;
    //   }
    // }
    std::vector<signed char> p(a, a+counts);
    output.data=p;
    pub4_.publish(output);
  }


  void find_gezi(const int hang,const int counts,int* a,int floor,set<int>& s){
    if (floor > 0 )
    {
      int f[8]={ };
      for (int i = 0; i < counts; ++i)
      {
        //代价值为100（障碍物占用）
        if (a[i]==100)
        {
          //标记需要膨胀的栅格
          f[0]=i-hang-1;
          f[3]=i-1;
          f[6]=i+hang-1;
          f[1]=i-hang;
          f[7]=i+hang;
          f[2]=i-hang+1;
          f[5]=i+1;
          f[8]=i+hang+1;
        }
        for (int j = 0; j < 8; j++ )
        { 
          //将所有要膨胀的栅格赋给s
          s.insert(f[j]);
        }
      }
      //给标记为膨胀的栅格膨胀，赋100，为障碍物
      for( auto beg=s.begin(),end=s.end(); beg!=end ; ++beg)
      {
        if (-1 < *beg && *beg < counts)
          a[*beg]=100;
      }
    }else {
      return;
    }
    find_gezi(hang,counts,a,floor-1,s);//递归，直到floor=0结束膨胀
  }

  /*
  void merge_scan(void){
    if(!is_has_map){
      return;
    }
    // carto_map_real_time_ = carto_map_grid;
    copy_map(carto_map_grid, carto_map_real_time_);

    // https://www.jianshu.com/p/864b9a67dc20
    tf::TransformListener listener;
    tf::StampedTransform transform_map_baseScan;
    try{
      listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(.0).sleep();
    }
    // try{
      //向侦听器查询特定的转换，坐标转换
      // listener.lookupTransform(carto_map_real_time_.header.frame_id, scan_cur_.header.frame_id,
      //                          ros::Time(0), transform_map_baseScan);
      // listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(3.0));
      // listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
                        // listener.lookupTransform("/base_scan", "/map", 
                        //        ros::Time(0), transform_map_baseScan);
      // printf("transform: origin: [%f, %f, %f]\n", 
      //   transform_map_baseScan.getOrigin().x(), 
      //   transform_map_baseScan.getOrigin().y(), 
      //   transform_map_baseScan.getOrigin().z());
      // printf("transform Quaternion: [%f, %f, %f, %f]\n", 
      //   transform_map_baseScan.getRotation().x(), 
      //   transform_map_baseScan.getRotation().y(),
      //   transform_map_baseScan.getRotation().z(),
      //   transform_map_baseScan.getRotation().w());
      //printf("\n");
    
    // }
    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    int b[carto_map_grid.info.width * carto_map_grid.info.height] = { };
    for(int i=0; i<carto_map_grid.info.width * carto_map_grid.info.height; i++)
    {
      b[i] = -2;
    }

    float range_min = scan_cur_.range_min;
    float range_max = scan_cur_.range_max;
    // printf("***********************scan size: %d, map [h, w]: [%d, %d]\n", 
    //   scan_cur_.ranges.size(), 
    //   carto_map_real_time_.info.height, 
    //   carto_map_real_time_.info.width);
    
    for(int i=0; i<scan_cur_.ranges.size(); i++){
      
      float angle_i= scan_cur_.angle_min + i*scan_cur_.angle_increment;
      
      float range_i = scan_cur_.ranges[i];

      if(range_i<range_min){
        continue;
      }

      bool is_out_of_range =false;
      if(range_i>range_max||isinf(range_i)){
        // continue;
        is_out_of_range = true;
        range_i = range_max;
      }
      // 
      float r_cos = cos(angle_i);
      float r_sin = sin(angle_i);
      //printf("range_i: %f  scan size: %d, i: %d\n", range_i, scan_cur_.ranges.size(), i);
      int f=0;

      for(float jf=range_i; jf>= 0; ){
        float px = jf * r_cos;
        float py = jf * r_sin;

        //tf::Vector3 Vec3_in_basecan(px, py, 0);
        //tf::Vector3 Vec3_in_map = transform_map_baseScan(Vec3_in_basecan);
        // geometry_msgs::Pose current_pose_ros;
        // geometry_msgs::TransformStamped transform_pose;
        // tf::transformStampedTFToMsg(transform_map_baseScan,transform_pose);
        // float xmap=transform_map_baseScan.getOrigin().x();
        // float ymap=transform_map_baseScan.getOrigin().y();
        // try{
        //   listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(5.0));
        //   listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
        // }
        // catch (tf::TransformException &ex) {
        //   ROS_ERROR("%s",ex.what());
        //   ros::Duration(5.0).sleep();
        // }
        tf::Quaternion quat(transform_map_baseScan.getRotation().getW(),transform_map_baseScan.getRotation().getX(),transform_map_baseScan.getRotation().getY(),transform_map_baseScan.getRotation().getZ());
        //Eigen::Matrix<float,3,3> rotate_ma;
        //rotate_ma=quat.matrix();
        double roll,pitch,yaw;
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
        Matrix3f rotate_ma;
        rotate_ma<<cos(yaw),-sin(yaw),(float)transform_map_baseScan.getOrigin().x(),sin(yaw), cos(yaw),(float)transform_map_baseScan.getOrigin().y(),0,0,1.0;          
        //Eigen::Vector3f v2((float)transform_map_baseScan.getOrigin().x(),(float)transform_map_baseScan.getOrigin().y(),(float)transform_map_baseScan.getOrigin().z());
        Eigen::Vector3f v3=rotate_ma*Eigen::Vector3f(px,py,1);
        // odom_now.pose.pose.position.x=v3(0);
        // odom_now.pose.pose.position.y=v3(1);
        // float xmap = Vec3_in_map.x();
        // float ymap = Vec3_in_map.y();
        float xmap=v3(0);
        float ymap=v3(1);
        jf -= carto_map_.info.resolution;
        // int xy_to_index(const float &x, const float &y, int &xi, int &yi, const nav_msgs::OccupancyGrid &map_in)
        int xmap_i, ymap_j=0;
        int index_xy = xy_to_index(xmap, ymap, xmap_i, ymap_j, carto_map_real_time_);
        // printf("x: %f, y: %f, xi: %d, yi: %d\n", xmap, ymap, xmap_i, ymap_j);

        //printf("index_xy: %d, xmap: %f, ymap: %f, xmap_i: %d, ymap_j: %d\n", index_xy, xmap, ymap, xmap_i, ymap_j);

        if ((-1<index_xy) && (index_xy<carto_map_real_time_.info.width * carto_map_real_time_.info.height))
        {
          if(f!=0 && ((int)(carto_map_real_time_.data[index_xy]))!=100){
          b[index_xy] = 0;
        }
        else if(carto_map_real_time_.data[index_xy] == 100){
          b[index_xy]=100;
        }
        else if(f==0 && !is_out_of_range){
          b[index_xy]=100;
        }
        else{
          continue;
        }
      }
      f++;
    }
  }

    for (int k = 0; k < carto_map_real_time_.info.width * carto_map_real_time_.info.height; k++)
    {
      if (b[k]==-2)
      {
        b[k]=(int)carto_map_real_time_.data[k];
      }
      //b[k]=(int)carto_map_real_time_.data[k];
        // cout<<b[k]<<endl;
    }

    // std::vector<signed char> q(b, b+carto_map_real_time_.info.width*carto_map_real_time_.info.height);  
    // carto_map_real_time_.data=q;

    for(int i=0; i<carto_map_real_time_.info.width * carto_map_real_time_.info.height;i++)
    {
       carto_map_real_time_.data[i]=b[i];
    }
    //printf("************ transform sucess!\n");
    pub1_.publish(carto_map_real_time_);
    //printf("************* publish transform sucess!\n");
   }
  */


//获取雷达数据后的回调
void SCAN_callback(const sensor_msgs::LaserScan& scan_cur_){
 if(!is_has_map){
      return;
    }
    // carto_map_real_time_ = carto_map_grid;
    copy_map(carto_map_inflation, carto_map_real_time_);//得到膨胀（1格）后的实时地图

    // https://www.jianshu.com/p/864b9a67dc20
    tf::TransformListener listener;
    tf::StampedTransform transform_map_baseScan;
    try{
      listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(10.0));
      listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(.0).sleep();
    }
    // try{
      //向侦听器查询特定的转换，坐标转换
      // listener.lookupTransform(carto_map_real_time_.header.frame_id, scan_cur_.header.frame_id,
      //                          ros::Time(0), transform_map_baseScan);
      // listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(3.0));
      // listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
                        // listener.lookupTransform("/base_scan", "/map", 
                        //        ros::Time(0), transform_map_baseScan);
      /*printf("transform: origin: [%f, %f, %f]\n", 
        transform_map_baseScan.getOrigin().x(), 
        transform_map_baseScan.getOrigin().y(), 
        transform_map_baseScan.getOrigin().z());
      printf("transform Quaternion: [%f, %f, %f, %f]\n", 
        transform_map_baseScan.getRotation().x(), 
        transform_map_baseScan.getRotation().y(),
        transform_map_baseScan.getRotation().z(),
        transform_map_baseScan.getRotation().w());
      //printf("\n");
    */
    // }
    // catch (tf::TransformException &ex) {
    //   ROS_ERROR("%s",ex.what());
    //   ros::Duration(1.0).sleep();
    // }

    int b[carto_map_inflation.info.width * carto_map_inflation.info.height] = { };
    for(int i=0; i<carto_map_inflation.info.width * carto_map_inflation.info.height; i++)
    {
      b[i] = -2;//初始化
    }
    //雷达扫描角度范围
    float range_min = scan_cur_.range_min;
    float range_max = scan_cur_.range_max;
    /*printf("***********************scan size: %d, map [h, w]: [%d, %d]\n", 
      scan_cur_.ranges.size(), 
      carto_map_real_time_.info.height, 
      carto_map_real_time_.info.width);
    */
    for(int i=0; i<scan_cur_.ranges.size(); i++){
      
      float angle_i= scan_cur_.angle_min + i*scan_cur_.angle_increment;
      
      float range_i = scan_cur_.ranges[i];

      if(range_i<range_min){
        continue;
      }

      bool is_out_of_range =false;
      if(range_i>range_max||isinf(range_i)){
        // continue;
        is_out_of_range = true;
        range_i = range_max;
      }
      // 
      float r_cos = cos(angle_i);
      float r_sin = sin(angle_i);
      //printf("range_i: %f  scan size: %d, i: %d\n", range_i, scan_cur_.ranges.size(), i);
      int f=0;

      for(float jf=range_i; jf>= 0; ){
        float px = jf * r_cos;
        float py = jf * r_sin;

        //tf::Vector3 Vec3_in_basecan(px, py, 0);
        //tf::Vector3 Vec3_in_map = transform_map_baseScan(Vec3_in_basecan);
        // geometry_msgs::Pose current_pose_ros;
        // geometry_msgs::TransformStamped transform_pose;
        // tf::transformStampedTFToMsg(transform_map_baseScan,transform_pose);
        // float xmap=transform_map_baseScan.getOrigin().x();
        // float ymap=transform_map_baseScan.getOrigin().y();
        // try{
        //   listener.waitForTransform("/map", "/base_scan", ros::Time(0), ros::Duration(5.0));
        //   listener.lookupTransform("/map", "/base_scan",ros::Time(0), transform_map_baseScan);
        // }
        // catch (tf::TransformException &ex) {
        //   ROS_ERROR("%s",ex.what());
        //   ros::Duration(5.0).sleep();
        // }
        tf::Quaternion quat(transform_map_baseScan.getRotation().getW(),transform_map_baseScan.getRotation().getX(),transform_map_baseScan.getRotation().getY(),transform_map_baseScan.getRotation().getZ());
        //Eigen::Matrix<float,3,3> rotate_ma;
        //rotate_ma=quat.matrix();
        double roll,pitch,yaw;
        tf::Matrix3x3(quat).getRPY(roll,pitch,yaw);
        Matrix3f rotate_ma;
        rotate_ma<<cos(yaw),-sin(yaw),(float)transform_map_baseScan.getOrigin().x(),sin(yaw), cos(yaw),(float)transform_map_baseScan.getOrigin().y(),0,0,1.0;          
        //Eigen::Vector3f v2((float)transform_map_baseScan.getOrigin().x(),(float)transform_map_baseScan.getOrigin().y(),(float)transform_map_baseScan.getOrigin().z());
        Eigen::Vector3f v3=rotate_ma*Eigen::Vector3f(px,py,1);
        // odom_now.pose.pose.position.x=v3(0);
        // odom_now.pose.pose.position.y=v3(1);
        // float xmap = Vec3_in_map.x();
        // float ymap = Vec3_in_map.y();
        float xmap=v3(0);
        float ymap=v3(1);
        jf -= carto_map_.info.resolution;
        // int xy_to_index(const float &x, const float &y, int &xi, int &yi, const nav_msgs::OccupancyGrid &map_in)
        int xmap_i, ymap_j=0;
        int index_xy = xy_to_index(xmap, ymap, xmap_i, ymap_j, carto_map_real_time_);
        // printf("x: %f, y: %f, xi: %d, yi: %d\n", xmap, ymap, xmap_i, ymap_j);

        //printf("index_xy: %d, xmap: %f, ymap: %f, xmap_i: %d, ymap_j: %d\n", index_xy, xmap, ymap, xmap_i, ymap_j);

        if ((-1<index_xy) && (index_xy<carto_map_real_time_.info.width * carto_map_real_time_.info.height))
        {
          if(f!=0 && ((int)(carto_map_real_time_.data[index_xy]))!=100){
          b[index_xy] = 0;//空闲
        }
        else if(carto_map_real_time_.data[index_xy] == 100){
          b[index_xy]=100;//障碍
        }
        else if(f==0 && !is_out_of_range){
          continue;
          b[index_xy]=100;
        }
        else{
          continue;
        }
      }
      f++;
    }
  }

    for (int k = 0; k < carto_map_real_time_.info.width * carto_map_real_time_.info.height; k++)
    {
      if (b[k]==-2)
      {
        b[k]=(int)carto_map_real_time_.data[k];
      }
      //b[k]=(int)carto_map_real_time_.data[k];
        // cout<<b[k]<<endl;
    }

    // std::vector<signed char> q(b, b+carto_map_real_time_.info.width*carto_map_real_time_.info.height);  
    // carto_map_real_time_.data=q;

    for(int i=0; i<carto_map_real_time_.info.width * carto_map_real_time_.info.height;i++)
    {
       carto_map_real_time_.data[i]=b[i];
    }
    //printf("************ transform sucess!\n");
    pub1_.publish(carto_map_real_time_);
  }





private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Publisher pub1_;
  ros::Publisher pub2_;
  ros::Publisher pub3_;
  ros::Publisher pub4_;
  ros::Subscriber sub_;
  ros::Subscriber sub1_;
  ros::Subscriber sub2_;
  ros::Subscriber sub3_;
  ros::Subscriber sub_can_;
  bool is_has_map = false;
  // 
  // 

  nav_msgs::OccupancyGrid carto_map_;  // cartographer output map [-1, [0, 100]]
  nav_msgs::OccupancyGrid carto_map_grid;  // grid map [-1, {0, 1}}]
  nav_msgs::OccupancyGrid carto_map_inflation;
  nav_msgs::OccupancyGrid carto_map_real_time_; // map with scan
  //sensor_msgs::LaserScan scan_cur_;
};


int main(int argc, char **argv)
{
	ros::init(argc, argv, "map_solver");
  // SubscribeAndPublish SAPObject;
  // ros::Rate loop_rate(20);
  // while (ros::ok())
  // {
  //   SAPObject.get_pub2().publish(SAPObject.get_carto_map_inflation());
  //   ros::spinOnce();
  //   loop_rate.sleep();
  // }
	SubscribeAndPublish SAPObject;
	ros::spin();
	return 0;
}


// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// float32 angle_min
// float32 angle_max
// float32 angle_increment
// float32 time_increment
// float32 scan_time
// float32 range_min
// float32 range_max
// float32[] ranges
// float32[] intensities

// frame_id: "base_scan"
// frame_id: "map"


// nav_msgs::OccupancyGrid

// std_msgs/Header header
//   uint32 seq
//   time stamp
//   string frame_id
// nav_msgs/MapMetaData info
//   time map_load_time
//   float32 resolution
//   uint32 width
//   uint32 height
//   geometry_msgs/Pose origin
//     geometry_msgs/Point position
//       float64 x
//       float64 y
//       float64 z
//     geometry_msgs/Quaternion orientation
//       float64 x
//       float64 y
//       float64 z
//       float64 w
// int8[] data


// [x, y] in map frame
int is_in_map(const int x, const int y, const nav_msgs::OccupancyGrid &map_in){

  // if(!is_has_map){
  //   printf("\n");
  //   return -1;
  // }

  int map_height = map_in.info.height;
  int map_width = map_in.info.width;
  float map_resolution = map_in.info.resolution;
  geometry_msgs::Pose map_origin = map_in.info.origin;
  float origin_y = map_origin.position.y;
  float origin_x = map_origin.position.x;

  // int index = (int)((x - origin_x) / map_resolution) + ((int)((y - origin_y) / map_resolution)) * map_width;
  // if(map_in.data[index]==1){
  //   return -1;
  // }
  if(x<=0||y>=map_width||
    y<=0||x>=map_height){
    return -1; // not in map
  }
  return 1;
}

int is_in_map(const float x, const float y, const nav_msgs::OccupancyGrid &map_in){
  // if(!is_has_map){
  //   printf("\n");
  //   return -1;
  // }

  int map_height = map_in.info.height;
  int map_width = map_in.info.width;
  float map_resolution = map_in.info.resolution;
  geometry_msgs::Pose map_origin = map_in.info.origin;
  float origin_y = map_origin.position.y;
  float origin_x = map_origin.position.x;

  // int index = (int)((x - origin_x) / map_resolution) + ((int)((y - origin_y) / map_resolution)) * map_width;
  // if(map_in.data[index]==1){
  //   return -1;
  // }
  if(x<origin_x||(int)((x - origin_x) / map_resolution)>map_width||
    y<origin_y||(int)((y - origin_y) / map_resolution)>map_height){
    return -1; // not in map
  }
  return 1;

}

// [float x, float y] --> [int x, int y], 
// return index (-1 is fail)
int xy_to_index(const float &x, const float &y, int &xi, int &yi, const nav_msgs::OccupancyGrid &map_in)
{
  // if(!is_has_map){
  //   return -1;
  // }
  int map_height = map_in.info.height;
  int map_width = map_in.info.width;
  float map_resolution = map_in.info.resolution;

  geometry_msgs::Pose map_origin = map_in.info.origin;

  float origin_y = map_origin.position.y;
  float origin_x = map_origin.position.x;
  xi = (int)((x - origin_x) / map_resolution);
  yi = (int)((y - origin_y) / map_resolution);

  // printf("map_height: %d, width: %d,  res: %f, origin_x: %f, origin_y: %f\n", map_height, map_width, map_resolution, origin_x, origin_y);

  if(is_in_map(xi, yi, map_in) == -1){
    return -1; // fail
  }
  else if(is_in_map(xi, yi, map_in) == 1){
    int index = (int)(xi + yi * map_width);
    return index;
  }
  
}

int index_to_float(const int &xi, const int &yi, float &x, float &y, const nav_msgs::OccupancyGrid &map_in){
  // if(!is_has_map){
  //   return -1;
  // }
  int map_height = map_in.info.height;
  int map_width = map_in.info.width;
  float map_resolution = map_in.info.resolution;
  geometry_msgs::Pose map_origin = map_in.info.origin;
  float origin_y = map_origin.position.y;
  float origin_x = map_origin.position.x;
  if(is_in_map(xi, yi, map_in) == -1){
    return -1; // fail
  }
  else if(is_in_map(xi, yi, map_in) == 1){
    x = (xi+0.5) * map_resolution + origin_x;
    y = (yi+0.5) * map_resolution + origin_y;

    int index = xi + yi * map_width;
    return index;
  }

  return 1;
}


void copy_map(const nav_msgs::OccupancyGrid &input, nav_msgs::OccupancyGrid &output){
  int a[input.info.width * input.info.height] = {-1};
  output.header.seq = input.header.seq; 
  output.header.frame_id = input.header.frame_id;
  output.header.stamp = input.header.stamp;
  output.info.map_load_time = input.info.map_load_time;
  output.info.resolution = input.info.resolution;
  //printf("*****copy_map res: %f\n", output.info.resolution);
  output.info.width = input.info.width;
  output.info.height = input.info.height;
  output.info.origin = input.info.origin;
  //(sizeof(input.data)/sizeof(input.data[0]))
  for(int i=0; i<input.info.width * input.info.height;i++)
 {
    a[i]=(int)input.data[i];
  }

  std::vector<signed char> p(a, a+input.info.width*input.info.height);
  output.data=p;
}
