/**
 * @brief 碰撞检测函数集
 */
#include "hybrid_astar_searcher/collisiondetection.h"
#include <iostream>
#include <chrono>
using namespace planning;

CollisionDetection::CollisionDetection() {
  this->grid = nullptr;
  // collisionLookup就是生成的覆盖栅格模板


  auto dp_map_start = std::chrono::high_resolution_clock::now();

  
  Lookup::collisionLookup(collisionLookup);

  auto dp_map_end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
  std::cout << "占据栅格模板生成耗时:" << dp_map_use_time.count() * 1000 << "ms" << std::endl;

  std::cout << "CollisionDetection Init!" << std::endl;
}

// 输入真实坐标和偏航角(x,y,theta)
bool CollisionDetection::configurationTest(float x, float y, float theta) {
  //真实坐标转行列索引
  int X = std::floor((x - grid->info.origin.position.x) / grid->info.resolution + 0.5);
  int Y = std::floor((y - grid->info.origin.position.y) / grid->info.resolution + 0.5);

  //原始theta在[-pai,+pai]必须把theta转到0-360度内
  if(theta < 0) theta+=(2 * M_PI);
  // 在进行强制类型转换为 int 时，结果会被转换为 -1，因为小数部分会被舍弃，这就导致越界
  unsigned int idx = (unsigned int)(theta / Constants::deltaHeadingRad); //匹配最接近当前偏航角的角度索引       
 
  // int X = (int)x;
  // int Y = (int)y;
  // int iX = (int)((x - (long)x) * Constants::positionResolution);//得出X方向在cell中的偏移量
  // iX = iX > 0 ? iX : 0;
  // int iY = (int)((y - (long)y) * Constants::positionResolution);//Y方向在cell中的偏移量
  // iY = iY > 0 ? iY : 0;
  // 我们就是在0.1分辨率下的栅格原点，没有偏移，只有那些低分辨率下才有偏移
  // int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  // unsigned int idx = iT;
  
  // std::cout << "idx:" << idx << std::endl;
  int cX;
  int cY;

  // idx我们只用它表示某一个角度下的覆盖栅格表格
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    // 拿出这个角度模板对应的覆盖栅格，有length个，我们设置的都是轮廓，没用内部
    // 再加上偏移（X，Y）
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    // if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {

    if(cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
      || grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1 ){ 
      // if (grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1) {
        return false;
      // }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
    }
  }

  return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}


// 输入栅格坐标和偏航角(X,Y,Theta)
bool CollisionDetection::configurationTest_(int X, int Y, float Theta) {
  //真实坐标转行列索引
  // int X = std::floor((x - grid->info.origin.position.x) / grid->info.resolution + 0.5);
  // int Y = std::floor((y - grid->info.origin.position.y) / grid->info.resolution + 0.5);

  //原始theta在[-pai,+pai]必须把theta转到0-360度内
  if(Theta < 0) Theta+=2 * M_PI;
  int iT = (int)(Theta / Constants::deltaHeadingRad); //匹配最接近当前偏航角的角度索引       
 
  // int X = (int)x;
  // int Y = (int)y;
  // int iX = (int)((x - (long)x) * Constants::positionResolution);//得出X方向在cell中的偏移量
  // iX = iX > 0 ? iX : 0;
  // int iY = (int)((y - (long)y) * Constants::positionResolution);//Y方向在cell中的偏移量
  // iY = iY > 0 ? iY : 0;
  // 我们就是在0.1分辨率下的栅格原点，没有偏移，只有那些低分辨率下才有偏移
  // int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  int idx = iT;
  int cX;
  int cY;

  // idx我们只用它表示某一个角度下的覆盖栅格表格
  for (int i = 0; i < collisionLookup[idx].length; ++i) {
    // 拿出这个角度模板对应的覆盖栅格，有length个，我们设置的都是轮廓，没用内部
    // 再加上偏移（X，Y）
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);

    // make sure the configuration coordinates are actually on the grid
    // if (cX >= 0 && (unsigned int)cX < grid->info.width && cY >= 0 && (unsigned int)cY < grid->info.height) {

    if(cX < 0 || (unsigned int)cX >= grid->info.width || cY < 0 || (unsigned int)cY >= grid->info.height
      || grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1 ){ 
      // if (grid->data[cY * grid->info.width + cX]==100 || grid->data[cY * grid->info.width + cX]==-1) {
        return false;
      // }//若grid的某个小网格存在值，说明有障碍，则返回false表示不在自由网格
    }
  }

  return true;//所有检测都没有检测到被占用，说明没有障碍，可以通行
}