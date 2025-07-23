#include "hybrid_astar_searcher/dynamicvoronoi.h"

#include <math.h>
#include <iostream>


DynamicVoronoi::DynamicVoronoi() {
  sqrt2 = sqrt(2.0);
  data = NULL;
  gridMap = NULL;
}

DynamicVoronoi::~DynamicVoronoi() {
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  if (gridMap) {
    for (int x=0; x<sizeX; x++) delete[] gridMap[x];
    delete[] gridMap;
  }
}


// 初始化data数组，即地图上每个单元格的数据结构
void DynamicVoronoi::initializeEmpty(int _sizeX, int _sizeY, bool initGridMap) {
  sizeX = _sizeX;//地图尺寸
  sizeY = _sizeY;
  // 如果data数组已经存在，则先释放其内存空间。
  if (data) {
    for (int x=0; x<sizeX; x++) delete[] data[x];
    delete[] data;
  }
  // 分配新的data数组
  data = new dataCell*[sizeX];
  for (int x=0; x<sizeX; x++) data[x] = new dataCell[sizeY];
  // 如果initGridMap为true（我们是false），则进行以下操作：
  if (initGridMap) {
    // 如果gridMap数组已经存在(我们存在)，则先释放其内存空间。
    if (gridMap) {
      for (int x=0; x<sizeX; x++) delete[] gridMap[x];
      delete[] gridMap;
    }
    // 分配新的gridMap数组，并将其所有元素初始化为0。
    gridMap = new bool*[sizeX];
    for (int x=0; x<sizeX; x++) gridMap[x] = new bool[sizeY];
  }

// 将每个单元格的初始属性值设置为无穷大（dist）、最大整数（sqdist）、无效的障碍物数据（obstX和obstY）、自由状态（voronoi）、未加入队列（queueing）以及不需要提升（needsRaise）。
  dataCell c;
  c.dist = INFINITY;
  c.sqdist = INT_MAX;
  c.obstX = invalidObstData;
  c.obstY = invalidObstData;
  c.voronoi = free;//初始化
  c.queueing = fwNotQueued;
  c.needsRaise = false;

  for (int x=0; x<sizeX; x++)
    for (int y=0; y<sizeY; y++) data[x][y] = c;//赋值

  //我们initGridMap是false
  if (initGridMap) {
    for (int x=0; x<sizeX; x++) 
      for (int y=0; y<sizeY; y++) gridMap[x][y] = 0;//清空栅格状态
  }
}


//处理所有障碍栅格，特别关注边界障碍栅格
void DynamicVoronoi::initializeMap(int _sizeX, int _sizeY, bool** _gridMap) {
  gridMap = _gridMap;
  initializeEmpty(_sizeX, _sizeY, false);//初始化data数组
  // 遍历整个网格地图
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) {

      // 对于每个障碍物单元格（true），进行以下操作：
      if (gridMap[x][y]) {
        dataCell c = data[x][y];

        // 检查给定的坐标（x和y）是否为障碍物，其实这里表示该障碍格还没有初始化
        if (!isOccupied(x,y,c)) {
          // 未初始化：
          bool isSurrounded = true;//初始化为true
          // 检查给定坐标（x和y）所在的单元格是否被其周围的8个单元格包围
          // 使用两层循环遍历当前点周围的8个单元格（不包括自身）
          for (int dx=-1; dx<=1; dx++) {
            int nx = x+dx;
            if (nx<=0 || nx>=sizeX-1) continue;
            for (int dy=-1; dy<=1; dy++) {
              if (dx==0 && dy==0) continue;
              int ny = y+dy;
              if (ny<=0 || ny>=sizeY-1) continue;
              // 对于每个相邻的单元格，如果该单元格没有被占用（即gridMap[nx][ny]为false），则将isSurrounded设置为false，并跳出内层循环。
              if (!gridMap[nx][ny]) {
                // 表示当前单元格至少有一个相邻单元格未被占用，是边界单元格
                isSurrounded = false;
                break;
              }
            }
          }
          // 如果isSurrounded为true，则表示当前单元格周围的8个单元格都是障碍格
          if (isSurrounded) {
            // obstX和obstY：该单元格最近的障碍物单元格的坐标，就是它自己
            c.obstX = x;
            c.obstY = y;
            // sqdist：该单元格到最近障碍物单元格的距离的平方，它本身
            c.sqdist = 0;
            // dist：该单元格到最近障碍物单元格的距离
            c.dist=0;
            c.voronoi=occupied;//完全的障碍格
            c.queueing = fwProcessed;//已处理
            data[x][y] = c;
            // 边界单元格：将其标记为障碍物单元格。里面的单元格不管
          } else setObstacle(x,y);//我们只看重边界障碍物
        }
      }
    }
  }
}

void DynamicVoronoi::occupyCell(int x, int y) {
  gridMap[x][y] = 1;
  setObstacle(x,y);
}
void DynamicVoronoi::clearCell(int x, int y) {
  gridMap[x][y] = 0;
  removeObstacle(x,y);
}



void DynamicVoronoi::setObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c)) return;//已设置为障碍
  
  addList.push_back(INTPOINT(x,y));//加入addList，这个表存储所有更新后的边界障碍
  c.obstX = x;//本身
  c.obstY = y;
  data[x][y] = c;
}

void DynamicVoronoi::removeObstacle(int x, int y) {
  dataCell c = data[x][y];
  if(isOccupied(x,y,c) == false) return;

  removeList.push_back(INTPOINT(x,y));
  c.obstX = invalidObstData;
  c.obstY  = invalidObstData;    
  c.queueing = bwQueued;
  data[x][y] = c;
}

void DynamicVoronoi::exchangeObstacles(const std::vector<INTPOINT>& points) {

  for (unsigned int i=0; i<lastObstacles.size(); i++) {
    int x = lastObstacles[i].x;
    int y = lastObstacles[i].y;

    bool v = gridMap[x][y];
    if (v) continue;
    removeObstacle(x,y);
  }  

  lastObstacles.clear();
  lastObstacles.reserve(points.size());

  for (unsigned int i=0; i<points.size(); i++) {
    int x = points[i].x;
    int y = points[i].y;
    bool v = gridMap[x][y];
    if (v) continue;
    setObstacle(x,y);
    lastObstacles.push_back(points[i]);
  }  
}



void DynamicVoronoi::update(bool updateRealDist) {

  commitAndColorize(updateRealDist);//增删障碍格，更新障碍格信息

  // 计算每个单元格到最近障碍物的距离和Voronoi图
  while (!open.empty()) {
    INTPOINT p = open.pop();//从open队列中弹出一个具有最小sqdist值的坐标p，open里一开始是一堆边界障碍，后来传播到空闲栅格
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if(c.queueing==fwProcessed) continue; 

    // RAISE
    if (c.needsRaise) {
      // 遍历当前单元格周围的8个单元格
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];//邻近单元格
          if (nc.obstX!=invalidObstData && !nc.needsRaise) {
            // 如果相邻单元格nc不是障碍物，将其加入open队列，并将其queueing属性设置为fwQueued
            if(!isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])) {
              open.push(nc.sqdist, INTPOINT(nx,ny));//根据sqdist加入open
              nc.queueing = fwQueued;//入列
              nc.needsRaise = true;
              nc.obstX = invalidObstData;
              nc.obstY = invalidObstData;
              // 如果updateRealDist为true，则将相邻单元格nc的dist属性设置为无穷大，表示从该单元格到障碍物的距离为无穷大
              if (updateRealDist) nc.dist = INFINITY;//初始化
              nc.sqdist = INT_MAX;
              data[nx][ny] = nc;
              //如果相邻单元格nc是障碍物，将其加入open队列，并将其queueing属性设置为fwQueued。
            } else {
              if(nc.queueing != fwQueued){
                open.push(nc.sqdist, INTPOINT(nx,ny));//加入open
                nc.queueing = fwQueued;
                data[nx][ny] = nc;
              }
            }      
          }
        }
      }
      c.needsRaise = false;
      c.queueing = bwProcessed;//将当前单元格c的queueing属性设置为bwProcessed，表示该单元格已经被处理过
      data[x][y] = c;
    }

    // 如果数据单元格c属于障碍物
    else if (c.obstX != invalidObstData && isOccupied(c.obstX,c.obstY,data[c.obstX][c.obstY])) {
      // LOWER
      c.queueing = fwProcessed;
      c.voronoi = occupied;//障碍

      // 遍历当前单元格周围的8个单元格
      for (int dx=-1; dx<=1; dx++) {
        int nx = x+dx;
        if (nx<=0 || nx>=sizeX-1) continue;
        for (int dy=-1; dy<=1; dy++) {
          if (dx==0 && dy==0) continue;
          int ny = y+dy;
          if (ny<=0 || ny>=sizeY-1) continue;
          dataCell nc = data[nx][ny];
          if(!nc.needsRaise) {
            int distx = nx-c.obstX;
            int disty = ny-c.obstY;
            int newSqDistance = distx*distx + disty*disty;	//计算每个相邻单元格到该障碍物的平方距离
            bool overwrite =  (newSqDistance < nc.sqdist);
            // 如果相邻单元格nc到该障碍物的平方距离等于nc原来保存的距离，并且nc原来的obstX不是无效值，并且该障碍物没有被重新插入：
            if(!overwrite && newSqDistance==nc.sqdist) { 
              if (nc.obstX == invalidObstData || isOccupied(nc.obstX,nc.obstY,data[nc.obstX][nc.obstY])==false) overwrite = true;
            }
            // 如果相邻单元格nc到该障碍物的平方距离小于nc原来保存的距离（即nc.sqdist）：
            if (overwrite) {
              // 将相邻单元格nc的坐标添加到open队列中
              open.push(newSqDistance, INTPOINT(nx,ny));
              nc.queueing = fwQueued;
              // 如果updateRealDist为true，则计算相邻单元格nc到该障碍物的真实距离，并更新nc的dist属性。
              if (updateRealDist) {
                nc.dist = sqrt((double) newSqDistance);
              }
              // 将相邻单元格nc的sqdist属性设置为新的距离值。
              nc.sqdist = newSqDistance;
              // 更新相邻单元格nc的obstX和obstY属性为该障碍物的坐标。
              nc.obstX = c.obstX;
              nc.obstY = c.obstY;
            } else { 
              // 调用checkVoro函数，检查当前单元格c和相邻单元格nc之间的Voronoi边界，并更新Voronoi图。
              //对多个障碍点距离相同的栅格会进入这里
              checkVoro(x,y,nx,ny,c,nc);
            }
            // 将更新后的相邻单元格nc写回data数组的相应位置。
            data[nx][ny] = nc;
          }
        }
      }
    }
    data[x][y] = c;
  }
}


//获取其离最近障碍栅格的栅格距离
float DynamicVoronoi::getDistance( int x, int y ) const {
  if( (x>0) && (x<sizeX) && (y>0) && (y<sizeY)) return data[x][y].dist; 
  else return -INFINITY;
}


// 判断给定坐标 (x,y) 是否属于 Voronoi 图
bool DynamicVoronoi::isVoronoi( int x, int y ) const {
  dataCell c = data[x][y];
  
  return (c.voronoi==free || c.voronoi==voronoiKeep);//合法的Voronoi节点
}


//增删障碍格，更新障碍格信息
void DynamicVoronoi::commitAndColorize(bool updateRealDist) {
  // ADD NEW OBSTACLES
  // 遍历addList列表（存储所有更新后的边界障碍格），处理新增的障碍物。对于每个坐标p，获取其x和y坐标，并获取data[x][y]对应的数据单元格c
  for (unsigned int i=0; i<addList.size(); i++) {
    INTPOINT p = addList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    // 如果数据单元格c的queueing属性不等于fwQueued（表示该单元格没有被标记为障碍物）
    if(c.queueing != fwQueued){
      if (updateRealDist) c.dist = 0;
      c.sqdist = 0;//本身
      c.obstX = x;
      c.obstY = y;
      c.queueing = fwQueued;//入列
      c.voronoi = occupied;//表示该单元格属于障碍物
      data[x][y] = c;
      open.push(0, INTPOINT(x,y));//addList转存入open队列
    }
  }

  // REMOVE OLD OBSTACLES
  // 遍历removeList列表，处理移除的障碍物
  for (unsigned int i=0; i<removeList.size(); i++) {
    INTPOINT p = removeList[i];
    int x = p.x;
    int y = p.y;
    dataCell c = data[x][y];

    if (isOccupied(x,y,c)==true) continue; // obstacle was removed and reinserted 表示该单元格仍然被标记为障碍物
    open.push(0, INTPOINT(x,y));//存入open，变更的都要存入,标记？
    if (updateRealDist) c.dist  = INFINITY;//无穷大，初始化？
    c.sqdist = INT_MAX;
    c.needsRaise = true;
    data[x][y] = c;
  }
  removeList.clear();
  addList.clear();
}


// 检查当前单元格c和相邻单元格nc之间的Voronoi边界，并更新Voronoi图
/*如果当前单元格c和相邻单元格nc之间存在一个障碍物，那么它们之间就有一个Voronoi边界。
为了计算Voronoi边界，该函数首先计算了从当前单元格c到相邻单元格nc所对应障碍物的距离（即stability_xy变量），以及从相邻单元格nc到当前单元格c所对应障碍物的距离（即stability_nxy变量）。
然后，根据两个距离的大小关系，判断哪个单元格应该被添加到Voronoi图中。*/
void DynamicVoronoi::checkVoro(int x, int y, int nx, int ny, dataCell& c, dataCell& nc) {

  if ((c.sqdist>1 || nc.sqdist>1) && nc.obstX!=invalidObstData) { 
    if (abs(c.obstX-nc.obstX) > 1 || abs(c.obstY-nc.obstY) > 1) {
      //compute dist from x,y to obstacle of nx,ny	 
      int dxy_x = x-nc.obstX;
      int dxy_y = y-nc.obstY;
      int sqdxy = dxy_x*dxy_x + dxy_y*dxy_y;
      int stability_xy = sqdxy - c.sqdist;
      if (sqdxy - c.sqdist<0) return;

      //compute dist from nx,ny to obstacle of x,y
      int dnxy_x = nx - c.obstX;
      int dnxy_y = ny - c.obstY;
      int sqdnxy = dnxy_x*dnxy_x + dnxy_y*dnxy_y;
      int stability_nxy = sqdnxy - nc.sqdist;
      if (sqdnxy - nc.sqdist <0) return;

      //which cell is added to the Voronoi diagram?
      /*如果当前单元格c到相邻单元格nc所对应障碍物的距离stability_xy小于等于相邻单元格nc到当前单元格c所对应障碍物的距离stability_nxy，
      并且当前单元格c到该障碍物的平方距离c.sqdist大于2，则将当前单元格c添加到Voronoi图中。
      为此，需要将当前单元格c的voronoi属性设置为free，表示该单元格属于Voronoi图，并且将该单元格周围的八个相邻单元格的voronoi属性也设置为free，以便后续处理。*/
      if(stability_xy <= stability_nxy && c.sqdist>2) {
        if (c.voronoi != free) {
          c.voronoi = free;//原本是occupied
          reviveVoroNeighbors(x,y);
          // 如果当前单元格c或相邻单元格nc被添加到Voronoi图中，那么需要将它们的坐标添加到pruneQueue队列中，以便在Voronoi图修剪过程中处理它们
          pruneQueue.push(INTPOINT(x,y));
        }
      }
      /*如果相邻单元格nc到当前单元格c所对应障碍物的距离stability_nxy小于等于当前单元格c到相邻单元格nc所对应障碍物的距离stability_xy，并且相邻单元格nc到该障碍物的平方距离nc.sqdist大于2，则将相邻单元格nc添加到Voronoi图中。为此，需要将相邻单元格nc的voronoi属性设置为free，并将该单元格周围的八个相邻单元格的voronoi属性也设置为free。*/
      if(stability_nxy <= stability_xy && nc.sqdist>2) {
        if (nc.voronoi != free) {
          nc.voronoi = free;
          reviveVoroNeighbors(nx,ny);
          pruneQueue.push(INTPOINT(nx,ny));
        }
      }
    }
  }
}


// 指定坐标(x, y)周围的符合条件的相邻单元格从Voronoi图中移除，并将其加入到待处理的修剪队列中。
void DynamicVoronoi::reviveVoroNeighbors(int &x, int &y) {
  // 遍历其周围的相邻单元格
  for (int dx=-1; dx<=1; dx++) {
    int nx = x+dx;
    if (nx<=0 || nx>=sizeX-1) continue;
    for (int dy=-1; dy<=1; dy++) {
      if (dx==0 && dy==0) continue;
      int ny = y+dy;
      if (ny<=0 || ny>=sizeY-1) continue;
      dataCell nc = data[nx][ny];
      // 检查相邻单元格的距离值 nc.sqdist 是否不等于最大整数值 INT_MAX，且它不需要提升(needsRaise为false)，并且其voronoi属性为voronoiKeep或voronoiPrune。
      if (nc.sqdist != INT_MAX && !nc.needsRaise && (nc.voronoi == voronoiKeep || nc.voronoi == voronoiPrune)) {
        // 如果上述条件成立，就将相邻单元格的voronoi属性设置为free
        nc.voronoi = free;
        // 然后将更新后的相邻单元格数据重新写回数据数组中，并将其坐标添加到pruneQueue队列中，以备在Voronoi图修剪过程中进一步处理。
        data[nx][ny] = nc;
        pruneQueue.push(INTPOINT(nx,ny));
      }
    }
  }
}


bool DynamicVoronoi::isOccupied(int x, int y) const {
  dataCell c = data[x][y];
  return (c.obstX==x && c.obstY==y);
}


// 检查给定的坐标（x和y）是否为障碍物
bool DynamicVoronoi::isOccupied(int &x, int &y, dataCell &c) { 
  return (c.obstX==x && c.obstY==y);
}


// 将 Voronoi 图绘制到文件中
void DynamicVoronoi::visualize(const char *filename) {
  // write pgm files
  std::cout << "keshihua" << std::endl;
  FILE* F = fopen(filename, "w");
  if (!F) {
    std::cerr << "could not open 'result.pgm' for writing!\n";
    return;
  }
  // P6 格式是一种二进制格式，每个像素由三个字节表示，分别对应于红、绿、蓝三个通道的值
  fprintf(F, "P6\n");
  fprintf(F, "%d %d 255\n", sizeX, sizeY);

  // 遍历 Voronoi 图中的每个像素,根据其 Voronoi 状态和距离值，将相应的颜色值写入文件中
  for(int y = sizeY-1; y >=0; y--){      
    for(int x = 0; x<sizeX; x++){	
      unsigned char c = 0;
      if (isVoronoi(x,y)) {//当前像素属于 Voronoi 图
        fputc( 255, F );//红色
        fputc( 0, F );
        fputc( 0, F );
      } else if (data[x][y].sqdist==0) {//源点
        fputc( 0, F );//黑色
        fputc( 0, F );
        fputc( 0, F );
      } 
      else {
        float f = 80+(data[x][y].dist*5);
        if (f>255) f=255;
        if (f<0) f=0;
        c = (unsigned char)f;
        fputc( c, F );//渐变的灰度值表示距离信息
        fputc( c, F );
        fputc( c, F );
      }
    }
  }
  fclose(F);
}


// 剪枝
// 将不必要的节点设置为剪枝状态（voronoiPrune），或者保留状态（voronoiKeep），或者重新尝试状态（voronoiRetry）
void DynamicVoronoi::prune() {
  // filler
  // 遍历pruneQueue队列中的点
  while(!pruneQueue.empty()) {
    INTPOINT p = pruneQueue.front();//从pruneQueue队列中取出一个点p
    pruneQueue.pop();
    int x = p.x;
    int y = p.y;

    if (data[x][y].voronoi==occupied) continue;
    if (data[x][y].voronoi==freeQueued) continue;

    data[x][y].voronoi = freeQueued;
    open.push(data[x][y].sqdist, p);//加入到open优先队列中

    /* tl t tr
       l c r
       bl b br */

    // 右上、右下、左上、左下
    dataCell tr,tl,br,bl;
    tr = data[x+1][y+1];
    tl = data[x-1][y+1];
    br = data[x+1][y-1];
    bl = data[x-1][y-1];

    // 右侧、左侧、上方和下方
    dataCell r,b,t,l;
    r = data[x+1][y];
    l = data[x-1][y];
    t = data[x][y+1];
    b = data[x][y-1];

    if (x+2<sizeX && r.voronoi==occupied) { 
      // fill to the right
      if (tr.voronoi!=occupied && br.voronoi!=occupied && data[x+2][y].voronoi!=occupied) {
        r.voronoi = freeQueued;
        open.push(r.sqdist, INTPOINT(x+1,y));//加入到open优先队列中
        data[x+1][y] = r;
      }
    } 
    if (x-2>=0 && l.voronoi==occupied) { 
      // fill to the left
      if (tl.voronoi!=occupied && bl.voronoi!=occupied && data[x-2][y].voronoi!=occupied) {
        l.voronoi = freeQueued;
        open.push(l.sqdist, INTPOINT(x-1,y));
        data[x-1][y] = l;
      }
    } 
    if (y+2<sizeY && t.voronoi==occupied) { 
      // fill to the top
      if (tr.voronoi!=occupied && tl.voronoi!=occupied && data[x][y+2].voronoi!=occupied) {
        t.voronoi = freeQueued;
        open.push(t.sqdist, INTPOINT(x,y+1));
        data[x][y+1] = t;
      }
    } 
    if (y-2>=0 && b.voronoi==occupied) { 
      // fill to the bottom
      if (br.voronoi!=occupied && bl.voronoi!=occupied && data[x][y-2].voronoi!=occupied) {
        b.voronoi = freeQueued;
        open.push(b.sqdist, INTPOINT(x,y-1));
        data[x][y-1] = b;
      }
    } 
  }


  while(!open.empty()) {
    INTPOINT p = open.pop();
    dataCell c = data[p.x][p.y];
    int v = c.voronoi;
    if (v!=freeQueued && v!=voronoiRetry) { // || v>free || v==voronoiPrune || v==voronoiKeep) {
      //      assert(v!=retry);
      continue;
    }

    markerMatchResult r = markerMatch(p.x,p.y);
    if (r==pruned) c.voronoi = voronoiPrune;
    else if (r==keep) c.voronoi = voronoiKeep;
    else { // r==retry
      c.voronoi = voronoiRetry;
      //      printf("RETRY %d %d\n", x, sizeY-1-y);
      pruneQueue.push(p);
    }
    data[p.x][p.y] = c;

    if (open.empty()) {
      while (!pruneQueue.empty()) {
        INTPOINT p = pruneQueue.front();
        pruneQueue.pop();
        open.push(data[p.x][p.y].sqdist, p);
      }
    }
  }
  //  printf("match: %d\nnomat: %d\n", matchCount, noMatchCount);
}


DynamicVoronoi::markerMatchResult DynamicVoronoi::markerMatch(int x, int y) {
  // implementation of connectivity patterns
  bool f[8];

  int nx, ny;
  int dx, dy;

  int i=0;
  int count=0;
  //  int obstacleCount=0;
  int voroCount=0;
  int voroCountFour=0;

  for (dy=1; dy>=-1; dy--) {
    ny = y+dy;
    for (dx=-1; dx<=1; dx++) {
      if (dx || dy) {
        nx = x+dx;
        dataCell nc = data[nx][ny];
        int v = nc.voronoi;
        bool b = (v<=free && v!=voronoiPrune); 
        //	if (v==occupied) obstacleCount++;
        f[i] = b;
        if (b) {
          voroCount++;
          if (!(dx && dy)) voroCountFour++;
        }
        if (b && !(dx && dy) ) count++;
        //	if (v<=free && !(dx && dy)) voroCount++;
        i++;
      }
    }
  }
  if (voroCount<3 && voroCountFour==1 && (f[1] || f[3] || f[4] || f[6])) {
    //    assert(voroCount<2);
    //    if (voroCount>=2) printf("voro>2 %d %d\n", x, y);
    return keep;
  }

  // 4-connected
  if ((!f[0] && f[1] && f[3]) || (!f[2] && f[1] && f[4]) || (!f[5] && f[3] && f[6]) || (!f[7] && f[6] && f[4])) return keep;
  if ((f[3] && f[4] && !f[1] && !f[6]) || (f[1] && f[6] && !f[3] && !f[4])) return keep;
  


  // keep voro cells inside of blocks and retry later
  if (voroCount>=5 && voroCountFour>=3 && data[x][y].voronoi!=voronoiRetry) {
    return retry;
  }

  return pruned;
}
