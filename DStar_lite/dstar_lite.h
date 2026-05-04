#ifndef __DSTAR_LITE_H__
#define __DSTAR_LITE_H__

#include <vector>
#include <string>
#include <set>
#include <limits>
#include <utility>
#include "globalvariables.h"

// 嗯，这里先前向声明一下，不想在头文件里把 GridWorld 全 include 进来
class GridWorld; 

// 这个类就是 D* Lite 的主体…就按书上的变量名来，方便对比
class DStarLite {
public:
    // 构造函数：就存下尺寸/参数，然后把二维数组先开出来
    DStarLite(int rows_, int cols_, unsigned int heuristic_, std::string gridWorldName_);

    // 初始化：把起点终点喂进来，清状态，塞一个 key(goal) 到 OPEN
    void initialise(int startX, int startY, int goalX, int goalY);

    // 第一次规划：其实就是跑一遍 ComputeShortestPath
    void Search();       // First planning

    // 反复重规划直到到达或者无路可走…这里面就是循环 ReplanStep
    void Replan();       // Continuous reprogramming until arrival or roadlessness

    // 单步重规划并把“机器人”挪一格…返回 true 表示到目标了
    bool ReplanStep();   

    // 这俩是为了重算显示用的 h、key（调试/展示会用到）
    void updateHValues();
    void updateAllKeyValues();

    // 统计量都丢这儿…填表会用到（扩展次数、访问次数、OPEN 最大长度、路径长、耗时）
    int stateExpansions = 0; // 展开了多少次
    int vertexAccesses  = 0; // 从 OPEN 里摸了多少次
    int maxQLength      = 0; // OPEN 的峰值
    double pathLength   = 0.0; // 实走路径长度（1 或 √2 累加）
    double runningTime  = 0.0; // 累计用时（秒）

    // 简单 getter：主要给 UI 同步小绿点小蓝点用
    LpaStarCell* getStart() const { return start; }
    LpaStarCell* getGoal()  const { return goal;  }

    // 网格尺寸 + 启发式类型（CHEBYSHEV / EUCLIDEAN）
    int rows = 0, cols = 0;
    unsigned int heuristic = CHEBYSHEV;
    std::string gridWorldName;

    // 算法内部用的网格（g、rhs、key、type 都在这里）
    std::vector<std::vector<LpaStarCell>> maze;

    // 跟显示层的桥…为了“观察”周围 8 邻域（把 9/8 转成 1/0）
    GridWorld* world = nullptr;

private:
    // —— 下面是算法的核心零件 —— //

    // 启发式：从当前 start 到 (x,y)…就两种，欧氏/切比雪夫
    double calc_H_fromStart(int x, int y) const;

    // 生成 key：[k1, k2] = [min(g,rhs)+h+km, min(g,rhs)]
    std::pair<double,double> calculateKey(LpaStarCell* s) const;

    // 经典的 updateVertex(u)…该进 OPEN 进，该出出
    void updateVertex(LpaStarCell* u);

    // 主循环：ComputeShortestPath…一直把队头处理好为止
    void computeShortestPath();

    // 走路的时候挑个最优后继：argmin (g(successor) + c)
    LpaStarCell* bestSuccessor(LpaStarCell* s) const;

    // 小“传感器”：围着 start 看一圈，把 9→1、8→0，并触发更新
    void discoverAroundStart(); 

    // —— OPEN 集合（按 key 排序，坐标只是打平手） —— //
    struct OpenRef {
        double k1, k2; int y, x; LpaStarCell* ptr;
    };
    struct OpenCmp {
        bool operator()(const OpenRef& a, const OpenRef& b) const {
            if (a.k1 != b.k1) return a.k1 < b.k1;
            if (a.k2 != b.k2) return a.k2 < b.k2;
            if (a.y  != b.y)  return a.y  < b.y;
            return a.x < b.x;
        }
    };
    std::set<OpenRef, OpenCmp> open;

    // 这仨就是对 OPEN 的小包装
    void openInsert(LpaStarCell* s);
    void openRemove(LpaStarCell* s);
    bool openEmpty() const { return open.empty(); }
    OpenRef openTop() const { return *open.begin(); }
    double topK1() const { return open.empty()? INF : open.begin()->k1; }
    double topK2() const { return open.empty()? INF : open.begin()->k2; }

    // 当前的 start/goal，以及 km（关键字里那一项的增量）
    LpaStarCell* start = nullptr;
    LpaStarCell* goal  = nullptr;
    double km = 0.0;
};

// 这俩是桥函数：把显示层 ←→ 算法层 的数据做同步
void copyDisplayMapToDstarMaze(GridWorld& gWorld, DStarLite* dstar);
void copyDstarMazeToDisplayMap(GridWorld& gWorld, DStarLite* dstar);

#endif
