#include <cmath>
#include <algorithm>
#include <chrono>
#include "dstar_lite.h"
#include "gridworld.h"

// Just keeping INF sticky: if either is INF -> INF; otherwise normal sum.
static inline double plus_inf(double a, double b){
    if(a >= INF || b >= INF) return INF;
    return a + b;
}

// Store sizes/params and allocate the 2D maze array.
DStarLite::DStarLite(int rows_, int cols_, unsigned int heuristic_, std::string gridWorldName_){
    rows = rows_;
    cols = cols_;
    heuristic = heuristic_;
    gridWorldName = gridWorldName_;
    maze.resize(rows);
    for(int r=0;r<rows;++r) maze[r].resize(cols);
}

// Init step: reset counters, wire 8-neighbours (1 or sqrt(2)),
// set start/goal, push goal with key into OPEN, and precompute h for display.
void DStarLite::initialise(int startX, int startY, int goalX, int goalY){
    stateExpansions = 0; vertexAccesses = 0; maxQLength = 0;
    pathLength = 0.0; runningTime = 0.0; km = 0.0;
    open.clear();

    // per-cell defaults
    for(int y=0;y<rows;++y){
        for(int x=0;x<cols;++x){
            auto &c = maze[y][x];
            c.g = INF; c.rhs = INF; c.h = 0.0;
            c.key[0]=c.key[1]=0.0; c.x=x; c.y=y;
            for(int m=0;m<DIRECTIONS;++m){
                c.move[m]=nullptr; c.predecessor[m]=nullptr; c.linkCost[m]=1.0;
            }
        }
    }

    // hook neighbours + back-pointers; set diagonal costs
    for(int y=0;y<rows;++y){
        for(int x=0;x<cols;++x){
            for(int m=0;m<DIRECTIONS;++m){
                int nx = x + neighbours[m].x;
                int ny = y + neighbours[m].y;
                if(nx>=0 && nx<cols && ny>=0 && ny<rows){
                    maze[y][x].move[m] = &maze[ny][nx];
                    maze[ny][nx].predecessor[(m+4)%DIRECTIONS] = &maze[y][x];
                    bool diag = (neighbours[m].x!=0 && neighbours[m].y!=0);
                    maze[y][x].linkCost[m] = diag ? SQRT_2 : 1.0;
                }
            }
        }
    }

    // set pointers for start/goal
    start = &maze[startY][startX];
    goal  = &maze[goalY][goalX];

    // rhs(goal)=0 and insert goal into OPEN
    goal->rhs = 0.0;
    auto k = calculateKey(goal);
    goal->key[0]=k.first; goal->key[1]=k.second;
    openInsert(goal);

    // h depends on current start; recompute once
    updateHValues();
}

// Recompute all h-values from current start. Simple full sweep.
void DStarLite::updateHValues(){
    for(int y=0;y<rows;++y)
        for(int x=0;x<cols;++x)
            maze[y][x].h = calc_H_fromStart(x,y);
}

// Refresh all key[ ] values (mainly for display/debug).
void DStarLite::updateAllKeyValues(){
    for(int y=0;y<rows;++y)
        for(int x=0;x<cols;++x){
            auto k = calculateKey(&maze[y][x]);
            maze[y][x].key[0]=k.first; maze[y][x].key[1]=k.second;
        }
}

// Heuristic from start to (x,y). Here I switch EUCLIDEAN/CHEBYSHEV.
double DStarLite::calc_H_fromStart(int x, int y) const {
    int dx = std::abs(start->x - x);
    int dy = std::abs(start->y - y);
    if(heuristic == EUCLIDEAN) return std::sqrt((double)(dx*dx + dy*dy));
    return (double)std::max(dx, dy); // Chebyshev
}

// Key builder: [k1, k2] = [min(g,rhs)+h+km, min(g,rhs)].
std::pair<double,double> DStarLite::calculateKey(LpaStarCell* s) const {
    double min_grhs = std::min(s->g, s->rhs);
    return { min_grhs + s->h + km,  min_grhs };
}

// Insert into OPEN (ordered set). Also track peak size.
void DStarLite::openInsert(LpaStarCell* s){
    auto k = calculateKey(s);
    s->key[0]=k.first; s->key[1]=k.second;
    open.insert(OpenRef{ k.first, k.second, s->y, s->x, s });
    if((int)open.size() > maxQLength) maxQLength = (int)open.size();
}

// Remove specific state from OPEN via (y,x) match.
void DStarLite::openRemove(LpaStarCell* s){
    for(auto it = open.begin(); it != open.end(); ++it){
        if(it->y == s->y && it->x == s->x){
            open.erase(it);
            return;
        }
    }
}

// updateVertex(u): recompute rhs(u) and fix OPEN membership.
void DStarLite::updateVertex(LpaStarCell* u){
    if(u != goal){
        double best = INF;
        for(int m=0;m<DIRECTIONS;++m){
            LpaStarCell* s = u->move[m];
            if(!s || s->type=='1') continue;
            double cand = plus_inf(s->g, u->linkCost[m]);
            if(cand < best) best = cand;
        }
        u->rhs = best;
    }
    openRemove(u);
    if(u->g != u->rhs){
        openInsert(u);
    }
}

// Main loop (ComputeShortestPath). Keep time and counters for the table.
void DStarLite::computeShortestPath(){
    auto t0 = std::chrono::high_resolution_clock::now();

    while( !openEmpty() &&
           ( std::make_pair(topK1(), topK2()) < calculateKey(start)
             || start->rhs != start->g ) )
    {
        auto top = openTop();
        open.erase(open.begin());
        LpaStarCell* u = top.ptr;
        vertexAccesses++;

        auto k_old = std::make_pair(top.k1, top.k2);
        auto k_new = calculateKey(u);
        if( k_old < k_new ){
            // key got worse -> reinsert with fresh key
            openInsert(u);
            continue;
        }
        if( u->g > u->rhs ){
            // improve g; push impact to predecessors
            u->g = u->rhs;
            stateExpansions++;
            for(int m=0;m<DIRECTIONS;++m){
                LpaStarCell* p = u->predecessor[m];
                if(!p || p->type=='1') continue;
                updateVertex(p);
            }
        }else{
            // raise to INF and repair around it
            u->g = INF;
            updateVertex(u);
            for(int m=0;m<DIRECTIONS;++m){
                LpaStarCell* p = u->predecessor[m];
                if(!p || p->type=='1') continue;
                updateVertex(p);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    runningTime += std::chrono::duration<double>(t1 - t0).count();
}

// Pick successor with minimal (g + edge cost) — this is the next step.
LpaStarCell* DStarLite::bestSuccessor(LpaStarCell* s) const {
    LpaStarCell* best = nullptr;
    double bestCost = INF;
    for(int m=0;m<DIRECTIONS;++m){
        LpaStarCell* nb = s->move[m];
        if(!nb || nb->type=='1') continue;
        double cand = plus_inf(nb->g, s->linkCost[m]);
        if(cand < bestCost){
            bestCost = cand;
            best = nb;
        }
    }
    return best;
}

// Tiny “sensor” around start: map '9'->'1' (blocked), '8'->'0' (free).
// If anything changes, touch that cell + its neighbours.
void DStarLite::discoverAroundStart(){
    if(!world) return;
    for(int dy=-1; dy<=1; ++dy){
        for(int dx=-1; dx<=1; ++dx){
            int nx = start->x + dx;
            int ny = start->y + dy;
            if(nx<0||nx>=cols||ny<0||ny>=rows) continue;

            char observed = world->getMapTypeValue(ny, nx);
            LpaStarCell* cell = &maze[ny][nx];

            char newType = observed;
            if(observed=='9') newType='1';
            if(observed=='8') newType='0';

            if(cell->type != newType){
                cell->type = newType;
                updateVertex(cell);
                for(int m=0;m<DIRECTIONS;++m){
                    if(cell->predecessor[m]) updateVertex(cell->predecessor[m]);
                    if(cell->move[m])        updateVertex(cell->move[m]);
                }
            }
        }
    }
}

// One-shot planning: just run ComputeShortestPath once.
void DStarLite::Search(){
    computeShortestPath();
}

// One replan step and move one cell; advance km and refresh h.
bool DStarLite::ReplanStep(){
    if(start==goal) return true;

    discoverAroundStart();
    computeShortestPath();

    LpaStarCell* nxt = bestSuccessor(start);
    if(!nxt) return true; // no route
    double step = (std::abs(nxt->x - start->x)==1 && std::abs(nxt->y - start->y)==1) ? SQRT_2 : 1.0;
    pathLength += step;

    double inc = calc_H_fromStart(nxt->x, nxt->y);
    km += inc;
    start = nxt;
    updateHValues();
    return (start == goal);
}

// Convenience loop: keep stepping until done
void DStarLite::Replan(){
    int guard = rows*cols*10;
    while(start!=goal && guard-- > 0){
        if(ReplanStep()) break;
    }
}


// Copy visible world into internal maze, then call initialise.
void copyDisplayMapToDstarMaze(GridWorld& gWorld, DStarLite* dstar){
    dstar->world = &gWorld;
    int R = gWorld.getGridWorldRows();
    int C = gWorld.getGridWorldCols();

    for(int y=0;y<R;++y){
        for(int x=0;x<C;++x){
            dstar->maze[y][x].type = gWorld.getMapTypeValue(y,x);
            dstar->maze[y][x].x = x;
            dstar->maze[y][x].y = y;
        }
    }
    vertex s = gWorld.getStartVertex();
    vertex t = gWorld.getGoalVertex();
    dstar->initialise(s.col, s.row, t.col, t.row);
}

// Push g/rhs/h/type back to display; also sync start/goal markers.
void copyDstarMazeToDisplayMap(GridWorld& gWorld, DStarLite* dstar){
    int R = gWorld.getGridWorldRows();
    int C = gWorld.getGridWorldCols();

    // write costs/types out
    for(int y=0;y<R;++y){
        for(int x=0;x<C;++x){
            gWorld.setMapTypeValue(y,x, dstar->maze[y][x].type); // '0','1','8','9'
            gWorld.setMapGValue   (y,x, dstar->maze[y][x].g);
            gWorld.setMapRhsValue (y,x, dstar->maze[y][x].rhs);
            gWorld.setMapHValue   (y,x, dstar->maze[y][x].h);
        }
    }

    // clear old markers then set current ones
    vertex oldS = gWorld.getStartVertex();
    vertex oldG = gWorld.getGoalVertex();
    if(oldS.row >= 0 && oldS.col >= 0){
        if (gWorld.getMapTypeValue(oldS.row, oldS.col) == '6')
            gWorld.setMapTypeValue(oldS.row, oldS.col, '0');
    }
    if(oldG.row >= 0 && oldG.col >= 0){
        if (gWorld.getMapTypeValue(oldG.row, oldG.col) == '7')
            gWorld.setMapTypeValue(oldG.row, oldG.col, '0');
    }

    int sx = dstar->getStart()->x;
    int sy = dstar->getStart()->y;
    int gx = dstar->getGoal()->x;
    int gy = dstar->getGoal()->y;

    vertex newS; newS.row = sy; newS.col = sx;
    vertex newG; newG.row = gy; newG.col = gx;

    gWorld.setStartVertex(newS);
    gWorld.setGoalVertex(newG);

    gWorld.setMapTypeValue(sy, sx, '6');  // current robot
    gWorld.setMapTypeValue(gy, gx, '7');  // target
}
