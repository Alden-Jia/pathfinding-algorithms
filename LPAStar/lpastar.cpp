// Fast visualization throttle (single-file helper, no extra headers).
// It tries not to call delay() too often so the window stays responsive.
#ifndef VIZ_MIN_DELAY_MS
#define VIZ_MIN_DELAY_MS 12
#endif

#if !defined(__VIZ_THROTTLE_ONCE)
#define __VIZ_THROTTLE_ONCE 1
#include <chrono>
#include <thread>

#if defined(_WIN32) || defined(__WIN32__)
  #include <windows.h>
  inline void __viz_sleep_1ms__(){ Sleep(1); }
#else
  inline void __viz_sleep_1ms__(){ std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
#endif

#if defined(delay)
  #undef delay
#endif

// Very small helper to decide when to sleep ~1ms.
// We only sleep if at least VIZ_MIN_DELAY_MS passed since last draw tick.
namespace __viz_throttle_ns__ {
  inline bool should_delay() {
    using clock = std::chrono::steady_clock;
    static auto last = clock::time_point{};
    if (VIZ_MIN_DELAY_MS <= 0) return false;
    auto now = clock::now();
    if (last.time_since_epoch().count() == 0) { last = now; return true; }
    auto dms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last).count();
    if (dms >= VIZ_MIN_DELAY_MS) { last = now; return true; }
    return false;
  }
}
#define delay(ms) do { if (__viz_throttle_ns__::should_delay()) { __viz_sleep_1ms__(); } } while(0)
#endif

#include "lpastar.h"
#include "gridworld.h"
#include <algorithm>
#include <cstdio>

// Heartbeat prints (basic progress indicator):
// 0 = off; 1 = stderr single line; 2 = Windows: update console title (quiet).
#ifndef LPA_HEARTBEAT_MODE
#define LPA_HEARTBEAT_MODE 2
#endif

#if LPA_HEARTBEAT_MODE == 2
  #if defined(_WIN32) || defined(__WIN32__)
    #include <windows.h>  // SetConsoleTitleA
  #else
    #undef LPA_HEARTBEAT_MODE
    #define LPA_HEARTBEAT_MODE 1  // fallback on non-Windows
  #endif
#endif

// Compliance switch:
// 1 = submission/quiet (no fallbacks, minimal logs)
// 0 = debug (more logs, lenient)
#ifndef LPA_COMPLIANCE
#define LPA_COMPLIANCE 1
#endif

#if LPA_COMPLIANCE
  #define LPA_ENABLE_FALLBACK  0
  #define LPA_STRICT           1
  #ifndef LPA_HEARTBEAT_MODE
  #define LPA_HEARTBEAT_MODE   2
  #endif
  #ifndef LPA_DEBUG_LOGS
  #define LPA_DEBUG_LOGS       0
  #endif
#else
  #define LPA_ENABLE_FALLBACK  1
  #define LPA_STRICT           0
  #ifndef LPA_HEARTBEAT_MODE
  #define LPA_HEARTBEAT_MODE   1
  #endif
  #ifndef LPA_DEBUG_LOGS
  #define LPA_DEBUG_LOGS       1
  #endif
#endif

// Lightweight debug print (does nothing in submission mode).
#define LPA_DPRINT(...) do{ if (LPA_DEBUG_LOGS) std::fprintf(stderr, __VA_ARGS__); }while(0)

// Note: WinBGI provides delay() in graphics.h (already pulled by gridworld.h).

// -----------------------------------------------------------------------------
// Constructor: set sizes/heuristic/name, make empty grid and set viz cadence.
// -----------------------------------------------------------------------------
LpaStar::LpaStar(int rows_, int cols_, unsigned int heuristic_, const std::string& gridWorldName_)
: rows(rows_), cols(cols_), heuristic(heuristic_), gridWorldName(gridWorldName_) {
    strHeuristic = (heuristic == CHEBYSHEV) ? "CHEBYSHEV" : "EUCLIDEAN";
    makeEmptyMaze();
    vizStep = recommendedVizStep(); // pick a starting draw step
}

// Update all keys (useful when we want to draw key overlays).
void LpaStar::updateAllKeyValues(){
    if(!goal) return;
    for(int y = 0; y < rows; ++y){
        for(int x = 0; x < cols; ++x){
            calcKey(&maze[y][x]);
        }
    }
    if(start) calcKey(start);
    if(goal)  calcKey(goal);
}

// Create an empty grid of cells and reset pointers.
void LpaStar::makeEmptyMaze(){
    maze.assign(rows, std::vector<LpaStarCell>(cols));
    for(int y=0;y<rows;++y){
        for(int x=0;x<cols;++x){
            auto &c = maze[y][x];
            c.x = x; c.y = y;
            c.g = INF_D(); c.rhs = INF_D();
            c.h = 0.0; c.key[0]=c.key[1]=INF_D();
            c.in_open   = false;
            c.lastEnqK0 = INF_D();
            c.lastEnqK1 = INF_D();
        }
    }
    start = goal = nullptr;
}

// Bind start/goal by coordinates and refresh heuristics.
// We also clamp/repair (x,y) if swapped or out of bounds.
void LpaStar::initialise(int startX, int startY, int goalX, int goalY){
    // Rebuild empty maze (keeps rows/cols).
    makeEmptyMaze();

    // If (x,y) is out but (y,x) is valid, swap once (simple user safety).
    bool s_ok     = (startX>=0 && startX<cols && startY>=0 && startY<rows);
    bool s_sw_ok  = (startY>=0 && startY<cols && startX>=0 && startX<rows);
    if(!s_ok && s_sw_ok) { std::swap(startX,startY); }

    bool g_ok     = (goalX>=0 && goalX<cols && goalY>=0 && goalY<rows);
    bool g_sw_ok  = (goalY>=0 && goalY<cols && goalX>=0 && goalX<rows);
    if(!g_ok && g_sw_ok) { std::swap(goalX,goalY); }

    // Clamp to bounds.
    if(startX < 0) startX = 0; else if(startX >= cols) startX = cols-1;
    if(startY < 0) startY = 0; else if(startY >= rows) startY = rows-1;
    if(goalX  < 0) goalX  = 0; else if(goalX  >= cols) goalX  = cols-1;
    if(goalY  < 0) goalY  = 0; else if(goalY  >= rows) goalY  = rows-1;

    // Note: maze index is [row=y][col=x].
    start = &maze[startY][startX];
    goal  = &maze[goalY][goalX];

    // Heuristics depend on goal, so update now.
    updateHValues();

    // One-time print to quickly verify binding (ok to comment out later).
    std::fprintf(stderr,
        "[init] rows=%d cols=%d  bind S=(x=%d,y=%d)  G=(x=%d,y=%d)\n",
        rows, cols, startX, startY, goalX, goalY);
}

// Recompute h for every cell (depends on goal).
void LpaStar::updateHValues(){
    if(!goal) return;
    for(int y=0;y<rows;++y)
        for(int x=0;x<cols;++x)
            maze[y][x].h = calc_H(x,y);
}

// Heuristic for 8-connected grid.
// If CHEBYSHEV: use Chebyshev distance (matches 8-neighbor steps nicely).
// Else: plain Euclidean distance.
double LpaStar::calc_H(int x,int y) const{
    int dx = std::abs(goal->x - x);
    int dy = std::abs(goal->y - y);
    if (heuristic == CHEBYSHEV) {
        return static_cast<double>(std::max(dx, dy));
    }
    return std::sqrt(static_cast<double>(dx*dx + dy*dy));
}

// Move cost: straight=1, diagonal=sqrt(2).
double LpaStar::moveCost(int dx,int dy){
    return (dx==0 || dy==0) ? 1.0 : std::sqrt(2.0);
}

// LPA* key(s) = (min(g,rhs)+h, min(g,rhs))
void LpaStar::calcKey(LpaStarCell* s){
    double m = minval(s->g, s->rhs);
    s->key[0] = m + s->h;
    s->key[1] = m;
}

// Return current smallest key at the top of OPEN.
// We drop stale entries (lazy deletion).
std::pair<double,double> LpaStar::topKey(){
    auto eq = [](double a,double b){
        if(std::isinf(a) && std::isinf(b)) return true;
        return std::fabs(a-b) <= 1e-9;
    };
    while(!U.empty()){
        const auto &t = U.top();
        if( eq(t.k0, t.node->key[0]) && eq(t.k1, t.node->key[1]) )
            return {t.k0, t.k1};
        U.pop(); // stale
    }
    return {INF_D(), INF_D()};
}

// Pop the minimum valid entry from OPEN.
// If we only hit stale entries, keep popping.
bool LpaStar::popMinWithKey(LpaStarCell*& out, double& k0, double& k1){
    auto eq = [](double a,double b){
        if (std::isinf(a) && std::isinf(b)) return true;
        return std::fabs(a-b) <= 1e-9;
    };
    while(!U.empty()){
        auto t = U.top(); U.pop();
        if (eq(t.k0, t.node->key[0]) && eq(t.k1, t.node->key[1])) {
            out = t.node; k0 = t.k0; k1 = t.k1;
            out->in_open = false;     // now it's truly out of OPEN
            queuePops++; queueAccesses++;
            return true;
        }
        // else stale -> ignore
    }
    return false;
}

// Push a node using its current key (only if g != rhs).
// If key got larger and it's already in OPEN, we skip pushing;
// older smaller entry is still there and will force a reinsert later.
void LpaStar::pushWithCurrentKey(LpaStarCell* s){
    if (approxEq(s->g, s->rhs)) return;  // consistent -> no need in OPEN

    bool last_le_current = !lexLess(s->key[0], s->key[1], s->lastEnqK0, s->lastEnqK1);
    if (s->in_open && last_le_current) return;

    U.push(PQItem{ s, s->key[0], s->key[1] });
    s->in_open   = true;
    s->lastEnqK0 = s->key[0];
    s->lastEnqK1 = s->key[1];

    queuePushes++;
    queueAccesses++;
    if((long)U.size() > maxQLength) maxQLength = (long)U.size();
}

// A cell is traversable if it's not a wall ('1').
// Start/goal are always treated as traversable (so relaxations can propagate).
// During INITIAL planning we also avoid '8' (unknown-free) by default.
// If your assignment asks to avoid '9' initially too, add it here.
bool LpaStar::traversable(int x,int y,bool initialPhase) const{
    const LpaStarCell* cell = &maze[y][x];

    if (cell == start || cell == goal) return true;

    const char tp = cell->type;

    if (initialPhase) {
        if (tp == '8') return false;
        // if (tp == '9') return false; // enable if required by your PDF
    }

    return (tp != '1');
}

// Visit predecessors of s (8-neighborhood). Call fn(pred, cost).
void LpaStar::forEachPred(LpaStarCell* s,
                          const std::function<void(LpaStarCell*, double)>& fn,
                          bool initialPhase){
    if(!traversable(s->x, s->y, initialPhase)) return;

    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    auto inb = [&](int x,int y){ return x>=0 && x<cols && y>=0 && y<rows; };

    for(int k=0;k<8;++k){
        int nx = s->x + dx[k], ny = s->y + dy[k];
        if(!inb(nx,ny)) continue;
        if(!traversable(nx, ny, initialPhase)) continue;

        double c = moveCost(std::abs(dx[k]), std::abs(dy[k]));
        fn(&maze[ny][nx], c);
    }
}

// Visit successors of s (8-neighborhood). Call fn(succ, cost).
void LpaStar::forEachSucc(LpaStarCell* s,
                          const std::function<void(LpaStarCell*, double)>& fn,
                          bool initialPhase){
    if(!traversable(s->x, s->y, initialPhase)) return;

    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    auto inb = [&](int x,int y){ return x>=0 && x<cols && y>=0 && y<rows; };

    for(int k=0;k<8;++k){
        int nx = s->x + dx[k], ny = s->y + dy[k];
        if(!inb(nx,ny)) continue;
        if(!traversable(nx, ny, initialPhase)) continue;

        double c = moveCost(std::abs(dx[k]), std::abs(dy[k]));
        fn(&maze[ny][nx], c);
    }
}

// Standard LPA* vertex update.
// If not start: rhs(u) = min over predecessors (g(pred) + cost).
// Then push u if inconsistent (lazy insertion).
void LpaStar::updateVertex(LpaStarCell* u, bool initialPhase){
    vertexAccesses++;
    if(u != start){
        double minrhs = INF_D();
        forEachPred(u, [&](LpaStarCell* p, double c){
            minrhs = std::min(minrhs, p->g + c);
        }, initialPhase);
        u->rhs = minrhs;
    }
    calcKey(u);
    pushWithCurrentKey(u);
}

// Main loop: keep improving until OPEN's best key is no better than goal's key
// and goal is locally consistent (g == rhs).
void LpaStar::computeShortestPath(bool initialPhase){
    if(!start || !goal) return;

#if LPA_HEARTBEAT_MODE != 0
    // Heartbeat config: just to show some progress on large maps.
    const int HEARTBEAT_EVERY_POP = 5000;
    const int HEARTBEAT_EVERY_MS  = 250;
    auto hbStart  = std::chrono::steady_clock::now();
    auto lastBeat = hbStart;
    size_t pops   = 0;

    auto hb_emit = [&](bool force=false){
        auto now = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastBeat).count();
        if(!force && ms < HEARTBEAT_EVERY_MS) return;

        double elapsed = std::chrono::duration<double>(now - hbStart).count();
      #if LPA_HEARTBEAT_MODE == 1
        static const char SPIN[4] = {'|','/','-','\\'};
        static int si = 0;
        std::fprintf(stderr, "\r[%c] LPA* running... pops=%zu OPEN=%zu t=%.2fs",
                     SPIN[(si++) & 3], pops, this->U.size(), elapsed);
        std::fflush(stderr);
      #elif LPA_HEARTBEAT_MODE == 2
        char title[128];
        std::snprintf(title, sizeof(title),
                      "LPA* running... pops=%zu OPEN=%zu t=%.2fs",
                      pops, this->U.size(), elapsed);
        SetConsoleTitleA(title);
      #endif
        lastBeat = now;
    };
#endif

    const double EPS = 1e-8;

    while(true){
        auto tk = topKey();
        calcKey(goal);
        const double kg0 = goal->key[0], kg1 = goal->key[1];

        bool needMore =
            ( !U.empty() && ( tk.first < kg0 - EPS ||
                              (std::fabs(tk.first - kg0) <= EPS && tk.second < kg1 - EPS) ) )
            || ( std::fabs(goal->g - goal->rhs) > EPS );

        if(!needMore) break;
        if(U.empty()){
            LPA_DPRINT("[LPA*] OPEN empty but goal inconsistent (g!=rhs).\n");
            break;
        }

        LpaStarCell* u = nullptr; double oldk0=0.0, oldk1=0.0;
        if(!popMinWithKey(u, oldk0, oldk1)){
#if LPA_HEARTBEAT_MODE != 0
            hb_emit(false);  // still emit sometimes
#endif
            continue;
        }

#if LPA_HEARTBEAT_MODE != 0
        ++pops;
        if((pops % HEARTBEAT_EVERY_POP) == 0) hb_emit(true);
        else                                   hb_emit(false);
#endif

        // If key grew, reinsert with new (bigger) key.
        calcKey(u);
        if (lexLess(oldk0, oldk1, u->key[0], u->key[1])) {
            pushWithCurrentKey(u);
            continue;
        }

        if (u->g > u->rhs + EPS) {
            // improve g(u) to rhs(u)
            u->g = u->rhs;
            ++stateExpansions;

            // relax successors
            forEachSucc(u, [&](LpaStarCell* s, double c){
                const double cand = u->g + c;
                if (cand + EPS < s->rhs) s->rhs = cand;
                updateVertex(s, initialPhase);
            }, initialPhase);
        } else {
            // make u inconsistent and fix neighbors relying on old g(u)
            const double g_old = u->g;
            u->g = INF_D();
            updateVertex(u, initialPhase);
            forEachSucc(u, [&](LpaStarCell* s, double c){
                if (approxEq(s->rhs, g_old + c)) {
                    double minrhs = INF_D();
                    forEachPred(s, [&](LpaStarCell* p, double cp){
                        minrhs = std::min(minrhs, p->g + cp);
                    }, initialPhase);
                    s->rhs = minrhs;
                }
                updateVertex(s, initialPhase);
            }, initialPhase);
        }
    }

#if LPA_HEARTBEAT_MODE == 1
    std::fprintf(stderr, "\n");
#elif LPA_HEARTBEAT_MODE == 2
    SetConsoleTitleA("LPA* done");
#endif

    queueSizeAfter = (long)U.size();

    // If goal.g is INF -> no path found (we report 0.0 as path length).
    if (std::isinf(goal->g)) {
        pathLength = 0.0;
    } else {
        pathLength = reconstructPathLength();
    }
}

// Reconstruct the path length by walking from goal backwards to start.
// We always pick the neighbor with smallest (g + stepCost).
double LpaStar::reconstructPathLength() const{
    if(!start || !goal) return 0.0;
    const LpaStarCell* cur = goal;
    double len = 0.0;
    int guard = rows*cols + 5; // just-in-case safety
    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};
    while(cur && cur!=start && guard-- > 0){
        const LpaStarCell* best = nullptr;
        double bestVal = INF_D();
        for(int k=0;k<8;++k){
            int nx = cur->x + dx[k], ny = cur->y + dy[k];
            if(nx<0||nx>=cols||ny<0||ny>=rows) continue;
            const LpaStarCell* p = &maze[ny][nx];
            if(std::isinf(p->g)) continue;
            double c = moveCost(std::abs(dx[k]), std::abs(dy[k]));
            double v = p->g + c;
            if(v < bestVal){ bestVal = v; best = p; }
        }
        if(!best) break;
        len += moveCost(std::abs(best->x - cur->x), std::abs(best->y - cur->y));
        cur = best;
    }
    return len;
}

// Reset counters/metrics before a run.
void LpaStar::resetStats(){
    stateExpansions = 0;
    maxQLength = 0;
    vertexAccesses = 0;
    pathLength = 0.0;
    runningTime = 0.0;
    queuePushes = queuePops = queueAccesses = 0;
    queueSizeAfter = 0;
}

// Initial planning run (before we "reveal" 8/9).
void LpaStar::initialPlanning(){
    strEpisode = "INITIAL";
    resetStats();

    // Start with everything inconsistent (g=INF, rhs=INF).
    for(int y=0; y<rows; ++y){
        for(int x=0; x<cols; ++x){
            maze[y][x].g      = INF_D();
            maze[y][x].rhs    = INF_D();
            maze[y][x].key[0] = INF_D();
            maze[y][x].key[1] = INF_D();
            maze[y][x].in_open   = false;
            maze[y][x].lastEnqK0 = INF_D();
            maze[y][x].lastEnqK1 = INF_D();
        }
    }

    updateHValues();

    // Reset OPEN and seed with start (rhs=0).
    U = decltype(U)();
    if(start){
        start->rhs = 0.0;
        calcKey(start);
        pushWithCurrentKey(start);
        maxQLength = std::max<long>(maxQLength, (long)U.size());
    }

    // Run LPA* in "initial" phase (treat '8' as blocked).
    auto t0 = std::chrono::steady_clock::now();
    computeShortestPath(true);
    auto t1 = std::chrono::steady_clock::now();
    runningTime = std::chrono::duration<double>(t1 - t0).count();

    // Store a polyline for drawing the "initial" path in cyan.
    storeInitialPathPolyline();
}

// Incremental "final" planning: reveal cells (9->1, 8->0) and replan.
// We do NOT reset g/rhs here; we try to reuse work (incremental).
void LpaStar::finalPlanningIncremental(){
    if(!start || !goal) return;

    strEpisode = "FINAL";

    // 1) Apply reveal: 9->1 (becomes walls), 8->0 (becomes free).
    //    Track which cells changed so we can update neighbors.
    std::vector<std::pair<int,int>> changed;
    auto reveal = [](char t){ return (t=='9') ? '1' : ((t=='8') ? '0' : t); };

    for(int y=0; y<rows; ++y){
        for(int x=0; x<cols; ++x){
            char nt = reveal(maze[y][x].type);
            if(nt != maze[y][x].type){
                maze[y][x].type = nt;
                changed.emplace_back(y,x);
            }
        }
    }

    // Heuristics do not depend on obstacles, but refreshing is cheap.
    updateHValues();

    // 2) If nothing changed, still store the final path (so the yellow line appears).
    if (changed.empty()) {
        updateHValues();
        storeFinalPathPolyline();
        return;
    }

    // 3) Touch changed cells and their neighbors (updateVertex) to fix rhs/g.
    auto touch = [this](int r,int c){
        if(r<0 || r>=rows || c<0 || c>=cols) return;
        updateVertex(&maze[r][c], /*initialPhase=*/false);
    };
    for(const auto& rc : changed){
        int r = rc.first, c = rc.second;
        touch(r,c);
        for(int dr=-1; dr<=1; ++dr)
            for(int dc=-1; dc<=1; ++dc)
                if(dr || dc) touch(r+dr, c+dc);
    }

    // 4) Make sure start is in OPEN (seed).
    start->rhs = 0.0;
    calcKey(start);
    pushWithCurrentKey(start);

    // 5) Replan incrementally (now treat '8' as free, '9' as walls).
    auto t0 = std::chrono::steady_clock::now();
    computeShortestPath(false);
    auto t1 = std::chrono::steady_clock::now();
    runningTime = std::chrono::duration<double>(t1 - t0).count();

    // 6) Try to build the path polyline. If failed, do a full final run as fallback.
    bool goal_ok = !std::isinf(goal->g);
    auto rc = _buildPathRC();
    if(!goal_ok || rc.size() < 2){
        finalPlanning();          // full reset fallback
        rc = _buildPathRC();      // try again for drawing
    }
    _finalPathRC = rc;            // save the yellow line data
}

// Full "final" planning: reveal and rebuild from scratch (non-incremental).
void LpaStar::finalPlanning(){
    if(!start || !goal){
        std::fprintf(stderr, "[LPA*] ERROR: start/goal not set.\n");
        return;
    }

    strEpisode = "FINAL";
    resetStats();

    // Apply reveal and reset all search state.
    for(int y=0; y<rows; ++y){
        for(int x=0; x<cols; ++x){
            char &t = maze[y][x].type;
            if (t == '9') t = '1';
            else if (t == '8') t = '0';
            maze[y][x].g      = INF_D();
            maze[y][x].rhs    = INF_D();
            maze[y][x].key[0] = INF_D();
            maze[y][x].key[1] = INF_D();
            maze[y][x].in_open   = false;
            maze[y][x].lastEnqK0 = INF_D();
            maze[y][x].lastEnqK1 = INF_D();
        }
    }

    // Reset OPEN and seed with start.
    U = decltype(U)();
    if(start){
        start->rhs = 0.0;
        calcKey(start);
        pushWithCurrentKey(start);
        maxQLength = std::max<long>(maxQLength, (long)U.size());
    }

    auto t0 = std::chrono::steady_clock::now();
    computeShortestPath(false);
    auto t1 = std::chrono::steady_clock::now();
    runningTime = std::chrono::duration<double>(t1 - t0).count();

    // Save the final polyline so we can draw a yellow path.
    storeFinalPathPolyline();
}

// Print a CSV-style line with metrics (used by "all" mode).
void LpaStar::printResults() const{
    // Algorithm,Heuristic,Map,Episode,Expansions,MaxQ,Accesses,PathLen,Time(s)
    std::string algo = "LPA_STAR";
    std::cout << std::left << std::setw(12) << algo
              << std::left << std::setw(2)  << "," << std::left << std::setw(12) << strHeuristic
              << std::left << std::setw(2)  << "," << std::left << std::setw(24) << gridWorldName
              << std::left << std::setw(2)  << "," << std::left << std::setw(8)  << strEpisode
              << std::right << std::fixed;

    std::cout << std::setprecision(0) << std::setw(1) << " " << "," << std::setw(10) << stateExpansions
              << std::setw(1) << " " << "," << std::setw(10) << maxQLength
              << std::setw(1) << " " << "," << std::setw(10) << vertexAccesses;

    std::cout << std::setprecision(2)
              << std::setw(1) << " " << "," << std::setw(10) << pathLength
              << std::setw(1) << " " << "," << std::setw(10) << std::setprecision(6) << runningTime
              << std::endl;
}

// Very small helper to pick an initial draw step from grid size.
int LpaStar::recommendedVizStep() const{
    long cells = 1L * rows * cols;
    int est = (int)(cells / 80);
    if(est < 10) est = 10;
    if(est > 300) est = 300;
    return est;
}

// Try to keep drawing neither too fast nor too slow.
void LpaStar::autoTuneVizOnDraw(){
    using clock = std::chrono::steady_clock;
    auto now = clock::now();
    if(lastVizTp.time_since_epoch().count() == 0){
        lastVizTp = now;
        return;
    }
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastVizTp).count();
    lastVizTp = now;

    if(dt < 5) {
        if(vizStep < 1000) vizStep = std::min(1000, vizStep * 2);
    } else if(dt > 60) {
        if(vizStep > 1) vizStep = std::max(1, vizStep / 2);
    }
}

// Build the path as a list of (row, col), from start to goal.
// We walk from goal backwards choosing neighbor with smallest (g + stepCost).
std::vector<std::pair<int,int>> LpaStar::_buildPathRC() const {
    std::vector<std::pair<int,int>> rc;
    if (!start || !goal) return rc;
    if (std::isinf(goal->g)) return rc;  // unreachable -> empty

    const LpaStarCell* cur = goal;
    static const int dx[8] = {-1, 0, 1, -1, 1, -1, 0, 1};
    static const int dy[8] = {-1,-1,-1,  0, 0,  1, 1, 1};

    int guard = rows * cols + 5; // safety break
    while (cur && guard-- > 0) {
        rc.emplace_back(cur->y, cur->x); // store as (row, col)
        if (cur == start) break;

        const LpaStarCell* best = nullptr;
        double bestVal = std::numeric_limits<double>::infinity();

        for (int k = 0; k < 8; ++k) {
            int nx = cur->x + dx[k], ny = cur->y + dy[k];
            if (nx < 0 || nx >= cols || ny < 0 || ny >= rows) continue;
            if (!traversable(nx, ny, /*initialPhase=*/false)) continue;

            const LpaStarCell* p = &maze[ny][nx];
            if (std::isinf(p->g)) continue;

            double step = moveCost(std::abs(dx[k]), std::abs(dy[k]));
            double v = p->g + step;
            if (v < bestVal) { bestVal = v; best = p; }
        }

        if (!best) break;
        cur = best;
    }

    std::reverse(rc.begin(), rc.end());  // now it's start -> goal
    return rc;
}

// Convert (row, col) polyline to pixel centers and draw it as a line path.
void LpaStar::_drawPolylineFromRC(const std::vector<std::pair<int,int>>& rcPath,
                                  int color, int thickness) const {
    if(!gworld || rcPath.size() < 2) return;

    const int FX1 = gworld->getFieldX1();
    const int FY1 = gworld->getFieldY1();
    const int FX2 = gworld->getFieldX2();
    const int FY2 = gworld->getFieldY2();

    const int cw = (FX2 - FX1 + 1) / cols;
    const int ch = (FY2 - FY1 + 1) / rows;

    auto cellCenter = [&](int r, int c, int& x, int& y){
        x = FX1 + c * cw + cw/2;
        y = FY1 + r * ch + ch/2;
    };

    setcolor(color);
    setlinestyle(SOLID_LINE, 0, thickness);

    for(size_t i=1;i<rcPath.size();++i){
        int x0,y0,x1,y1;
        cellCenter(rcPath[i-1].first, rcPath[i-1].second, x0, y0);
        cellCenter(rcPath[i].first,   rcPath[i].second,   x1, y1);
        line(x0,y0,x1,y1);
    }
}

// Store the polyline built from current g-values for the "initial" run.
void LpaStar::storeInitialPathPolyline(){ _initialPathRC = _buildPathRC(); }

// Store the polyline built from current g-values for the "final" run.
void LpaStar::storeFinalPathPolyline(){   _finalPathRC   = _buildPathRC(); }

// Draw both polylines (cyan for initial, yellow for final).
void LpaStar::drawStoredPolylines() const {
#if defined(_WIN32) || defined(__WIN32__)
    _drawPolylineFromRC(_initialPathRC, LIGHTCYAN, 2);  // initial path
    _drawPolylineFromRC(_finalPathRC,  YELLOW,    3);   // final path
#else
    _drawPolylineFromRC(_initialPathRC, CYAN, 2);
    _drawPolylineFromRC(_finalPathRC,  YELLOW, 3);
#endif
}
