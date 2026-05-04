///////////////////////////////////////////////////////////////////////////////////////////
//
//
//  
//                        
//
//            Program Name: Incremental Search 
//             Description: start-up code for simulating LPA* and D*Lite
//                        - implements a gridworld that can be loaded from file, and 
//                          modified through a user-interface 
//
//        Run Parameters: 
//
//    Keys for Operation: 
//
//                  History:  date of revision
//                         13/Aug/2025  
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <algorithm>
#include <iostream>
#include <chrono>
#include <queue>
#include <cmath>



#if defined __unix__ || defined __APPLE__
  #include <graphics.h>
#elif defined __WIN32__
  #include "graphics.h"
#endif

#include "globalvariables.h"
#include "transform.h"
#include "gridworld.h"
#include "dstar_lite.h"

// Beginner note: keeping a global heuristic just like the starter code did.
// This one is only for display header; the actual planner uses D_HEURISTIC below.
unsigned int HEURISTIC = CHEBYSHEV;


// -------------------- globals --------------------
// Beginner note: tiny UI state flags. Not fancy, just convenient for toggles.
int BACKGROUND_COLOUR, LINE_COLOUR;
int GRIDWORLD_ROWS, GRIDWORLD_COLS;
bool SHOW_MAP_DETAILS = false;
bool CONTROL_KEY_FLAG = false;

// Beginner note: these switches control which numbers I draw inside each cell.
bool SHOW_G   = false;
bool SHOW_RHS = false;
bool SHOW_H   = false;
bool SHOW_KEY = false; 

// Beginner note: singletons for the whole run.
GridWorld grid_world;
DStarLite* dstar = nullptr;

// Beginner note: this is the heuristic actually passed to D* Lite via argv.
unsigned int D_HEURISTIC = CHEBYSHEV;

// 计算“当前显示地图”上的计划路径长度（不依赖 D* 内部 g/rhs）
static double planned_len_astar_on_display(bool treat9asBlocked)
{
    int R = grid_world.getGridWorldRows();
    int C = grid_world.getGridWorldCols();
    vertex s = grid_world.getStartVertex();
    vertex g = grid_world.getGoalVertex();
    if (s.row < 0 || s.col < 0 || g.row < 0 || g.col < 0) return INF;

    auto idx = [C](int r,int c){ return r*C + c; };
    std::vector<double> dist((size_t)R*C, INF);

    struct Node{int r,c; double g,f;};
    struct Cmp { bool operator()(const Node& a, const Node& b) const { return a.f > b.f; } };
    auto inb  = [&](int r,int c){ return r>=0 && r<R && c>=0 && c<C; };
    auto step = [&](int dy,int dx){ return (dy!=0 && dx!=0)? SQRT_2 : 1.0; };
    auto hfun = [&](int r,int c){
        int dr = std::abs(g.row - r), dc = std::abs(g.col - c);
        if (D_HEURISTIC == EUCLIDEAN) return std::hypot((double)dr, (double)dc);
        return (double)std::max(dr, dc); // CHEBYSHEV
    };

    std::priority_queue<Node, std::vector<Node>, Cmp> open;
    dist[idx(s.row, s.col)] = 0.0;
    open.push({s.row, s.col, 0.0, hfun(s.row, s.col)});

    while(!open.empty()){
        Node cur = open.top(); open.pop();
        int id = idx(cur.r, cur.c);
        if (cur.g > dist[id] + 1e-12) continue;
        if (cur.r == g.row && cur.c == g.col) return cur.g;

        for (int dy=-1; dy<=1; ++dy){
            for (int dx=-1; dx<=1; ++dx){
                if (dy==0 && dx==0) continue;
                int nr = cur.r + dy, nc = cur.c + dx;
                if (!inb(nr,nc)) continue;

                char t = grid_world.getMapTypeValue(nr, nc);
                if (t == '1') continue;
                if (treat9asBlocked && t == '9') continue; 

                double ng = cur.g + step(dy,dx);
                int nid = idx(nr,nc);
                if (ng + 1e-12 < dist[nid]){
                    dist[nid] = ng;
                    open.push({nr,nc,ng, ng + hfun(nr,nc)});
                }
            }
        }
    }
    return INF; 
}


// -------------------- helpers --------------------
// Beginner note: whenever start/goal/map changes, I simply recreate D* Lite.
// Heavy-handed but safe for coursework; avoids edge-case state bugs.
static void recreate_dstar()
{
    if (dstar) { delete dstar; dstar = nullptr; }
    dstar = new DStarLite(grid_world.getGridWorldRows(),
                          grid_world.getGridWorldCols(),
                          D_HEURISTIC,
                          std::string("grid"));
    copyDisplayMapToDstarMaze(grid_world, dstar); // Beginner note: sets dstar->world pointer too.
}

// Beginner note: double-buffer redraw (page flip) to reduce flicker.
static void redraw(bool pageFlip=true)
{
    static bool page=false;
    if (pageFlip) page = !page;
    setactivepage(page);
    cleardevice();

    // Beginner note: prefer selected-details view if any detail flags are on.
    if (SHOW_G || SHOW_RHS || SHOW_H || SHOW_KEY) {
        grid_world.displayMapWithSelectedDetails(SHOW_G, SHOW_RHS, SHOW_H, SHOW_KEY);
    } else if (SHOW_MAP_DETAILS) {
        grid_world.displayMapWithDetails();
    } else {
        grid_world.displayMap();
    }
}

// Beginner note: after planning or stepping, I sync costs back to GridWorld,
// draw the polyline path, and dump one stats line (for the spreadsheet).
static void draw_path_and_info()
{
    copyDstarMazeToDisplayMap(grid_world, dstar);
    grid_world.displayPath_for_dStarLite();


    double planLen = planned_len_astar_on_display(false);
    if (!(planLen < INF*0.5)) {
        planLen = planned_len_astar_on_display(true);
    }

    std::cout << "DSTAR Exp=" << dstar->stateExpansions
              << " VA=" << dstar->vertexAccesses
              << " Qmax=" << dstar->maxQLength
              << " PathLen=" << ( (planLen < INF*0.5) ? planLen : -1.0 )
              << " Time=" << dstar->runningTime << "s\n";
}




// Beginner note: Ctrl+Space → build the initial plan (no movement yet).
static void initial_plan()
{
    if (!dstar) recreate_dstar();
    dstar->Search();
    redraw();
    draw_path_and_info();
}

// Beginner note: Enter → do one replan step and move one cell along the path.
static void replan_single_step()
{
    if (!dstar) recreate_dstar();
    bool done = dstar->ReplanStep();
    redraw();
    draw_path_and_info();
    if (done) std::cout << "Reached goal or no path.\n";
}

// Beginner note: Ctrl+Enter → loop steps until goal (ESC to break).
static void replan_to_goal()
{
    if (!dstar) recreate_dstar();
    int guard = GRIDWORLD_ROWS * GRIDWORLD_COLS * 10; // quick safety guard
    while (!dstar->ReplanStep() && guard-- > 0) {
        redraw(false);
        draw_path_and_info();
        delay(10);
        // Beginner note: allow ESC to break out early. (was: 可加入 ESC 中断)
        if (GetAsyncKeyState(VK_ESCAPE)) break;
    }
    redraw();
    draw_path_and_info();
}

// -------------------- input --------------------
// Beginner note: simple key poller (Windows). I just return small codes
// so the main loop's switch stays readable.
static int getKey()
{
#if defined __WIN32__
  if(GetAsyncKeyState(VK_F5)      < 0) return 105;
  if(GetAsyncKeyState(0x53)       < 0) return 6;   // S (set START)
  if(GetAsyncKeyState(0x58)       < 0) return 7;   // X (set GOAL)
  if(GetAsyncKeyState(0x42)       < 0) return 1;   // B (block)
  if(GetAsyncKeyState(0x55)       < 0) return 12;  // U (unblock)
  if(GetAsyncKeyState(0x59)       < 0) return 20;  // Y (mark '9' unknown->blocked later)
  if(GetAsyncKeyState(0x5A)       < 0) return 21;  // Z (mark '8' unknown->free later)
  if(GetAsyncKeyState(0x47)       < 0) return 9;   // G (toggle g/rhs/h display)
  if(GetAsyncKeyState(VK_CONTROL) < 0) { CONTROL_KEY_FLAG=true; return 111; }
  if(GetAsyncKeyState(VK_SPACE)   < 0) { if(CONTROL_KEY_FLAG){ while(GetAsyncKeyState(VK_SPACE)){} return 100; } }
  if(GetAsyncKeyState(VK_RETURN)  < 0) {
      if(CONTROL_KEY_FLAG){ while(GetAsyncKeyState(VK_RETURN)){} return 200; } // Ctrl+Enter
      else { while(GetAsyncKeyState(VK_RETURN)){} return 201; }               // Enter (step)
  }
#endif
  return 0;
}

// -------------------- simulation (Windows) --------------------
// Beginner note: main event loop (mouse + keyboard). I keep it straightforward.
#if defined __WIN32__
void run_dstar_windows(const char* fileName, bool autorun_replan)
{
    // init world + draw initial map
    grid_world.initSystemOfCoordinates();
    grid_world.loadMapAndDisplay(fileName);
    grid_world.initialiseMapConnections();

    GRIDWORLD_ROWS = grid_world.getGridWorldRows();
    GRIDWORLD_COLS = grid_world.getGridWorldCols();

    // create planner from the currently loaded map
    recreate_dstar();

    // either auto-run or just compute first plan
    if (autorun_replan) {
        replan_to_goal();
    } else {
        initial_plan(); // I also draw the first path to make it obvious
    }

    // mouse & keyboard loop
    bool validCellSelected=false, blockedCellSelected=false;
    int rowSelected=-1, colSelected=-1;
    static bool page=false;

    while ((GetAsyncKeyState(VK_ESCAPE)) == 0) {
        // handle mouse select (highlight a cell and record its row/col)
        if (mousedown()) {
            int mX = mousecurrentx();
            int mY = mousecurrenty();
            if(mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX()
            && mY >= grid_world.getFieldY1() && mY <= grid_world.getGridMaxY()){
                validCellSelected = true;
                CellPosition p = grid_world.getCellPosition_markCell(mX, mY);
                rowSelected = p.row - 1;
                colSelected = p.col - 1;
                blockedCellSelected = (grid_world.getMapTypeValue(rowSelected, colSelected) == '1');
            } else {
                validCellSelected = false;
                rowSelected = colSelected = -1;
            }
        }

        // keys
        int action = getKey();
        switch(action){
            case 105: { // F5 toggle details
                SHOW_MAP_DETAILS = !SHOW_MAP_DETAILS;
                redraw();
            } break;

            case 9: {   // Beginner note: G toggles g/rhs/h all together.
                        // I also hide the "key legend" when numbers are on.
                bool newState = !(SHOW_G && SHOW_RHS && SHOW_H);
                SHOW_G   = newState;
                SHOW_RHS = newState;
                SHOW_H   = newState;
                SHOW_KEY = false;   
                redraw();           
            } break;

            case 6: { // S set start (selected cell -> '6')
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    vertex s = grid_world.getStartVertex();
                    if(s.row!=-1 && s.col!=-1) grid_world.setMapTypeValue(s.row, s.col, '0');
                    grid_world.setMapTypeValue(rowSelected, colSelected, '6');
                    s.row=rowSelected; s.col=colSelected;
                    grid_world.setStartVertex(s);
                    grid_world.initialiseMapConnections();
                    recreate_dstar();   // re-init planner from the new map
                    redraw();
                }
            } break;

            case 7: { // X set goal (selected cell -> '7')
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    vertex g = grid_world.getGoalVertex();
                    if(g.row!=-1 && g.col!=-1) grid_world.setMapTypeValue(g.row, g.col, '0');
                    grid_world.setMapTypeValue(rowSelected, colSelected, '7');
                    g.row=rowSelected; g.col=colSelected;
                    grid_world.setGoalVertex(g);
                    grid_world.initialiseMapConnections();
                    recreate_dstar();
                    redraw();
                }
            } break;

            case 1: { // B block -> '1'
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    vertex s = grid_world.getStartVertex();
                    vertex g = grid_world.getGoalVertex();
                    if(!((rowSelected==s.row && colSelected==s.col) ||
                         (rowSelected==g.row && colSelected==g.col)))
                    {
                        grid_world.setMapTypeValue(rowSelected, colSelected, '1');
                        grid_world.initialiseMapConnections();
                        recreate_dstar();
                        redraw();
                    }
                }
            } break;

            case 12: { // U un-block -> '0'
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    vertex s = grid_world.getStartVertex();
                    vertex g = grid_world.getGoalVertex();
                    if(!((rowSelected==s.row && colSelected==s.col) ||
                         (rowSelected==g.row && colSelected==g.col)))
                    {
                        grid_world.setMapTypeValue(rowSelected, colSelected, '0');
                        grid_world.initialiseMapConnections();
                        recreate_dstar();
                        redraw();
                    }
                }
            } break;

            case 20: { // Y -> '9' (unknown, later treated as blocked by D* sensor)
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    grid_world.setMapTypeValue(rowSelected, colSelected, '9');
                    grid_world.initialiseMapConnections();
                    recreate_dstar();
                    redraw();
                }
            } break;

            case 21: { // Z -> '8' (unknown, later treated as free by D* sensor)
                if(validCellSelected && rowSelected>=0 && colSelected>=0){
                    grid_world.setMapTypeValue(rowSelected, colSelected, '8');
                    grid_world.initialiseMapConnections();
                    recreate_dstar();
                    redraw();
                }
            } break;

            case 100: { // Ctrl+Space : initial plan
                CONTROL_KEY_FLAG = false;
                initial_plan();
            } break;

            case 201: { // Enter : single replan step
                replan_single_step();
            } break;

            case 200: { // Ctrl+Enter : auto replan to goal
                CONTROL_KEY_FLAG = false;
                replan_to_goal();
            } break;

            default: break;
        }

        delay(8); // Beginner note: tiny sleep so the loop doesn't hammer CPU.
    }
}
#endif

// -------------------- UNIX stub --------------------
// Beginner note: I only tested interaction on Windows.
// This just opens a window so at least something shows up.
#if defined __unix__ || defined __APPLE__
void init_sdlbgi (void)
{
  setwinoptions ("",
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOWPOS_CENTERED,
    SDL_WINDOW_SHOWN);
  initwindow (1280, 720);
  sdlbgifast ();
}
#endif

// -------------------- main --------------------
// Beginner note: minimal CLI:
//   .\search.exe <mapfile> [euc|cheb] [replan]
int main(int argc, char *argv[])
{
#if defined __unix__ || defined __APPLE__
  init_sdlbgi();
#elif defined __WIN32__
  int graphDriver = 0, graphMode = 0;
  initgraph(&graphDriver, &graphMode, "", 1360, 768);
#endif

  if (argc < 2) {
    std::cout << "Usage:\n"
              << "  .\\search.exe <mapfile> [euc|cheb] [replan]\n";
    return 0;
  }

  char gridFileName[256]; strcpy(gridFileName, argv[1]);

  // Beginner note: read heuristic from argv[2] (default Chebyshev).
  if (argc >= 3) {
    std::string h = argv[2];
    std::transform(h.begin(), h.end(), h.begin(), ::tolower);
    if (h=="euc" || h=="euclidean" || h=="e") D_HEURISTIC = EUCLIDEAN;
    else if (h=="cheb" || h=="chebyshev" || h=="c") D_HEURISTIC = CHEBYSHEV;
  }

  // Beginner note: if argv[3] == "replan" then auto-run to goal.
  bool autorun_replan = (argc >= 4 && std::string(argv[3])=="replan");

  BACKGROUND_COLOUR = WHITE;
  LINE_COLOUR = GREEN;
  GRIDWORLD_ROWS = GRIDWORLD_COLS = 0;
  SHOW_MAP_DETAILS = false;
  CONTROL_KEY_FLAG = false;

#if defined __WIN32__
  run_dstar_windows(gridFileName, autorun_replan);
#else
  std::cout << "D* interactive mode is Windows-first. This UNIX build shows a window but interaction may be limited.\n";
#endif

  if (dstar) { delete dstar; dstar = nullptr; }
  std::cout << "----<< The End.>>----\n";
  return 0;
}
