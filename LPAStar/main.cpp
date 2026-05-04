///////////////////////////////////////////////////////////////////////////////////////////
//
//          Program Name: Incremental Search
//           Description: start-up code for simulating LPA* and D*Lite
//                        - implements a gridworld that can be loaded from file, and
//                          modified through a user-interface
//
//        Run Parameters:
//
//    Keys for Operation:
//
//              History:  date of revision
//                         13/Aug/2025
//
//      Start-up code by:    n.h.reyes@massey.ac.nz
//
///////////////////////////////////////////////////////////////////////////////////////////

#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <deque>
#include <set>
#include <vector>
#include <chrono>
#include <memory>

#if defined(__unix__) || defined(__APPLE__)
  #include <graphics.h> // linux, mac os
#elif defined(_WIN32) || defined(__WIN32__)
  #include "graphics.h"
#endif

//-------------------------
#include "globalvariables.h"
#include "transform.h"
#include "astarsearch.h"
#include "lpastar.h"
#include "gridworld.h"

// color constants (used by the tiny graphics lib)
int BACKGROUND_COLOUR;
int LINE_COLOUR;

int robotWidth;
int GRIDWORLD_ROWS; // duplicated in GridWorld (ok for this project)
int GRIDWORLD_COLS; // duplicated in GridWorld (ok for this project)

//----------------------------
unsigned int HEURISTIC;
int numberOfExpandedStates;
int MAX_MOVES;
int maxQLength;
int qLengthAfterSearch;

///////////////////////////////////////////////////////////////////////////////
LpaStar* lpa_star;
GridWorld grid_world;

bool SHOW_MAP_DETAILS;
bool CONTROL_KEY_FLAG;
///////////////////////////////////////////////////////////////////////////////

using std::cout;
using std::endl;
using std::string;

// just a small list of map basenames (without ".map")
string list_of_grid_worlds[6] = {
  "grid_dstar_journal", "grid_lpa_journal", "grid_lpa_journal_big",
  "grid_spiral", "grid_trap", "grid_big"
};

// --- helpers ---------------------------------------------------------------
// clamp (row,col) into [0..R-1]/[0..C-1]. If (row,col) is out but (col,row) is
// valid, we swap once. This helps when someone mixes row/col by accident.
static inline void fix_rc_inplace(int &row, int &col, int R, int C){
    auto in = [&](int r, int c){ return (r>=0 && r<R && c>=0 && c<C); };
    if(!in(row,col) && in(col,row)) std::swap(row,col);
    if(row < 0) row = 0;
    if(row >= R) row = R-1;
    if(col < 0) col = 0;
    if(col >= C) col = C-1;
}

// draw one frame: clear -> draw map -> overlay path lines -> flip page
static void renderFrame(GridWorld& gw, LpaStar* lpa, bool& page){
    setactivepage(page);
    cleardevice();
    (SHOW_MAP_DETAILS ? gw.displayMapWithDetails() : gw.displayMap());
    if (lpa) lpa->drawStoredPolylines();   // draw initial (cyan) and final (yellow) polylines
    setvisualpage(page);
    page = !page;                          // double buffering
}

// small bounds check
static inline bool in_bounds(int r, int c, int rows, int cols){
    return (r >= 0 && r < rows && c >= 0 && c < cols);
}

// print current context to stderr (not to normal stdout)
// helps debugging without messing the assignment output
static void dump_context_stderr(const char* tag,
                                int rows, int cols,
                                int sRow, int sCol,
                                int gRow, int gCol){
    std::fprintf(stderr,
        "[%s] rows=%d cols=%d  S=(%d,%d)  G=(%d,%d)\n",
        tag, rows, cols, sRow, sCol, gRow, gCol);
}

// validate start/goal before replan. We only read from GridWorld getters,
// so we don't touch any private stuff in LpaStar.
static bool validate_start_goal_before_replan(GridWorld& gw, LpaStar* lpa, const char* where_tag){
    const auto sV = gw.getStartVertex();
    const auto gV = gw.getGoalVertex();

    const int rows = gw.getGridWorldRows();
    const int cols = gw.getGridWorldCols();
    dump_context_stderr(where_tag, rows, cols, sV.row, sV.col, gV.row, gV.col);

    if(!in_bounds(sV.row, sV.col, rows, cols)){
        std::fprintf(stderr, "[%s] ERROR: Start out of range!\n", where_tag);
        return false;
    }
    if(!in_bounds(gV.row, gV.col, rows, cols)){
        std::fprintf(stderr, "[%s] ERROR: Goal out of range!\n", where_tag);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// copy from LPA* "maze" into GridWorld "map" (for overlays)
// NOTE: we just mirror g/rhs/h/key and the type so the UI can display numbers.
void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa){
  for(int i=0; i < gWorld.getGridWorldRows(); i++){
    for(int j=0; j < gWorld.getGridWorldCols(); j++){
      gWorld.map[i][j].type = lpa->maze[i][j].type;
      gWorld.map[i][j].h    = lpa->maze[i][j].h;
      gWorld.map[i][j].g    = lpa->maze[i][j].g;
      gWorld.map[i][j].rhs  = lpa->maze[i][j].rhs;
      gWorld.map[i][j].row  = lpa->maze[i][j].y;
      gWorld.map[i][j].col  = lpa->maze[i][j].x;
      for(int k=0;k<2;k++) gWorld.map[i][j].key[k] = lpa->maze[i][j].key[k];
    }
  }
  gWorld.map[lpa->start->y][lpa->start->x].h   = lpa->start->h;
  gWorld.map[lpa->start->y][lpa->start->x].g   = lpa->start->g;
  gWorld.map[lpa->start->y][lpa->start->x].rhs = lpa->start->rhs;
  gWorld.map[lpa->start->y][lpa->start->x].row = lpa->start->y;
  gWorld.map[lpa->start->y][lpa->start->x].col = lpa->start->x;
  for(int k=0;k<2;k++) gWorld.map[lpa->start->y][lpa->start->x].key[k] = lpa->start->key[k];

  gWorld.map[lpa->goal->y][lpa->goal->x].h   = lpa->goal->h;
  gWorld.map[lpa->goal->y][lpa->goal->x].g   = lpa->goal->g;
  gWorld.map[lpa->goal->y][lpa->goal->x].rhs = lpa->goal->rhs;
  gWorld.map[lpa->goal->y][lpa->goal->x].row = lpa->goal->y;
  gWorld.map[lpa->goal->y][lpa->goal->x].col = lpa->goal->x;
  for(int k=0;k<2;k++) gWorld.map[lpa->goal->y][lpa->goal->x].key[k] = lpa->goal->key[k];
}

// ---------------------------------------------------------------------------
// copy from GridWorld "map" into LPA* "maze"
// we only sync the terrain type here, x/y were already set in makeEmptyMaze()
// we also rebind start/goal pointers (x=col, y=row)
void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa){
    const int R = gWorld.getGridWorldRows();
    const int C = gWorld.getGridWorldCols();

    for (int i = 0; i < R; ++i){
        for (int j = 0; j < C; ++j){
            lpa->maze[i][j].type = gWorld.map[i][j].type;
        }
    }

    vertex s = gWorld.getStartVertex();
    vertex g = gWorld.getGoalVertex();
    fix_rc_inplace(s.row, s.col, R, C);
    fix_rc_inplace(g.row, g.col, R, C);
    lpa->start = &lpa->maze[s.row][s.col];
    lpa->goal  = &lpa->maze[g.row][g.col];
}

///////////////////////////////////////////////////////////////////////////////
// tiny text helper
void drawInformationPanel(int x, int y, char* info){
  settextstyle(SMALL_FONT, HORIZ_DIR, 4);
  settextjustify(LEFT_TEXT,CENTER_TEXT);
  setcolor(YELLOW);
  if (info != NULL) outtextxy(x, y, info);
}

// poll keys (Windows branch only). We map keys to small integers to switch on.
int getKey(){
#if defined(_WIN32) || defined(__WIN32__)
  if(GetAsyncKeyState(VK_F4)  < 0){ SHOW_MAP_DETAILS=false; return 104; }
  if(GetAsyncKeyState(VK_F5)  < 0){ SHOW_MAP_DETAILS=true;  return 105; }
  if(GetAsyncKeyState(VK_F6)  < 0){ return 106; }
  if(GetAsyncKeyState(VK_F7)  < 0){ while(GetAsyncKeyState(VK_F7)!=0){} return 107; }
  if(GetAsyncKeyState(VK_F8)  < 0){ while(GetAsyncKeyState(VK_F8)!=0){} return 108; }
  if(GetAsyncKeyState(VK_F9)  < 0){ if(CONTROL_KEY_FLAG){ while(GetAsyncKeyState(VK_F9)!=0){} return 109; } }
  if(GetAsyncKeyState(VK_F10) < 0){ while(GetAsyncKeyState(VK_F10)!=0){} return 110; }
  if(GetAsyncKeyState(VK_F12) < 0){ if(CONTROL_KEY_FLAG){ while(GetAsyncKeyState(VK_F12)!=0){} return 19; } }
  if(GetAsyncKeyState(0x54)   < 0){ if(CONTROL_KEY_FLAG){ while(GetAsyncKeyState(0x54)!=0){} return 22; } } // T
  if(GetAsyncKeyState(0x53)   < 0){ while(GetAsyncKeyState(0x53)!=0){} return 6; }  // S
  if(GetAsyncKeyState(0x58)   < 0){ while(GetAsyncKeyState(0x58)!=0){} return 7; }  // X
  if(GetAsyncKeyState(0x42)   < 0){ while(GetAsyncKeyState(0x42)!=0){} return 1; }  // B
  if(GetAsyncKeyState(0x47)   < 0){ return 9;  } // G
  if(GetAsyncKeyState(0x48)   < 0){ return 10; } // H
  if(GetAsyncKeyState(0x4B)   < 0){ return 11; } // K
  if(GetAsyncKeyState(0x52)   < 0){ return 13; } // R

  if(GetAsyncKeyState(0x55)   < 0){ while(GetAsyncKeyState(0x55)!=0){} return 12; } // U
  if(GetAsyncKeyState(0x50)   < 0){ while(GetAsyncKeyState(0x50)!=0){} return 14; } // P
  if(GetAsyncKeyState(0x43)   < 0){ return 15; } // C
  if(GetAsyncKeyState(0x4D)   < 0){ return 16; } // M
  if(GetAsyncKeyState(0x59)   < 0){ while(GetAsyncKeyState(0x59)!=0){} return 20; } // Y
  if(GetAsyncKeyState(0x5A)   < 0){ while(GetAsyncKeyState(0x5A)!=0){} return 21; } // Z

  if(GetAsyncKeyState(VK_CONTROL) < 0){ CONTROL_KEY_FLAG=true; return 111; }
  if(GetAsyncKeyState(VK_SPACE) < 0){
    if(CONTROL_KEY_FLAG){ cout << "Initial planning..." << endl; while(GetAsyncKeyState(VK_SPACE)!=0){} return 100; }
  }
  if(GetAsyncKeyState(VK_RETURN) < 0){
    if(CONTROL_KEY_FLAG){ cout << "Re-planning..." << endl; while(GetAsyncKeyState(VK_RETURN)!=0){} return 200; }
  }
#endif
  return 0;
}

// batch runner for "main.exe all"
// NOTE: per your last request, we run C (Chebyshev) first, then E (Euclidean).
void runExperiments(){
  static bool page = false;

  const int num = (int)(sizeof(list_of_grid_worlds)/sizeof(list_of_grid_worlds[0]));
  const unsigned int Hs[2] = { CHEBYSHEV, EUCLIDEAN }; // run C then E

  for (unsigned int h : Hs) {
    for (int j = 0; j < num; ++j) {
      // (1) load and display the map
      grid_world.initSystemOfCoordinates();
      char path[256] = {0};
      std::snprintf(path, sizeof(path), "./grids/%s.map",
                    list_of_grid_worlds[j].c_str());
      grid_world.loadMapAndDisplay(path);
      grid_world.initialiseMapConnections();

      // (2) make an LPA* with current heuristic and attach GridWorld
      std::unique_ptr<LpaStar> lpa(new LpaStar(
          grid_world.getGridWorldRows(),
          grid_world.getGridWorldCols(),
          h,
          std::string(path)
      ));
      lpa->attachGridWorld(&grid_world);

      // (3) bind start/goal (x=col, y=row) with a small safety fix
      vertex s = grid_world.getStartVertex();
      vertex g = grid_world.getGoalVertex();
      int R = grid_world.getGridWorldRows();
      int C = grid_world.getGridWorldCols();
      fix_rc_inplace(s.row, s.col, R, C);
      fix_rc_inplace(g.row, g.col, R, C);
      lpa->initialise(/*startX*/ s.col, /*startY*/ s.row,
                      /*goalX */ g.col, /*goalY */ g.row);

      // copy types from display-map to the algorithm grid
      copyDisplayMapToMaze(grid_world, lpa.get());

      // (4) initial plan (cyan), draw it, print stats
      lpa->initialPlanning();
      renderFrame(grid_world, lpa.get(), page);
      lpa->printResults();

      // (5) final plan (yellow), draw it, print stats
      // If you want the incremental version instead, call finalPlanningIncremental().
      lpa->finalPlanning();
      renderFrame(grid_world, lpa.get(), page);
      lpa->printResults();
    }
  }
}

// Windows interactive simulation (keyboard/mouse)
void runSimulation_windows(char *fileName){
#if defined(_WIN32) || defined(__WIN32__)
  WorldBoundaryType worldBoundary;
  DevBoundaryType deviceBoundary;
  [[maybe_unused]] bool ANIMATE_MOUSE_FLAG = false;
  bool validCellSelected=false;
  bool blockedCellSelected=false;
  static bool page=false;
  [[maybe_unused]] int mX = 0;
  [[maybe_unused]] int mY = 0;

  [[maybe_unused]] CellPosition p;

  int rowSelected=-1, colSelected=-1;
  [[maybe_unused]] int mouseRadius = 1;

  grid_world.initSystemOfCoordinates();
  grid_world.loadMapAndDisplay(fileName);
  grid_world.initialiseMapConnections();

  // LPA* object
  unsigned int heuristic = HEURISTIC;
  lpa_star = new LpaStar(grid_world.getGridWorldRows(),
                         grid_world.getGridWorldCols(),
                         heuristic,
                         std::string(fileName));
  lpa_star->attachGridWorld(&grid_world); // for drawing polylines

  // read S/G from the map
  vertex start = grid_world.getStartVertex();
  vertex goal  = grid_world.getGoalVertex();

  // fix (row,col) if needed, then pass as (x=col, y=row)
  int R = grid_world.getGridWorldRows();
  int C = grid_world.getGridWorldCols();
  int sRow = start.row, sCol = start.col;
  int gRow = goal.row,  gCol = goal.col;
  fix_rc_inplace(sRow, sCol, R, C);
  fix_rc_inplace(gRow, gCol, R, C);

  cout << "(start.col = " << sCol << ", start.row = " << sRow << ")\n";
  cout << "(goal.col  = " << gCol << ", goal.row  = " << gRow << ")\n";

  lpa_star->initialise(sCol, sRow, gCol, gRow);

  // if still null, better bail out to avoid UI glitches
  if(!lpa_star->start || !lpa_star->goal){
      std::fprintf(stderr, "[fatal] start/goal still null after initialise().\n");
      return;
  }

  copyDisplayMapToMaze(grid_world, lpa_star);

  worldBoundary = grid_world.getWorldBoundary();
  deviceBoundary= grid_world.getDeviceBoundary();
  GRIDWORLD_ROWS = grid_world.getGridWorldRows();
  GRIDWORLD_COLS = grid_world.getGridWorldCols();

  int action=-1, prevAction=-1;
  renderFrame(grid_world, lpa_star, page);

  while((GetAsyncKeyState(VK_ESCAPE))==0){
    prevAction = action;
    action = getKey();

    if((action != prevAction) && action != -1){
      switch(action){
        case 1: { // block cell
          vertex s = grid_world.getStartVertex();
          vertex g = grid_world.getGoalVertex();
          if(rowSelected != -1 && colSelected != -1){
            if(!((rowSelected-1)==s.row && (colSelected-1)==s.col) &&
               !((rowSelected-1)==g.row && (colSelected-1)==g.col)){
              grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '1');
              grid_world.initialiseMapConnections();
              rowSelected=-1; colSelected=-1;
            }
          }
        } break;

        case 12: { // unblock cell
          vertex s = grid_world.getStartVertex();
          vertex g = grid_world.getGoalVertex();
          if(rowSelected != -1 && colSelected != -1){
            if(!((rowSelected-1)==s.row && (colSelected-1)==s.col) &&
               !((rowSelected-1)==g.row && (colSelected-1)==g.col)){
              grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '0');
              grid_world.initialiseMapConnections();
              rowSelected=-1; colSelected=-1;
            }
          }
        } break;

        case 20: { // set type '9'
          vertex s = grid_world.getStartVertex();
          vertex g = grid_world.getGoalVertex();
          if(rowSelected != -1 && colSelected != -1){
            if(!((rowSelected-1)==s.row && (colSelected-1)==s.col) &&
               !((rowSelected-1)==g.row && (colSelected-1)==g.col)){
              grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '9');
              grid_world.initialiseMapConnections();
              rowSelected=-1; colSelected=-1;
            }
          }
        } break;

        case 21: { // set type '8'
          vertex s = grid_world.getStartVertex();
          vertex g = grid_world.getGoalVertex();
          if(rowSelected != -1 && colSelected != -1){
            if(!((rowSelected-1)==s.row && (colSelected-1)==s.col) &&
               !((rowSelected-1)==g.row && (colSelected-1)==g.col)){
              grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '8');
              grid_world.initialiseMapConnections();
              rowSelected=-1; colSelected=-1;
            }
          }
        } break;

        case 105: { // F5: show details
          if(grid_world.isGridMapInitialised()){
            cout << "\nletter F5, pressed.\n";
            page = !page; setactivepage(page);
            (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
            if (lpa_star) lpa_star->drawStoredPolylines();
            setvisualpage(page);
            while(GetAsyncKeyState(VK_F5)!=0){}
          } else {
            cout << "map has not been initialised yet.\n";
          }
          SHOW_MAP_DETAILS=false; action=-1;
        } break;

        case 15: { // C: show local connections
          if(grid_world.isGridMapInitialised()){
            if(validCellSelected && rowSelected!=-1 && colSelected!=-1 && !blockedCellSelected){
              cout << "\nletter c, pressed.\n";
              page=!page; setactivepage(page);
              (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
              grid_world.displayVertexConnections(colSelected-1, rowSelected-1);
              if (lpa_star) lpa_star->drawStoredPolylines();
              setvisualpage(page);
              rowSelected=-1; colSelected=-1;
              while(GetAsyncKeyState(0x43)!=0){}
            }
          } else cout << "please select a valid cell first.\n";
          if(validCellSelected && !blockedCellSelected){ rowSelected=-1; colSelected=-1; action=-1; }
        } break;

        case 16: { // M: show all connections
          if(grid_world.isGridMapInitialised()){
            cout << "\nletter m, pressed.\n";
            page=!page; setactivepage(page);
            (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
            grid_world.displayMapConnections();
            if (lpa_star) lpa_star->drawStoredPolylines();
            setvisualpage(page);
            while(GetAsyncKeyState(0x4D)!=0){}
          } else cout << "map has not been initialised yet.\n";
        } break;

        case 6: { // set START to selected
          vertex s = grid_world.getStartVertex();
          if((s.row!=-1)&&(s.col!=-1)){
            grid_world.setMapTypeValue(s.row, s.col, '0');
            grid_world.initialiseMapConnections();
          } else { cout << "invalid START vertex\n"; break; }
          if(rowSelected!=-1 && colSelected!=-1){
            grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '6');
            s.row=rowSelected-1; s.col=colSelected-1; grid_world.setStartVertex(s);
            rowSelected=-1; colSelected=-1;
          } else { cout << "invalid new START vertex, please select first.\n"; }
        } break;

        case 7: { // set GOAL to selected
          vertex s = grid_world.getGoalVertex();
          if((s.row!=-1)&&(s.col!=-1)){
            grid_world.setMapTypeValue(s.row, s.col, '0');
          } else { cout << "invalid GOAL vertex\n"; action=-1; break; }
          if(rowSelected!=-1 && colSelected!=-1){
            grid_world.setMapTypeValue(rowSelected-1, colSelected-1, '7');
            s.row=rowSelected-1; s.col=colSelected-1; grid_world.setGoalVertex(s);
            grid_world.initialiseMapConnections();
            rowSelected=-1; colSelected=-1;
          } else { cout << "invalid new GOAL vertex, please select first.\n"; action=-1; }
        } break;

        case 22: { // Ctrl+T : copy display -> maze
          if(CONTROL_KEY_FLAG){ CONTROL_KEY_FLAG=false; copyDisplayMapToMaze(grid_world, lpa_star); cout << "copied display map to algorithm's maze\n"; }
        } break;

        case 100: { // Ctrl+Space : initial planning
          if(CONTROL_KEY_FLAG){
            CONTROL_KEY_FLAG=false;
            copyDisplayMapToMaze(grid_world, lpa_star);
            lpa_star->initialPlanning();       // compute (treat '9' free, avoid '8')
            lpa_star->updateAllKeyValues();    // for overlays
            copyMazeToDisplayMap(grid_world, lpa_star);
            lpa_star->printResults();
          }
        } break;

        case 200: { // Ctrl+Enter : re-planning (incremental final)
          if (CONTROL_KEY_FLAG) {
            CONTROL_KEY_FLAG = false;
            try {
              // (A) pre-check + copy
              if(!validate_start_goal_before_replan(grid_world, lpa_star, "replan:before")) break;

              copyDisplayMapToMaze(grid_world, lpa_star);

              int R = grid_world.getGridWorldRows();
              int C = grid_world.getGridWorldCols();
              auto s = grid_world.getStartVertex();
              auto g = grid_world.getGoalVertex();
              fix_rc_inplace(s.row, s.col, R, C);
              fix_rc_inplace(g.row, g.col, R, C);
              // if you want to write the fixed S/G back to UI:
              // grid_world.setStartVertex(s);
              // grid_world.setGoalVertex(g);

              if(!validate_start_goal_before_replan(grid_world, lpa_star, "replan:after-copy")) break;

              // (B) incremental reveal + replan
              lpa_star->finalPlanningIncremental();

              // (C) refresh overlays
              lpa_star->updateAllKeyValues();
              copyMazeToDisplayMap(grid_world, lpa_star);
              lpa_star->printResults();

            } catch (const std::exception& e) {
              std::fprintf(stderr, "[replan] exception: %s\n", e.what());
            } catch (...) {
              std::fprintf(stderr, "[replan] unknown exception\n");
            }
          }
        } break;

        case 110: { action=-1; } break;

        case 9: { // show g
          cout << "\nletter g, pressed.\n";
          page=!page; setactivepage(page);
          (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
          grid_world.displayMapWithSelectedDetails(true,false,false,false);
          if (lpa_star) lpa_star->drawStoredPolylines();
          setvisualpage(page);
          while(GetAsyncKeyState(0x47)!=0){}
          action=-1;
        } break;

        case 10: { // show h
          cout << "\nletter h, pressed.\n";
          page=!page; setactivepage(page);
          (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
          grid_world.displayMapWithSelectedDetails(false,false,true,false);
          if (lpa_star) lpa_star->drawStoredPolylines();
          setvisualpage(page);
          while(GetAsyncKeyState(0x48)!=0){}
          action=-1;
        } break;

        case 11: { // show key
          cout << "\nletter k, pressed.\n";
          page=!page; setactivepage(page);
          (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
          grid_world.displayMapWithSelectedDetails(false,false,false,true);
          if (lpa_star) lpa_star->drawStoredPolylines();
          setvisualpage(page);
          while(GetAsyncKeyState(0x4B)!=0){}
          action=-1;
        } break;

        case 13: { // show rhs (letter 'r')
          cout << "\nletter r, pressed.\n";
          page=!page; setactivepage(page);
          (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
          grid_world.displayMapWithSelectedDetails(false, true, false, false);
          if (lpa_star) lpa_star->drawStoredPolylines();
          setvisualpage(page);
          while(GetAsyncKeyState(0x52)!=0){}
          action=-1;
        } break;

        case 14: { // show positions
          cout << "\nletter p, pressed.\n";
          page=!page; setactivepage(page);
          (SHOW_MAP_DETAILS ? grid_world.displayMapWithDetails() : grid_world.displayMap());
          grid_world.displayMapWithPositionDetails();
          if (lpa_star) lpa_star->drawStoredPolylines();
          setvisualpage(page);
          while(GetAsyncKeyState(0x50)!=0){}
          action=-1;
        } break;

        case 19: { // save map
          if(CONTROL_KEY_FLAG){
            CONTROL_KEY_FLAG=false;
            string targetFileName;
            cout << "Save map to file.\nEnter destination filename: ";
            std::cin >> targetFileName;
            grid_world.saveNewMap(targetFileName.c_str());
          }
          action=-1;
        } break;
      }

      renderFrame(grid_world, lpa_star, page);

      // tiny heartbeat in console, just to see loop ticks
      static int iter=0;
      cout << "\niter = " << iter++ << endl;
    }

    // --- mouse picking (draw a small ring on the cell) ---------------------
    bool ANIMATE_MOUSE_FLAG=false;
    int mX=0,mY=0;
    if(!SHOW_MAP_DETAILS){
      if(mousedown()){
        ANIMATE_MOUSE_FLAG=true;
        mX = mousecurrentx();
        mY = mousecurrenty();
        if(mX >= grid_world.getFieldX1() && mX <= grid_world.getGridMaxX()
        && mY >= grid_world.getFieldY1() && mY <= grid_world.getGridMaxY()){
          blockedCellSelected=false;
          circle(mX, mY, 3);
          validCellSelected = true;
          CellPosition p = grid_world.getCellPosition_markCell(mX, mY);
          rowSelected = p.row;
          colSelected = p.col;
          char tp = grid_world.getMapTypeValue(rowSelected-1, colSelected-1);
          cout << "\n(row = " << rowSelected << ", col = " << colSelected << endl;
          if(tp == '1'){ blockedCellSelected=true; cout << "\nblocked cell selected.\n"; }
        } else validCellSelected = false;
      }
    }

    int mouseRadius=1;
    while(ANIMATE_MOUSE_FLAG){
      if(mouseRadius < 40) mouseRadius += 1;
      if(mouseRadius >= 40){ ANIMATE_MOUSE_FLAG=false; mouseRadius=0; }
      setactivepage(page); cleardevice();
      grid_world.displayMap();
      if (lpa_star) lpa_star->drawStoredPolylines();
      if(ANIMATE_MOUSE_FLAG){
        setcolor(RED); circle(mX, mY, 20); line(mX,mY-20,mX,mY+20); line(mX-20,mY,mX+20,mY);
        setcolor(YELLOW); circle(mX, mY, mouseRadius);
      }
      char info[256]; info[0]='\0';
      if(validCellSelected){
        CellPosition p=grid_world.getCellPosition_markCell(mX, mY);
        rowSelected = p.row; colSelected = p.col;
        snprintf(info,sizeof(info),"row: %d, col: %d",rowSelected, colSelected);
        drawInformationPanel(grid_world.getFieldX2(),grid_world.getFieldY1() + textheight("H")*6, info);
      }
      setvisualpage(page); page = !page;
    }
  } // while ESC not pressed
#endif
}

void runSimulation_unix(char *fileName){
#if defined(__unix__) || defined(__APPLE__)
  // You can fill this if you want the interactive mode on Unix.
#endif
}

//---
void init_sdlbgi (void){
#if defined(__unix__) || defined(__APPLE__)
  setwinoptions ("", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SDL_WINDOW_SHOWN);
  initwindow (800, 600);
  sdlbgifast ();
#endif
}

//---
int main(int argc, char *argv[]) {
#if defined(__unix__) || defined(__APPLE__)
  init_sdlbgi();
#elif defined(_WIN32) || defined(__WIN32__)
  int graphDriver = 0, graphMode = 0;
  initgraph(&graphDriver, &graphMode, "", 1360, 768); // Start Window - LAPTOP SCREEN
#endif

  char gridFileName[80];

  if (argc == 3){
    string heuristic(argv[2]);
    std::transform(heuristic.begin(), heuristic.end(), heuristic.begin(), ::tolower);

    // safe copy with explicit NUL-termination
    memset(gridFileName, 0, sizeof(gridFileName));
    snprintf(gridFileName, sizeof(gridFileName), "%s", argv[1]);

    if((heuristic=="euclidean") || (heuristic=="e")){
      HEURISTIC = EUCLIDEAN; cout << "Heuristic function = EUCLIDEAN\n";
    } else if((heuristic=="chebyshev") || (heuristic=="c")){
      HEURISTIC = CHEBYSHEV; cout << "Heuristic function = CHEBYSHEV\n";
    } else {
      cout << "Invalid parameters:  gridworld heuristic\n";
      cout << "Example: ./main .\\grids\\grid_Dstar_journal.map m\n";
    }

    BACKGROUND_COLOUR = WHITE;
    LINE_COLOUR = GREEN;
    GRIDWORLD_ROWS = 0;
    GRIDWORLD_COLS = 0;
    SHOW_MAP_DETAILS=false;
    CONTROL_KEY_FLAG=false;

    try{
#if defined(__unix__) || defined(__APPLE__)
      // runSimulation_unix(gridFileName);
      cout << "Use the unix branch here if needed.\n";
#elif defined(_WIN32) || defined(__WIN32__)
      runSimulation_windows(gridFileName);
#endif
    } catch(const std::exception& e){
      std::cerr << "[FATAL] " << e.what() << "\n";
      throw;  // or std::abort();
    } catch(...){
      std::cerr << "[FATAL] unknown exception\n";
      throw;  // or std::abort();
    }

  } else if(argc == 2){
    string run_mode(argv[1]);
    std::transform(run_mode.begin(), run_mode.end(), run_mode.begin(), ::tolower);
    if((run_mode=="all") || (run_mode=="a")){
      using std::chrono::system_clock;
      system_clock::time_point start = std::chrono::system_clock::now();
      runExperiments();
      auto end = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsed_seconds = end-start;
      string timeStr = std::to_string(elapsed_seconds.count());
      timeStr += " sec.";
      cout << "\nTotal time = " << timeStr << endl;
    } else {
      cout << "Invalid parameter.\nExample: ./main all\n";
    }
  } else {
    cout << "\nmissing parameters!\n";
  }

  cout << "----<< The End.>>----" << endl;
  return 0;
}
