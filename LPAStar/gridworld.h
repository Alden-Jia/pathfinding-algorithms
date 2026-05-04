#ifndef __GRIDWORLD_H__
#define __GRIDWORLD_H__

#if defined(__unix__) || defined(__APPLE__)
  #include <graphics.h>   // Linux / macOS (SDL_bgi)
#elif defined(_WIN32) || defined(__WIN32__)
  #include "graphics.h"   // Windows (WinBGI)
#endif

#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>        // memset

#include "transform.h"
#include "globalvariables.h"
#include "lpastar.h"

// This class owns the grid map used for visualization.
// It knows how to:
//  - load a map from file
//  - draw the map and little overlays (g, rhs, h, keys, etc.)
//  - wire up neighbors (8-connected) with costs
//  - show paths produced by LPA* / D* Lite (just drawing)
class GridWorld {

public:
    GridWorld(){
      // keep the constructor super simple
      xInc = 0;
      yInc = 0;
      cellWidth = 0;
      cellHeight = 0;
      GRIDWORLD_ROWS = 0;
      GRIDWORLD_COLS = 0;
      fieldX1 = fieldY1 = fieldX2 = fieldY2 = 0;
      WORLD_MAXX = WORLD_MAXY = 0.0f;
      MAP_INITIALISED = false;
      std::memset(fileName, 0, sizeof(fileName));
      // start/goal will be set when loading the map
      startVertex = {};
      goalVertex  = {};
    }

    // simple helper: have we loaded a map already?
    bool isGridMapInitialised(){
        return MAP_INITIALISED;
    }

    // coordinate system (where the grid sits on the window)
    void initSystemOfCoordinates();

    // connect every cell to its 8 neighbors (and set edge costs)
    void initialiseMapConnections();

    // read a map file and paint it
    void loadMapAndDisplay(const char* fn);

    // header text (instructions) at the top of the window
    void displayHeader();

    // draw the map (no numbers)
    void displayMap();

    // draw the map and print g/rhs/h for each cell
    void displayMapWithDetails();

    // draw the map and print "row/col" for each cell
    void displayMapWithPositionDetails();

    // draw the map and print only keys (plus g/rhs)
    void displayMapWithKeyDetails();

    // choose exactly what to show per cell
    void displayMapWithSelectedDetails(bool display_g, bool display_rhs, bool display_h, bool display_key);

    // debug: show edges from one cell
    void displayVertexConnections(int i, int j);

    // debug: show edges for the whole map
    void displayMapConnections();

    // draw paths (these functions only draw; they do not compute the path)
    void displayPath_for_dStarLite();
    void displayPath_for_lpaStar();

    // draw faint grid lines in the background
    void drawGrid();

    // paint a single cell, returns its center (in pixels)
    Coordinates markCell_col_row(int col, int row, int fillColour, int outlineColour);

    // draw a small blue rectangle at the cell that contains (x,y)
    void markCell(int x, int y);

    // same as above but also return the (row, col) of that cell
    CellPosition getCellPosition_markCell(int x, int y);

    // pixel bounds of the drawn grid
    int getGridMaxX();
    int getGridMaxY();

    // world/device rectangles (just getters)
    WorldBoundaryType getWorldBoundary(){ return worldBoundary; }
    DevBoundaryType   getDeviceBoundary(){ return deviceBoundary; }

    // cell size in pixels
    int getCellWidth(){  return cellWidth; }
    int getCellHeight(){ return cellHeight; }

    // rectangle where the grid is drawn (pixel coordinates)
    int getFieldX1(){ return fieldX1; }
    int getFieldY1(){ return fieldY1; }
    int getFieldX2(){ return fieldX2; }
    int getFieldY2(){ return fieldY2; }

    // grid size (rows/cols)
    int getGridWorldRows(){ return GRIDWORLD_ROWS; }
    int getGridWorldCols(){ return GRIDWORLD_COLS; }

    // basic setters: change values inside the map
    void setMapTypeValue (int row, int col, char   tp){ map[row][col].type = tp; }
    void setMapGValue    (int row, int col, double g ){ map[row][col].g    = g;  }
    void setMapRhsValue  (int row, int col, double rhs){ map[row][col].rhs = rhs; }
    void setMapHValue    (int row, int col, double h ){ map[row][col].h    = h;  }
    void setMapFValue    (int row, int col, double f ){ map[row][col].f    = f;  }
    void setMapStatusValue(int row, int col, char s   ){ map[row][col].status = s; }

    // set start/goal (simple safety checks)
    void setStartVertex(vertex s){
        if(!MAP_INITIALISED){
            std::cout << "Error in getStartVertex()" << std::endl;
            std::exit(1);
        }
        if((s.row != -1) && (s.col != -1)) startVertex = s;
        else{
            std::cout << "Error in GridWorld::setStartVertex()" << std::endl;
            std::exit(1);
        }
    }

    void setGoalVertex(vertex s){
        if(!MAP_INITIALISED){
            std::cout << "Error in getGoalVertex()" << std::endl;
            std::exit(1);
        }
        if((s.row != -1) && (s.col != -1)) goalVertex = s;
        else{
            std::cout << "Error in GridWorld::setGoalVertex()" << std::endl;
            std::exit(1);
        }
    }

    // basic getters for cell values
    char   getMapTypeValue(int row, int col){ return map[row][col].type; }
    double getMapRhsValue (int row, int col){ return map[row][col].rhs;  }
    double getMapGValue   (int row, int col){ return map[row][col].g;    }
    double getMapFValue   (int row, int col){ return map[row][col].f;    }
    double getMapHValue   (int row, int col){ return map[row][col].h;    }

    // get start/goal (require a map to be loaded first)
    vertex getStartVertex(){
        if(!MAP_INITIALISED){
            std::cout << "Error in getStartVertex()" << std::endl;
            std::exit(1);
        }
        return startVertex;
    }

    vertex getGoalVertex(){
        if(!MAP_INITIALISED){
            std::cout << "Error in getStartVertex()" << std::endl;
            std::exit(1);
        }
        return goalVertex;
    }

    // save current map to a file (text, very simple)
    void saveNewMap(const char* fn);

    // draw a cell and print numbers inside it
    void markCell_col_row_details(int col, int row, int fillColour, int outlineColour);

    // same, but you can choose which numbers to show
    void markCell_col_row_details(int col, int row, int fillColour, int outlineColour,
                                  bool display_g, bool display_rhs, bool display_h, bool display_key);

    // draw a cell and print "col=?, row=?"
    void markCell_col_row_details_xy(int col, int row, int fillColour, int outlineColour);

    // allow helper functions in main/lpastar to read/write the map quickly
    friend void copyMazeToDisplayMap(GridWorld &gWorld, LpaStar* lpa);
    friend void copyDisplayMapToMaze(GridWorld &gWorld, LpaStar* lpa);

private:
    // cell size in pixels
    int cellWidth;
    int cellHeight;

    // grid size (cells)
    int GRIDWORLD_ROWS;
    int GRIDWORLD_COLS;

    // world and device rectangles (for drawing)
    WorldBoundaryType worldBoundary;
    DevBoundaryType   deviceBoundary;

    // world extent (not that important here)
    float WORLD_MAXX;
    float WORLD_MAXY;

    // pixel rectangle where the grid is drawn
    int fieldX1, fieldY1, fieldX2, fieldY2;

    // the grid itself
    std::vector<std::vector<vertex>> map;

    // special cells
    vertex startVertex;
    vertex goalVertex;

    // state flags and drawing increments
    bool MAP_INITIALISED;
    int xInc;
    int yInc;

    // where we loaded the map from (longer buffer avoids overflow)
    char fileName[260];
};

#endif
