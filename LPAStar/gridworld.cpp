#include "gridworld.h"
#include <cstdio>    // snprintf
#include <cstdlib>   // exit
#include <cstring>   // memset

// define COLOR if not provided (WinBGI uses 0x00RRGGBB)
#ifndef COLOR
#define COLOR(r,g,b) ( ((int)((((unsigned char)(r))<<16) | (((unsigned char)(g))<<8) | ((unsigned char)(b)) )) )
#endif

// add two numbers but keep INF "sticky" (INF + x = INF)
static inline double sum(double a, double b){
    if(a == INF || b == INF) return INF;
    return (a + b);
}

// draw LPA* path by following best g+cost from the GOAL backwards
// note: this only draws; it does not compute the path
void GridWorld::displayPath_for_lpaStar(){
    vertex* current = &map[goalVertex.row][goalVertex.col];
    if(!current) return;

    int guard = GRIDWORLD_ROWS * GRIDWORLD_COLS + 5; // avoid infinite loops
    while(current && guard-- > 0){
        vertex* origin = current;
        vertex* best = nullptr;
        double bestVal = INF;

        // check the 8 neighbors
        for(int m = 0; m < DIRECTIONS; ++m){
            vertex* nb = origin->move[m];
            if(nb && nb->type != '1'){          // ignore walls
                double linkCost = origin->linkCost[m];
                double g = nb->g;               // neighbor's g
                double val = sum(g, linkCost);  // g(nei) + cost(nei->cur)
                if(val < bestVal){
                    bestVal = val;
                    best = nb;
                }
            }
        }

        // if nothing better, we stop (path might be missing)
        if(!best || bestVal == INF) break;

        // draw a small segment from current to best
        setcolor(RED);
        setlinestyle(SOLID_LINE, 1, 1);
        line(best->centre.x, best->centre.y, origin->centre.x, origin->centre.y);

        current = best;
        if(current == &map[startVertex.row][startVertex.col]) break; // reached start
    }
}

// draw D* Lite path by following best g+cost from the START forward
void GridWorld::displayPath_for_dStarLite(){
    vertex* currentVertex = &map[startVertex.row][startVertex.col];
    vertex* originVertex  = currentVertex;
    vertex* min_neighbour = nullptr;

    double min_g_plus_c = INF;
    double linkCost, g;

    while(1){
        min_g_plus_c = INF;

        // choose neighbor with smallest g + cost
        for(int m = 0; m < DIRECTIONS; m++){
            vertex* neighbour = originVertex->move[m];
            if(neighbour != nullptr && neighbour->type != '1'){
                linkCost = originVertex->linkCost[m];
                g = (originVertex->move[m])->g;
                if(min_g_plus_c > sum(g, linkCost)){
                    min_g_plus_c = sum(g, linkCost);
                    min_neighbour = neighbour;
                }
            }
        }

        // draw the chosen step
        if(min_neighbour != nullptr){
            setcolor(RED);
            setlinestyle(SOLID_LINE, 1, 1);
            line(min_neighbour->centre.x, min_neighbour->centre.y,
                 currentVertex->centre.x, currentVertex->centre.y);
        }

        currentVertex = min_neighbour;
        originVertex  = currentVertex;
        if(currentVertex == &map[goalVertex.row][goalVertex.col]) break; // reached goal
    }
}

// print a header text at the top (UI only)
void GridWorld::displayHeader(){
    int x = getmaxx() / 2;
    int y = textheight("H") * 1;

    settextstyle(SMALL_FONT, HORIZ_DIR, 5);
    settextjustify(CENTER_TEXT, TOP_TEXT);
    setcolor(YELLOW);
    setbkcolor(BLACK);
    outtextxy(x, y, "INCREMENTAL OPTIMAL SEARCH (8-CONNECTED GRIDWORLD)");

    y = y + textheight("_");
    settextjustify(LEFT_TEXT, TOP_TEXT);
    outtextxy(fieldX2 - textwidth("start-up codes by n.h.reyes@massey.ac.nz"),
              fieldY2, "start-up codes by n.h.reyes@massey.ac.nz");

    settextjustify(CENTER_TEXT, TOP_TEXT);
    setcolor(GREEN);
    outtextxy(x, y, "F4: hide details, F5: show details, F9: copy display map to Algorithm data structure(maze), F10: run Search");

    y = y + textheight("_");
    setcolor(WHITE);
    outtextxy(x, y, "B: block cell, U: unblock cell, H: h-values, K: key-values, S: new START, X: new GOAL, P: cell positions, C: local connections, M: all connections ");

    y = y + textheight("_");
    setcolor(WHITE);

    char info[256] = {0};
    char heuristicStr[80] = {0};

    if (HEURISTIC == EUCLIDEAN) {
        std::snprintf(heuristicStr, sizeof(heuristicStr), "%s", "EUCLIDEAN");
    } else if (HEURISTIC == CHEBYSHEV) {
        std::snprintf(heuristicStr, sizeof(heuristicStr), "%s", "CHEBYSHEV");
    }

    std::snprintf(info, sizeof(info), "fileName = %s, heuristic = %s", fileName, heuristicStr);
    outtextxy(x, y, info);
}

// load a map file and paint it
// file format: first two numbers = rows, cols; then rows*cols of char types
void GridWorld::loadMapAndDisplay(const char* fn){
    std::ifstream i_file;
    i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

    MAP_INITIALISED = false;
    std::memset(fileName, 0, sizeof(fileName));
    std::snprintf(fileName, sizeof(fileName), "%s", (fn ? fn : ""));

    try{
        i_file.open(fn, std::ifstream::in);
        if(DEBUG) std::cout << "file opened." << std::endl;

        displayHeader();

        // read grid size
        i_file >> GRIDWORLD_ROWS;
        i_file >> GRIDWORLD_COLS;

        // compute cell size on screen
        xInc = std::abs(fieldX2 - fieldX1) / GRIDWORLD_COLS;
        cellWidth  = xInc;
        yInc = std::abs(fieldY2 - fieldY1) / GRIDWORLD_ROWS;
        cellHeight = yInc;

        // allocate and read cells
        map.resize(GRIDWORLD_ROWS);
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            map[j].resize(GRIDWORLD_COLS);
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                map[j][i].g = map[j][i].rhs = map[j][i].f = 0.0;
                map[j][i].row = j; map[j][i].col = i;
                map[j][i].key[0] = map[j][i].key[1] = 0.0;
                for(int m = 0; m < DIRECTIONS; m++){
                    map[j][i].move[m] = NULL;     // set neighbor pointers later
                    map[j][i].linkCost[m] = 1.0;  // default cost
                }

                // read cell type: '0','1','9','8','6','7','4'
                i_file >> map[j][i].type;

                // paint the cell (and remember start/goal)
                if(map[j][i].type == '0') map[j][i].centre = markCell_col_row(i, j, LIGHTGRAY, WHITE);
                if(map[j][i].type == '1') map[j][i].centre = markCell_col_row(i, j, BLACK,     WHITE);
                if(map[j][i].type == '9') map[j][i].centre = markCell_col_row(i, j, LIGHTRED,  WHITE);
                if(map[j][i].type == '8') map[j][i].centre = markCell_col_row(i, j, LIGHTBLUE, WHITE);
                if(map[j][i].type == '6'){ startVertex.row=j; startVertex.col=i; map[j][i].centre = markCell_col_row(i, j, GREEN, WHITE); }
                if(map[j][i].type == '7'){ goalVertex.row =j; goalVertex.col =i; map[j][i].centre = markCell_col_row(i, j, BLUE,  WHITE); }
                if(map[j][i].type == '4') map[j][i].centre = markCell_col_row(i, j, CYAN,      WHITE);
            }
        }

        i_file.close();
        if(DEBUG) std::cout << "file closed." << std::endl;
        MAP_INITIALISED = true;
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception opening/reading/closing file\n";
    }
    if(DEBUG){
        std::cout << "\nGridworld loaded from file: " << (fn ? fn : "") << std::endl;
        std::cout << "GRIDWORLD_ROWS = " << GRIDWORLD_ROWS << std::endl;
        std::cout << "GRIDWORLD_COLS = " << GRIDWORLD_COLS << std::endl;
    }
}

// set up neighbor pointers and edge costs for every cell
// rule used here:
//  - '1' means wall → edges touching it are INF
//  - diagonal is allowed; it is blocked only when BOTH side cells are walls
//    (a soft "no corner cut" rule)
// if you want to allow all diagonals, remove the BOTH-sides-wall check
void GridWorld::initialiseMapConnections(){
    vertex* originVertex;
    vertex* neighbour;
    int neighbourY = -1, neighbourX = -1;

    try{
        displayHeader();

        // diagonal step cost (sqrt(2) ≈ 1.4142)
        static const double DIAG_COST = 1.4142135623730951;

        for(int r = 0; r < GRIDWORLD_ROWS; ++r){
            for(int c = 0; c < GRIDWORLD_COLS; ++c){
                originVertex = &map[r][c];

                for(int m = 0; m < DIRECTIONS; ++m){
                    // offsets for this neighbor
                    const int dy = neighbours[m].y;
                    const int dx = neighbours[m].x;

                    neighbourY = r + dy;
                    neighbourX = c + dx;

                    // skip if outside the grid
                    if(neighbourX < 0 || neighbourX >= GRIDWORLD_COLS ||
                       neighbourY < 0 || neighbourY >= GRIDWORLD_ROWS){
                        continue;
                    }

                    neighbour = &map[neighbourY][neighbourX];

                    // check walls
                    const bool origin_wall = (originVertex->type == '1');
                    const bool neigh_wall  = (neighbour->type     == '1');

                    bool blocked = origin_wall || neigh_wall;

                    // optional diagonal restriction: block only if BOTH side cells are walls
                    const bool is_diag = (dx != 0 && dy != 0);
                    if(is_diag){
                        const bool side1_wall = (map[r][c + dx].type == '1'); // same row
                        const bool side2_wall = (map[r + dy][c].type == '1'); // same col
                        if(side1_wall && side2_wall){
                            blocked = true;
                        }
                    }

                    // write pointer and cost
                    originVertex->move[m] = neighbour;
                    if(blocked){
                        originVertex->linkCost[m] = INF;                // blocked edge
                    }else{
                        originVertex->linkCost[m] = is_diag ? DIAG_COST // diagonal edge
                                                            : 1.0;      // straight edge
                    }
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception displaying Map\n";
    }
}

// show edges for one selected cell (debug helper)
void GridWorld::displayVertexConnections(int i, int j){
    try{
        std::cout << "vertex(x = " << i << ", y= " << j << std::endl;

        if(map[j][i].type != '1'){
            vertex* originVertex = &map[j][i];
            for(int m = 0; m < DIRECTIONS; m++){
                vertex* neighbour = map[j][i].move[m];
                if(neighbour != NULL && neighbour->type != '1'){
                    std::cout << "cost[" << m << "] = " << originVertex->linkCost[m] << std::endl;
                    setcolor(YELLOW);
                    setlinestyle (0, 0, THICK_WIDTH);
                    line(neighbour->centre.x, neighbour->centre.y,
                         originVertex->centre.x, originVertex->centre.y);
                } else if(neighbour != NULL && neighbour->type == '1'){
                    setcolor(RED);
                    setlinestyle (0, 0, THICK_WIDTH);
                    line(neighbour->centre.x, neighbour->centre.y,
                         originVertex->centre.x, originVertex->centre.y);
                    std::cout << "cost[" << m << "] = " << originVertex->linkCost[m] << std::endl;
                }
            }
            setlinestyle (0, 0, NORM_WIDTH);
        }
        setlinestyle (0, 0, NORM_WIDTH);
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// show all edges (debug helper)
void GridWorld::displayMapConnections(){
    try{
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type != '1'){
                    vertex* originVertex = &map[j][i];
                    for(int m = 0; m < DIRECTIONS; m++){
                        vertex* neighbour = map[j][i].move[m];
                        if(neighbour != NULL && neighbour->type != '1'){
                            setcolor(YELLOW);
                            setlinestyle (0, 0, THICK_WIDTH);
                            line(neighbour->centre.x, neighbour->centre.y,
                                 originVertex->centre.x, originVertex->centre.y);
                        } else if(neighbour != NULL && neighbour->type == '1'){
                            setcolor(RED);
                            setlinestyle (0, 0, THICK_WIDTH);
                            line(neighbour->centre.x, neighbour->centre.y,
                                 originVertex->centre.x, originVertex->centre.y);
                        }
                    }
                    setlinestyle (0, 0, NORM_WIDTH);
                }
            }
        }
        setlinestyle (0, 0, NORM_WIDTH);
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// draw the map (no numbers). colors only.
void GridWorld::displayMap(){
    try{
        displayHeader();
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type == '0') markCell_col_row(i, j, LIGHTGRAY,  WHITE);
                if(map[j][i].type == '1') markCell_col_row(i, j, BLACK,      WHITE);
                if(map[j][i].type == '9') markCell_col_row(i, j, LIGHTRED,   WHITE);
                if(map[j][i].type == '8') markCell_col_row(i, j, LIGHTBLUE,  WHITE);
                if(map[j][i].type == '6'){
                    startVertex.row = j; startVertex.col = i;
                    markCell_col_row(i, j, GREEN, WHITE);
                }
                if(map[j][i].type == '7'){
                    goalVertex.row = j; goalVertex.col = i;
                    markCell_col_row(i, j, BLUE, WHITE);
                }
                if(map[j][i].type == '4'){
                    map[j][i].centre = markCell_col_row(i, j, CYAN, WHITE);
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// draw the map and print "col=?, row=?" inside each cell
void GridWorld::displayMapWithPositionDetails(){
    try{
        displayHeader();
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type == '0') markCell_col_row_details_xy(i, j, LIGHTGRAY,   WHITE);
                if(map[j][i].type == '1') markCell_col_row_details_xy(i, j, BLACK,       WHITE);
                if(map[j][i].type == '9') markCell_col_row_details_xy(i, j, LIGHTRED,    WHITE);
                if(map[j][i].type == '8') markCell_col_row_details_xy(i, j, LIGHTBLUE,   WHITE);
                if(map[j][i].type == '6'){
                    startVertex.row = j; startVertex.col = i;
                    markCell_col_row_details_xy(i, j, GREEN, WHITE);
                }
                if(map[j][i].type == '7'){
                    goalVertex.row = j; goalVertex.col = i;
                    markCell_col_row_details_xy(i, j, BLUE, WHITE);
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// draw the map and print g/rhs/h (and optionally key) inside each cell
void GridWorld::displayMapWithDetails(){
    try{
        displayHeader();
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type == '0') markCell_col_row_details(i, j, LIGHTGRAY,   WHITE);
                if(map[j][i].type == '1') markCell_col_row_details(i, j, BLACK,       WHITE);
                if(map[j][i].type == '9') markCell_col_row_details(i, j, LIGHTRED,    WHITE);
                if(map[j][i].type == '8') markCell_col_row_details(i, j, LIGHTBLUE,   WHITE);
                if(map[j][i].type == '6'){
                    startVertex.row = j; startVertex.col = i;
                    markCell_col_row_details(i, j, GREEN, WHITE);
                }
                if(map[j][i].type == '7'){
                    goalVertex.row = j; goalVertex.col = i;
                    markCell_col_row_details(i, j, BLUE, WHITE);
                }
                if(map[j][i].type == '4'){
                    markCell_col_row_details(i, j, CYAN, WHITE);
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// draw the map and print only selected fields per cell
void GridWorld::displayMapWithKeyDetails(){
    try{
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type == '0')
                    markCell_col_row_details(i, j, LIGHTGRAY, WHITE, true, true, false, true);
                if(map[j][i].type == '1')
                    markCell_col_row_details(i, j, BLACK,     WHITE, true, true, false, true);
                if(map[j][i].type == '9')
                    markCell_col_row_details(i, j, LIGHTRED,  WHITE, true, true, false, true);
                if(map[j][i].type == '8')
                    markCell_col_row_details(i, j, LIGHTBLUE, WHITE, true, true, false, true);
                if(map[j][i].type == '6'){
                    startVertex.row = j; startVertex.col = i;
                    markCell_col_row_details(i, j, GREEN, WHITE, true, true, false, true);
                }
                if(map[j][i].type == '7'){
                    goalVertex.row = j; goalVertex.col = i;
                    markCell_col_row_details(i, j, BLUE,  WHITE, true, true, false, true);
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// same as displayMapWithDetails but you choose which numbers to show
void GridWorld::displayMapWithSelectedDetails(bool display_g, bool display_rhs,
                                              bool display_h, bool display_key){
    try{
        displayHeader();
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                if(map[j][i].type == '0')
                    markCell_col_row_details(i, j, LIGHTGRAY, WHITE, display_g, display_rhs, display_h, display_key);
                if(map[j][i].type == '1')
                    markCell_col_row_details(i, j, BLACK,     WHITE, display_g, display_rhs, display_h, display_key);
                if(map[j][i].type == '9')
                    markCell_col_row_details(i, j, LIGHTRED,  WHITE, display_g, display_rhs, display_h, display_key);
                if(map[j][i].type == '8')
                    markCell_col_row_details(i, j, LIGHTBLUE, WHITE, display_g, display_rhs, display_h, display_key);
                if(map[j][i].type == '6'){
                    startVertex.row = j; startVertex.col = i;
                    markCell_col_row_details(i, j, GREEN, WHITE, display_g, display_rhs, display_h, display_key);
                }
                if(map[j][i].type == '7'){
                    goalVertex.row = j; goalVertex.col = i;
                    markCell_col_row_details(i, j, BLUE,  WHITE, display_g, display_rhs, display_h, display_key);
                }
            }
        }
    }
    catch (std::ifstream::failure&){
        std::cerr << "Exception dispaying Map\n";
    }
}

// set up coordinate system (screen rectangles)
void GridWorld::initSystemOfCoordinates(){
    WORLD_MAXX = 220.0f;
    WORLD_MAXY = 180.0f;

    // where to draw the grid area on the window
    fieldX1 = getmaxx() / 10;
    fieldX2 = getmaxx() - (getmaxx() / 10);
    fieldY1 = getmaxy() / 6;
    fieldY2 = getmaxy() - (getmaxy() / 10);

    if(DEBUG) {
        std::cout << "Device Coordinates(" << fieldX1 << ", " << fieldY1 << ", "
                  << fieldX2 << ", " << fieldY2 << ")\n";
    }

    // world rect (not used much but kept)
    worldBoundary.x1 = 0.0;           worldBoundary.y1 = WORLD_MAXY;
    worldBoundary.x2 = WORLD_MAXX;    worldBoundary.y2 = 0.0;

    // device rect (pixels)
    deviceBoundary.x1 = fieldX1;      deviceBoundary.y1 = fieldY1;
    deviceBoundary.x2 = fieldX2;      deviceBoundary.y2 = fieldY2;
}

// draw grid lines (just background lines)
void GridWorld::drawGrid(){
    setlinestyle(WIDE_DOT_FILL, 1, 1);
    setcolor(LINE_COLOUR);

    for(int x = deviceBoundary.x1; x <= deviceBoundary.x2; x += xInc){
        line(x, fieldY1, x, fieldY2);
    }
    for(int y = deviceBoundary.y1; y <= deviceBoundary.y2; y += yInc){
        line(fieldX1, y, fieldX2, y);
    }
}

// draw a blue rectangle on the cell that contains (x,y)
void GridWorld::markCell(int x, int y){
    int cellX = deviceBoundary.x1 + (((x - deviceBoundary.x1) / cellWidth) * cellWidth);
    int cellY = deviceBoundary.y1 + (((y - deviceBoundary.y1) / cellHeight) * cellHeight);
    setcolor(BLUE);
    rectangle(cellX, cellY, cellX + cellWidth, cellY + cellHeight);
}

// return the cell position (row/col) of (x,y) and draw a small blue box
CellPosition GridWorld::getCellPosition_markCell(int x, int y){
    int cellX = deviceBoundary.x1 + (((x - deviceBoundary.x1) / cellWidth) * cellWidth);
    int cellY = deviceBoundary.y1 + (((y - deviceBoundary.y1) / cellHeight) * cellHeight);
    setcolor(BLUE);
    rectangle(cellX, cellY, cellX + cellWidth, cellY + cellHeight);

    CellPosition p;
    p.col = (int(x - deviceBoundary.x1) / cellWidth) + 1;
    p.row = (int(y - deviceBoundary.y1) / cellHeight) + 1;
    return p;
}

// helpers to know the pixel bounds of the grid
int GridWorld::getGridMaxX(){
    if((cellWidth < 0) || (GRIDWORLD_COLS < 0)){
        std::cout << "Error in GridWorld::getGridMaxX()" << std::endl;
        std::exit(-1);
    }
    return (deviceBoundary.x1 + ((GRIDWORLD_COLS) * cellWidth));
}

int GridWorld::getGridMaxY(){
    if((cellHeight < 0) || (GRIDWORLD_ROWS < 0)){
        std::cout << "Error in GridWorld::getGridMaxy()" << std::endl;
        std::exit(-1);
    }
    return (deviceBoundary.y1 + ((GRIDWORLD_ROWS) * cellHeight));
}

// fill and outline a cell; also draw a tiny circle in the center
Coordinates GridWorld::markCell_col_row(int col, int row, int fillColour, int outlineColour){
    int cellX, cellY;

    if(col == 0) cellX = deviceBoundary.x1;
    else         cellX = deviceBoundary.x1 + ((col) * cellWidth);

    if(row == 0) cellY = deviceBoundary.y1;
    else         cellY = deviceBoundary.y1 + ((row) * cellHeight);

    setcolor(fillColour);
    setfillstyle(SOLID_FILL, fillColour);
    bar(cellX, cellY, cellX + cellWidth, cellY + cellHeight);
    setcolor(outlineColour);
    rectangle(cellX, cellY, cellX + cellWidth, cellY + cellHeight);

    Coordinates c;
    c.x = (cellX + cellX + cellWidth) / 2;
    c.y = (cellY + cellY + cellHeight) / 2;
    circle(c.x, c.y, cellWidth/12);
    return c;
}

// draw a cell and print numbers inside (g / rhs / h / key if asked)
void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour,
                                         bool display_g, bool display_rhs, bool display_h, bool display_key){
    int cellX, cellY; char info[256]; int x, y;

    if(col == 0) cellX = deviceBoundary.x1;
    else         cellX = deviceBoundary.x1 + ((col) * cellWidth);

    if(row == 0) cellY = deviceBoundary.y1;
    else         cellY = deviceBoundary.y1 + ((row) * cellHeight);

    setcolor(fillColour);
    setfillstyle(SOLID_FILL, fillColour);
    bar(cellX, cellY, cellX + cellWidth, cellY + cellHeight);
    setcolor(WHITE);
    rectangle(cellX, cellY, cellX + cellWidth, cellY + cellHeight);

    y = cellY;
    settextstyle(SMALL_FONT, HORIZ_DIR, 4);
    settextjustify(LEFT_TEXT, TOP_TEXT);
    setcolor(RED);

    if(display_g){
        x = cellX + textwidth("I"); y = y + textheight(";");
        double g = map[row][col].g;
        if(g > (INF - 0.0001) && g < (INF + 0.0001)) std::snprintf(info, sizeof(info), "g = INF");
        else                                         std::snprintf(info, sizeof(info), "g = %3.1f", g);
        outtextxy(x, y, info);
    }

    if(display_rhs){
        x = cellX + textwidth("I"); y = y + textheight(";");
        double rhs = map[row][col].rhs;
        if(rhs > (INF - 0.0001) && rhs < (INF + 0.0001)) std::snprintf(info, sizeof(info), "rhs = INF");
        else                                             std::snprintf(info, sizeof(info), "rhs = %3.1f", rhs);
        outtextxy(x, y, info);
    }

    if(display_h){
        x = cellX + textwidth("I"); y = y + textheight(";");
        double h = map[row][col].h;
        if(h > (INF - 0.0001) && h < (INF + 0.0001)) std::snprintf(info, sizeof(info), "h = INF");
        else                                         std::snprintf(info, sizeof(info), "h = %3.1f", h);
        outtextxy(x, y, info);
    }

    if(display_key){
        char info1[256], info2[256];
        x = cellX + textwidth("I"); y = y + textheight(";");
        double k1 = map[row][col].key[0];
        double k2 = map[row][col].key[1];

        if(k1 > (INF - 0.0001)) std::snprintf(info1, sizeof(info1), "INF");
        else                    std::snprintf(info1, sizeof(info1), "%3.1f", k1);
        if(k2 > (INF - 0.0001)) std::snprintf(info2, sizeof(info2), "INF");
        else                    std::snprintf(info2, sizeof(info2), "%3.1f", k2);

        std::snprintf(info, sizeof(info), "[%s, %s]", info1, info2);
        outtextxy(x, y, info);
    }
}

// convenience overload: show g/rhs/h and hide key by default
void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour){
    markCell_col_row_details(col, row, fillColour, outlineColour,
                             /*display_g=*/true,
                             /*display_rhs=*/true,
                             /*display_h=*/true,
                             /*display_key=*/false);
}

// draw a cell and print text like "col=?, row=?"
void GridWorld::markCell_col_row_details_xy(int col, int row, int fillColour, int outlineColour){
    int cellX, cellY; char info[128]; int x, y;

    if(col == 0) cellX = deviceBoundary.x1;
    else         cellX = deviceBoundary.x1 + ((col) * cellWidth);

    if(row == 0) cellY = deviceBoundary.y1;
    else         cellY = deviceBoundary.y1 + ((row) * cellHeight);

    setcolor(fillColour);
    setfillstyle(SOLID_FILL, fillColour);
    setbkcolor(fillColour);
    bar(cellX, cellY, cellX + cellWidth, cellY + cellHeight);
    setcolor(outlineColour);
    rectangle(cellX, cellY, cellX + cellWidth, cellY + cellHeight);

    y = cellY;
    settextstyle(SMALL_FONT, HORIZ_DIR, 4);
    settextjustify(LEFT_TEXT, TOP_TEXT);
    setcolor(WHITE);

    char info1[6]; char info2[6];
    x = cellX + textwidth("I"); y = y + textheight(";");

    int cellCol = map[row][col].col;
    int cellRow = map[row][col].row;

    if(cellCol == INF) std::snprintf(info1, sizeof(info1), "INF");
    else               std::snprintf(info1, sizeof(info1), "%d", cellCol);

    if(cellRow == INF) std::snprintf(info2, sizeof(info2), "INF");
    else               std::snprintf(info2, sizeof(info2), "%d", cellRow);

    std::snprintf(info, sizeof(info), "col=%s,row=%s", info1, info2);
    setbkcolor(fillColour);
    outtextxy(x, y, info);
}

// save the current map to a file (very simple format)
void GridWorld::saveNewMap(const char* fn){
    std::ofstream i_file;
    char saveName[260] = {0};
    std::snprintf(saveName, sizeof(saveName), "%s", (fn ? fn : ""));

    i_file.exceptions ( std::ofstream::failbit | std::ofstream::badbit );
    try{
        std::cout << "\n\nSaving new map into file: " << saveName << "..." << std::endl;
        i_file.open (fn, std::ofstream::out | std::ofstream::trunc);

        std::cout << "file opened." << std::endl;

        // write rows/cols
        i_file << GRIDWORLD_ROWS << std::endl;
        i_file << GRIDWORLD_COLS;

        // write the grid of types (as chars)
        for(int j = 0; j < GRIDWORLD_ROWS; j++){
            i_file << std::endl;
            for(int i = 0; i < GRIDWORLD_COLS; i++){
                i_file << map[j][i].type;
            }
        }

        i_file.close();
        std::cout << "file closed." << std::endl;
    }
    catch (std::ofstream::failure&){
        std::cerr << "saveNewMap(), exception opening/saving/closing file\n";
    } catch (...){
        std::cerr << "saveNewMap(), exception occurred." << std::endl;
    }

    std::cout << "GRIDWORLD_ROWS = " << GRIDWORLD_ROWS << std::endl;
    std::cout << "GRIDWORLD_COLS = " << GRIDWORLD_COLS << std::endl;
    std::cout << "\nGridworld saved into file: " << (fn ? fn : "") << "." << std::endl;
}
