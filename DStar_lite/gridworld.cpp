#include "gridworld.h"

// Beginner note: tiny helper. If any term is INF, the sum should stay INF.
// I keep it separate so I don't repeat this check everywhere.
//---
double sum(double a, double b){
		if(a == INF || b == INF){
			return INF;
		}else{
			return (a+b);
		}
    }


// Beginner note: This one is similar but for D* Lite. Here I start at 'start'
// and walk towards the goal by always choosing the neighbor with min(g + cost).
//---
void GridWorld::displayPath_for_dStarLite(){

     vertex* currentVertex = NULL;
     vertex* min_neighbour = NULL;

     currentVertex = &map[startVertex.row][startVertex.col];

     //---
     vertex* neighbour; 
     vertex* originVertex;

     double min_g_plus_c = INF;
     double linkCost,g;    
     //---

     originVertex = currentVertex;

 while(1){
     min_g_plus_c = INF;
     for(int m=0; m < DIRECTIONS; m++){

           neighbour = originVertex->move[m];

           if(neighbour != NULL && neighbour->type != '1'){

               linkCost = originVertex->linkCost[m];

               g=(originVertex->move[m])->g;

               if(min_g_plus_c > sum(g,linkCost)){

                       min_g_plus_c = sum(g,linkCost);

                       min_neighbour = neighbour;
                }              
           }                   
     }
 
     if(min_neighbour != NULL){
     	setcolor(RED);
     	setlinestyle(SOLID_LINE, 1, 1);

     	line(min_neighbour->centre.x, min_neighbour->centre.y, currentVertex->centre.x, currentVertex->centre.y);
     }     
 
     currentVertex = min_neighbour; //added this
     originVertex = currentVertex; //added this

     if(currentVertex == &map[goalVertex.row][goalVertex.col])

           break;
 }
}
//---

// Beginner note: This draws the text header on top of the window.
// I also print the heuristic in use so I don't forget what I'm testing.
void GridWorld::displayHeader(){
		 int x,y;
		 x = getmaxx() /2;
		 y = textheight("H")*1;
		 settextstyle(SMALL_FONT, HORIZ_DIR, 5);
		 settextjustify(CENTER_TEXT,TOP_TEXT);
		 setcolor(YELLOW);				
		 setbkcolor(BLACK);			
		 outtextxy(x ,y, "INCREMENTAL OPTIMAL SEARCH (8-CONNECTED GRIDWORLD)");
		 y = y + textheight("_");
	    settextjustify(LEFT_TEXT,TOP_TEXT);    
	    outtextxy(fieldX2-textwidth("start-up codes by n.h.reyes@massey.ac.nz"),fieldY2,"start-up codes by n.h.reyes@massey.ac.nz");
	    settextjustify(CENTER_TEXT,TOP_TEXT);    
	    setcolor(GREEN);
		 outtextxy(x ,y, "F4: hide details, F5: show details, F9: copy display map to Algorithm data structure(maze), F10: run Search");
	    y = y + textheight("_");
	    setcolor(WHITE);
   	    outtextxy(x ,y, "B: block cell, U: unblock cell, H: h-values, K: key-values, S: new START, X: new GOAL, P: cell positions, C: local connections, M: all connections ");
	    
	    y = y + textheight("_");
	    setcolor(WHITE);
	
	    char info[256];
		char heuristicStr[80];

		strcpy(info, "");
		strcpy(heuristicStr, "");
	    //-------------------------------------------
		 if (HEURISTIC == EUCLIDEAN){
			strcpy(heuristicStr,"EUCLIDEAN");			
		 }else if (HEURISTIC == CHEBYSHEV){
			strcpy(heuristicStr,"CHEBYSHEV");			
		 }	
	    //-------------------------------------------
	    // sprintf(info,"fileName = %s, heuristic = %s", fileName, heuristicStr);
		 snprintf(info, sizeof(info),"fileName = %s, heuristic = %s", fileName, heuristicStr);
   	 //outtextxy(x ,y, "fileName: ");
	    outtextxy(x ,y, info);
}


// Beginner note: read the map file, set up sizes/colors and draw cells.
// I also remember start and goal when I see '6' or '7' in the file.
void GridWorld::loadMapAndDisplay(const char* fn) //,int totalRows, int totalColumns)
{
	// int cellX, cellY;
	ifstream i_file;
	
	// Beginner note: I turn on exceptions so file errors throw and I can catch them.
	i_file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
	
	try{
		
		
		i_file.open(fn, std::ifstream::in);

		if(DEBUG){
		   cout << "file opened." << endl;
	    }
		strcpy(fileName, fn);
		
		//---------------------------------------
		displayHeader();
		//---------------------------------------
		
		i_file >> GRIDWORLD_ROWS;
		i_file >> GRIDWORLD_COLS;
		//-------------------------------
		xInc = abs(fieldX2-fieldX1)/GRIDWORLD_COLS;
	    cellWidth = xInc;
	    yInc = abs(fieldY2-fieldY1)/GRIDWORLD_ROWS;
        cellHeight = yInc;	
		//-------------------------------
		
		map.resize(GRIDWORLD_ROWS); 
			
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				map[j].resize(GRIDWORLD_COLS);
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					// Beginner note: init per-vertex data so numbers draw nicely later.
					map[j][i].g = 0.0;
					map[j][i].rhs = 0.0;
					map[j][i].f = 0.0;
					map[j][i].row = j;
					map[j][i].col = i;
					map[j][i].key[0] = 0.0;
					map[j][i].key[1] = 0.0;
					
					for(int m=0; m < DIRECTIONS; m++){
					   map[j][i].move[m] = NULL;  	
						map[j][i].linkCost[m] = 1.0; 
					}
					
					//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
					
					i_file >> map[j][i].type;
					
					if(map[j][i].type == '0') //traversable cell
					{						
						map[j][i].centre = markCell_col_row(i,j, LIGHTGRAY, WHITE);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						map[j][i].centre = markCell_col_row(i,j, BLACK, WHITE);
					}
					if(map[j][i].type == '9') //unknown to be blocked
					{

#if defined __unix__ || defined __APPLE__
					map[j][i].centre = markCell_col_row(i,j, COLOR(231,163,139), WHITE);
#elif defined __WIN32__

					map[j][i].centre = markCell_col_row(i,j, 47, WHITE);
#endif

	
					
					}
					if(map[j][i].type == '8') //unknown to be traversable
					{
						map[j][i].centre = markCell_col_row(i,j, LIGHTBLUE, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						map[j][i].centre = markCell_col_row(i,j, GREEN, WHITE);
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						map[j][i].centre = markCell_col_row(i,j, BLUE, WHITE);
					}
					
					if(map[j][i].type == '4') //'SUB-GOAL'
					{
						map[j][i].centre = markCell_col_row(i,j, CYAN, WHITE);
					}
					
				}
			}
		

		i_file.close();
		if(DEBUG){
		   cout << "file closed." << endl;
		}
		
		MAP_INITIALISED = true;
	}
	catch (std::ifstream::failure& e) {
      std::cerr << "Exception opening/reading/closing file\n";
    }
 
  if(DEBUG){
  	cout << "\nGridworld loaded from file: " << fn << endl;
  	cout << "GRIDWORLD_ROWS = " << GRIDWORLD_ROWS << endl;
  	cout << "GRIDWORLD_COLS = " << GRIDWORLD_COLS << endl;
  }
  
  //getch();
}

// Beginner note: after loading, I connect each cell to its valid neighbors.
// I also remember which are start and goal again (redundant but safe).
void GridWorld::initialiseMapConnections() 
{
	// int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	int neighbourY, neighbourX;
	
	neighbourY=-1;
	neighbourX=-1;
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					originVertex = &map[j][i];
					for(int m=0; m < DIRECTIONS; m++){
						
						//originVertex->linkCost[m] = INF;
						
						neighbourY = originVertex->row + neighbours[m].y;
						neighbourX = originVertex->col + neighbours[m].x;
						//for debugging only
						//cout << "neighbour["<< m << "]: (neighbourX = " << neighbourX << ", neighbourY = " << neighbourY << ")" << endl;
						if( (neighbourX >= 0) && (neighbourX < GRIDWORLD_COLS) && (neighbourY >= 0) && (neighbourY < GRIDWORLD_ROWS)){
						   neighbour = &map[neighbourY][neighbourX];
						   
							if(neighbour->type != '1'){ //if the neighbour is not BLOCKED, then it is reacheable
							   //map[j][i].move[m] = neighbour;
								originVertex->move[m] = neighbour;
								originVertex->linkCost[m] = 1.0;
							} else if(neighbour->type == '1'){
								
								originVertex->move[m] = neighbour; //THIS IS ONLY A TEST
								originVertex->linkCost[m] = INF;
							}
							
						}
					   
					}
					
					
					if(map[j][i].type == '0') //traversable cell
					{						
						//markCell_col_row(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						//markCell_col_row(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						//markCell_col_row(i,j, DARKGRAY, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						//markCell_col_row(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						//markCell_col_row(i,j, BLUE, WHITE);
						
					}
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
}

// Beginner note: helper to draw connections of one vertex so I can debug quickly.
//inputs: col, row of vertex 
//output: display vertex connections
void GridWorld::displayVertexConnections(int i, int j) 
{
	// int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	// int neighbourY, neighbourX;
		
	try{
		//---------------------------------------
		 // displayHeader();
		//---------------------------------------		
		
		cout << "vertex(x = " << i << ", y= " << j << endl;
		
		         //If the cell is not a BLOCKED CELL
		         if(map[j][i].type != '1'){
		         	    cout << "\ndraw lines." << endl;
		
						originVertex = &map[j][i];
						for(int m=0; m < DIRECTIONS; m++){
							neighbour = map[j][i].move[m];
							if(neighbour != NULL && neighbour->type != '1'){
								cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; 
								//setcolor(RED);
								setcolor(YELLOW);
								setlinestyle (0, 0, THICK_WIDTH);
								// setlinestyle(WIDE_DOT_FILL, 2, 2);
							    line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
							}  else if(neighbour->type == '1'){
								setcolor(RED);
								setlinestyle (0, 0, THICK_WIDTH);
								// setlinestyle(WIDE_DOT_FILL, 2, 2);
							    line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
								cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; //<< "blocked cell" << endl; 
								
							}
							
						}
						setlinestyle (0, 0, NORM_WIDTH);
				   } 
					
			
			setlinestyle (0, 0, NORM_WIDTH);
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}



// Beginner note: same as above but for the whole grid. Useful to see the topology.
//output: display entire map connections
void GridWorld::displayMapConnections() 
{
	// int cellX, cellY;
	vertex* originVertex;
	vertex* neighbour;
	// int neighbourY, neighbourX;
		
	try{
		//---------------------------------------
		 // displayHeader();
		//---------------------------------------		
		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
		
		         //If the cell is not a BLOCKED CELL
		         if(map[j][i].type != '1'){
		
						originVertex = &map[j][i];
						for(int m=0; m < DIRECTIONS; m++){
							neighbour = map[j][i].move[m];
							if(neighbour != NULL && neighbour->type != '1'){
								//for debugging only
								//cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; 
								
								setcolor(YELLOW);
								// setlinestyle(WIDE_DOT_FILL, 2, 2);
								setlinestyle (0, 0, THICK_WIDTH);
							    line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
							}  else if(neighbour->type == '1'){
								setcolor(RED);
								// setlinestyle(WIDE_DOT_FILL, 2, 2);
								setlinestyle (0, 0, THICK_WIDTH);
							    line(neighbour->centre.x, neighbour->centre.y, originVertex->centre.x, originVertex->centre.y);	
								//for debugging only
								//cout << "cost[" << m << "] = " << originVertex->linkCost[m] << endl; //<< "blocked cell" << endl; 
								
							}
							
						}
						setlinestyle (0, 0, NORM_WIDTH);
				   } 
					
					
					
				} //for next Cols
			} //for next Rows
		   // setlinestyle(SOLID_LINE, 1, 1);
		   setlinestyle (0, 0, NORM_WIDTH);
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}


// Beginner note: plain view, just color the cells. I call this when I don't need numbers.
void GridWorld::displayMap() 
{
	// int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown to be blocked
					{
						//

#if defined __unix__ || defined __APPLE__

					markCell_col_row(i,j, COLOR(231,163,139), WHITE);
#elif defined __WIN32__

					markCell_col_row(i,j, 47, WHITE);
					
#endif



						
					}
					if(map[j][i].type == '8') //unknown to be traversable
					{
						markCell_col_row(i,j, LIGHTBLUE, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row(i,j, BLUE, WHITE);
						
					}
					
					if(map[j][i].type == '4') //'SUB-GOAL'
					{
						map[j][i].centre = markCell_col_row(i,j, CYAN, WHITE);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
  
 
  
}

// Beginner note: same as displayMap, but I also print cell indices (x,y).
void GridWorld::displayMapWithPositionDetails() 
{
	// int cellX, cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details_xy(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details_xy(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details_xy(i,j, DARKGRAY, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details_xy(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details_xy(i,j, BLUE, WHITE);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
}

// Beginner note: draw cells and show g/rhs/h (I call a helper that prints numbers).
void GridWorld::displayMapWithDetails() 
{
	// int cellX;
	// int cellY;
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						//markCell_col_row_details(i,j, LIGHTGRAY, WHITE);
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE);
						
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						//markCell_col_row_details(i,j, BLACK, WHITE);
						markCell_col_row_details(i,j, BLACK, WHITE);
						
					}
					if(map[j][i].type == '9') //unknown
					{
						markCell_col_row_details(i,j, DARKGRAY, WHITE);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE);
						
					}
					
					if(map[j][i].type == '4') //SUB GOAL
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, CYAN, WHITE);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
}



// Beginner note: Like the function above, but here I can choose which values to show.
// This is used by my 'G' key toggle in main.cpp.
void GridWorld::displayMapWithKeyDetails() 
{
	// int cellX;
	// int cellY;
	
		
	try{
		//---------------------------------------
		 // displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE, true, true, false, true);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details(i,j, BLACK, WHITE, true, true, false, true);
						
					}
					if(map[j][i].type == '9') //unknown
					{

#if defined __unix__ || defined __APPLE__

					markCell_col_row_details(i,j, COLOR(231,163,139), WHITE, true, true, false, true);
#elif defined __WIN32__
					markCell_col_row_details(i,j, 47, WHITE, true, true, false, true);

#endif
						
					}

					if(map[j][i].type == '8') //unknown
					{
						markCell_col_row_details(i,j, LIGHTBLUE, WHITE, true, true, false, true);
					    // map[j][i].centre = markCell_col_row(i,j, LIGHTBLUE, WHITE);	
					}


					
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE, true, true, false, true);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE, true, true, false, true);
						
					}
					
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
}


// Beginner note: fully generic version, I pass which numbers to draw (g/rhs/h/key).
void GridWorld::displayMapWithSelectedDetails(bool display_g, bool display_rhs, bool display_h, bool display_key) 
{
	
	
		
	try{
		//---------------------------------------
		 displayHeader();
		//---------------------------------------		
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					if(map[j][i].type == '0') //traversable cell
					{						
						markCell_col_row_details(i,j, LIGHTGRAY, WHITE, display_g, display_rhs, display_h, display_key);
					}
					
					if(map[j][i].type == '1') //'B' - blocked cell
					{
						markCell_col_row_details(i,j, BLACK, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '9') //unknown to be blocked
					{

#if defined __unix__ || defined __APPLE__

					markCell_col_row_details(i,j, COLOR(231,163,139), WHITE, display_g, display_rhs, display_h, display_key);
#elif defined __WIN32__
					markCell_col_row_details(i,j, 47, WHITE, display_g, display_rhs, display_h, display_key);

#endif
						
					}
					if(map[j][i].type == '8') //unknown to be traversable
					{
						markCell_col_row_details(i,j, LIGHTBLUE, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '6') //'S' - start vertex
					{
						startVertex.row = j;
						startVertex.col = i;
						markCell_col_row_details(i,j, GREEN, WHITE, display_g, display_rhs, display_h, display_key);
						
					}
					if(map[j][i].type == '7') //'G' - goal vertex
					{
						goalVertex.row = j;
						goalVertex.col = i;
						markCell_col_row_details(i,j, BLUE, WHITE, display_g, display_rhs, display_h, display_key);	
					}
				}
			}
		
	}
   catch (std::ifstream::failure &e) {
    std::cerr << "Exception dispaying Map\n";
   }
}



// Beginner note: set up the drawing area (device coordinates) and
// the world bounds I use to compute cell sizes.
void GridWorld::initSystemOfCoordinates(){
   

	 WORLD_MAXX = 220.0f;
    WORLD_MAXY = 180.0f;
	
	
	
	
	 fieldX1 = getmaxx() / 10;
    fieldX2 = getmaxx() - (getmaxx() / 10);
    fieldY1 = getmaxy() / 6; //top of the screen
    fieldY2 = getmaxy() - (getmaxy() / 10); //bottom of the screen
	
	if(DEBUG) {
		cout << "Device Coordinates(" << fieldX1 << ", " << fieldY1 << ", " << fieldX2 << ", " << fieldY2 << ")\n";	
	}
	

//--------------------------------------------------------	
		    
//5 vs 5 MIROSOT FIELD    
	//World boundaries
    worldBoundary.x1 = 0.0;
    worldBoundary.y1 = WORLD_MAXY;
    worldBoundary.x2 = WORLD_MAXX;
    worldBoundary.y2 = 0.0;

	//Device boundaries
    deviceBoundary.x1 = fieldX1;
    deviceBoundary.y1 = fieldY1;
    deviceBoundary.x2 = fieldX2;
    deviceBoundary.y2 = fieldY2;
	 	
}
///////////////////////////////////////////////////////////////////////////

// Beginner note: draw grid lines so I can see each cell.
void GridWorld::drawGrid(){
	// int countVertLines, countHorizLines;
	
	// countVertLines=0;
	// countHorizLines=0;
	
	
   setlinestyle(WIDE_DOT_FILL, 1, 1);
   //setcolor(DARKGRAY);
	
   
	setcolor(LINE_COLOUR);
   //draw vertical bars
	
	
	//for(int x=deviceBoundary.x1; x <= deviceBoundary.x2; x += robotWidth){
	for(int x=deviceBoundary.x1; x <= deviceBoundary.x2; x += xInc){
		line(x, fieldY1, x, fieldY2);
	}
	
	//for(int y=deviceBoundary.y1; y <= deviceBoundary.y2; y += robotWidth){
	for(int y=deviceBoundary.y1; y <= deviceBoundary.y2; y += yInc){
		line(fieldX1, y, fieldX2, y);
	}
   	
	
	//setcolor(WHITE);
	//rectangle(fieldX1, fieldY1, fieldX2, fieldY2);
	
	//cout << "drawGrid complete." << endl;
}

// Beginner note: little helper to put a blue rectangle on the cell under the mouse.
void GridWorld::markCell(int x, int y){
	
	int cellX, cellY;
	

	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/cellWidth) * cellWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/cellHeight) * cellHeight);
	
	setcolor(BLUE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
}

// Beginner note: this returns the (row,col) for a pixel and draws a blue box on it.
// I use it when clicking to set start/goal/blocks.
CellPosition GridWorld::getCellPosition_markCell(int x, int y){
	
	int cellX, cellY;
	CellPosition p;

	cellX = deviceBoundary.x1 + (((x-deviceBoundary.x1)/cellWidth) * cellWidth);
	cellY = deviceBoundary.y1 + (((y-deviceBoundary.y1)/cellHeight) * cellHeight);
	
	setcolor(BLUE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	int col = (int(x-deviceBoundary.x1)/cellWidth)+1;
	int row = (int(y-deviceBoundary.y1)/cellHeight)+1;
	
	p.col = col;
	p.row = row;
	return p;
	
}

// Beginner note: a couple of helpers that return the max pixel coordinates for the grid.
int GridWorld::getGridMaxX(){
	if( (cellWidth < 0) || (GRIDWORLD_COLS < 0) ){
		cout << "Error in GridWorld::getGridMaxX()" << endl;
		exit(-1);
	}
	//return (deviceBoundary.x1 + ((GRIDWORLD_COLS-1) * cellWidth));
	return (deviceBoundary.x1 + ((GRIDWORLD_COLS) * cellWidth));
}

int GridWorld::getGridMaxY(){
	if( (cellHeight < 0) || (GRIDWORLD_ROWS < 0) ){
		cout << "Error in GridWorld::getGridMaxy()" << endl;
		exit(-1);
	}
	//return (deviceBoundary.y1 + ((GRIDWORLD_ROWS-1) * cellHeight));
	return (deviceBoundary.y1 + ((GRIDWORLD_ROWS) * cellHeight));
}

// Beginner note: draw a colored cell and a tiny circle in the center. Also return center.
Coordinates GridWorld::markCell_col_row(int col, int row, int fillColour, int outlineColour){
	
	int cellX, cellY;
	
 	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	
	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
   setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	Coordinates c;
	c.x = (cellX + cellX+cellWidth)/2;
	c.y = (cellY + cellY+cellHeight)/2;
	
	circle(c.x, c.y, cellWidth/12);
	
	return c;
}

// Beginner note: same as above but now I can choose which numbers to draw for a cell.
void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour, bool display_g, bool display_rhs, bool display_h, bool display_key){
	
	int cellX, cellY;
	char info[256]; //napoleon 2022
	int x,y;
	
	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	

    setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	// setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(WHITE);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	//--------------------------------------------
	y = cellY; 
		
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,TOP_TEXT);

	setcolor(RED);
	
	if(display_g){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		
		double g = map[row][col].g;
		strcpy(info, ""); //2022 - napoleon
		if(g > (INF - 0.0001) && g < (INF + 0.0001)){ //tolerance = 0.0001
			// sprintf(info,"g = INF");
			snprintf(info, sizeof(info),"g = INF");
		} else {
			// sprintf(info,"g = %3.1f",g);
			snprintf(info,sizeof(info),"g = %3.1f",g);
		}
		// setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }
	
	//--------------------------------------------
	
	if(display_rhs){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		
		double rhs = map[row][col].rhs;
		strcpy(info, ""); //2022 - napoleon
		if(rhs > (INF - 0.0001) && rhs < (INF + 0.0001)){ //tolerance = 0.0001
			// sprintf(info,"rhs = INF");
			snprintf(info, sizeof(info),"rhs = INF");
		} else {
			// sprintf(info,"rhs = %3.1f",rhs);
			snprintf(info, sizeof(info) ,"rhs = %3.1f",rhs);
		}
		// setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }
	//--------------------------------------------
		
	if(display_h){
		x = cellX + textwidth("I");
		y = y + textheight(";");
		double h = map[row][col].h;
		strcpy(info, ""); //2022 - napoleon
		if(h > (INF - 0.0001) && h < (INF + 0.0001)){ //tolerance = 0.0001
			// sprintf(info,"h = INF");
			snprintf(info,sizeof(info),"h = INF");
		} else {
			// sprintf(info,"h = %3.1f",h);
			snprintf(info,sizeof(info),"h = %3.1f",h);
			
		}
		// setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }
	
	//--------------------------------------------
		
	if(display_key){
		char info1[256]; //2022
		char info2[256]; //2022
		x = cellX + textwidth("I");
		y = y + textheight(";");
		double k1 = map[row][col].key[0];
		double k2 = map[row][col].key[1];
		
		strcpy(info1, ""); //2022 - napoleon
		strcpy(info2, ""); //2022 - napoleon

		if(k1 > (INF - 0.0001)){ //tolerance = 0.0001
			// sprintf(info1,"INF");
			snprintf(info1,sizeof(info1),"INF");
		} else {
			
			// sprintf(info1,"%3.1f",k1);
			snprintf(info1,sizeof(info1),"%3.1f",k1);
		}
		
		if(k2 > (INF - 0.0001)){ //tolerance = 0.0001
			// sprintf(info2,"INF");
			snprintf(info2,sizeof(info2),"INF");
		} else {
			// sprintf(info2,"%3.1f",k2);
			snprintf(info2,sizeof(info2),"%3.1f",k2);
		}
		
		// sprintf(info,"[%s, %s]",info1, info2);
		snprintf(info,sizeof(info),"[%s, %s]",info1, info2);
		
		// setbkcolor(fillColour);
		outtextxy(x ,y, info);
   }
}



// Beginner note: same “details” function but shows cell’s (col,row).
void GridWorld::markCell_col_row_details_xy(int col, int row, int fillColour, int outlineColour){
	
	int cellX, cellY;
	char info[128];
	int x,y;
	
	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	

	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
	
   setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	
	//--------------------------------------------
			
	y = cellY; // + textheight("_");
		
	settextstyle(SMALL_FONT, HORIZ_DIR, 4);
	settextjustify(LEFT_TEXT,TOP_TEXT);
	setcolor(WHITE);
	
	
	//--------------------------------------------
		
	
	char info1[6];
	char info2[6];
	x = cellX + textwidth("I");
	y = y + textheight(";");
	int cellCol = map[row][col].col;
	int cellRow = map[row][col].row;
	
	if(cellCol == INF){ 
		// sprintf(info1,"INF");
		snprintf(info1,sizeof(info1),"INF");

		
	} else {
		// sprintf(info1,"%d",cellCol);
		snprintf(info1,sizeof(info1),"%d",cellCol);
	}
	
	if(cellRow == INF){ 
		// sprintf(info2,"INF");
		snprintf(info2,sizeof(info2),"INF");
	} else {
		// sprintf(info2,"%d",cellRow);
		snprintf(info2,sizeof(info2),"%d",cellRow);
		
	}
	
	// sprintf(info,"col=%s,row=%s",info1, info2);
	snprintf(info,sizeof(info), "col=%s,row=%s",info1, info2);
	
	
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
}


// Beginner note: another variant that prints g/rhs/h stacked inside the cell.
void GridWorld::markCell_col_row_details(int col, int row, int fillColour, int outlineColour){
	
	int cellX, cellY;
	
	if(col == 0){
		cellX = deviceBoundary.x1;
	} else {
	   //cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
		cellX = deviceBoundary.x1 + ((col) * cellWidth);
	}
	
	if(row == 0){
	   cellY = deviceBoundary.y1;
	} else {
		//cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
		cellY = deviceBoundary.y1 + ((row) * cellHeight);
	}
	

	//~ cellX = deviceBoundary.x1 + ((col-1) * cellWidth);
	//~ cellY = deviceBoundary.y1 + ((row-1) * cellHeight);
	
    setcolor(fillColour);
	setfillstyle(SOLID_FILL, fillColour);
	setbkcolor(fillColour);
	bar(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	setcolor(outlineColour);
	rectangle(cellX, cellY, cellX+cellWidth, cellY + cellHeight);
	//--------------------------------------------
	int x,y;
	
	x = cellX + textwidth("I");
	y = cellY + textheight("_");
	//y = cellY; //  + textheight(";");
	settextstyle(SMALL_FONT, HORIZ_DIR, 5);
	settextjustify(LEFT_TEXT,TOP_TEXT);
	setcolor(WHITE);
	
	char info[256];
	double g = map[row][col].g;
	
	strcpy(info, ""); //2022 - napoleon
	if(g > (INF - 0.0001) && g < (INF + 0.0001)){ //tolerance = 0.0001
		// sprintf(info,"g = INF");
		snprintf(info,sizeof(info),"g = INF");
	} else {
	   // sprintf(info,"g = %3.1f",g);
		snprintf(info,sizeof(info),"g = %3.1f",g);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
	//--------------------------------------------
	y = y + textheight(";");
	
	double rhs = map[row][col].rhs;
	
	if(rhs > (INF - 0.0001) && rhs < (INF + 0.0001)){ //tolerance = 0.0001
		// sprintf(info,"rhs = INF");
		snprintf(info,sizeof(info),"rhs = INF");
	} else {
	   snprintf(info,sizeof(info),"rhs = %3.1f",rhs);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
	//--------------------------------------------
	y = y + textheight(";");
	
	double h = map[row][col].h;
	
	if(h > (INF - 0.0001) && h < (INF + 0.0001)){ //tolerance = 0.0001
		snprintf(info,sizeof(info),"h = INF");

	} else {
	   snprintf(info,sizeof(info),"h = %3.1f",h);
	}
	setbkcolor(fillColour);
	outtextxy(x ,y, info);
	
	
}


// Beginner note: write the current map (including edited cells) back to a file.
// I use it to save custom test cases for the report.
void GridWorld::saveNewMap(const char* fn) 
{
	// int cellX, cellY;
	ofstream i_file;
	char fileName[25];
	
	
	strcpy(fileName, fn);

	i_file.exceptions ( std::ofstream::failbit | std::ofstream::badbit );
	
	try{
		
		cout << "\n\nSaving new map into file: " << fileName << "..." << endl;
		
		i_file.open (fn, std::ofstream::out | std::ofstream::trunc); //clear contents if the file already exists

		
		cout << "file opened." << endl;
		
		
		
		i_file << GRIDWORLD_ROWS << endl;
		i_file << GRIDWORLD_COLS;
		//-------------------------------
		
			
			for(int j =0; j < GRIDWORLD_ROWS; j++) //row
			{
				i_file << endl;
				for(int i =0;i < GRIDWORLD_COLS; i++) //col
				{
					
					i_file << map[j][i].type;
					
				}
			}
		
		i_file.close();
		cout << "file closed." << endl;
		
		
  }	catch (std::ofstream::failure &e) {
    std::cerr << "saveNewMap(), exception opening/saving/closing file\n";


  } catch (...) {
    std::cerr << "saveNewMap(), exception occurred." << endl;
  }

  cout << "GRIDWORLD_ROWS = " << GRIDWORLD_ROWS << endl;
  cout << "GRIDWORLD_COLS = " << GRIDWORLD_COLS << endl;
  
  cout << "\nGridworld saved into file: " << fn << "." << endl;
  
}
