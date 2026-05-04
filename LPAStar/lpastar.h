#ifndef __LPASTAR_H__
#define __LPASTAR_H__

#include <vector>
#include <queue>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <limits>
#include <chrono>
#include <functional>   // std::function for callbacks
#include <utility>      // std::pair

#include "globalvariables.h"

// Forward declaration to avoid circular include.
class GridWorld;

// Just keeping the original defines in case other code uses them.
#define INITIAL_PLANNING 1
#define FINAL_PLANNING   2

// LPA* (Lifelong Planning A*)
// This class stores the grid (maze), the priority queue (OPEN),
// and all the little helpers needed by the algorithm.
// I try to keep the comments simple and short.
class LpaStar {
public:
    // Public data used by the rest of the project.
    // "maze" is a 2D array of cells. Each cell has g, rhs, h, key[], type, etc.
    std::vector<std::vector<LpaStarCell>> maze;

    // Start and goal pointers point inside "maze".
    LpaStarCell* start {nullptr};
    LpaStarCell* goal  {nullptr};

    // Basic constructor: tell us the grid size, which heuristic to use,
    // and a short name for printing results (usually the map file name).
    LpaStar(int rows_, int cols_, unsigned int heuristic_, const std::string& gridWorldName_);

    // Bind start/goal positions (x=col, y=row) and refresh h-values.
    // Note: we clamp/repair coordinates if out of range or swapped.
    void initialise(int startX, int startY, int goalX, int goalY);

    // Two main runs:
    // - initialPlanning() is before revealing 8/9 cells.
    // - finalPlanning()    is a full rebuild after reveal (non-incremental).
    // - finalPlanningIncremental() tries to reuse work (incremental).
    void initialPlanning();
    void finalPlanning();
    void finalPlanningIncremental();

    // Recompute all h-values (depends on "goal").
    void updateHValues();

    // Recompute all keys k = (min(g,rhs)+h, min(g,rhs)).
    // Handy when we want to draw key overlays.
    void updateAllKeyValues();

    // Print one CSV-like line with stats (used by "all" mode).
    void printResults() const;

    // Hook a GridWorld pointer so we can draw polylines (paths) on screen.
    void attachGridWorld(GridWorld* gw) { gworld = gw; }

    // Optional: turn on/off expansion visualization and set a step.
    // If you pass a non-positive step, we clamp it to 1.
    void setExpansionVisualization(bool enable, int step = 50){
        visualizeExpansions = enable;
        vizStep = (step <= 0 ? 1 : step);
        manualVizConfigured = true; // manual config wins; stop auto tuning
    }

    // Path polyline helpers (for drawing lines over the grid).
    // We store (row,col) points for both the initial and final paths.
    void storeInitialPathPolyline();   // call after initial plan
    void storeFinalPathPolyline();     // call after final plan
    void drawStoredPolylines() const;  // draw both paths (cyan/yellow)

    // Public stats so callers can read them easily.
    long   stateExpansions {0};   // how many times we "improved" g(u)=rhs(u)
    long   maxQLength {0};        // max size of OPEN
    long   vertexAccesses {0};    // how many times we touched/updateVertex()
    double pathLength {0.0};      // length of the path we reconstructed
    double runningTime {0.0};     // wall-clock seconds for last run

    // Extra queue stats (some PDFs ask for this).
    long queuePushes {0};         // how many pushes into OPEN
    long queuePops   {0};         // how many valid pops (stale entries not counted)
    long queueAccesses {0};       // we define as pushes + valid pops
    long queueSizeAfter {0};      // OPEN size right after a run ends

    // Grid size copied here for convenience.
    int rows {0};
    int cols {0};

private:
    // OPEN list item: we snapshot the key when pushing (lazy deletion).
    struct PQItem{
        LpaStarCell* node;
        double k0, k1; // these were node->key[0], node->key[1] at push time
    };
    struct PQGreater{
        // std::priority_queue is a max-heap, so we invert comparison
        // to put the smallest (k0,k1) at the top.
        bool operator()(const PQItem& a, const PQItem& b) const{
            if(a.k0 != b.k0) return a.k0 > b.k0;
            return a.k1 > b.k1;
        }
    };
    std::priority_queue<PQItem, std::vector<PQItem>, PQGreater> U;

    // We store two polylines as (row,col) lists for drawing:
    // one for the initial run and one for the final run.
    std::vector<std::pair<int,int>> _initialPathRC; // start -> goal
    std::vector<std::pair<int,int>> _finalPathRC;   // start -> goal

    // Build a polyline from current g-values (walk goal->start by best neighbor).
    // Returns points from start to goal.
    std::vector<std::pair<int,int>> _buildPathRC() const;

    // Convert (row,col) to pixel centers and draw a line strip.
    void _drawPolylineFromRC(const std::vector<std::pair<int,int>>& rcPath,
                             int color, int thickness) const;

    // A simple Infinity helper (to avoid clashing with INF macro).
    inline static double INF_D(){ return std::numeric_limits<double>::infinity(); }
    inline static double minval(double a,double b){ return a<b?a:b; }

    // Reset counters and timers before a run.
    void resetStats();

    // Re-create an empty grid of cells and reset pointers.
    void makeEmptyMaze();

    // The main LPA* loop.
    // If initialPhase==true: we treat '8' as blocked (unknown-free).
    void computeShortestPath(bool initialPhase);

    // Standard LPA* update: compute rhs(u), then (re)insert into OPEN if needed.
    void updateVertex(LpaStarCell* u, bool initialPhase);

    // Visit predecessors/successors in 8-neighborhood.
    // We call "fn(neighbor, moveCost)" for each valid neighbor.
    void forEachPred(LpaStarCell* s,
                     const std::function<void(LpaStarCell*, double)>& fn,
                     bool initialPhase);
    void forEachSucc(LpaStarCell* s,
                     const std::function<void(LpaStarCell*, double)>& fn,
                     bool initialPhase);

    // Key helpers.
    void calcKey(LpaStarCell* s);
    std::pair<double,double> topKey();                        // read smallest key (skip stale)
    bool popMinWithKey(LpaStarCell*& out, double& k0, double& k1); // pop valid min
    void pushWithCurrentKey(LpaStarCell* s);                  // push if inconsistent

    // Heuristic and move cost:
    // - calc_H: Chebyshev or Euclidean, depending on the mode.
    // - moveCost: 1 for straight, sqrt(2) for diagonal.
    double calc_H(int x,int y) const;
    static double moveCost(int dx,int dy);

    // A couple of small numeric helpers to compare doubles safely.
    static bool approxEq(double a,double b){
        if(std::isinf(a) && std::isinf(b)) return true;
        return std::fabs(a-b) <= 1e-12;
    }
    static bool lexLess(double a0,double a1,double b0,double b1){
        // Lexicographic compare with a tiny epsilon so we don't loop forever
        // due to floating point noise.
        const double EPS = 1e-12;
        if (a0 < b0 - EPS) return true;
        if (a0 > b0 + EPS) return false;
        return (a1 < b1 - EPS);
    }

    // Cell traversability rule (handles initial/final phases).
    bool traversable(int x,int y,bool initialPhase) const;

    // Compute path length from the current g-values (goal -> start walk).
    double reconstructPathLength() const;

    // Visualization helpers (auto throttle).
    int   recommendedVizStep() const;  // pick a starting step from grid size
    void  autoTuneVizOnDraw();         // adjust step based on actual frame time

private:
    // Heuristic mode and some labels for printing.
    unsigned int heuristic {0};
    std::string strHeuristic;
    std::string gridWorldName;
    std::string strEpisode;

    // Optional pointer to the grid drawer (so we can draw lines).
    GridWorld* gworld {nullptr};

    // Visualization flags/knobs.
    bool visualizeExpansions {true};
    int  vizStep {50};

    // If user called setExpansionVisualization(), we stop auto tuning.
    bool manualVizConfigured {false};
    bool autoAdaptiveViz {true};
    std::chrono::steady_clock::time_point lastVizTp{};
};

#endif // __LPASTAR_H__
