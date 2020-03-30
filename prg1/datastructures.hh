// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <map>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <unordered_map>
#include<bits/stdc++.h>
#include <functional>
#include <deque>

using namespace std;
// Types for IDs
using StopID = long int;
using RegionID = std::string;
using Name = std::string;

// Return values for cases where required thing was not found
StopID const NO_STOP = -1;
RegionID const NO_REGION = "!!NO_REGION!!";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};


// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};


// This is the class you are supposed to implement

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance:
    // Short rationale for estimate:
    int stop_count();

    // Estimate of performance:
    // Short rationale for estimate:
    void clear_all();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> all_stops();

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_stop(StopID id, Name const& name, Coord xy);

    // Estimate of performance:
    // Short rationale for estimate:
    Name get_stop_name(StopID id);

    // Estimate of performance:
    // Short rationale for estimate:
    Coord get_stop_coord(StopID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> stops_alphabetically();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> stops_coord_order();

    // Estimate of performance:
    // Short rationale for estimate:
    StopID min_coord();

    // Estimate of performance:
    // Short rationale for estimate:
    StopID max_coord();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> find_stops(Name const& name);

    // Estimate of performance:
    // Short rationale for estimate:
    bool change_stop_name(StopID id, Name const& newname);

    // Estimate of performance:
    // Short rationale for estimate:
    bool change_stop_coord(StopID id, Coord newcoord);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_region(RegionID id, Name const& name);

    // Estimate of performance:
    // Short rationale for estimate:
    Name get_region_name(RegionID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<RegionID> all_regions();

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_stop_to_region(StopID id, RegionID parentid);

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_subregion_to_region(RegionID id, RegionID parentid);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<RegionID> stop_regions(StopID id);

    // Non-compulsory operations

    // Estimate of performance:
    // Short rationale for estimate:
    void creation_finished();

    // Estimate of performance:
    // Short rationale for estimate:
    std::pair<Coord, Coord> region_bounding_box(RegionID id);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> stops_closest_to(StopID id);

    // Estimate of performance:
    // Short rationale for estimate:
    bool remove_stop(StopID id);

    // Estimate of performance:
    // Short rationale for estimate:
    RegionID stops_common_region(StopID id1, StopID id2);

private:
    // Add stuff needed for your class implementation here
    struct RegionNode {
        RegionID regionId;
        Name regionName;
        std::vector<StopID>stops;
        RegionNode *child;
        RegionNode *next;
        RegionNode *prev;
        RegionNode *parent;
    };

    struct BusStop {
        StopID id;
        Name name;
        Coord coord;
        double dist;
        RegionNode* main_region;

    };


    std::multimap<Name,StopID>name_stop;
    std::multimap<double,StopID>new_coord;
    std::unordered_map<StopID,BusStop> stops;
    std::vector<StopID> new_stop_coord;
    std::vector<StopID> all_stop;
    std::vector<StopID> stop_coord;
    std::vector<RegionID>allRegions;
    std::unordered_map<RegionID,RegionNode*> regions;

    bool sorted_coord = true;
    bool sorted_name = true;



    RegionNode *newNode (RegionID regionID, Name regionName){
        RegionNode* node = new RegionNode();
        node->regionId = regionID;
        node->regionName = regionName;
        node->stops = {};
        node->child =  NULL;
        node->next = NULL;
        node->prev = NULL;
        node->parent = NULL;
        return node;
      }


    void traverseTree(RegionNode * root,std::vector<int>&x_coord,std::vector<int>&y_coord)
    {
        if (root == NULL)
            return;
        while (root->prev) {
            root = root->prev;
        }

        std::stack<RegionNode*> s;
        s.push(root);
        while (!s.empty())
        {
            RegionNode* top = s.top();
            std::vector<StopID> temp = top->stops;

            if (temp.size() >0 ){
                for (auto s:temp) {
                    x_coord.push_back(stops[s].coord.x);
                    y_coord.push_back(stops[s].coord.y);
                }
            }
            s.pop();
            if (top->next) {
                s.push(top->next);
            }
            if (top->child)
                s.push(top->child);
        }
    }

    inline bool sortByCoord(const StopID &lhs, const StopID &rhs) {
        if (stops[lhs].dist < stops[rhs].dist) {return true;}
        else if (stops[lhs].dist > stops[rhs].dist) {return false;}
        else {return stops[lhs].coord.y < stops[rhs].coord.y; }
    }


    inline bool sortByName(const StopID &lhs, const StopID &rhs) { return stops[lhs].name < stops[rhs].name; }

    inline bool sortByDistance(const StopID &lhs, const StopID &rhs, Coord xy) {
        double dist1 = pow(stops[lhs].coord.x - xy.x,2) + pow(stops[lhs].coord.y - xy.y,2);
        double dist2 = pow(stops[rhs].coord.x - xy.x,2) + pow(stops[rhs].coord.y - xy.y,2);
        if (dist1 < dist2)  {return true;}
        else if (dist1 > dist2) {return false;}
        else {return stops[lhs].coord.y < stops[rhs].coord.y; }
    }

    inline bool CoordCmp (const BusStop &lhs, const BusStop &rhs) {
        if (lhs.dist < rhs.dist) {return true;}
        else if (lhs.dist > rhs.dist) {return false;}
        else {return lhs.coord.y < rhs.coord.y; }
    }
    std::vector<StopID> mergeSort(std::vector<StopID>&v1, std::vector<StopID>&v2, bool(Datastructures::*func)(const StopID&,const StopID&)) {
        std::vector<StopID> stops;
        int n1 = int(v1.size());
        int n2 = int(v2.size());
        int i=0;
        int j=0;
        while (i<n1 && j < n2) {
            if ((this->*func)(v1[i],v2[j])) {
                stops.push_back(v1[i]);
                i++;
            }
            else {
                stops.push_back(v2[j]);
                j++;
            }
        }

        while (i<n1) {
            stops.push_back(v1[i]);
            i++;
        }
        while (j <n2) {
            stops.push_back(v2[j]);
            j++;
        }
        return stops;
    }



};

#endif // DATASTRUCTURES_HH
