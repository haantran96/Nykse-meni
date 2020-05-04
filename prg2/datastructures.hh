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
#include <bits/stdc++.h>
#include <functional>
#include <tuple>
#include <queue>
using namespace std;

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>

// Types for IDs
using StopID = long int;
using RegionID = std::string;
using RouteID = std::string;
using Name = std::string;

// Return values for cases where required thing was not found
RouteID const NO_ROUTE = "!!NO_ROUTE!!";
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

// Type for time of day in minutes from midnight (i.e., 60*hours + minutes)
using Time = int;

// Return value for cases where color was not found
Time const NO_TIME = std::numeric_limits<Time>::min();

// Type for a duration of time (in minutes)
using Duration = int;

// Return value for cases where Duration is unknown
Duration const NO_DURATION = NO_VALUE;

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Duration is unknown
Distance const NO_DISTANCE = NO_VALUE;



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

    // Phase 2 operations

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<RouteID> all_routes();

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_route(RouteID id, std::vector<StopID> stops);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::pair<RouteID, StopID>> routes_from(StopID stopid);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<StopID> route_stops(RouteID id);

    // Estimate of performance:
    // Short rationale for estimate:
    void clear_routes();

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_any(StopID fromstop, StopID tostop);

//    // Non-compulsory operations

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_least_stops(StopID fromstop, StopID tostop);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_with_cycle(StopID fromstop);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<StopID, RouteID, Distance>> journey_shortest_distance(StopID fromstop, StopID tostop);

    // Estimate of performance:
    // Short rationale for estimate:
    bool add_trip(RouteID routeid, const std::vector<Time> &stop_times);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::pair<Time, Duration> > route_times_from(RouteID routeid, StopID stopid);

    // Estimate of performance:
    // Short rationale for estimate:
    std::vector<std::tuple<StopID, RouteID, Time>> journey_earliest_arrival(StopID fromstop, StopID tostop, Time starttime);

    // Estimate of performance:
    // Short rationale for estimate:
    void add_walking_connections(); // Note! This method is completely optional, and not part of any testing

private:
    // Add stuff needed for your class implementation here

    struct RouteStop {
        RouteID routeId;
        StopID stopId;
        int index;
        std::vector<std::pair<Time,Duration>>trips;
    };

    // Stop's information
    struct BusStop {
        StopID id;
        int int_id;
        Name name;
        Coord coord;
        std::vector<RouteStop> routes;
        std::unordered_map<StopID,RouteID> next_stop;
        std::vector<std::tuple<Time,Duration,StopID>>trips;
    };

    // Region's information. This is similar to sibling-child tree representation, but adding parent node
    std::unordered_map<StopID,BusStop> stops_main;
    std::vector<StopID> all_stop;

    Distance calculateDistance(StopID lhs, StopID rhs) {
        double dist = pow(stops_main[lhs].coord.x - stops_main[rhs].coord.x,2) + pow(stops_main[lhs].coord.y - stops_main[rhs].coord.y,2);
        return int(sqrt(dist));
    }

    double eucl_distance(Coord lhs, Coord rhs) {
        double dist = pow(lhs.x - rhs.x,2) + pow(lhs.y - rhs.y,2);
        return int(sqrt(dist));
    }

    struct AstarNode {
        int id;
        double dist_f;
        int num_nodes;
    };
    struct Astar_dist {
        double dist_g;
        double dist_f;
        int s_parent;
        bool visited;
    };
    struct Astar_stops {
        double dist_g;
        int s_parent;
        int num_nodes;
        bool visited;
    };


    struct Graph {
        Distance dist_g;
        int s_parent;
        bool visited;
    };

    struct compare{
        bool operator() (const AstarNode& lhs,const AstarNode& rhs ){
             return (lhs.dist_f > rhs.dist_f);
        }
    };

    struct compare_leaststop{
        bool operator() (const AstarNode& lhs,const AstarNode& rhs ){
          if (lhs.num_nodes > rhs.num_nodes) {return true;}
             else if (lhs.num_nodes < rhs.num_nodes) { return false;}
             else {return lhs.dist_f > rhs.dist_f;}
        }
    };

    struct Connection {
        int src;
        int tgt;
        Time src_ts;
        Time tgt_ts;
        //RouteID routeId;
        int route_track;
    };

    std::vector<Connection> connections;

    std::unordered_map<RouteID,std::vector<RouteStop>>routes_main;
    int V = 0;
    int connection_count = 0;


    std::vector<std::vector<int>> adjacency_list;
    std::vector<std::vector<int>> adj_list_back;

    std::vector<RouteID>connection_routes;
    void addEdge(int u, int v);

    void DFS(list<int>*queue, std::vector<Graph>& nodes, int& found,int&found_parent, int& final_dist);

    inline bool sortConnection(const Connection &lhs, const Connection &rhs) {
       return lhs.src_ts < rhs.src_ts;
    }

    void A_star(std::priority_queue<AstarNode, std::vector<AstarNode>, compare> *queue, StopID tgt,
                std::vector<std::vector<int>> &adj_list, std::vector<Astar_dist>& node, int multiplier);

    void Astar_leaststop(std::priority_queue<AstarNode, std::vector<AstarNode>, compare_leaststop> *queue, StopID tgt,
                        std::vector<std::vector<int>> &adj_list, std::vector<Astar_stops>& node);

    void connection_scan(std::vector<int>&track_connections, std::vector<Time>&earliest_arrival, int tgt,std::vector<Connection>& all_connections);
    bool sorted_ts = false;

    template <class AstarType>
    int intersection_node(std::vector<AstarType>& s_nodes, std::vector<AstarType>& t_nodes)
    {
        for (int i=0; i< V; i++) {
            if (s_nodes[i].visited && t_nodes[i].visited) {
                return i;
            }
        }
        return -1;
    }

};

#endif // DATASTRUCTURES_HH
