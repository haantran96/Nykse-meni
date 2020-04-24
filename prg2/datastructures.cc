// Datastructures.cc

#include "datastructures.hh"

#include <random>
#include <cmath>
#include <stdexcept>
#include <set>
#include <algorithm>
#include <iostream>
#include <random>
#include <cmath>
#include <functional>



std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{
    // Replace this comment with your implementation 
}

Datastructures::~Datastructures()
{
    // Replace this comment with your implementation
}

int Datastructures::stop_count()
{
    // Replace this comment and the line below with your implementation
    return stops_main.size();
}

void Datastructures::clear_all()
{
    // Replace this comment with your implementation
    stops_main.clear();
    regions.clear();
    allRegions.clear();
    all_stop.clear();
    clear_routes();
    adjacency_list.clear();
    adj_list_back.clear();
    V=0;
}

std::vector<StopID> Datastructures::all_stops()
{
    // Replace this comment and the line below with your implementation
    return all_stop;
}

bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    double dist = pow(xy.x,2)+pow(xy.y,2);
    if (stops_main.find(id) == stops_main.end()) {
        stops_main[id].id = id;
        stops_main[id].name = name;
        stops_main[id].coord = xy;
        stops_main[id].dist = dist;
        all_stop.push_back(id);
        vertex_id[id] = V;
        V += 1;
        adjacency_list.resize(V);
        adj_list_back.resize(V);
        return true;
    }
    else
        return false;
}

Name Datastructures::get_stop_name(StopID id)
{
    if (stops_main.find(id) == stops_main.end()) {
        return NO_NAME;
    } else {
        return stops_main[id].name;
    }

}

Coord Datastructures::get_stop_coord(StopID id)
{
    if (stops_main.find(id) == stops_main.end())
        return NO_COORD;
    return stops_main[id].coord;
}

std::vector<StopID> Datastructures::stops_alphabetically()
{
    // Sort stop by name
    return {NO_STOP};
}
std::vector<StopID> Datastructures::stops_coord_order()
{
    return {NO_STOP};
}

StopID Datastructures::min_coord()
{
    return NO_STOP;
}

StopID Datastructures::max_coord()
{
    return NO_STOP;
}

std::vector<StopID> Datastructures::find_stops(Name const& name)
{
    return {NO_STOP};
}

bool Datastructures::change_stop_name(StopID id, const Name& newname)
{
    return false;
}

bool Datastructures::change_stop_coord(StopID id, Coord newcoord)
{
    return false;
}

bool Datastructures::add_region(RegionID id, const Name &name)
{
     return false;
}

Name Datastructures::get_region_name(RegionID id)
{
    return NO_NAME;
}

std::vector<RegionID> Datastructures::all_regions()
{
    // Replace this comment and the line below with your implementation
    return {NO_REGION};
}

bool Datastructures::add_stop_to_region(StopID id, RegionID parentid)
{
    return false;
}


bool Datastructures::add_subregion_to_region(RegionID id, RegionID parentid)
{
    return false;
}


std::vector<RegionID> Datastructures::stop_regions(StopID id)
{
    return {NO_REGION};
}

void Datastructures::creation_finished()
{
    // Replace this comment with your implementation
    // You don't have to use this method for anything, if you don't need it

}

std::pair<Coord,Coord> Datastructures::region_bounding_box(RegionID id)
{
    return {NO_COORD, NO_COORD};
}


std::vector<StopID> Datastructures::stops_closest_to(StopID id)
{
    return {NO_STOP};
}

bool Datastructures::remove_stop(StopID id)
{
    return false;
}

RegionID Datastructures::stops_common_region(StopID id1, StopID id2)
{
    return {NO_REGION};
}

std::vector<RouteID> Datastructures::all_routes()
{
    // Replace this comment and the line below with your implementation
    std::vector<RouteID> all_route;
    for (auto r:routes_main) {
        all_route.push_back(r.first);
    }
    return all_route;
}

void Datastructures::addEdge(int u, int v)
{
//    auto it1 = std::find(adjacency_list[u].begin(),adjacency_list[u].end(),v);
//    auto it2 = std::find(adjacency_list[v].begin(),adjacency_list[v].end(),u);
//    if (it1 == adjacency_list[u].end())
      adjacency_list[u].push_back(v);
//    if (it2 == adjacency_list[v].end())
      adj_list_back[v].push_back(u);
};
bool Datastructures::add_route(RouteID id, std::vector<StopID> stops)
{   
    // Replace this comment and the line below with your implementation
    if (routes_main.find(id) != routes_main.end())
        return false;
    if (stops.size() == 1)
        return false;
    std::vector<RouteStop> stop_info;
    int stop_counts = int(stops.size());
    for (int i=0; i < stop_counts;i++) {
        if (stops_main.find(stops[i]) == stops_main.end())
             return false;
        RouteStop route_stop = {id,stops[i],i};
        stop_info.push_back(route_stop);
        stops_main[stops[i]].routes.push_back(route_stop);

        if (i != stop_counts-1) {
            int src = vertex_id[stops[i]];
            int dest = vertex_id[stops[i+1]];
            addEdge(src,dest);
        }
    }
    routes_main[id] = stop_info;
    return true;
}

std::vector<std::pair<RouteID, StopID>> Datastructures::routes_from(StopID stopid)
{
    // Replace this comment and the line below with your implementation
    if (routes_main.size() == 0)
        return {};
    std::vector<RouteStop>routes = stops_main[stopid].routes;
    std::vector<std::pair<RouteID, StopID>> route_stop;
    for (auto r:routes) {
        if (r.index < int(routes_main[r.routeId].size())-1) {
            int next_stop = r.index+1;
            route_stop.push_back({r.routeId,routes_main[r.routeId].at(next_stop).stopId});
        }
    }
    return route_stop;
}

std::vector<StopID> Datastructures::route_stops(RouteID id)
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> route_stop;
    if (routes_main.find(id) == routes_main.end())
        return {NO_STOP};
    else {
        for (auto r:routes_main[id])
            route_stop.push_back(r.stopId);
        return route_stop;
    }
}

void Datastructures::clear_routes()
{
    // Replace this comment and the line below with your implementation
    for (int i=0; i < int(stops_main.size());i++) {
        stops_main[i].routes.clear();
    }

    routes_main.clear();
    //adjacency_list.clear();
}
void Datastructures::BFS(list<int> *queue, bool *visited,
                                    int *parent, vector<vector<int>> adjList)
{
    int current = queue->front();
    queue->pop_front();
    //vector<vector<int>>::iterator i;
    for (auto i=adjList[current].begin();i != adjList[current].end();i++)
    {
        // If adjacent vertex is not visited earlier
        // mark it visited by assigning true value
        if (!visited[*i])
        {
            // set current as parent of this vertex
            parent[*i] = current;

            // Mark this vertex visited
            visited[*i] = true;

            // Push to the end of queue
            queue->push_back(*i);
        }
    }
};

// check for intersecting vertex
int Datastructures::isIntersecting(bool *s_visited, bool *t_visited)
{
    int intersectNode = -1;
    for(int i=0;i<V;i++)
    {
        // if a vertex is visited by both front
        // and back BFS search return that node
        // else return -1
        if(s_visited[i] && t_visited[i])
            return i;
    }
    return intersectNode;
};

// Print the path from source to target
std::vector<int> Datastructures::printPath(int *s_parent, int *t_parent,
                  int s, int t, int intersectNode)
{
    vector<int> path;
    path.push_back(intersectNode);
    int i = intersectNode;
    while (i != s)
    {
        path.push_back(s_parent[i]);
        i = s_parent[i];
    }
    reverse(path.begin(), path.end());
    i = intersectNode;
    while(i != t)
    {
        path.push_back(t_parent[i]);
        i = t_parent[i];
    }
    return path;
};

// Method for bidirectional searching
vector<int> Datastructures::biDirSearch(int s, int t)
{
    // boolean array for BFS started from
    // source and target(front and backward BFS)
    // for keeping track on visited nodes
    bool s_visited[V], t_visited[V];
    vector<int>path;
    // Keep track on parents of nodes
    // for front and backward search
    int s_parent[V], t_parent[V];

    // queue for front and backward search
    list<int> s_queue, t_queue;

    int intersectNode = -1;

    // necessary initialization
    for(int i=0; i<V; i++)
    {
        s_visited[i] = false;
        t_visited[i] = false;
    }

    s_queue.push_back(s);
    s_visited[s] = true;

    // parent of source is set to -1
    s_parent[s]=-1;

    t_queue.push_back(t);
    t_visited[t] = true;

    // parent of target is set to -1
    t_parent[t] = -1;

    while (!s_queue.empty() && !t_queue.empty())
    {
        // Do BFS from source and target vertices
        BFS(&s_queue, s_visited, s_parent,adjacency_list);
        BFS(&t_queue, t_visited, t_parent,adj_list_back);

        // check for intersecting vertex
        intersectNode = isIntersecting(s_visited, t_visited);

        // If intersecting vertex is found
        // that means there exist a path
        if(intersectNode != -1)
        {
            // print the path and exit the program
            path = printPath(s_parent, t_parent, s, t, intersectNode);
        }
    }
    return path;
}
std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_any(StopID fromstop, StopID tostop)
{
    // Replace this comment and the line below with your implementation
    if (stops_main.find(fromstop) == stops_main.end() || stops_main.find(tostop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    else {
        std::vector<bool>discovered(V,false);
        int src = vertex_id[fromstop];
        int dest = vertex_id[tostop];
        queue<int> q;
        vector<int> path;

        std::vector<std::tuple<StopID, RouteID, Distance>>journey;
        path = biDirSearch(src,dest);
        if (path.size()>0) {
            int dist = 0;
            StopID temp = all_stop.at(path[0]);
            for (auto i:path) {
                dist += calculateDistance(all_stop.at(i),temp);
                temp = all_stop.at(i);
                journey.push_back({all_stop.at(i),NO_ROUTE,dist});
            }
        }
        return journey;
    }
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_least_stops(StopID fromstop, StopID tostop)
{
    // Replace this comment and the line below with your implementation
    return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    journey_any(fromstop,tostop);
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_with_cycle(StopID fromstop)
{
    // Replace this comment and the line below with your implementation
    return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_shortest_distance(StopID fromstop, StopID tostop)
{
    // Replace this comment and the line below with your implementation
    return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
}

bool Datastructures::add_trip(RouteID routeid, std::vector<Time> const& stop_times)
{
    // Replace this comment and the line below with your implementation
    return false;
}

std::vector<std::pair<Time, Duration>> Datastructures::route_times_from(RouteID routeid, StopID stopid)
{
    // Replace this comment and the line below with your implementation
    return {{NO_TIME, NO_DURATION}};
}

std::vector<std::tuple<StopID, RouteID, Time> > Datastructures::journey_earliest_arrival(StopID fromstop, StopID tostop, Time starttime)
{
    // Replace this comment and the line below with your implementation
    return {{NO_STOP, NO_ROUTE, NO_TIME}};
}

void Datastructures::add_walking_connections()
{
    // Replace this comment and the line below with your implementation
}
