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

#include <limits>

const double INF_DOUBLE = std::numeric_limits<double>::max();
const Time INF_TIME = std::numeric_limits<Time>::max();
const int INF = std::numeric_limits<int>::max();


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
    all_stop.clear();
    clear_routes();
    V=0;
    sorted_ts = false;

}

std::vector<StopID> Datastructures::all_stops()
{
    // Replace this comment and the line below with your implementation
    return all_stop;
}

bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    if (stops_main.find(id) == stops_main.end()) {
        stops_main[id] = {id,V,name,xy,{},{},{}};
        all_stop.push_back(id);
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
    if (routes_main.size()==0) {
        return {};
    }
    std::vector<RouteID> all_route;
    for (auto r:routes_main) {
        all_route.push_back(r.first);
    }
    return all_route;
}

void Datastructures::addEdge(int u, int v)
{
    auto it1 = std::find(adjacency_list[u].begin(),adjacency_list[u].end(),v);
    auto it2 = std::find(adj_list_back[v].begin(),adj_list_back[v].end(),u);
    if (it1 == adjacency_list[u].end())
        adjacency_list[u].push_back(v);
    if (it2 == adj_list_back[v].end())
    adj_list_back[v].push_back(u);

};
bool Datastructures::add_route(RouteID id, std::vector<StopID> stops)
{
    // Replace this comment and the line below with your implementation
    if (routes_main.find(id) != routes_main.end())
        return false;
    if (stops.size() == 1)
        return false;
    sorted_ts = false;

    std::vector<RouteStop> stop_info;
    int stop_counts = int(stops.size());

    for (int i=0; i < stop_counts;i++) {
        if (stops_main.find(stops[i]) == stops_main.end())
             return false;
        RouteStop route_stop = {id,stops[i],i,{}};
        stop_info.push_back(route_stop);
        stops_main[stops[i]].routes.push_back(route_stop);

        if (i != stop_counts-1) {
            stops_main[stops[i]].next_stop.insert(std::pair(stops[i+1],id));
            int src = stops_main[stops[i]].int_id;
            int dest = stops_main[stops[i+1]].int_id;
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
    if (routes_main.size() == 0)
        return {};

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
        stops_main[i].trips.clear();
    }

    routes_main.clear();
    connections.clear();
    adjacency_list.clear();
    adj_list_back.clear();
    sorted_ts = false;

    adjacency_list.resize(V);
    adj_list_back.resize(V);
    connection_routes.clear();
    connection_count = 0;
}



void Datastructures::A_star(std::priority_queue<AstarNode, std::vector<AstarNode>, compare> *queue, StopID tgt,
                            std::vector<std::vector<int> > &adj_list, std::vector<Astar_dist>&node, int multiplier)
{
    AstarNode current = queue->top();
    queue->pop();

    node[current.id].visited = true;
    if (current.id == stops_main[tgt].int_id) {
        return;
    }
    for (auto i=adj_list[current.id].begin(); i!=adj_list[current.id].end(); i++) {
        if (!node[*i].visited) {
            int parent = current.id;
            Coord coord_i = stops_main[all_stop[*i]].coord;
            Coord coord_p = stops_main[all_stop[parent]].coord;
            double dst_h = int(eucl_distance(coord_i,stops_main[tgt].coord) / multiplier);
            double dst_g =  node[parent].dist_g + eucl_distance(coord_p,coord_i);
            if (node[*i].dist_f > dst_g+dst_h){
                node[*i].dist_f = dst_g+dst_h;
                node[*i].dist_g = dst_g;
                node[*i].s_parent = current.id;
            }
            queue->push({*i,dst_g+dst_h,0});
        }
    }

}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_any(StopID fromstop, StopID tostop)
{
    // Replace this comment and the line below with your implementation
    if (stops_main.find(fromstop) == stops_main.end() || stops_main.find(tostop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    else {
        int s = stops_main[fromstop].int_id;
        int t = stops_main[tostop].int_id;
        std::priority_queue<AstarNode, vector<AstarNode>, compare> s_queue;
        std::priority_queue<AstarNode, vector<AstarNode>, compare> t_queue;

        std::vector<std::tuple<StopID, RouteID, Distance>>journey;

        Coord src = stops_main[s].coord;
        Coord tgt = stops_main[t].coord;
        std::vector<Astar_dist> s_nodes;
        std::vector<Astar_dist> t_nodes;
        s_nodes.assign(V, {0,INF_DOUBLE,-1,false});
        t_nodes.assign(V, {0,INF_DOUBLE,-1,false});

        int multiplier = 1;
        s_queue.push({s, eucl_distance(src,tgt) / multiplier,0});
        s_nodes[s].visited = true;
        s_nodes[s].s_parent = -1;

        t_queue.push({t,eucl_distance(src,tgt) / multiplier,0});
        t_nodes[t].visited = true;
        t_nodes[t].s_parent = -1;

        int intersection = -1;

        while (!s_queue.empty() && !t_queue.empty()) {
            A_star(&s_queue,tostop, adjacency_list, s_nodes,multiplier);
            A_star(&t_queue,fromstop, adj_list_back, t_nodes,multiplier);
            intersection = intersection_node(s_nodes, t_nodes);

            if (intersection != -1) {
                RouteID routeId;
                StopID curr,par;
                int i = intersection;
                int temp;
                while (i != s) {
                    temp = s_nodes[i].s_parent;
                    curr = all_stop[i];
                    par = all_stop[temp];
                    routeId = stops_main[par].next_stop[curr];
                    journey.push_back({all_stop[temp],routeId,s_nodes[temp].dist_g});
                    i = s_nodes[i].s_parent;
                }
                std::reverse(journey.begin(),journey.end());

                int dist = s_nodes[intersection].dist_g;

                i = intersection;
                while (i != -1) {
                    temp = t_nodes[i].s_parent;

                    curr = all_stop[i];

                    par = all_stop[temp];
                    if (t_nodes[i].s_parent == -1) {
                        routeId = NO_ROUTE;
                    } else {
                        routeId = stops_main[curr].next_stop[par];
                    }
                    journey.push_back({curr,routeId,dist});
                    dist += calculateDistance(curr,par);

                    i = t_nodes[i].s_parent;
                }
                break;
            }

        }
       return journey;
    }

}

void Datastructures::Astar_leaststop(std::priority_queue<AstarNode, std::vector<AstarNode>, compare_leaststop> *queue, StopID tgt,
                                     std::vector<std::vector<int>> &adj_list, std::vector<Astar_stops>& node)
{
    AstarNode current = queue->top();
    queue->pop();
    if (current.id == stops_main[tgt].int_id) {
        return;
    }
    node[current.id].visited = true;
    for (auto i=adj_list[current.id].begin(); i!=adj_list[current.id].end(); i++) {
        if (!node[*i].visited) {
            int parent = current.id;
            double dst_f = calculateDistance(tgt,all_stop[*i]);
            double dst_g = node[parent].dist_g + calculateDistance(all_stop[parent],all_stop[*i]);

            int nodes = node[parent].num_nodes + 1;

            if (node[*i].num_nodes > nodes ) {
                node[*i].num_nodes = nodes;
                node[*i].dist_g = dst_g;
                node[*i].s_parent = parent;
            }
            queue->push({*i,dst_f,nodes});
        }
    }
}

std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_least_stops(StopID fromstop, StopID tostop)
{
    if (stops_main.find(fromstop) == stops_main.end() || stops_main.find(tostop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    else {
        int s = stops_main[fromstop].int_id;
        int t = stops_main[tostop].int_id;
        std::priority_queue<AstarNode, vector<AstarNode>, compare_leaststop> s_queue;
        std::priority_queue<AstarNode, vector<AstarNode>, compare_leaststop> t_queue;

        std::vector<std::tuple<StopID, RouteID, Distance>>journey;

        std::vector<Astar_stops> s_nodes;
        std::vector<Astar_stops> t_nodes;

        s_nodes.assign(V, {INF_DOUBLE,-1,INF, false});
        t_nodes.assign(V, {INF_DOUBLE,-1,INF,false});
        s_queue.push({s, 0,0});
        s_nodes[s].visited = true;
        s_nodes[s].s_parent = -1;
        s_nodes[s].num_nodes = 0;
        s_nodes[s].dist_g = 0;

        t_queue.push({t,0,0});
        t_nodes[t].visited = true;
        t_nodes[t].s_parent = -1;
        t_nodes[t].num_nodes = 0;
        t_nodes[t].dist_g = 0;


        int intersection = -1;

        while (!s_queue.empty() && !t_queue.empty()) {
            Astar_leaststop(&s_queue,tostop, adjacency_list, s_nodes);
            Astar_leaststop(&t_queue,fromstop, adj_list_back, t_nodes);
            intersection = intersection_node(s_nodes, t_nodes);

            if (intersection != -1) {
                RouteID routeId;
                StopID curr,par;
                int i = intersection;
                int temp;
                while (i != s) {
                    temp = s_nodes[i].s_parent;
                    curr = all_stop[i];
                    par = all_stop[temp];
                    routeId = stops_main[par].next_stop[curr];
                    journey.push_back({all_stop[temp],routeId,s_nodes[temp].dist_g});
                    i = s_nodes[i].s_parent;
                }
                std::reverse(journey.begin(),journey.end());

                int dist = s_nodes[intersection].dist_g;

                i = intersection;
                while (i != -1) {
                    temp = t_nodes[i].s_parent;

                    curr = all_stop[i];

                    par = all_stop[temp];
                    if (t_nodes[i].s_parent == -1) {
                        routeId = NO_ROUTE;
                    } else {
                        routeId = stops_main[curr].next_stop[par];
                    }
                    journey.push_back({curr,routeId,dist});
                    dist += calculateDistance(curr,par);

                    i = t_nodes[i].s_parent;
                }
                break;
            }

        }
       return journey;
    }

}
void Datastructures::DFS(list<int>*queue, std::vector<Graph>& nodes, int& found, int&found_parent, int& final_dist) {

    if (found != -1) {
        return;
    } else {
        int current = queue->back();
        queue->pop_back();
        nodes[current].visited = true;

        if (adjacency_list[current].size() >0 ) {
            for (auto i = adjacency_list[current].begin();i != adjacency_list[current].end();++i) {
                if (!nodes[*i].visited) {
                    queue->push_back(*i);
                    nodes[*i].s_parent = current;
                    nodes[*i].dist_g = nodes[current].dist_g + calculateDistance(all_stop[*i],all_stop[current]);
                    DFS(queue,nodes,found,found_parent, final_dist);
                } else {
                    found = *i;
                    found_parent = current;
                    final_dist = nodes[found_parent].dist_g + calculateDistance(all_stop[found_parent],all_stop[found]);

                    break;
                }
            }
        }
    }
}


std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_with_cycle(StopID fromstop)
{
    // Replace this comment and the line below with your implementation
    if (stops_main.find(fromstop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    else {
        list<int> s_queue;
        std::vector<Graph> nodes;
        nodes.assign(V, {INF_DOUBLE,-1,false});
//        for(int i=0; i<V; i++)
//        {
//            nodes[i].visited = false;
//            nodes[i].dist_g = 0;
//        }

        int s = stops_main[fromstop].int_id;
        nodes[s].visited = true;
        s_queue.push_back(s);
        nodes[s].s_parent= -1;
        nodes[s].dist_g= 0;

        int final_dist = -1;
        int found = -1;
        int found_parent = -1;
        std::vector<std::tuple<StopID, RouteID, Distance>>journey;

        // searching for path using DFS
        while (!s_queue.empty()) {
            DFS(&s_queue, nodes, found,found_parent, final_dist);
            if (found != -1) {
                break;
            }
        }
        if (found != -1) {
            StopID current = all_stop[found];
            journey.push_back({current,NO_ROUTE,final_dist});
            int i = found_parent;
            int next = found;
            RouteID routeId;
            while (i != nodes[s].s_parent) {
                current = all_stop[i];
                routeId = stops_main[current].next_stop[all_stop[next]];
                journey.push_back({current,routeId,nodes[i].dist_g});
                next = i;

                i = nodes[i].s_parent;
            }
        }
        std::reverse(journey.begin(),journey.end());
        return journey;
    }
}


std::vector<std::tuple<StopID, RouteID, Distance>> Datastructures::journey_shortest_distance(StopID fromstop, StopID tostop)
{
    // Replace this comment and the line below with your implementation
    if (stops_main.find(fromstop) == stops_main.end() || stops_main.find(tostop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_DISTANCE}};
    else {
        int s = stops_main[fromstop].int_id;
        int t = stops_main[tostop].int_id;
        std::priority_queue<AstarNode, vector<AstarNode>, compare> s_queue;
        std::priority_queue<AstarNode, vector<AstarNode>, compare> t_queue;

        std::vector<std::tuple<StopID, RouteID, Distance>>journey;

        Coord src = stops_main[s].coord;
        Coord tgt = stops_main[t].coord;
        std::vector<Astar_dist> s_nodes;
        std::vector<Astar_dist> t_nodes;
        s_nodes.assign(V, {0,INF_DOUBLE,-1,false});
        t_nodes.assign(V, {0,INF_DOUBLE,-1,false});

        int multiplier = 3;
        s_queue.push({s, eucl_distance(src,tgt) / multiplier,0});
        s_nodes[s].visited = true;
        s_nodes[s].s_parent = -1;

        t_queue.push({t,eucl_distance(src,tgt) / multiplier,0});
        t_nodes[t].visited = true;
        t_nodes[t].s_parent = -1;

        int intersection = -1;

        while (!s_queue.empty() && !t_queue.empty()) {
            A_star(&s_queue,tostop, adjacency_list, s_nodes,multiplier);
            A_star(&t_queue,fromstop, adj_list_back, t_nodes,multiplier);
            intersection = intersection_node(s_nodes, t_nodes);

            if (intersection != -1) {
                RouteID routeId;
                StopID curr,par;
                int i = intersection;
                int temp;
                while (i != s) {
                    temp = s_nodes[i].s_parent;
                    curr = all_stop[i];
                    par = all_stop[temp];
                    routeId = stops_main[par].next_stop[curr];
                    journey.push_back({all_stop[temp],routeId,s_nodes[temp].dist_g});
                    i = s_nodes[i].s_parent;
                }
                std::reverse(journey.begin(),journey.end());

                int dist = s_nodes[intersection].dist_g;

                i = intersection;
                while (i != -1) {
                    temp = t_nodes[i].s_parent;

                    curr = all_stop[i];

                    par = all_stop[temp];
                    if (t_nodes[i].s_parent == -1) {
                        routeId = NO_ROUTE;
                    } else {
                        routeId = stops_main[curr].next_stop[par];
                    }
                    journey.push_back({curr,routeId,dist});
                    dist += calculateDistance(curr,par);

                    i = t_nodes[i].s_parent;
                }
                break;
            }

        }
       return journey;
    }

}

bool Datastructures::add_trip(RouteID routeid, std::vector<Time> const& stop_times)
{
    // Replace this comment and the line below with your implementation
    if (routes_main.find(routeid) == routes_main.end())
        return false;
    else {
        sorted_ts = false;
        std::vector<RouteStop>&route_stop = routes_main[routeid];
        for (int i = 0; i < int(stop_times.size()); i++) {
            StopID stopid = route_stop[i].stopId;
            Duration duration;
            int next_stop_int;
            StopID next_stop;
            Time arrival_time;
            if (i == int(stop_times.size())-1) {

                duration = 0;
                next_stop_int = -1;
                arrival_time = 0;
                next_stop = -1;
            }
            else {

                arrival_time = stop_times[i+1];
                duration = arrival_time - stop_times[i];
                next_stop_int = stops_main[route_stop[i+1].stopId].int_id;
                next_stop = route_stop[i+1].stopId;
            }

            stops_main[stopid].trips.push_back(std::tuple(stop_times[i],duration,next_stop));

            route_stop[i].trips.push_back(std::pair(stop_times[i],duration));
            connections.push_back({stops_main[stopid].int_id,next_stop_int,stop_times[i],arrival_time,connection_count});
            connection_routes.push_back(route_stop[i].routeId);
            connection_count += 1;
        }
        return true;
    }
}

void Datastructures::connection_scan(std::vector<int>&track_connections, std::vector<Time>&earliest_arrival, int tgt, std::vector<Connection>& all_connections) {

    Time earliest = INF_TIME;
    for (size_t i = 0; i < all_connections.size(); i++) {
        Connection connection = all_connections[i];

        if (connection.src_ts >= earliest_arrival[connection.src] &&
            connection.tgt_ts < earliest_arrival[connection.tgt]  ) {
            earliest_arrival[connection.tgt] = connection.tgt_ts;
            track_connections[connection.tgt] = i;

            if (connection.tgt == tgt) {
                earliest = std::min(earliest,connection.tgt_ts);
            } else if (connection.tgt_ts > earliest) {
                return;
            }
        }
    }
}
std::vector<std::pair<Time, Duration>> Datastructures::route_times_from(RouteID routeid, StopID stopid)
{
    // Replace this comment and the line below with your implementation
    if (routes_main.find(routeid) == routes_main.end() || stops_main.find(stopid) == stops_main.end())
        return {{NO_TIME, NO_DURATION}};
    else {
        std::vector<RouteStop> route_stop = routes_main[routeid];
        std::vector<std::pair<Time, Duration>> route_time;
        auto it = std::find_if(
            route_stop.begin(),
            route_stop.end(),
            [stopid](const RouteStop& item) //you want to compare an item
                {return item.stopId == stopid; } );
        if (it == route_stop.end())
            return {{NO_TIME, NO_DURATION}};
        else {
            route_time.insert(route_time.end(),it->trips.begin(),it->trips.end());
        }
        return route_time;
    }
}

std::vector<std::tuple<StopID, RouteID, Time> > Datastructures::journey_earliest_arrival(StopID fromstop, StopID tostop, Time starttime)
{
    // Replace this comment and the line below with your implementation
    if (stops_main.find(fromstop) == stops_main.end() || stops_main.find(tostop) == stops_main.end())
        return {{NO_STOP, NO_ROUTE, NO_TIME}};
    else {
        if (!sorted_ts) {
            std::sort(connections.begin(),connections.end(),
                      [&] (const Connection& lhs, const Connection& rhs) { return sortConnection(lhs,rhs); });
            sorted_ts = true;
        }

        auto it = std::upper_bound(connections.begin(), connections.end(), starttime,
                         [](Time lhs, const Connection& rhs) {
            return lhs <= rhs.src_ts;
        });

        std::vector<Connection> new_connections(it, connections.end());

        std::vector<std::tuple<StopID, RouteID, Time> > journey;

        std::vector<int> track_connections;
        std::vector<Time> earliest_arrival;

        track_connections.assign(V,INF);
        earliest_arrival.assign(V,INF);
        int s = stops_main[fromstop].int_id;
        int t = stops_main[tostop].int_id;

        earliest_arrival[s] = starttime;
        connection_scan(track_connections,earliest_arrival,t, new_connections);
        if (track_connections[t] != INF) {
            int i = track_connections[t];

            StopID src_stop = all_stop[new_connections[i].src];
            StopID tgt_stop = all_stop[new_connections[i].tgt];

            RouteID routeId;
            journey.push_back({tgt_stop,NO_ROUTE,new_connections[i].tgt_ts});
            while (i != INF) {
                Connection c = new_connections[i];
                src_stop = all_stop[c.src];
                tgt_stop = all_stop[c.tgt];

                //routeId = stops_main[src_stop].next_stop[tgt_stop];
                routeId = connection_routes[c.route_track];
                journey.push_back({src_stop,routeId,new_connections[i].src_ts});
                i = track_connections[c.src];
            }

            std::reverse(journey.begin(),journey.end());
        }
        return journey;
    }
}

void Datastructures::add_walking_connections()
{
    // Replace this comment and the line below with your implementation
}

