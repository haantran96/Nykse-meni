## datastructures.hh

**Note**: In my implementation, the bus will have 2 IDs: the given StopID from the program, and an Integer ID. The Integer ID is determined by the time a stop is added by calling add_stop (i.e: if the stop is the 10th stop added by the program, Integer ID = 10). The reason for Integer ID is for easier implementation of my adjacency list, which stores stops, and linked stops to them (similar to a Hash Table)
    
    struct BusStop {
        StopID id;
        int int_id;
        Name name;
        Coord coord;
        std::vector<RouteStop> routes;
        std::unordered_map<StopID,RouteID> next_stop;
        std::vector<std::tuple<Time,Duration,StopID>>trips;
    };
    --> Store information of a Stop (StopID, Integer Id, Name, Coordinates, Routes, Infromation of Next Stop, Trips)
    
    struct RouteStop {
        RouteID routeId;
        StopID stopId;
        int index;
        std::vector<std::pair<Time,Duration>>trips;
    }; 
    --> Link a Route and Stop. It stores a RouteID, StopID, the position of a stop in a route (index) and related trips. To clarify about variable index: if add_route A 1 3 4, then index of 1 is 0, 3 is 1 and 4 is 2.
    
    struct Graph {
        Distance dist_g;
        int s_parent;
        bool visited;
    };
    --> Implemented in journey_with_cylcle. 
        dist_g: Aggregated distance of a stop from the source stop
        s_parent: parent stop of a given stop.
        visited: check if a stop is visited by the program
    
    struct AstarNode {
        int id;
        double dist_f;
        int num_nodes;
    };
    --> Implemented in journey_with_cylcle: 
        dist_g: Aggregated distance of a stop from the source stop
        s_parent: parent stop of a given stop.
        visited: check if a stop is visited by the program
        
    struct Astar_dist {
        double dist_g;
        double dist_f;
        int s_parent;
        bool visited;
    };
    --> Implemented in journey_shortest_distance:
        dist_g: aggregated distance of a stop from the source stop
        dist_f: dist_g + dist_h (dist_h: heuristic distance to target - Euclidean distance)
        s_parent: parent stop of a given stop. 
        visited: check if a stop is visited by the program
    
    struct Astar_stops {
        double dist_g;
        int s_parent;
        int num_nodes;
        bool visited;
    };
    --> Implemented in journey_least_stops
        dist_g: aggregated distance of a stop from the source stop
        s_parent: parent stop of a given stop. 
        num_nodes: aggregated number of stops that a path passes     
        visited: check if a stop is visited by the program
        
    struct Connection {
        int src;
        int tgt;
        Time src_ts;
        Time tgt_ts;
        int route_track;
    };
    --> Implemented in journey_least_arrival:
        src, tgt: Integer ID of source and target stop
        src_ts, tgt_ts: Departure and arrival time of a trip
        route_track: integer ID of a connection. This is used to track RouteID of a certain trip.
        
    struct compare{
        bool operator() (const AstarNode& lhs,const AstarNode& rhs ){
             return (lhs.dist_f > rhs.dist_f);
        }
    };
    --> Comparator used in priority_queue for journey_shortest_distance and journey_any
    
    struct compare_leaststop{
        bool operator() (const AstarNode& lhs,const AstarNode& rhs ){
          if (lhs.num_nodes > rhs.num_nodes) {return true;}
             else if (lhs.num_nodes < rhs.num_nodes) { return false;}
             else {return lhs.dist_f > rhs.dist_f;}
        }
    };
    --> Comparator used in priority_queue for journey_least_stops
    
    inline bool sortConnection(const Connection &lhs, const Connection &rhs) {
       return lhs.src_ts < rhs.src_ts;
    }
    -->
    
    bool sorted_ts = false;
    
    int V = 0;
    int connection_count = 0;
    --> V: Interger ID of Stops
        connection_count: Integer ID of Connections
    
    std::unordered_map<StopID,BusStop> stops_main;
    --> unordered_map with StopID as key and BusStop struct as value -> Easy and fast to retrieve information with a given id
    
    std::vector<StopID> all_stop;
    --> vector that contains all the stop IDs. I use this to convert back from Integer ID to StopID
    
    std::unordered_map<RouteID,std::vector<RouteStop>>routes_main;
    --> unordered_map with RouteID as key and vector of RouteStop struct as value -> Easy and fast to retrieve information with a given id
    
    std::vector<std::vector<int>> adjacency_list;
    --> Adjacency list of Stops. This is similar to a hash table. For example: 1: [2, 3], 2: [3, 4].  
    
    
    std::vector<std::vector<int>> adj_list_back;
    --> Adjacency list of Stops, but backwards. (for bi-birectional search)
    
    std::vector<Connection> connections;
    --> vector that contain all the stop IDs. I use this to convert back from Integer ID to RouteID of a stops

    std::vector<RouteID>connection_routes;
    --> Storing routes of each connection

    void addEdge(int u, int v);
    --> adding Edge to adjacency_list and adj_list_back. u and v are integer ID of the StopIDs.
    
    void DFS(list<int>*queue, std::vector<Graph>& nodes, int& found,int&found_parent, int& final_dist);
    --> Depth first seach, used in journey_with_cycle. Time complexity: O(V+E) where V is number of vertices in the graph and E is number of edges in the graph
    
    void A_star(std::priority_queue<AstarNode, std::vector<AstarNode>, compare> *queue, StopID tgt,
                std::vector<std::vector<int>> &adj_list, std::vector<Astar_dist>& node, int multiplier);
    --> A star search, used in journey_with_distance. Time complexity: O(b^d), where b is the branching factor.
    
    void Astar_leaststop(std::priority_queue<AstarNode, std::vector<AstarNode>, compare_leaststop> *queue, StopID tgt,
                        std::vector<std::vector<int>> &adj_list, std::vector<Astar_stops>& node);
    --> A star search, used in journey_least_stops. Time complexity: O(b^d), where b is the branching factor.
    
    void connection_scan(std::vector<int>&track_connections, std::vector<Time>&earliest_arrival, int tgt,std::vector<Connection>& all_connections);
    --> Connection scan algorithm, used in journey_earliest_arrival.
    
    
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
    --> Because I implemented A* search bi-directionally, this function finds the intersection point of 2 searches from fromstop and from tostop 
    
    Distance calculateDistance(StopID lhs, StopID rhs) {
        double dist = pow(stops_main[lhs].coord.x - stops_main[rhs].coord.x,2) + pow(stops_main[lhs].coord.y - stops_main[rhs].coord.y,2);
        return int(sqrt(dist));
    }
    double eucl_distance(Coord lhs, Coord rhs) {
        double dist = pow(lhs.x - rhs.x,2) + pow(lhs.y - rhs.y,2);
        return int(sqrt(dist));
    }
    --> Calculate Euclidean distance of 2 stops with StopID or Coordinates


## Performance test
I run perftest-all.txt in TUNI linux machine and get these results:
[release build](https://course-gitlab.tuni.fi/tie-2010x_tiraka_dsa_2019-2020/an_tran/-/blob/master/prg1/performance-release.txt)
[debug build](https://course-gitlab.tuni.fi/tie-2010x_tiraka_dsa_2019-2020/an_tran/-/blob/master/prg1/performance-debug.txt)

-In general, release mode yields better performance compared to debug mode, but most of the operations can be finished within 20 seconds after adding 1 million stops and regions.
-Function remove_stop takes more time than the benchmark, because I have a lot of containers to remove the stop from.

