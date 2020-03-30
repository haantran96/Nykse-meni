## datastructures.hh
    struct RegionNode {
        RegionID regionId;
        Name regionName;
        std::vector<StopID>stops;
        RegionNode *child; // Child 
        RegionNode *next; // Next Sibling
        RegionNode *prev; // Previous sibling
        RegionNode *parent; //Parent
    };
        -> Store information of a region: RegionID, Name, vector of belong stops, pointers to next/previous siblings, parent and child nodes.
    
    struct BusStop {
        StopID id;
        Name name;
        Coord coord;
        double dist; // distance to origion
        RegionNode* main_region;
    };
        -> Store information of a stop: StopID, Name, Coordinate, distance to original, pointer to the region that directly has the stop.


    std::multimap<Name,StopID>name_stop;
    std::multimap<double,StopID>new_coord;
        -> Multimaps that contain the Name/Distance to origins as key and StopID as values. When inserted, the names and distances are always sorted.
    
    std::unordered_map<StopID,BusStop> stops;
        -> unordered_map with ID of the stop as key and BusStop struct as value -> Easy and fast to retrieve information with a given id

    std::vector<StopID> all_stop;
    -> vector that contain all the stop IDs
    
    std::vector<StopID> stop_coord;
    std::vector<StopID> new_stop_coord;
        -> Stop Ids in sorted order (based on coordinates) and of newly inserted stops. 
        -> Instead of sorting all stops, I save the already sorted stops in 1 vector and only sort the newly inserted stops.
        -> By this, we can save some time on the already sorted elements, which works a lot of stops are added (in the perftest-sorting function)
        -> After sorting, new_stop_coord vector is cleared to empty again.
    
    std::vector<RegionID>allRegions;
        -> vector contains all region IDs
    
    std::unordered_map<RegionID,RegionNode*> regions;
        -> map that stores region ID as key and its information as values
    
    bool sorted_coord = true;
        -> Because stop_coord is always sorted, this value is always true. However, when the coordinates of the stops are changed, set this to false.
    
    ## Private functions:
    RegionNode *newNode (RegionID regionID, Name regionName): O(1)
        create a new node for the region when it is added
        
    void traverseTree(RegionNode * root,std::vector<int>&x_coord,std::vector<int>&y_coord): O(n)
        traverse through all children nodes from the root node and push the x and y coordinates to
        Instead of recursion, I implement this using a stack (time complexity is same, but safer for memory)
        
    inline bool sortByCoord: O(1)
        sort the xy coordinates by distance to origin.
        
    inline bool sortByName: O(1)
        sort by name
    
    inline bool sortByDistance: O(1)
        sort the xy coordinates by distance to another point.
        
    std::vector<StopID> mergeSort(std::vector<StopID>&v1, std::vector<StopID>&v2, bool(Datastructures::*func)(const StopID&,const StopID&)): O(n)
        merge 2 sorted vectors v1, v2 using merge sort algorithm
        func is the function that defines the rules for sorting.
        
## datastructures.cc
    -int stop_count() : O(1)
        return size() of unordered_map
    
    -void clear_all() : O(n)
        removing multimaps, unordered_maps and vectors takes linear time complexity 
    
    -std::vector<StopID> all_stops() : O(1)
        return the all_stop vector
    
    -bool add_stop(StopID id, Name const& name, Coord xy) : O(logn) for each new added stop
        insert to unordered_map (stops) and multimap (name_stop, new_coord): O(logn)
        push_back to vectors (all_stop, new_stop_coord): O(1)
    
    -Name get_stop_name(StopID id) : O(1)
        return value of an unordered_map with the key given
    
    -Coord get_stop_coord(StopID id) : O(1)
        return value of an unordered_map with the key given
            
    -std::vector<StopID> stops_alphabetically() : O(n)
        multimap name_stop already sorts the names alphabetically -> inserting StopIDs to a vector takes linear time.        
    
    -std::vector<StopID> stops_coord_order() : O(nlogn)
            + If coord_stop is unsorted (the coordinates are changed): sort again the coord_stop -> O(nlogn)
            + Sort new_coord_stop -> O(nlogn)
            + Merge 2 sorted vectors using mergeSort -> O(n1+n2), where n1 and n2 are sizes of coord_stop and new_coord_stop
            -> Overall O(nlogn)
            + After merging, clear the content of new_coord_stop -> O(n)
        
    StopID min_coord(), max_coord : O(n)
        + If new_stop_coord.size() = 0 (the coordinates are sorted): return first/last values of coord_stop -> O(1) 
        + Otherwise, find min/max distance to origin in the multimap (first/last element) -> O(1). 
        + Then search for elements with the same distance in the multimap with equal_range -> O(logn)
        + Iterate through the found iterators to the elements, compare the y coordinates between the found elements and get the ones with min/max y 
            -> O(m) where m is the number of StopIDs with the same distance.
        + Compare the min/max element of coord_stop and new_coord_stop.
        
    std::vector<StopID> find_stops(Name const& name): O(logn+m) where m is the number of stops with the same name -> Overall O(n)
        + Use equal_range to search for name in multimap name_stop -> O(logn)
        + Iterate through the found iterators to the elements -> O(m), m is the number of found stops with the given name
    
    bool change_stop_name(StopID id, Name const& newname): O(n)
        + Change stop name in unordered_map -> O(1)
        + Change stop_name in multimap (searching takes O(n), then erase the element O(n), insert the new pair (name.id) (O(logn)): overall O(n)
        
    bool change_stop_coord(StopID id, Coord newcoord) : O(n)
        + Change stop coord in unordered_map -> O(1)
        + Change stop coord in multimap -> O(n)
        + Check if newcoord belongs to the already sorted array, if yes then set the boolean sorted_coord = false -> O(n)
        
    bool add_region(RegionID id, Name const& name) : O(logn)
        + Add the region to the unordered_map regions -> O(logn)
        + Add the region ID to the vector allRegions -> O(1)
        
    Name get_region_name(RegionID id) : O(1)
        + return value of an unordered_map with the key given
    
    std::vector<RegionID> all_regions() : O(1)
        + return allRegions vector
        
    bool add_stop_to_region(StopID id, RegionID parentid): O(n)
        + check if the stops and regions are existed in unordered_maps: O(1) on average
        + check if the StopID is already added to the region: O(n) (search in vector)
        + If not -> add to the vector and assign the region as the stops[id].main_region -> O(1)
        
    bool add_subregion_to_region(RegionID id, RegionID parentid): O(n)
        + Check if subregion and region are existed: O(1) on average
        + Insert parent-child relationship to the regions. Two cases:
            + If the parent region doesn't have a child -> O(1)
            + If the parent region already have child/children 
                -> Traverse through all the children and push the subregion to the last -> O(n) with n is the number of children of the region
            
    std::vector<RegionID> stop_regions(StopID id): O(n)
        + Find the main_region from the StopID -> O(1)
        + From the found main_region node, traverse to its parents until reach NULL -> O(n) where n is the height from root to the found region
    
    void creation_finished();
    
    std::pair<Coord, Coord> region_bounding_box(RegionID id) : O(n)
        + Find the region from unodered_map regions -> O(1) on average
        + Traverse through its children (subregions) and push the x and y coordinates of the stops to x_coord and y_coord vectors -> O(n)
        + Find min and max x, y coordinates with std::minmax_element-> O(n)
        
    std::vector<StopID> stops_closest_to(StopID id): O(nlogn)
        + Sort all stops based on the distances to the given stop: O(nlogn)
        
    bool remove_stop(StopID id);
        + Remove stop in the unodered_map stops: O(n)
        + Remove stop in 2 multimaps name_stop and coord_stop/new_coord: O(n)
        + Remove stop in all relevant vectors (stop_coord, new_stop_coord, all_stop): 
            1) Find the stop id -> O(n)
            2) swap the found iterator with the end() -> O(1)
            3) Pop_back to remove the element -> O(1)
            -> Overall the time complexity is linear, but I found out that this method takes faster time than the erase() method.
            
        
    RegionID stops_common_region(StopID id1, StopID id2): O(n)
        + Find vectors of regions+subregions with the given StopIDs using the above stop_regions(StopID) -> O(n)
        + Use std::find_first_of the 2 vectors to find first common element -> worse case O(m*k) with m,k are sizes of the 2 vectors -> O(n) on average