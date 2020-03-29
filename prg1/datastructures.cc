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
#include <stdexcept>
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
    return stops.size();
}

void Datastructures::clear_all()
{
    // Replace this comment with your implementation
    stops.clear();
    regions.clear();
    allRegions.clear();
    all_stop.clear();
    sorted_coord = false;
    sorted_name = false;
    if (new_stop_coord.size()>0)
        new_stop_coord.clear();
    if (new_stop_name.size() >0)
        new_stop_name.clear();
    stop_names.clear();

}

std::vector<StopID> Datastructures::all_stops()
{
    // Replace this comment and the line below with your implementation
    return all_stop;
}

bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    // Replace this comment and the line below with your implementation
    double dist = pow(xy.x,2)+pow(xy.y,2);
    if (stops.find(id) == stops.end()) {
        stops[id].name = name;
        stops[id].coord = xy;
        stops[id].dist = dist;
        all_stop.push_back(id);
        new_stop_name.push_back(id);
        new_stop_coord.push_back(id);
        stop_names.push_back(name);
        Coords stop = {xy,dist,id};
        if (minCoord.id == -999)
            minCoord = stop;
         else {
            if( coord_operator(stop,minCoord)) {
                minCoord = stop;
            }
        }
        if (maxCoord.id == -999)
            maxCoord = stop;
         else {
            if( coord_operator(maxCoord,stop)) {
                maxCoord = stop;
            }
        }

        name_stop.insert(std::pair<Name,StopID>(name,id));

        return true;
    }
    else {
        return false;
    }
}

Name Datastructures::get_stop_name(StopID id)
{
    // Replace this comment and the line below with your implementation
    if (stops.find(id) == stops.end()) {
        return NO_NAME;
    } else {
        return stops[id].name;
    }
}

Coord Datastructures::get_stop_coord(StopID id)
{
    // Replace this comment and the line below with your implementation
    if (stops.find(id) == stops.end()) {
        return NO_COORD;
    }
    else {
        return stops[id].coord;
    }
}

std::vector<StopID> Datastructures::stops_alphabetically()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
        return {NO_STOP};
    else {
//        if (!sorted_name && stop_alphabetical.size() > 0) {
//            std::sort(stop_alphabetical.begin(),stop_alphabetical.end(),
//                      [this] (StopID lhs, StopID rhs) { return sortByName(lhs, rhs); });
//            sorted_name = true;
//        }


//        if (new_stop_name.size() >0) {
//            if (new_stop_name.size() <= 300000)
//                std::sort(new_stop_name.begin(),new_stop_name.end(),[this] (StopID lhs, StopID rhs) { return sortByName(lhs, rhs); });
//            else {
//                std::vector<StopID> v1, v2;
//                v1 = std::vector<StopID>(new_stop_name.begin(),new_stop_name.begin()+int(new_stop_name.size() /2));
//                v2 = std::vector<StopID>(new_stop_name.begin()+int(new_stop_name.size() /2),new_stop_name.end());
//                std::sort(v1.begin(),v1.end(),
//                          [this] (StopID lhs, StopID rhs) { return sortByName(lhs, rhs); });
//                std::sort(v2.begin(),v2.end(),
//                          [this] (StopID lhs, StopID rhs) { return sortByName(lhs, rhs); });

//                new_stop_name = mergeSort(new_stop_name,stop_alphabetical,&Datastructures::sortByName);
//                v1.clear();
//                v2.clear();
//            }
//            if (stop_alphabetical.size() == 0){
//                stop_alphabetical = new_stop_name;
//                new_stop_name.clear();
//                return stop_alphabetical;
//            }


//            std::vector<StopID> stops = mergeSort(new_stop_name,stop_alphabetical,&Datastructures::sortByName);
//            new_stop_name.clear();
//            stop_alphabetical = stops;
//            return stop_alphabetical;
//        } else {
//            return stop_alphabetical;
//        }
        std::vector<StopID> stops;
        for (multimap<Name,StopID>::iterator it= name_stop.begin(); it != name_stop.end(); ++it){
            stops.push_back(it->second);
        }
        stop_alphabetical = stops;
        return stops;
    }
}
std::vector<StopID> Datastructures::stops_coord_order()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
        return {NO_STOP};
    else {
        if (!sorted_coord && stop_coord.size() > 0) {
            std::sort(stop_coord.begin(),stop_coord.end(),
                      [this] (StopID lhs, StopID rhs) { return sortByCoord(lhs, rhs); });
            sorted_coord = true;
        }

        if (new_stop_coord.size() >0) {
            std::sort(new_stop_coord.begin(),new_stop_coord.end(),
                      [this] (StopID lhs, StopID rhs) { return sortByCoord(lhs, rhs); });
            if (stop_coord.size() == 0){
                stop_coord = new_stop_coord;
                new_stop_coord.clear();

                return stop_coord;
            }
            std::vector<StopID> stops = mergeSort(new_stop_coord,stop_coord,&Datastructures::sortByCoord);

            new_stop_coord.clear();
            stop_coord = stops;
            return stop_coord;
        } else {
            return stop_coord;
        }
    }
}

StopID Datastructures::min_coord()
{
    // Replace this comment and the line below with your implementation
    return minCoord.id;
}

StopID Datastructures::max_coord()
{
    // Replace this comment and the line below with your implementation
    return maxCoord.id;
}

std::vector<StopID> Datastructures::find_stops(Name const& name)
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> foundStops;
//    for (auto& element: stops) {
//        if(element.second.name == name) {
//            foundStops.push_back(element.first);
//        }
//    }

//    if (new_stop_name.size()>0)
//         stops_alphabetically();

//    int l = 0;
//    int r = int(stop_alphabetical.size())-1;
//    int mid = -1;
//    while (l <= r)
//    {
//        int m = l + (r-l)/2;

//      if (stops[stop_alphabetical.at(m)].name == name) {
//          foundStops.push_back(stop_alphabetical.at(m));
//          mid = m;
//          break;
//      }
//      // If x greater, ignore left half
//      if (stops[stop_alphabetical.at(m)].name < name)
//          l = m + 1;

//      // If x is smaller, ignore right half
//      else
//           r = m - 1;
//    }

//    if (mid == -1) {
//        return foundStops;
//    }

//    int midLeft = mid -1;
//    int midRight = mid+1;

//    while (midLeft >=0 && stops[stop_alphabetical.at(midLeft)].name == name) {
//        foundStops.push_back(stop_alphabetical.at(midLeft));
//            midLeft --;
//    }
//    while (midRight < int(stop_alphabetical.size()) && stops[stop_alphabetical.at(midRight)].name == name ) {
//            foundStops.push_back(stop_alphabetical.at(midRight));
//            midRight++;
//    }

    std::pair <std::multimap<Name,StopID>::iterator, std::multimap<Name,StopID>::iterator> ret;
    ret = name_stop.equal_range(name);
    for (std::multimap<Name,StopID>::iterator it=ret.first; it!=ret.second; ++it) {
        foundStops.push_back(it->second);
    }

    return foundStops;
}

bool Datastructures::change_stop_name(StopID id, const Name& newname)
{
    // Replace this comment and the line below with your implementation
    if (stops.find(id) == stops.end()){
        return false;
    } else {
        stops.find(id)->second.name = newname;
        if (std::find(stop_alphabetical.begin(),stop_alphabetical.end(),id) != stop_alphabetical.end())
            sorted_name = false;

        return true;
    }
}

bool Datastructures::change_stop_coord(StopID id, Coord newcoord)
{
    // Replace this comment and the line below with your implementation

    if (stops.find(id) == stops.end()){
        return false;
    } else {
        stops.find(id)->second.coord = newcoord;
        stops.find(id)->second.dist = pow(newcoord.x,2)+pow(newcoord.y,2);
        if (std::find(stop_coord.begin(),stop_coord.end(),id) != stop_coord.end())
            sorted_coord = false;

        Coords new_stop = {newcoord,pow(newcoord.x,2)+pow(newcoord.y,2),id};
        if (coord_operator(new_stop,minCoord))
            minCoord = new_stop;
        if (coord_operator(maxCoord,new_stop))
            maxCoord = new_stop;
        return true;
    }
}

bool Datastructures::add_region(RegionID id, const Name &name)
{
    if (regions.find(id) == regions.end()) {
        regions[id] = newNode(id,name);
        allRegions.push_back(id);
        return true;
    } else {
        return false;
    }
}

Name Datastructures::get_region_name(RegionID id)
{
    // Replace this comment and the line below with your implementation
    if (regions.find(id) == regions.end()) {
        return NO_NAME;
    } else
        return regions.find(id)->second->regionName;
}

std::vector<RegionID> Datastructures::all_regions()
{
    // Replace this comment and the line below with your implementation
    return allRegions;
}

bool Datastructures::add_stop_to_region(StopID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
    auto it2 = regions.find(parentid);
    auto it1 = stops.find(id);
    if (it1 == stops.end() || it2 == regions.end()) {
        return false;
    } else {
        if (std::find(it2->second->stops.begin(),it2->second->stops.end(),id) == it2->second->stops.end()) {
            it2->second->stops.push_back(id);
            stops[id].main_region = it2->second;
            return true; }
        return false;
        }
    }


bool Datastructures::add_subregion_to_region(RegionID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
    bool foundParent = false;
    bool foundChild = false;
    std::unordered_map<RegionID,RegionNode*>::iterator it1 = regions.find(id);
    std::unordered_map<RegionID,RegionNode*>::iterator it2 = regions.find(parentid);

    if (it2 != regions.end()) {
        foundParent = true;
    }
    if (it1 != regions.end()) {
        foundChild =true;
    }

    if (foundParent == false || foundChild == false) {
        return false;
    } else {
        it1->second->parent = it2->second;
        if (it2->second->child) {
            while (it2->second->child->next) {
                it2->second->child = it2->second->child->next;
            }
            RegionNode* prev = it2->second->child;
            it2->second->child->next = it1->second;
            it2->second->child->next->prev = prev;
            return true;
        }
        it2->second->child = it1->second;
        it2->second->child->prev = NULL;
        return true;
    }
}


std::vector<RegionID> Datastructures::stop_regions(StopID id)
{
    // Replace this comment and the line below with your implementation

    std::vector<RegionID> st;
    RegionNode* node = stops[id].main_region;
    st.push_back(node->regionId);
    while (node->parent) {
        st.push_back(node->parent->regionId);
        node = node->parent;
    }
    return st;

}

void Datastructures::creation_finished()
{
    // Replace this comment with your implementation
    // You don't have to use this method for anything, if you don't need it

}

std::pair<Coord,Coord> Datastructures::region_bounding_box(RegionID id)
{
    // Replace this comment and the line below with your implementation
    std::vector<int>x_coord;
    std::vector<int>y_coord;
    Coord bottomLeft,topRight;
    std::unordered_map<RegionID,RegionNode*>::iterator it = regions.find(id);
    if (it == regions.end())
        return {NO_COORD, NO_COORD};
    else {
        RegionNode* found = it->second;
        std::vector<StopID> temp = found->stops;
        if (temp.size() >0 ){
            for (auto s:temp) {
                x_coord.push_back(stops[s].coord.x);
                y_coord.push_back(stops[s].coord.y);
            }
        }

        traverseTree(found->child,x_coord,y_coord);
        if (x_coord.size() == 0 ) {
            return {NO_COORD, NO_COORD};
        } else {

            auto x_ = std::minmax_element (x_coord.begin(),x_coord.end());
            auto y_ = std::minmax_element (y_coord.begin(), y_coord.end());
            bottomLeft.x = *x_.first;
            bottomLeft.y = *y_.first;
            topRight.x = *x_.second;
            topRight.y = *y_.second;
            return std::pair(bottomLeft,topRight);
        }
    }
}


std::vector<StopID> Datastructures::stops_closest_to(StopID id)
{
    // Replace this comment and the line below with your implementation

    std::unordered_map<StopID,BusStop>::iterator it = stops.find(id);
    if (it == stops.end())
        return {NO_STOP};
    else {
        Coord xy = it->second.coord;
        std::vector<StopID> stop_distance  = all_stop;
        std::sort(stop_distance.begin(),stop_distance.end(),[this,xy] (StopID lhs, StopID rhs) { return sortByDistance(lhs, rhs, xy); });

        return {stop_distance.begin()+1,stop_distance.begin()+6};
    }

}

bool Datastructures::remove_stop(StopID id)
{
    // Replace this comment and the line below with your implementation
    std::unordered_map<StopID,BusStop>::iterator it = stops.find(id);
    if (it == stops.end()) {
        return false;
    } else {
        std::vector<StopID>& v = regions[stops[id].main_region->regionId]->stops;
        v.erase(std::remove(v.begin(),v.end(),id),v.end());

        stops.erase(id);
        all_stop.erase(std::remove(all_stop.begin(),all_stop.end(), id),all_stop.end());
        if (creation_finish){
            stop_coord.erase(std::remove(stop_coord.begin(),stop_coord.end(), id),stop_coord.end());
            stop_alphabetical.erase(std::remove(stop_alphabetical.begin(),stop_alphabetical.end(), id),stop_alphabetical.end());
            return true;
        }
        if (std::find(new_stop_coord.begin(),new_stop_coord.end(), id) != new_stop_coord.end())
            stop_coord.erase(std::remove(stop_coord.begin(),stop_coord.end(), id),stop_coord.end());
        else
            new_stop_coord.erase(std::remove(new_stop_coord.begin(),new_stop_coord.end(), id),new_stop_coord.end());

        if (std::find(stop_alphabetical.begin(),stop_alphabetical.end(), id) != stop_alphabetical.end())
            stop_alphabetical.erase(std::remove(stop_alphabetical.begin(),stop_alphabetical.end(), id),stop_alphabetical.end());
        else
            new_stop_name.erase(std::remove(new_stop_name.begin(),new_stop_name.end(), id),new_stop_name.end());

        return true;
    }
}

RegionID Datastructures::stops_common_region(StopID id1, StopID id2)
{
    // Replace this comment and the line below with your implementation
    std::vector<RegionID> stop_regions1 = stop_regions(id1);
    std::vector<RegionID> stop_regions2 = stop_regions(id2);

    auto result = std::find_first_of (stop_regions1.begin(), stop_regions1.end(),
                        stop_regions2.begin(), stop_regions2.end());
    if (result == stop_regions1.end())
        return NO_REGION;
    else
        return  stop_regions1.at(std::distance(stop_regions1.begin(), result));
}
