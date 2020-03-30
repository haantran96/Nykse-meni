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
    name_stop.clear();
    sorted_coord = true;
    stop_coord.clear();
    new_coord.clear();
    if (new_stop_coord.size()>0)
        new_stop_coord.clear();
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
        stops[id].id = id;
        stops[id].name = name;
        stops[id].coord = xy;
        stops[id].dist = dist;
        all_stop.push_back(id);
        new_stop_coord.push_back(id);
        name_stop.insert(std::pair<Name,StopID>(name,id));
        new_coord.insert(std::pair<double,StopID>(dist,id));
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
        return {};
    else {
        std::vector<StopID> stops;
        for (multimap<Name,StopID>::iterator it= name_stop.begin(); it != name_stop.end(); ++it){
            stops.push_back(it->second);
        }
        return stops;
    }
}
std::vector<StopID> Datastructures::stops_coord_order()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
        return {};
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
    if (stop_count() == 0)
        return NO_STOP;
    else {
        if (new_stop_coord.size() == 0)
            return (*stop_coord.begin());
        else {
            double minDist = new_coord.begin()->first;
            int minY =-1;
            StopID mincoord;
            auto ret = new_coord.equal_range(minDist);
            for (std::multimap<double,StopID>::iterator it=ret.first; it!=ret.second; ++it) {
                if (minY == -1) {
                    mincoord = it->second;

                    minY = stops[it->second].coord.y;
                }
                else if (stops[it->second].coord.y < minY) {
                    mincoord = it->second;
                    minY = stops[it->second].coord.y;
                }
            }
            if (stop_coord.size() == 0)
                return mincoord;
            if (sortByCoord(mincoord,*stop_coord.begin()))
                return mincoord;
            else
                return (*stop_coord.begin());
        }
    }
}

StopID Datastructures::max_coord()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
        return NO_STOP;
    else {
        if (new_stop_coord.size() == 0)
            return (*(stop_coord.end()-1));
        else {
            double maxDist = new_coord.rbegin()->first;
            int maxY =-1;
            StopID maxcoord;
            auto ret = new_coord.equal_range(maxDist);
            for (auto it=ret.first; it!=ret.second; ++it) {
                if (maxY == -1) {
                    maxcoord = it->second;
                    maxY = stops[it->second].coord.y;
                }
                else if (stops[it->second].coord.y > maxY) {
                    maxcoord = it->second;
                    maxY = stops[it->second].coord.y;
                }
            }
            if (stop_coord.size() == 0)
                return maxcoord;

            if (sortByCoord(*(stop_coord.end()-1),maxcoord))
                return maxcoord;
            else
                return (*(stop_coord.end()-1));
        }

    }
}

std::vector<StopID> Datastructures::find_stops(Name const& name)
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> foundStops;
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
        Name oldname = stops.find(id)->second.name;

        stops.find(id)->second.name = newname;
        std::pair <std::multimap<Name,StopID>::iterator, std::multimap<Name,StopID>::iterator> ret;
        ret = name_stop.equal_range(oldname);
        for (std::multimap<Name,StopID>::iterator it=ret.first; it!=ret.second; ++it) {
            if (it->second == id) {
                it = name_stop.erase(it);
                name_stop.insert(std::pair<Name,StopID>(newname,id));
                break;
            }
        }
        return true;
    }
}

bool Datastructures::change_stop_coord(StopID id, Coord newcoord)
{
    // Replace this comment and the line below with your implementation

    if (stops.find(id) == stops.end()){
        return false;
    } else {
        double dist = stops[id].dist;
        double newdist = pow(newcoord.x,2)+pow(newcoord.y,2);
        stops.find(id)->second.coord = newcoord;
        stops.find(id)->second.dist = newdist;
        auto main_it = std::find(stop_coord.begin(),stop_coord.end(),id);
        if (main_it != stop_coord.end()){
            sorted_coord = false;
        }
        std::pair <std::multimap<double,StopID>::iterator, std::multimap<double,StopID>::iterator> ret = new_coord.equal_range(dist);
        for (auto it=ret.first; it!=ret.second; ++it) {
            if (it->second == id) {
                new_coord.erase(it);
                new_coord.insert(std::pair<double,StopID>(newdist,id));
                break;
            }
        }
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
        if (stop_distance.size() < 6) {
            return {stop_distance.begin()+1,stop_distance.end()};
        }
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
        Name name = stops[id].name;
        double dist = stops[id].dist;
        stops.erase(id);

        std::pair <std::multimap<Name,StopID>::iterator, std::multimap<Name,StopID>::iterator> ret;
        ret = name_stop.equal_range(name);
        for (std::multimap<Name,StopID>::iterator it=ret.first; it!=ret.second; ++it) {
            if (it->second == id) {
                it = name_stop.erase(it);
                break;
            }
        }

        auto it1 = std::find(stop_coord.begin(),stop_coord.end(),id);

        if (it1 != stop_coord.end())
            stop_coord.erase(stop_coord.begin()+std::distance(stop_coord.begin(),it1));
        else{
            auto it2 = std::find(new_stop_coord.begin(),new_stop_coord.end(),id);
            std::iter_swap(it2,new_stop_coord.end()-1);
            new_stop_coord.pop_back();

            std::pair <std::multimap<double,StopID>::iterator, std::multimap<double,StopID>::iterator> ret_;
            ret_ = new_coord.equal_range(dist);
            for (auto it=ret_.first; it!=ret_.second; ++it) {
                if (it->second == id) {
                    it = new_coord.erase(it);
                    break;
                }
            }
        }
        auto it3 = std::find(all_stop.begin(),all_stop.end(),id);
        std::iter_swap(it3,all_stop.end()-1);
        all_stop.pop_back();

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
