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
    root = new RegionNode();
}

Datastructures::~Datastructures()
{
    // Replace this comment with your implementation
    root = NULL;

}

int Datastructures::stop_count()
{
    // Replace this comment and the line below with your implementation
    return stop_name_coord.size();
}

void Datastructures::clear_all()
{
    // Replace this comment with your implementation
}

std::vector<StopID> Datastructures::all_stops()
{
    // Replace this comment and the line below with your implementation
    std::cout << "CMD ALL STOPS" << std::endl;
    std::vector<StopID> stops;

    for (auto& element : stop_name_coord) {
        stops.push_back(element.first);
     }

    //final_stops = stops;
    return stops;
}

bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    // Replace this comment and the line below with your implementation
    std::cout << "CMD add_stop" << std::endl;

    if (stop_name_coord.find(id) == stop_name_coord.end()) {

        stop_name_coord[id].name = name;
        stop_name_coord[id].coord = xy;
        return true;
    }
    else {
        return false;
    }
}

Name Datastructures::get_stop_name(StopID id)
{
    std::cout << "CMD get_stop_name" << std::endl;

    // Replace this comment and the line below with your implementation
    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (stop_name_coord.find(id) == stop_name_coord.end()) {
        return NO_NAME;
    } else {
        Name stop_name = stop_name_coord[id].name;
        return stop_name;
    }
}

Coord Datastructures::get_stop_coord(StopID id)
{
    std::cout << "CMD get_stop_coord" << std::endl;

    // Replace this comment and the line below with your implementation
    if (stop_name_coord.find(id) == stop_name_coord.end()) {
        return NO_COORD;
    }
    else {
        Coord stop_coord = stop_name_coord[id].coord;
        return stop_coord;
    }
}

std::vector<StopID> Datastructures::stops_alphabetically()
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> stops;
    std::multimap<Name,StopID> name_stop;
    if (stop_count() == 0)
    {
        return {NO_STOP};
    } else {
        for (auto element:stop_name_coord) {
            name_stop.emplace(std::make_pair(element.second.name,element.first));
        }
        for (auto& n:name_stop) {
            stops.push_back(n.second);
        }
        return stops;
    }
}
std::vector<StopID> Datastructures::stops_coord_order()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
    {
        return {NO_STOP};
    } else {
        std::vector<StopID> stops;
        std::multiset<CoordStop> coord_stops;
        for (auto& element:stop_name_coord) {
            CoordStop cs;
            cs.dist = pow(element.second.coord.x,2)+pow(element.second.coord.y,2);
            cs.y = element.second.coord.y;
            cs.stopid = element.first;
            coord_stops.emplace(cs);
        }
        for (auto& n:coord_stops) {
            stops.push_back(n.stopid);
        }
        return stops;
    }
}

StopID Datastructures::min_coord()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
    {
        return {NO_STOP};
    } else {
        CoordStop minCoord = {-1,-1,{}};
        for (auto& element: stop_name_coord) {
            double coord = sqrt(pow(element.second.coord.x,2)+pow(element.second.coord.y,2));
            if ((minCoord.dist == -1) || ((coord < minCoord.dist) && (element.second.coord.y < minCoord.y))){
                minCoord.dist = coord;
                minCoord.y = element.second.coord.y;
                minCoord.stopid = element.first;
            }
        }
        return minCoord.stopid;
    }
}

StopID Datastructures::max_coord()
{
    // Replace this comment and the line below with your implementation
    if (stop_count() == 0)
    {
        return {NO_STOP};
    } else {
        CoordStop maxCoord = {-1,-1,{}};
        for (auto& element: stop_name_coord) {
            double coord = sqrt(pow(element.second.coord.x,2)+pow(element.second.coord.y,2));
            if ((maxCoord.dist == -1) || ((coord > maxCoord.dist) && (element.second.coord.y > maxCoord.y))){
                maxCoord.dist = coord;
                maxCoord.y = element.second.coord.y;
                maxCoord.stopid = element.first;
            }
        }
        return maxCoord.stopid;
    }
}

std::vector<StopID> Datastructures::find_stops(Name const& name)
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> stops;
    for (auto& element: stop_name_coord) {
        if(element.second.name == name) {
            stops.push_back(element.first);
        }
    }
    return stops;
}

bool Datastructures::change_stop_name(StopID id, const Name& newname)
{
    // Replace this comment and the line below with your implementation
    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()){
        return false;
    } else {
        it->second.name = newname;
        return true;
    }
}

bool Datastructures::change_stop_coord(StopID id, Coord newcoord)
{
    // Replace this comment and the line below with your implementation

    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()){
        return false;
    } else {
        it->second.coord = newcoord;
        return true;
    }
}

bool Datastructures::add_region(RegionID id, const Name &name)
{
    std::cout << "CMD add_region" << std::endl;
    Region new_region = Region();
    new_region.regionId = id;
    new_region.regionName = name;
    if (regions.size() == 0) {
        regions.insert(std::make_pair(id,new_region));
        return true;
    } else {
        auto it = regions.find(id);
        if (it == regions.end()) {
            regions[id] = new_region;
            return true;
        } else {
            return false;
        }
    }
}

Name Datastructures::get_region_name(RegionID id)
{
    // Replace this comment and the line below with your implementation
//    std::unordered_map<RegionID,Region>::iterator it = regions.find(id);
//    if (it !=regions.end()) {
//        return regions[id].name;
//    } else {
//        return NO_NAME;
//    }

//    RegionNode* found = findRegion(root,id);
//    if (found == NULL)
//        return NO_NAME;
//    else {
//        return found->region.regionName;
//    }
    std::unordered_map<RegionID,Region>::iterator it = regions.find(id);
    if (it == regions.end()) {
        return NO_NAME;
    } else
        return it->second.regionName;
}

std::vector<RegionID> Datastructures::all_regions()
{
    // Replace this comment and the line below with your implementation
    std::vector<RegionID> allRegions;
//    RegionNode *pre, *current;

//    current = root;
//    while (current != NULL) {
//        if (current->next == NULL){
//            allRegions.push_back(current->region.regionId);
//            current = current->child;
//        } else {
//            pre = current->next;
//            while (pre->child != NULL && pre->child != current)
//                pre = pre->child;
//            if (pre->child == NULL) {
//                pre->child = current;
//                current = current->next;
//            } else {
//                pre->child = NULL;
//                allRegions.push_back(current->region.regionId);
//                current = current->child;
//            }
//        }
//    }
//    return allRegions;
    for (auto r:regions) {
        allRegions.push_back(r.first);
    }
    return allRegions;
}

bool Datastructures::add_stop_to_region(StopID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()) {
        return false;
    } else {
        std::unordered_map<RegionID,Region>::iterator it2 = regions.find(parentid);
        if (it2 == regions.end()) {
            return false;
        } else {
            it2->second.stops.push_back(id);
            stop_name_coord[id].main_region.push_back(parentid);
            RegionNode *found = findRegion(root,parentid);
            if (found) {
                found->region.stops.push_back(id);
            }
            return true;
        }
    }
}


bool Datastructures::add_subregion_to_region(RegionID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
//    std::vector<Region> *re = regions[parentid].subRegions;
//    std::vector<Region>::iterator it = std::find(std::begin(*re), std::end(*re), id);
//    if (it != re->end()) {
//        re->push_back(regions[id]);
//        return true;
//    } else {
//        return false; }
    RegionNode* parent, *child;
    Region parentR, childR;
    bool foundParent = false;
    bool foundChild = false;
    std::unordered_map<RegionID,Region>::iterator it1 = regions.find(id);
    std::unordered_map<RegionID,Region>::iterator it2 = regions.find(parentid);

    if (it2 != regions.end()) {
        foundParent = true;
        parentR = it2->second;
    }
    if (it1 != regions.end()) {
        foundChild =true;
        childR = it1->second;
    }

    if (foundParent == false || foundChild == false) {
        return false;
    } else {
        child = newNode(childR);
        if (root->region.regionId == "") {
            parent = newNode(parentR);
            root = parent;
            RegionNode* _ = addChild(root,child);
            child -> parent = root;
        } else {
            if (child->region.regionId == root->region.regionId) {
                parent = newNode(parentR);
                root->parent = parent;
                RegionNode* _ = addChild(parent,root);
                root = parent;
                return true;
            }
            RegionNode* found = findRegion(root, parentid);
            std::cout << "FOUND " << found->region.regionId << std::endl;
            RegionNode* _ = addChild(found,child);
            child->parent = found; }
            return true;

    }
}


std::vector<RegionID> Datastructures::stop_regions(StopID id)
{
    // Replace this comment and the line below with your implementation

    std::vector<RegionID> st;
    for (auto parentid:stop_name_coord[id].main_region) {

        RegionNode* found = findRegion(root,parentid);
        if (found) {
            st.push_back(found->region.regionId);

            while (found->parent) {
                std::cout << "PARENT " << found->parent->region.regionId << std::endl;
                st.push_back(found->parent->region.regionId);
                found = found->parent;
            }
        } else {
            std::unordered_map<RegionID,Region>::iterator it = regions.find(parentid);
            if (it == regions.end())
                st.push_back(NO_REGION);
            else {
                st.push_back(it->first);
            }
        }
    }
    // TODO: remove duplicates
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
    RegionNode* found = findRegion(root,id);
    std::vector<StopID>region_stops;
    std::vector<RegionID> foundRegions;
    std::vector<int>x_coord;
    std::vector<int>y_coord;
    Coord bottomLeft,topRight;
    if (found == NULL) {
        std::unordered_map<RegionID,Region>::iterator it = regions.find(id);
        if (it == regions.end()) {
            return {NO_COORD, NO_COORD};
        } else {
            foundRegions.push_back(it->first);
        }
    } else {
        while (found) {
            //std::vector<StopID> temp = found->region.stops;
            //std::copy (temp.begin(),temp.end(),back_inserter(region_stops));
            foundRegions.push_back(found->region.regionId);

            found = found->child;
        }
    }

    for (auto i:foundRegions) {
        std::vector<StopID> temp = regions[i].stops;
        std::copy (temp.begin(),temp.end(),back_inserter(region_stops));
    }

    for (auto c:region_stops) {
        x_coord.push_back(stop_name_coord[c].coord.x);
        y_coord.push_back(stop_name_coord[c].coord.y);
    }
    auto x = std::minmax_element (x_coord.begin(),x_coord.end());
    auto y = std::minmax_element (y_coord.begin(), y_coord.end());
    bottomLeft.x = *x.first;
    bottomLeft.y = *y.first;

    topRight.x = *x.second;
    topRight.y = *y.second;

    return {bottomLeft,topRight};
}


std::vector<StopID> Datastructures::stops_closest_to(StopID id)
{
    // Replace this comment and the line below with your implementation
    std::vector<StopID> stops;
    std::multiset<CoordStop> coord_stops;
    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    int count = 0;
    if (it == stop_name_coord.end())
        return {NO_STOP};
    else {
        Coord found = it->second.coord;
        for (auto& element:stop_name_coord) {
            CoordStop cs;
            cs.dist = pow(element.second.coord.x-found.x,2)+pow(element.second.coord.y-found.y,2);
            cs.y = element.second.coord.y;
            cs.stopid = element.first;
            coord_stops.emplace(cs);
        }
        for (auto& n:coord_stops) {
            if (count < 5 && n.stopid != id) {
                stops.push_back(n.stopid);
                count+=1;
            }
        }
        return stops;
    }

}

bool Datastructures::remove_stop(StopID id)
{
    // Replace this comment and the line below with your implementation
    std::unordered_map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()) {
        return false;
    } else {
        std::vector<RegionID> regionId = stop_name_coord[id].main_region;
        for (auto parentid:regionId) {
            std::unordered_map<RegionID,Region>::iterator it2 = regions.find(parentid);
            if (it2 != regions.end()) {
                auto it_ = std::find(it2->second.stops.begin(),it2->second.stops.end(),id);
                it2->second.stops.erase(it_);
                RegionNode *found = findRegion(root,parentid);
                if (found) {
                    auto it_ = std::find(found->region.stops.begin(),found->region.stops.end(),id);
                    found->region.stops.erase(it_);
                }
            }
        }
        stop_name_coord.erase(id);
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
