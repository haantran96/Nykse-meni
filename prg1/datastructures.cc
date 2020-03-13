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
}

Datastructures::~Datastructures()
{
    // Replace this comment with your implementation
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
    std::vector<StopID> stops;

    for (const auto& element : stop_name_coord) {
        stops.push_back(element.first);
     }

    //final_stops = stops;
    return stops;
}

bool Datastructures::add_stop(StopID id, const Name& name, Coord xy)
{
    // Replace this comment and the line below with your implementation

    std::map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()) {

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
    // Replace this comment and the line below with your implementation
    std::map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()) {
        return NO_NAME;
    } else {
        Name stop_name = stop_name_coord[id].name;
        return stop_name;
    }
}

Coord Datastructures::get_stop_coord(StopID id)
{
    // Replace this comment and the line below with your implementation
    std::map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()) {
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
    std::cout << "start";
    if (stop_count() == 0)
    {
        return {NO_STOP};
    } else {
        for (auto element:stop_name_coord) {
            std::cout << element.second.name << element.first;
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
    std::map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
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

    std::map<StopID,BusStop>::iterator it = stop_name_coord.find(id);
    if (it == stop_name_coord.end()){
        return false;
    } else {
        it->second.coord = newcoord;
        return true;
    }
}

bool Datastructures::add_region(RegionID id, const Name &name)
{
    // Replace this comment and the line below with your implementation
    std::map<RegionID,Region>::iterator it = regions.find(id);
    if (it ==regions.end()) {
        regions[id].name = name;
        return true;
    } else {
        return false;
    }
}

Name Datastructures::get_region_name(RegionID id)
{
    // Replace this comment and the line below with your implementation
    std::map<RegionID,Region>::iterator it = regions.find(id);
    if (it !=regions.end()) {
        return regions[id].name;
    } else {
        return NO_NAME;
    }
}

std::vector<RegionID> Datastructures::all_regions()
{
    // Replace this comment and the line below with your implementation
    std::vector<RegionID> allRegions;
    for (auto& element:regions) {
        allRegions.push_back(element.first);
    }
    return allRegions;
}

bool Datastructures::add_stop_to_region(StopID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
    std::map<RegionID,Region>::iterator it = regions.find(parentid);
    if (it !=regions.end()) {
        regions[parentid].busStops.push_back(stop_name_coord[id]);
        return true;
    } else {
        return false;
    }
}

bool Datastructures::add_subregion_to_region(RegionID id, RegionID parentid)
{
    // Replace this comment and the line below with your implementation
    std::vector<Region> *re = regions[parentid].subRegions;
    std::vector<Region>::iterator it = std::find(std::begin(*re), std::end(*re), id);
    if (it != re->end()) {
        re->push_back(regions[id]);
        return true;
    } else {
        return false; }
}

std::vector<RegionID> Datastructures::stop_regions(StopID id)
{
    // Replace this comment and the line below with your implementation
    return {NO_REGION};
}

void Datastructures::creation_finished()
{
    // Replace this comment with your implementation
    // You don't have to use this method for anything, if you don't need it
}

std::pair<Coord,Coord> Datastructures::region_bounding_box(RegionID id)
{
    // Replace this comment and the line below with your implementation
    return {NO_COORD, NO_COORD};
}

std::vector<StopID> Datastructures::stops_closest_to(StopID id)
{
    // Replace this comment and the line below with your implementation
    return {NO_STOP};
}

bool Datastructures::remove_stop(StopID id)
{
    // Replace this comment and the line below with your implementation
    return false;
}

RegionID Datastructures::stops_common_region(StopID id1, StopID id2)
{
    // Replace this comment and the line below with your implementation
    return NO_REGION;
}
