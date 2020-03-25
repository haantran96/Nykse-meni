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
#include <unordered_map>
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
    struct Region {
        Name regionName;
        RegionID regionId;
        std::vector<StopID>stops;
    };

    struct BusStop {
        Name name;
        Coord coord;
        std::vector<RegionID> main_region;
    };

    std::unordered_map<StopID,BusStop> stop_name_coord;

    struct CoordStop {
        double dist;
        int y;
        StopID stopid;
        bool operator<(const CoordStop& rhs) const {
                return std::tie(dist, y) < std::tie(rhs.dist, rhs.y);
            }
    };


    struct RegionNode {
        Region region;
        RegionNode *child;
        RegionNode *next;
        RegionNode *parent;
    };


    RegionNode *root;

    RegionNode *newNode (Region region){
        RegionNode *node = new RegionNode();
        node->region = region;
        node->child =  NULL;
        node->next = NULL;
        node->parent = NULL;
        return node;
      };

    RegionNode* findRegion(RegionNode* node, RegionID id) {
        if (node == NULL) {
            return node;
        }
        if (node->region.regionId == id)
            return node;
        RegionNode* res1 = findRegion(node->next, id);
        if (res1 != NULL)
            return res1;
        RegionNode* res2 = findRegion(node->child, id);
        return res2;
    }

    RegionNode* addSibling(RegionNode * node, RegionNode* sibling)
    {
        if (node == NULL)
            return NULL;

        while (node->next)
            node = node->next;

        return (node->next = sibling);
    }

    // Add child Node to a Node
    RegionNode* addChild(RegionNode* &node, RegionNode* child)
    {
    if (node == NULL)
        return NULL;
    if (node->child) {
        std::cout << "Add sibling " << child->region.regionId << std::endl;
        return addSibling(node->child, child);
    }
    else {
        std::cout << "add child " << child->region.regionId << std::endl;
        return (node->child = child);
        }

    }

    void traverseTree(RegionNode * root)
    {

        if (root == NULL)
            return;

        while (root)
        {
            std::cout << " " << root->region.regionId;
            if (root->child)
                traverseTree(root->child);
            root = root->next;
        }
    }

    std::vector<RegionID> findParents(RegionNode *root, StopID id)
    {
      /* base cases */
      std::vector<RegionID> st;
      if (root == NULL)
         return st;
      while (1) {
        std::vector<StopID> v = root->region.stops;
        if (std::find(v.begin(),v.end(),id) == v.end()) {
            st.push_back(root->region.regionId);
            root = root->child;
        } else {
            st.push_back(root->region.regionId);
            break;
            }

        root = root->next;
        }
      return st;
    }


    std::unordered_map<RegionID,Region> regions;
    //std::map<RegionID,Region> regions;


};

#endif // DATASTRUCTURES_HH
