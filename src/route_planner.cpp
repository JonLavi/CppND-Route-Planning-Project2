#include "route_planner.h"
#include "route_model.h"
#include <algorithm>
#include <vector>
using std::vector;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // save the closest nodes to the start and end
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // populate current_node.neighbors vector with all the neighbors.
  current_node->FindNeighbors();
  
  for (auto *neighbor : current_node->neighbors){
    // set the parent, the h_value, the g_value. 
  	neighbor->parent = current_node;
    neighbor->h_value = RoutePlanner::CalculateHValue(neighbor);
    neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
    //  add the neighbor to open_list and set the node's visited attribute to true.
    neighbor->visited = true ;
    open_list.push_back(neighbor);
  }
}

RouteModel::Node *RoutePlanner::NextNode() {
  
  //Sort the open_list according to the sum of the h value and g value.
  std::sort(open_list.begin(), open_list.end(), [](RouteModel::Node *node, RouteModel::Node *next_node) {
        return (node->h_value + node->g_value) < (next_node->h_value + next_node->g_value);   
    });
  
  //Create a pointer to the node in the list with the lowest sum.
  RouteModel::Node * next_node = open_list.front();
  
  // Remove that node from the open_list.
  open_list.erase(open_list.begin());
  
  //Remove that node from the open_list.
  return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    
  distance = 0.0f;
  // Create path_found vector
  std::vector<RouteModel::Node> path_found;
   
  //iteratively follow the chain of parents of nodes until the starting node is found.
  while (current_node != start_node){
    //add the distance from the node to its parent to the distance variable.
    RouteModel::Node * parent = current_node->parent;
    distance += current_node->distance(*parent);
    // add the node to the front of the vector
    path_found.insert(path_found.begin(), *current_node);
    current_node = parent;
  }
  
  // add the start node to front of the vector
  path_found.insert(path_found.begin(), *start_node);

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  return path_found;
}

void RoutePlanner::AStarSearch() {
  // start at the first node
  RouteModel::Node *current_node = start_node;
  start_node->visited = true;
  open_list.push_back(start_node);
  
  while (current_node != end_node){
    // add all of the neighbors of the current node to the open_list
    AddNeighbors(current_node);
    // sort the open_list and return the next node
    current_node = NextNode();
  }
  
  // save the final path that was found.
  m_Model.path = ConstructFinalPath(end_node);
}