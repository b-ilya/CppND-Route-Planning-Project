#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return end_node->distance(*node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
            neighbor->visited = true;
            open_list.push_back(neighbor);
        }
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    if (open_list.size() == 0) {
        return nullptr;
    }

    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](const auto& node_1, const auto& node_2) {
        float node_1_cost = node_1->g_value + node_1->h_value;
        float node_2_cost = node_2->g_value + node_2->h_value;
        return node_1_cost > node_2_cost;
    });
    auto next = open_list[0];
    open_list.pop_back();
    return next;
}


// This method take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    current_node->visited = true;
    path_found.push_back(*current_node);
    while (current_node->parent != nullptr) {
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
        path_found.push_back(*current_node);
    }

    // The returned vector have to be in the correct order: the start node should be the first element
    //   of the vector, the end node should be the last element.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = start_node;

    open_list.push_back(current_node);
    while (!open_list.empty()) {
        current_node = NextNode();
        if (current_node == end_node) {
            break;
        }
        AddNeighbors(current_node);
    }

    if (current_node == end_node) {
        m_Model.path = ConstructFinalPath(current_node);
    }
    // TODO: Implement your solution here.

}
