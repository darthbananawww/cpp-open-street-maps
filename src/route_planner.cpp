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
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for(int i = 0; i < current_node->neighbors.size(); i++) {
        current_node->neighbors[i]->parent = current_node;
        current_node->neighbors[i]->h_value = CalculateHValue(current_node->neighbors[i]);
        current_node->neighbors[i]->g_value = current_node->distance(*current_node->neighbors[i]);

        open_list.push_back(current_node->neighbors[i]);
        current_node->neighbors[i]->visited = true;
    }
}

bool NodeSort(const RouteModel::Node *i, const RouteModel::Node *j)  {i->g_value + i->h_value < j->g_value + j->h_value; }

RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), NodeSort);
    RouteModel::Node *i = open_list[0];
    open_list.erase(open_list.begin() + 0);
    return i;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    while(current_node != start_node) {
        path_found.push_back(*current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    AddNeighbors(start_node);
    while(open_list.size() > 0)
    {
        current_node = NextNode();
        if(current_node->distance(*end_node) == 0) {break;}
        AddNeighbors(current_node);
    }
    ConstructFinalPath(current_node);
    return;
}