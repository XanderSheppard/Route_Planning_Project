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

    //Debug nodes values
    std::cout << "\n Closest start_node:"<<start_node->x<<","<<start_node->y<<"\t| user input start:"<< start_x<<","<<start_y<<std::endl;
    std::cout << "Clostest end_node:" << end_node->x<<","<<end_node->y<<"\t| user input end:"<<end_x<<","<<end_y << std::endl;
}


// TODO 3: Implement the CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) 
{
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) 
{
    current_node->FindNeighbors();
    for (RouteModel::Node *n : current_node->neighbors)
    {
        n->parent = current_node;
        n->h_value = CalculateHValue(n);
        n->g_value += current_node->g_value + current_node->distance(*n);
        n->visited = true;
        open_list.push_back(n);
    }
}

// A comparison functor for the NextNode() sort function
bool Compare(const RouteModel:: Node *n1, const RouteModel::Node *n2)
{
    return(n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}

// Sort the open_list, get the node of lowest g+h value, remove it from open_list and return it.
RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare);
    auto next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}

// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    while (current_node !=start_node)
    {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    //Add start node to the path found
    path_found.push_back(*start_node);
    
    //Reverse vector order
    std::reverse(path_found.begin(), path_found.end());
    
    //Multiply the distance by the scale of map to get meters
    distance *= m_Model.MetricScale();
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.

void RoutePlanner::AStarSearch() 
{
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);


    do
    {
        current_node = NextNode();
        if(current_node == end_node)
        {
            m_Model.path = ConstructFinalPath(current_node);
            std::cout<< "\nPath Found!" <<std::endl;
            return;
        }
        AddNeighbors(current_node);
    } while (open_list.size()>0);

    std::cout<<"\nCan't find a path!"<<std::endl;
    m_Model.path = std::vector<RouteModel::Node>{};
}