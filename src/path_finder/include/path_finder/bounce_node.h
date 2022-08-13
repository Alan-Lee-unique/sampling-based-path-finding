#ifndef _BOUNCE_NODE_H_
#define _BOUNCE_NODE_H_

#include <ros/ros.h>
#include <Eigen/Eigen>

struct TreeNode
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    TreeNode *parent;
    Eigen::Vector3d x;
    Eigen::Vector3d dir;

    bool bounce_sample; //whether this node is sampled from collision
    TreeNode * bounce_parent;

    double cost_from_start; //dis cost + dir cost
    double cost_from_parent; //dis cost + dir cost
    double dis_cost_from_start; //distance cost from start
    double dir_cost_from_start; //direction_change cost from start
    double dis_cost_from_parent; //distance cost from parent
    double dir_cost_from_parent; //direction_change cost from parent

    double heuristic_to_goal;
    double fScore; //cost_from_start + heuristic_to_goal

    std::list<TreeNode *> children;

    TreeNode(): parent(NULL),cost_from_start(DBL_MAX),cost_from_parent(DBL_MAX),
        bounce_sample(false), bounce_parent(NULL){};
};

typedef TreeNode *RRTNode3DPtr;

struct NodeWithStates
{
    RRTNode3DPtr node_ptr;
    bool is_checked;
    bool is_valid;
};

struct Neighbour
{
    Eigen::Vector3d center; //center 
    std::vector<NodeWithStates> nearing_nodes; //neighbour nodes
};

#endif
