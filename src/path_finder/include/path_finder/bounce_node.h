#ifndef _BOUNCE_NODE_H_
#define _BOUNCE_NODE_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <utility>

namespace bounce{
    struct BounceTreeNode
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        
        BounceTreeNode *parent;
        Eigen::Vector3d x;
        // velocity direction, for normal node, pointing from parent to this; 
        // for bounce_sample, pointing from bounce_parent to this
        // 3 points needed for clac dir_change cost
        Eigen::Vector3d dir; 

        bool bounce_sample; //whether this node is sampled from collision
        Eigen::Vector3d bounce_parent;

        double cost_from_start; //dis cost + dir cost
        double cost_from_parent; //dis cost + dir cost
        double dis_cost_from_start; //distance cost from start
        double dir_cost_from_start; //direction_change cost from start
        double dis_cost_from_parent; //distance cost from parent, 2 points need for calc
        double dir_cost_from_parent; //direction_change cost from parent, 3 points need for calc

        double heuristic_to_goal;
        double fScore; //cost_from_start + heuristic_to_goal

        std::list<BounceTreeNode *> children;

        // TreeNode(): parent(NULL),cost_from_start(DBL_MAX),cost_from_parent(DBL_MAX),
        //     bounce_sample(false), bounce_parent(NULL){};
        BounceTreeNode()
        {
            parent = NULL;
            bounce_sample = false;

            cost_from_start = DBL_MAX;
            cost_from_parent = DBL_MAX;
            dis_cost_from_start = DBL_MAX;
            dir_cost_from_start = DBL_MAX;
            dis_cost_from_parent = DBL_MAX;
            dir_cost_from_parent = DBL_MAX;

            heuristic_to_goal = DBL_MAX;
            fScore = DBL_MAX;
        }
    };

    typedef BounceTreeNode *TreeNodePtr;

    struct NodeWithStates
    {
        TreeNodePtr node_ptr;
        bool is_checked;
        bool is_valid;
        
        NodeWithStates()
        {
            node_ptr = nullptr;
            is_checked = false;
            is_valid = false;
        }
        
        NodeWithStates(const TreeNodePtr &n, bool checked, bool valid) :
            node_ptr(n), is_checked(checked), is_valid(valid){}

    };

    struct BounceNeighbour
    {
        Eigen::Vector3d center; //center 
        std::vector<NodeWithStates> nearing_nodes; //neighbour nodes
    };
}
#endif
