#ifndef BOUNCE_RRT_STAR_H
#define BOUNCE_RRT_STAR_H

#include "occ_grid/occ_map.h"
#include "visualization/visualization.hpp"
#include "sampler.h"
#include "kdtree.h"

#include "bounce_node.h"

#include <ros/ros.h>
#include <utility>
#include <queue>
using namespace bounce;

namespace path_plan
{
    class BounceRRTStar
    {
    public:
        BounceRRTStar(){};
        BounceRRTStar(const ros::NodeHandle &nh, const env::OccMap::Ptr &mapPtr) 
            : nh_(nh), map_ptr_(mapPtr)
        {
            nh_.param("RRT_Star/steer_length", steer_length_, 0.0);
            nh_.param("RRT_Star/search_radius", search_radius_, 0.0);
            nh_.param("RRT_Star/search_time", search_time_, 0.0);
            nh_.param("RRT_Star/max_tree_node_nums", max_tree_node_nums_, 0);
            nh_.param("RRT_Star/use_informed_sampling", use_informed_sampling_, true);
            nh_.param("RRT_Star/use_GUILD_sampling",use_GUILD_sampling_, true);
            
            ROS_INFO("\033[1;32m----> [bounce_rrt_star]Now in bounce_mode.\033[0m"); //green info
            ROS_WARN_STREAM("[RRT*] param: steer_length: " << steer_length_);
            ROS_WARN_STREAM("[RRT*] param: search_radius: " << search_radius_);
            ROS_WARN_STREAM("[RRT*] param: search_time: " << search_time_);
            ROS_WARN_STREAM("[RRT*] param: max_tree_node_nums: " << max_tree_node_nums_);
            ROS_WARN_STREAM("[RRT*] param: use_informed_sampling: " << use_informed_sampling_);
            ROS_WARN_STREAM("[RRT*] param: use_GUILD_sampling: " << use_GUILD_sampling_);

            sampler.setSamplingRange(map_ptr_ ->getOrigin(), map_ptr_ -> getMapSize());
            
            valid_tree_node_nums_ = 0;
            nodes_pool_.resize(max_tree_node_nums_);
            for (int i = 0; i < max_tree_node_nums_; i++)
            {
                nodes_pool_[i] = new BounceTreeNode;
            }
        }
        ~BounceRRTStar(){};

        bool plan(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
        {
            reset();
            //check start and goal state
            if(!map_ptr_->isStateValid(s))
            {
                ROS_ERROR("[RRT*]: Start pos collide or out of bound");
                return false;
            }
            if (!map_ptr_->isStateValid(g))
            {
                ROS_ERROR("[RRT*]: Goal pos collide or out of bound");
                return false;
            }          

            /* construct start and goal nodes in nodes_pool_ */ 
            start_node_ =  nodes_pool_[1];
            start_node_ -> x = s;
            start_node_ -> dir = s; // start dir should be undifined
            start_node_ -> dis_cost_from_start = 0.0;
            start_node_ -> dir_cost_from_start = 0.0; //no dir change, so zero
            start_node_ -> cost_from_start = 0.0;
            start_node_ -> dis_cost_from_parent = 0.0;
            start_node_ -> dir_cost_from_parent = 0.0;
            start_node_ -> cost_from_parent = 0.0;

            goal_node_ =  nodes_pool_[0];
            goal_node_ -> x = g;
            goal_node_ -> dis_cost_from_start = DBL_MAX;
            goal_node_ -> dir_cost_from_start = DBL_MAX; //no dir change, so zero
            goal_node_ -> cost_from_start = DBL_MAX;
            goal_node_ -> dis_cost_from_parent = DBL_MAX; //may not used
            goal_node_ -> dir_cost_from_parent = DBL_MAX; //may not used
            goal_node_ -> cost_from_parent = DBL_MAX; //may not used

            valid_tree_node_nums_ = 2; // update cur nodes num in nodes_pool

            ROS_INFO("[Bounce RRT*]: Bounce RRT* starts planning a path");

            // TODO #2 informed sampling related code 
            sampler.reset(); // do not konw why
            // if(use_informed_sampling_)
            // {
            //     calInformedSet(10000000000.0, s, g, scale_, trans_, rot_);
            //     sampler.setInformedTransRot(trans_, rot_);
            // }

            return rrt_star(s, g);
        }

        vector<Eigen::Vector3d> getPath(){
            return final_path_;
        }

        vector<vector<Eigen::Vector3d>> getAllPaths()
        {
            return path_list_;
        }

        vector<std::pair<Eigen::Vector3d, double>> getSolutions()
        {
            return solution_cost_time_pair_list_;
        }

        void setVisualizer(const std::shared_ptr<visualization::Visualization> &visPtr)
        {
            vis_ptr_ = visPtr;
        }

    private:
        // nodehandle params
        ros::NodeHandle nh_;

        BiasSampler sampler;
        // for informed sampling
        Eigen::Vector3d trans_, scale_;
        Eigen::Matrix3d rot_;
        // for GUILD sampling
        Eigen::Vector3d scale1_, scale2_;
        Eigen::Vector3d trans1_, trans2_;
        Eigen::Matrix3d rot1_, rot2_;   

        bool use_informed_sampling_;
        bool use_GUILD_sampling_;
        double steer_length_;
        double search_radius_;
        double search_time_;
        int max_tree_node_nums_;
        int valid_tree_node_nums_;
        double first_path_use_time_;
        double final_path_use_time_;

        std::vector<TreeNodePtr> nodes_pool_;
        TreeNodePtr start_node_;
        TreeNodePtr goal_node_;
        vector<Eigen::Vector3d> final_path_;
        vector<vector<Eigen::Vector3d>> path_list_;
        vector<std::pair<Eigen::Vector3d, double>> solution_cost_time_pair_list_; //Vector3d cost(total_cost, dis_cost, dir_cost) time

        // environment
        env::OccMap::Ptr map_ptr_;
        std::shared_ptr<visualization::Visualization> vis_ptr_;

        //weights
        double dir_wei; // dir_cost weight in total cost calculation, range [0, steer_length/2.0]
        double dis_ratio = 0.8; // dis decay caused by bounce effect, range (0,1)

        void reset()
        {
            final_path_.clear();
            path_list_.clear();
            solution_cost_time_pair_list_.clear();

            for(int i = 0; i < valid_tree_node_nums_; i++)
            {
                nodes_pool_[i]->parent = nullptr;
                nodes_pool_[i]->children.clear();

                // for bounce_sample, reset its bounce_parent 
                if(nodes_pool_[i]->bounce_sample){
                    // nodes_pool_[i]->bounce_parent = nullptr;
                    nodes_pool_[i]->bounce_sample = false;
                }                    
            }

            valid_tree_node_nums_ = 0;
        }

        //calc params for informed sampling
        void calInformedSet(double a2, const Eigen::Vector3d &foci1, const Eigen::Vector3d &foci2,
                        Eigen::Vector3d &scale, Eigen::Vector3d &trans, Eigen::Matrix3d &rot)
        {
            trans = (foci1 + foci2) / 2.0;
            scale[0] = a2 / 2.0;
            Eigen::Vector3d diff(foci2 - foci1);
            double c_square = diff.squaredNorm() / 4.0;
            scale[1] = sqrt(scale[0] * scale[0] - c_square);
            scale[2] = scale[1];
            rot.col(0) = diff.normalized();
            diff[2] = 0.0;
            // project to the x-y plane and then rotate 90 degree;
            rot.col(1) = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ()) * diff.normalized(); 
            rot.col(2) = rot.col(0).cross(rot.col(1));
        }

        bool rrt_star(const Eigen::Vector3d &s, const Eigen::Vector3d &g)
        {
            bool goal_found = false;

            //  store all nodes in node_pool, search in kd_tree

            double rrt_start_time = ros::Time::now().toSec();

            /* create kd_tree and init  */
            kdtree *kd_tree = kd_create(3);
            kd_insert3(kd_tree, s(0), s(1), s(2), start_node_); // add start_node_ into kd_tree

            /* main loop: sample a node, then try to add into kd_tree*/
            int idx = 0;
            double t = ros::Time::now().toSec();
            for(idx = 0; (t - rrt_start_time) < search_time_ && valid_tree_node_nums_ < max_tree_node_nums_; idx++)
            {
                Eigen::Vector3d x_rand;
                sampler.samplingOnce(x_rand);
                if(!map_ptr_->isInMap(x_rand))  // not in the map, can be occupied
                {
                    continue;
                }

                kdres *p_nearest = kd_nearest3(kd_tree, x_rand(0), x_rand(1), x_rand(2));
                if(p_nearest == nullptr)
                {
                    ROS_ERROR("nearest query error");
                    continue;
                }
                TreeNodePtr nearest_node = (TreeNodePtr)kd_res_item_data(p_nearest);
                kd_res_free(p_nearest);

                Eigen::Vector3d x_new = steer(nearest_node->x, x_rand, steer_length_);
                //if has collision, store point before first collision in x_bounce
                Eigen::Vector3d x_bounce;
                BounceNeighbour neighbour_nodes; // to store neighbour info for normal point(at first)
                /* bounce case */
                if(!map_ptr_->isSegmentValid_B(nearest_node->x, x_new, x_bounce)) 
                {
                    //todo: calculate point after collision
                    Eigen::Vector3d x_post;
                    if(!calcPostCollisionPoint(nearest_node->x, x_bounce, x_post)) 
                        continue;
                    // check validity of x_post
                    // maybe it's better using x_post as the point before first occ_grid in ref_dir
                    if(!map_ptr_-> isSegmentValid(x_bounce, x_post)) 
                        continue;

                    // add x_post into kd_tree
                    TreeNodePtr newNode = nodes_pool_[valid_tree_node_nums_];
                    valid_tree_node_nums_++;

                    newNode->parent = nearest_node;
                    nearest_node->children.push_back(newNode);
                    newNode->x = x_post;
                    newNode->bounce_sample = true;
                    newNode->bounce_parent = x_bounce;
                    newNode->dir = (x_post - x_bounce).normalized();  // dir after bounce
                    
                    double dir_cos_p(0.0);
                    if(nearest_node->parent != nullptr)
                    {
                        Eigen::Vector3d dir1 = (x_bounce - nearest_node->x).normalized();
                        dir_cos_p = calDirCost(dir1, nearest_node->dir);
                    }    

                    newNode->dis_cost_from_parent = (x_bounce - nearest_node->x).norm() 
                        + (x_post - x_bounce).norm();    
                    newNode->dir_cost_from_parent = dir_cos_p;  // polyline dis
                    newNode->cost_from_parent = newNode->dis_cost_from_parent + newNode->dir_cost_from_parent; 

                    newNode->dis_cost_from_start = nearest_node->dis_cost_from_start + newNode->dis_cost_from_parent;
                    newNode->dir_cost_from_start = nearest_node->dir_cost_from_start + newNode->dir_cost_from_parent;           
                    newNode->cost_from_start = newNode->dis_cost_from_start + newNode->dir_cost_from_start;

                    kd_insert3(kd_tree, x_post(0), x_post(1), x_post(2), newNode);

                    // ROS_INFO("\033[1;32m[bounce_node] Now add a bounce point into kd_tree! \033[0m");
                }
                else  /* normal case */
                {
                    if(!map_ptr_ -> isStateValid(x_new))  // check end_point before add into kd_tree
                        continue;
                    
                    // find parent
                    neighbour_nodes.nearing_nodes.reserve(50);
                    neighbour_nodes.center = x_new;
                    kdres *nbr_set;
                    nbr_set = kd_nearest_range3(kd_tree, x_new(0), x_new(1), x_new(2), search_radius_);
                    if (nbr_set == nullptr)
                    {
                        ROS_ERROR("bkwd kd range query error");
                        continue;
                    }
                    while(!kd_res_end(nbr_set))
                    {
                        TreeNodePtr curr_node = (TreeNodePtr)kd_res_item_data(nbr_set);
                        neighbour_nodes.nearing_nodes.emplace_back(curr_node, false, false);
                        kd_res_next(nbr_set);
                    }   
                    kd_res_free(nbr_set);

                    /* choose parent in nearing_nodes */
                    double min_dis_p = (x_new - nearest_node->x).norm();
                    double min_dir_p = 0.0;
                    Eigen::Vector3d cur_dir = (x_new - nearest_node->x).normalized();
                    if(nearest_node->parent != nullptr)
                    {
                        // dir change cost
                        min_dir_p = calDirCost(cur_dir, nearest_node->dir);
                    }
                    double min_cos_p = min_dis_p + min_dir_p;
                    double min_cos_s = nearest_node->cost_from_start + min_cos_p; // cost from start
                    TreeNodePtr min_node(nearest_node);
                    
                    for(auto &node : neighbour_nodes.nearing_nodes)
                    {
                        if(node.node_ptr == nearest_node)
                            continue;
                        double tmp_cost_p, tmp_dis_p, tmp_dir_p(0.0);
                        tmp_dis_p = (x_new - node.node_ptr->x).norm();
                        Eigen::Vector3d tmp_dir = (x_new - node.node_ptr->x).normalized();
                        if(node.node_ptr->parent != nullptr)               // 3 nodes needed for cal dir_change_cost
                            tmp_dir_p = calDirCost(tmp_dir, nearest_node->dir);
                        tmp_cost_p = tmp_dis_p + tmp_dir_p;

                        double tmp_cost_s = node.node_ptr->cost_from_start + tmp_cost_p;

                        if(tmp_cost_s < min_cos_s)
                        {
                            bool connected = map_ptr_->isSegmentValid(node.node_ptr->x, x_new);
                            node.is_checked = true;
                            if(!connected)
                                continue;
                            
                            node.is_valid = true;
                            min_dis_p = tmp_dis_p;
                            min_dir_p = tmp_dir_p;

                            min_cos_p = tmp_cost_p;
                            min_cos_s = tmp_cost_s;
                            min_node = node.node_ptr;
                        }
                    }

                    // add x_new to kd_tree
                    TreeNodePtr newNode = nodes_pool_[valid_tree_node_nums_];
                    valid_tree_node_nums_++;
                    newNode->parent = min_node;
                    min_node->children.push_back(newNode);
                    newNode->x = x_new;
                    newNode->dir = (x_new - min_node->x).normalized();
                    newNode->cost_from_parent = min_cos_p;
                    newNode->cost_from_start = min_cos_s;
                    newNode->dis_cost_from_parent = min_dis_p;
                    newNode->dir_cost_from_parent = min_dir_p;
                    newNode->dis_cost_from_start = min_node->dis_cost_from_start + min_dis_p;
                    newNode->dir_cost_from_start = min_node->dir_cost_from_start + min_dir_p;
                    newNode->bounce_sample = false;

                    kd_insert3(kd_tree, x_new(0), x_new(1), x_new(2), newNode);
                }

                /* try to connect to goal */
                TreeNodePtr new_node = nodes_pool_[valid_tree_node_nums_-1]; //newly add node in pool
                double dist_to_goal = (new_node->x - goal_node_->x).norm();  //omit dir_change_cost to goal
                double dir_cos_goal = calDirCost((goal_node_->x - new_node->x).normalized(), new_node->dir);
                double cost_to_goal = dist_to_goal + dir_cos_goal;
                if(dist_to_goal <= search_radius_)
                {
                    bool connected2goal = map_ptr_->isSegmentValid(new_node->x, goal_node_->x);
                    bool better_path = goal_node_->cost_from_start > new_node->cost_from_start + cost_to_goal;
                    if(connected2goal && better_path)
                    {
                        if(!goal_found)
                            first_path_use_time_ = ros::Time::now().toSec() - rrt_start_time;
                        
                        goal_found = true;

                        ROS_INFO("\033[1;32m [bounce_rrt_star]: Now find a new way! \033[0m");
                        // connect new_node to goal_node
                        changeNodeParent(goal_node_, new_node, dist_to_goal, dir_cos_goal);
                        vector<Eigen::Vector3d> cur_best_path;
                        fillPath(goal_node_, cur_best_path);
                        path_list_.emplace_back(cur_best_path);
                        Eigen::Vector3d cost(goal_node_->cost_from_start, goal_node_->dis_cost_from_start, 
                            goal_node_->dir_cost_from_start);
                        solution_cost_time_pair_list_.emplace_back(cost, (ros::Time::now().toSec() - rrt_start_time));
                    }
                }

                /* rewire part */
                // construct neighbor info for bounce_sample
                if(new_node->bounce_sample)
                {
                    // bounce_sample could change parent, but could change child
                   
                    neighbour_nodes.nearing_nodes.reserve(50);
                    neighbour_nodes.center = new_node->x;
                    kdres *nbr_set;
                    nbr_set = kd_nearest_range3(kd_tree, new_node->x(0), new_node->x(1), new_node->x(2), search_radius_);
                    if(nbr_set == nullptr)
                    {
                        ROS_ERROR("bkwd kd range query error");
                        continue;
                    }
                    while(!kd_res_end(nbr_set))
                    {
                        TreeNodePtr cur_node = (TreeNodePtr) kd_res_item_data(nbr_set);
                        neighbour_nodes.nearing_nodes.emplace_back(cur_node, false, false);
                        kd_res_next(nbr_set);
                    }
                    kd_res_free(nbr_set);
                }
                // let cur_node be new_node's child, if it's better
                for (auto &cur_node : neighbour_nodes.nearing_nodes)
                {   
                    if(cur_node.node_ptr == new_node->parent)
                        continue;
                        
                    if(cur_node.node_ptr->bounce_sample)
                        continue;  //bounce node is not allowd to change parent now

                    // two parts, cost from start, cost to goal
                    double dis_to_child = (cur_node.node_ptr->x - new_node->x).norm();
                    Eigen::Vector3d tmp_dir = (cur_node.node_ptr->x - new_node->x).normalized();
                    double dir_cos_child = calDirCost(tmp_dir, new_node->dir); // satisfy at least 3 nodes, cal dir_change directly
                    double cos_to_child = dis_to_child + dir_cos_child;

                    bool better_from_start = new_node->cost_from_start + cos_to_child < cur_node.node_ptr->cost_from_start? 1 : 0;
                    bool better_to_goal =  new_node->cost_from_start + cos_to_child + (cur_node.node_ptr->x - goal_node_->x).norm()
                        < goal_node_->cost_from_parent ? 1 : 0; // omit dir_change_cost to goal
                        
                    if(better_from_start && better_to_goal) // only rewire when benefiting to reach goal
                    {
                        bool connected(false);
                        if(cur_node.is_checked)
                            connected = cur_node.is_valid;
                        else
                            connected = map_ptr_->isSegmentValid(new_node->x, cur_node.node_ptr->x);
                            
                        if(connected)
                        {
                            double goal_cost_before_rewire = goal_node_->cost_from_start;
                            changeNodeParent(cur_node.node_ptr, new_node, dis_to_child, dir_cos_child); // bounce_sample node is not permitted to change parent now
                            // check if goal_cost updated after rewire
                            if(goal_node_->cost_from_start < goal_cost_before_rewire)
                            {
                                vector<Eigen::Vector3d> cur_best_path;
                                fillPath(goal_node_, cur_best_path);
                                path_list_.push_back(cur_best_path);
                                Eigen::Vector3d cost(goal_node_->cost_from_start, goal_node_->dis_cost_from_start, 
                                    goal_node_->dir_cost_from_start);
                                solution_cost_time_pair_list_.emplace_back(cost, ros::Time::now().toSec() - rrt_start_time);
                            }
                        }
                    }/* go to next entry   */   
                } /* end of rewire  */    
            } /* end of sample once */

            /*** visulization part ***/
            vector<Eigen::Vector3d> vertice;
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;           
            // visualize for x_bounce and calculated normVec
            vector<Eigen::Vector3d> bounce_samples;
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> normVecs;

            sampleWholeTree(start_node_, vertice, edges, bounce_samples, normVecs);
            vector<visualization::BALL> balls;
            balls.reserve(vertice.size());
            visualization::BALL node_p;
            node_p.radius = 0.3;
            for(size_t i = 0; i < vertice.size(); i++)
            {
                node_p.center = vertice[i];
                balls.push_back(node_p);
            }
            vis_ptr_->visualize_balls(balls, "bounce_rrt_star/tree_vertice", visualization::Color::blue, 1.0);
            vis_ptr_->visualize_pairline(edges, "bounce_rrt_star/tree_edges", visualization::Color::green, 0.2);

            vis_ptr_->visualize_pointcloud(bounce_samples, "bounce_rrt_star/x_bounce");  // visualize x_bounces
            vis_ptr_->visualize_arrows(normVecs, "bounce_rrt_star/normVecs", visualization::orange);

            if(goal_found)
            {
                final_path_use_time_ = ros::Time::now().toSec() - rrt_start_time;
                fillPath(goal_node_, final_path_);
                ROS_INFO_STREAM("[Bounce RRT*]: first_path_use_time: " << first_path_use_time_ << 
                     ", total cost: " << solution_cost_time_pair_list_.front().first(0) <<
                     ", dis_cost: " << solution_cost_time_pair_list_.front().first(1) <<
                     ", dir_cost: " << solution_cost_time_pair_list_.front().first(2));
            }
            else if (valid_tree_node_nums_ == max_tree_node_nums_)
            {
                ROS_ERROR_STREAM("[Bounce RRT*]: Not connected to goal after " << max_tree_node_nums_ << " nodes added to rrt_tree.");
            }
            else 
            {
                ROS_ERROR_STREAM("[Bounce RRT*]: Not connected to goal after " << ros::Time::now().toSec() - rrt_start_time << " seconds.");
            }

            return goal_found; 
        }

        // dis <= steer_length, return x_rand; else return the steer node
        Eigen::Vector3d steer(const Eigen::Vector3d &nearest_node_p, const Eigen::Vector3d &rand_node_p, double len)
        {
            Eigen::Vector3d diff_vec = rand_node_p - nearest_node_p;
            double dist = diff_vec.norm();
            if (diff_vec.norm() <= len)
                return rand_node_p;
            else
                return nearest_node_p + diff_vec * len / dist;
        }
        
        //calc point after collision
        bool calcPostCollisionPoint(const Eigen::Vector3d &parent, const Eigen::Vector3d &x_bounce, Eigen::Vector3d &x_post)
        {
            // ROS_INFO("[bounce] now calculate a point after bounce!!!");

            Eigen::Vector3d normVec;
            // can not calc normVec
            if(!map_ptr_->calNormVector(x_bounce, normVec)) 
                return false;
            
            // Eigen::Vector3d incidence = x_bounce - parent;
            Eigen::Vector3d dir1 = parent  - x_bounce;
            double dis = dir1.norm() * dis_ratio; // reflect dis
            double d1 = dir1.dot(normVec);
            if(d1 <= 0)  // incocident angle >= 90 degrees
                return false;
            Eigen::Vector3d p2 = x_bounce + normVec * d1;
            Eigen::Vector3d p3 = parent + (p2 - parent) * 2.0;
            Eigen::Vector3d dir_ref = (p3 - x_bounce).normalized(); // reflect dir
            x_post = x_bounce + dir_ref * dis; 

            return true;
        }

        // calc dir change cost(dir1:latter vel, dir2:former vel)
        // return (1.0 - dir1.dot(dir2)) * dir_wei
        double calDirCost(const Eigen::Vector3d &dir1, const Eigen::Vector3d &dir2)
        {
            double cost = (1.0 - dir1.dot(dir2)) * dir_wei; // dir change cost

            return cost;
        }

        // change parent for &node, update children's cost of &node
        void changeNodeParent(TreeNodePtr &node, TreeNodePtr &parent, const double &dis_cos_p, const double &dir_cos_p)
        {
            if(node->bounce_sample)
            {
                ROS_WARN("[bounce rrt*] Node is a bounce_sample, now it's not permitted to change parent.");
                return;
            }

            if(node->parent)
                node->parent->children.remove(node);  // cut down 恩断义绝233
            
            node->parent = parent;
            node->dir = (node->x - parent->x).normalized(); //important
            node->dis_cost_from_parent = dis_cos_p;
            node->dir_cost_from_parent = dir_cos_p;
            node->cost_from_parent = node->dis_cost_from_parent + node->dir_cost_from_parent;

            node->dis_cost_from_start = parent->dis_cost_from_start + node->dis_cost_from_parent;
            node->dir_cost_from_start = parent->dir_cost_from_start + node->dir_cost_from_parent;
            node->cost_from_start = node->dis_cost_from_start + node->dir_cost_from_start;

            parent->children.push_back(node);

            // change node's descendants, change their cost from parent and start
            TreeNodePtr descendant(node);
            std::queue<TreeNodePtr> Q;
            Q.push(descendant);
            while(!Q.empty())
            {
                descendant = Q.front();
                Q.pop();
                for(auto &leafPtr : descendant->children)
                {
                    leafPtr->dis_cost_from_start = leafPtr->dis_cost_from_parent + descendant->dis_cost_from_start;
                    leafPtr->dir_cost_from_start = leafPtr->dir_cost_from_parent + descendant->dir_cost_from_start;
                    leafPtr->cost_from_start = leafPtr->dis_cost_from_start + leafPtr->dir_cost_from_start;
                    Q.push(leafPtr);
                }
            }
        }

        void fillPath(const TreeNodePtr &node, vector<Eigen::Vector3d> &path)
        {
            path.clear();
            TreeNodePtr nodePtr = node;
            while(nodePtr->parent)
            {
                path.push_back(nodePtr->x);  // add cur->x
                if(nodePtr->bounce_sample)
                    path.push_back(nodePtr->bounce_parent); //add x_bounce for show
                nodePtr = nodePtr->parent;   // last, add parent
            }
            path.push_back(nodePtr->x);

            // reverse(path.begin(),path.end());
            std::reverse(std::begin(path), std::end(path));
        }

        void sampleWholeTree(const TreeNodePtr &root, vector<Eigen::Vector3d> &vertice, 
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &edges,
            vector<Eigen::Vector3d> &bounce_samples,
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> &arrows)
        {
            if(root == nullptr)
                return;
            
            // get all nodes and edges in the tree
            TreeNodePtr node = root;
            std::queue<TreeNodePtr> Q;
            Q.push(node);
            vertice.push_back(node->x);  // add root node

            while(!Q.empty())
            {
                node = Q.front();
                Q.pop();
                for(const auto &leafPtr : node->children)
                {
                    if(leafPtr->bounce_sample)
                    {
                        vertice.push_back(leafPtr->bounce_parent); // add x-bounce
                        edges.emplace_back(std::make_pair(node->x, leafPtr->bounce_parent)); // incidence
                        vertice.push_back(leafPtr->x);
                        edges.emplace_back(std::make_pair(leafPtr->bounce_parent, leafPtr->x)); // reflect

                        bounce_samples.push_back(leafPtr->bounce_parent);
                        Eigen::Vector3d normVec;
                        if(map_ptr_->calNormVector(leafPtr->bounce_parent, normVec))
                            arrows.emplace_back(std::make_pair(leafPtr->bounce_parent, leafPtr->bounce_parent + normVec));
                    }
                    else
                    {
                        vertice.push_back(leafPtr->x);
                        edges.emplace_back(std::make_pair(node->x, leafPtr->x));
                    }

                    Q.push(leafPtr);
                }
            }
        }
        


    

    };
}

#endif