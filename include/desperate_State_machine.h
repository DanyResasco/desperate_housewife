#ifndef DESPERATE_MIND_H
#define DESPERATE_MIND_H

#include <ros/ros.h>
#include <state.h>
#include <state_machine.hpp>

#include <thread>

// #include "dual_manipulation_shared/state_manager_service.h"

// namespace dual_manipulation{
//     namespace state_manager{

        class Desp_state_server
        {
            public:
                Desp_state_server();
                ~Desp_state_server();
                void join();
                 ros::NodeHandle node;
                // shared_memory data;
            private:
                ros::AsyncSpinner aspin;
                void loop();
                void init();
                std::thread loop_thread;
                void reset();
                 // bool state_manager();
               
                // bool state_manager_ros_service(dual_manipulation_shared::state_manager_service::Request &req, dual_manipulation_shared::state_manager_service::Response &res);
                // ros::ServiceServer service;
                std::vector<std::tuple<state<transition>*,transition_type,state<transition>* > > Grafo;
                state_machine< state<transition>*,transition_type > sm;
                state<transition>* current_state;
                std::map<transition,bool> transition_map;
                ros::Publisher state_pub;
        };


//     }
// }
#endif 