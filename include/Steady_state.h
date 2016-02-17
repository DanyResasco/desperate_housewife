#ifndef STEADY_STATE_H
#define STEADY_STATE_H

#include "state.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <desperate_housewife/fittedGeometriesArray.h>

class steady_state : public state<transition>
{
public:
    steady_state();
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
    virtual void reset();
    
    private:
    	desperate_housewife::fittedGeometriesArray cylinder_geometry;

    	ros::Subscriber stream_subscriber_;
    	std::string geometries_topic_;
    	bool finish;
    	ros::NodeHandle nh;

        void steady_stateCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
        /*! 
          * \fn  steady_stateCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
          * \brief callback that store the number of cluster in the scene
          * \param  ros message
          * \return void
        */ 
};

#endif // STEADY_STATE_H