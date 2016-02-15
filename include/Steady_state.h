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
    	void steady_stateCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg);
    	ros::Subscriber stream_subscriber_;
    	std::string geometries_topic_;
    	bool finish;
    	ros::NodeHandle nh;
};

#endif // STEADY_STATE_H