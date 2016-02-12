#ifndef STARTING_STATE_H
#define STARTING_STATE_H

#include "state.h"

class starting_state : public state<transition>
{
public:
    starting_state();
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
};

#endif // STARTING_STATE_H