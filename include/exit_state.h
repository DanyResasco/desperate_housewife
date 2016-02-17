#ifndef EXIT_STATE_H
#define EXIT_STATE_H

#include "state.h"

class exit_state : public state<transition>
{
public:
    exit_state(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();

};

#endif // EXIT_STATE_H