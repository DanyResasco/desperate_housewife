#ifndef STATE_H
#define STATE_H
#include <map>
#include "transition.h"

template <class property>
class state
{
private:

public:
    state(){}
    virtual ~state(){}
    virtual bool isComplete()=0;
    virtual void run()=0;
    virtual std::map<property,bool> getResults()=0;
    virtual std::string get_type()=0;
    virtual void reset(){}
};

#endif 