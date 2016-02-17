#ifndef TABLE_STATE_H
#define TABLE_STATE_H

#include "state.h"

class Place_table_state : public state<transition>
{
public:
    Place_table_state(const shared& data);
    virtual std::map< transition, bool > getResults();
    virtual void run();
    virtual bool isComplete();
    virtual std::string get_type();
};

#endif // TABLE_STATE_H