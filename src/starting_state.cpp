#include "starting_state.h"

starting_state::starting_state(){}

std::map< transition, bool > starting_state::getResults()
{

}

void starting_state::run()
{
    usleep(50000);
}

bool starting_state::isComplete()
{
    return false;
}

std::string starting_state::get_type()
{
    return "starting_state";
}