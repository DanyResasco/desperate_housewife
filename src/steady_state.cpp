#include "Steady_state.h"

steady_state::steady_state()
{
  nh.param<std::string>("/BasicGeometriesNode/geometries_topic", geometries_topic_, "/BasicGeometriesNode/geometries");
  stream_subscriber_ = nh.subscribe(geometries_topic_, 1, &steady_state::steady_stateCallback, this);
  finish = false;
}

std::map< transition, bool > steady_state::getResults()
{
  std::map< transition, bool > results;
  if(cylinder_geometry.geometries.size() > 0)
    {
      results[transition::Geometries_ok] = true;
      finish = false;
    }

  return results;
}

void steady_state::run()
{
  usleep(200000);

}


void steady_state::steady_stateCallback(const desperate_housewife::fittedGeometriesArray::ConstPtr& msg)
{
  cylinder_geometry.geometries.resize(msg->geometries.size());

  for(unsigned int i = 0; i < msg->geometries.size(); i++ )
    {
      cylinder_geometry.geometries[i] = msg->geometries[i];
    }
  finish = true;
}


bool steady_state::isComplete()
{
  return finish;
}

std::string steady_state::get_type()
{
  return "Steady state";
}

void steady_state::reset()
{
}
