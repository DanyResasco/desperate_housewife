
#include <iostream>

#include <ecl/geometry.hpp>   




double interpolator(double t, double t_des){
    
    ecl::CubicSpline cubic;
    // ecl::CubicSpline cubic;
    // std::vector<double> t_set, y_set;
    ecl::Array<double> t_set(7);
    ecl::Array<double>  y_set(7);

    t_set << 0.0, 0.01, 0.02, 0.97,0.98,0.99,1.0;
      // std::cout<<"t_set: "<<t_set<<std::endl;


    y_set << 0.0,0.0,0.0,1.0,1.0,1.0;
    // std::cout<<"y_set: "<<y_set<<std::endl;

    cubic = ecl::CubicSpline::Natural(t_set,y_set);

    // cubic = ecl::CubicSpline::DerivativeHeuristic(t_set, y_set, 0, 0);
    // cubic = ecl::CubicSpline::ContinuousDerivatives(t_set, y_set, 0, 0);

    return cubic(t/t_des);
}

int main()
{

    double t_des = 1;
    for (double t= 0; t < t_des; t=t+.01)
    {
       std::cout << t << "\t" << interpolator(t, t_des) << std::endl;
    }
    return 0;
}
