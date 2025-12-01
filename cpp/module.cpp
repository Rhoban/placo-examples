#include "example.h"
#include <boost/python.hpp>
#include <eigenpy/eigenpy.hpp>
#include <ostream>

using namespace boost::python;

void expose_example() {
  class_<SolverExample>("SolverExample")
      .def("update_trajectory", &SolverExample::update_trajectory)
      .def("get_robot", &SolverExample::get_robot,
           return_value_policy<reference_existing_object>());
}

BOOST_PYTHON_MODULE(example) { expose_example(); }