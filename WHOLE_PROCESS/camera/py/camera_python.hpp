//
// Created by Jarry_Goon on 2023-07-18.
//

#ifndef HADA_CAMERA_PYTHON_HPP
#define HADA_CAMERA_PYTHON_HPP

#include "../../include/sensor/camera/camera.hpp"
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <utility>

namespace py = boost::python;
namespace np = boost::python::numpy;

class camera_python: public camera
{
public:
    camera_python(float height, float x_min, float x_max, float y_min, float y_max,
                  const py::str& cam_file, float pan, float tilt, float roll):
            camera(height, x_min, x_max, y_min, y_max, py::extract<std::string>(cam_file), pan, tilt, roll)
            {}
    
    np::ndarray bev(const np::ndarray& frame);
};

#endif //HADA_CAMERA_PYTHON_HPP
