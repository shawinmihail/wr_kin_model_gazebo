#include <boost/python.hpp>
#include <Python.h>
#include <surf_fcn.h>

double surf_alt(double x, double y)
{
    double out[4];
    surf_fcn(x, y, out);
    return out[0];
}

boost::python::list surf_grad(double x, double y)
{
    double out[4];
    surf_fcn(x, y, out);
    boost::python::list ret;
    ret.append(out[1]);
    ret.append(out[2]);
    ret.append(out[3]);
    return ret;
}

BOOST_PYTHON_MODULE(surf_fcn_python_wrapper)
{
    using namespace boost::python;
    def("surf_alt", surf_alt);
    def("surf_grad", surf_grad);
}
