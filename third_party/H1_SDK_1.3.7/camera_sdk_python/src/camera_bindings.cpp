#include <cstdint>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "camera_client.h"

namespace py = pybind11;

namespace {

py::array mat_to_numpy(const cv::Mat& mat) {
    if (mat.empty()) {
        return py::array();
    }

    cv::Mat contiguous = mat.isContinuous() ? mat : mat.clone();
    const int channels = contiguous.channels();
    const int depth = contiguous.depth();

    py::dtype dtype;
    switch (depth) {
        case CV_8U:
            dtype = py::dtype::of<std::uint8_t>();
            break;
        case CV_8S:
            dtype = py::dtype::of<std::int8_t>();
            break;
        case CV_16U:
            dtype = py::dtype::of<std::uint16_t>();
            break;
        case CV_16S:
            dtype = py::dtype::of<std::int16_t>();
            break;
        case CV_32S:
            dtype = py::dtype::of<std::int32_t>();
            break;
        case CV_32F:
            dtype = py::dtype::of<float>();
            break;
        case CV_64F:
            dtype = py::dtype::of<double>();
            break;
        default:
            throw std::runtime_error("Unsupported cv::Mat depth: " + std::to_string(depth));
    }

    std::vector<py::ssize_t> shape;
    std::vector<py::ssize_t> strides;
    const py::ssize_t elem_size = static_cast<py::ssize_t>(contiguous.elemSize1());

    if (channels == 1) {
        shape = {
            static_cast<py::ssize_t>(contiguous.rows),
            static_cast<py::ssize_t>(contiguous.cols),
        };
        strides = {
            static_cast<py::ssize_t>(contiguous.step),
            elem_size,
        };
    } else {
        shape = {
            static_cast<py::ssize_t>(contiguous.rows),
            static_cast<py::ssize_t>(contiguous.cols),
            static_cast<py::ssize_t>(channels),
        };
        strides = {
            static_cast<py::ssize_t>(contiguous.step),
            static_cast<py::ssize_t>(contiguous.elemSize()),
            elem_size,
        };
    }

    py::array result(dtype, shape, strides);
    std::memcpy(result.mutable_data(), contiguous.data, contiguous.total() * contiguous.elemSize());
    return result;
}

py::object frame_to_python(const std::optional<std::pair<cv::Mat, double>>& frame) {
    if (!frame) {
        return py::none();
    }

    return py::make_tuple(mat_to_numpy(frame->first), frame->second);
}

}  // namespace

PYBIND11_MODULE(_camera_client_native, m) {
    m.doc() = "Python 3.12 native bindings for Zerith camera_sdk_cpp";

    py::class_<CameraClient>(m, "CameraClient")
        .def(py::init<const std::string&>(), py::arg("grpc_target"))
        .def("start", [](CameraClient& self) {
            py::gil_scoped_release release;
            self.start();
        })
        .def("stop", [](CameraClient& self) {
            py::gil_scoped_release release;
            self.stop();
        })
        .def("get_latest_frame", [](CameraClient& self, const std::string& camera_name) {
            std::optional<std::pair<cv::Mat, double>> frame;
            {
                py::gil_scoped_release release;
                frame = self.getLatestFrame(camera_name);
            }
            return frame_to_python(frame);
        }, py::arg("camera_name"))
        .def("get_latest_depth", [](CameraClient& self, const std::string& camera_name) {
            std::optional<std::pair<cv::Mat, double>> frame;
            {
                py::gil_scoped_release release;
                frame = self.getLatestDepth(camera_name);
            }
            return frame_to_python(frame);
        }, py::arg("camera_name"))
        .def("get_state_bytes", [](CameraClient& self, const std::vector<std::string>& camera_names, float timeout_sec) {
            robot::RecorderStateResponse state;
            {
                py::gil_scoped_release release;
                state = self.get_state(camera_names, timeout_sec);
            }
            std::string serialized;
            if (!state.SerializeToString(&serialized)) {
                throw std::runtime_error("Failed to serialize RecorderStateResponse");
            }
            return py::bytes(serialized);
        }, py::arg("camera_names") = std::vector<std::string>{}, py::arg("timeout_sec") = 5.0f);
}
