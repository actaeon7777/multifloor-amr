// Batch existing PhysX scene queries without per-ray Python calls or callbacks.
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <omni/physx/IPhysxSceneQuery.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <vector>

namespace py = pybind11;
using FloatArray = py::array_t<float, py::array::c_style | py::array::forcecast>;

class BatchRaycaster
{
public:
    BatchRaycaster(py::object query, const std::vector<uint64_t>& ignored)
        : owner_(std::move(query)), thread_(std::this_thread::get_id())
    {
        // Use pybind11's registered type conversion. No integer pointer casts,
        // capsule layout assumptions, or access to private Python object memory.
        query_ = owner_.cast<omni::physx::IPhysxSceneQuery*>();
        if (!query_ || !query_->raycastAll)
            throw std::runtime_error("PhysX scene-query interface is unavailable");
        set_ignored(ignored);
    }

    void set_ignored(const std::vector<uint64_t>& ignored)
    {
        check_thread();
        ignored_.clear();
        ignored_.insert(ignored.begin(), ignored.end());
    }

    py::array_t<float> cast(const FloatArray& origin, const FloatArray& directions, float max_distance)
    {
        check_thread();
        if (!query_)
            throw std::runtime_error("Raycaster is closed");
        if (origin.ndim() != 1 || origin.shape(0) != 3)
            throw std::invalid_argument("origin must have shape (3,)");
        if (directions.ndim() != 2 || directions.shape(1) != 3)
            throw std::invalid_argument("directions must have shape (N, 3)");
        if (!std::isfinite(max_distance) || max_distance <= 0)
            throw std::invalid_argument("max_distance must be finite and positive");
        const auto o = origin.unchecked<1>();
        const auto rays = directions.unchecked<2>();
        for (int axis = 0; axis < 3; ++axis)
            if (!std::isfinite(o(axis)))
                throw std::invalid_argument("origin must be finite");
        for (py::ssize_t i = 0; i < rays.shape(0); ++i)
        {
            const float norm2 = rays(i, 0)*rays(i, 0) + rays(i, 1)*rays(i, 1) + rays(i, 2)*rays(i, 2);
            if (!std::isfinite(norm2) || std::abs(norm2 - 1.f) > .001f)
                throw std::invalid_argument("directions must be finite unit vectors");
        }
        py::array_t<float> result(rays.shape(0));
        auto distances = result.mutable_unchecked<1>();
        const carb::Float3 start{o(0), o(1), o(2)};
        // The caller runs on the simulation thread after a completed PhysX step.
        // Do not dispatch queries to other threads while the scene can mutate.
        {
            py::gil_scoped_release release;
            for (py::ssize_t i = 0; i < rays.shape(0); ++i)
            {
                float closest = std::numeric_limits<float>::infinity();
                const carb::Float3 direction{rays(i, 0), rays(i, 1), rays(i, 2)};
                query_->raycastAll(start, direction, max_distance,
                    [this, &closest](const omni::physx::RaycastHit& hit) {
                        if (ignored_.find(hit.collision) == ignored_.end())
                            closest = std::min(closest, hit.distance);
                        // Hit order is unspecified. Continue behind own housing
                        // and select the nearest environmental surface.
                        return true;
                    }, false);
                distances(i) = closest;
            }
        }
        return result;  // Python owns this array; it survives future physics steps.
    }

    size_t ignored_count() const { return ignored_.size(); }

    void close()
    {
        check_thread();
        query_ = nullptr;
        owner_ = py::none();
    }

private:
    void check_thread() const
    {
        if (std::this_thread::get_id() != thread_)
            throw std::runtime_error("PhysX queries must run on the owning simulation thread");
    }
    py::object owner_;
    omni::physx::IPhysxSceneQuery* query_ = nullptr;
    std::thread::id thread_;
    std::unordered_set<uint64_t> ignored_;
};

PYBIND11_MODULE(_amr_lidar_native, module)
{
    module.doc() = "PhysX batch LiDAR queries, native self filtering and nearest-hit reduction";
    module.attr("backend") = "physx_scene_query_cpp_v1";
    py::class_<BatchRaycaster>(module, "BatchRaycaster")
        .def(py::init<py::object, const std::vector<uint64_t>&>(), py::arg("query"), py::arg("ignored_paths"))
        .def("cast", &BatchRaycaster::cast, py::arg("origin"), py::arg("directions"), py::arg("max_distance"))
        .def("set_ignored", &BatchRaycaster::set_ignored)
        .def_property_readonly("ignored_count", &BatchRaycaster::ignored_count)
        .def("close", &BatchRaycaster::close);
}
