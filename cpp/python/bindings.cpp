// xblick._core: the C++ rings for Python. Buffers cross without a copy: host slots as NumPy arrays (views into the
// shared memory, kept alive by the ring object), device buffers as pointers that xblick.gpu wraps with
// __cuda_array_interface__. Copies and waits release the GIL.
#include <algorithm>
#include <cstring>
#include <vector>

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/tuple.h>
#include <nanobind/stl/vector.h>

#include "xblick/command.hpp"
#include "xblick/cuda_ring.hpp"
#include "xblick/pairing.hpp"
#include "xblick/paths.hpp"
#include "xblick/protocol.hpp"
#include "xblick/ray_cloud.hpp"
#include "xblick/rectify.hpp"
#include "xblick/ring.hpp"
#include "xblick/ros_cuda.hpp"
#include "xblick/ros_gid.hpp"
#include "xblick/rvl.hpp"
#include "xblick/version.hpp"

namespace nb = nanobind;
using namespace nb::literals;
using xblick::Written;

namespace
{
    using HostArray = nb::ndarray<nb::device::cpu, nb::c_contig>;          // written to (out)
    using InArray = nb::ndarray<nb::ro, nb::device::cpu, nb::c_contig>;    // only read: read-only arrays are fine
    using ByteView = nb::ndarray<nb::numpy, std::uint8_t, nb::ndim<1>>;
    using ReadOnlyByteView = nb::ndarray<nb::numpy, const std::uint8_t, nb::ndim<1>>;

    template <typename Array>
    std::uint64_t nbytes_of(const Array& a)
    {
        return static_cast<std::uint64_t>(a.size()) * a.itemsize();
    }

    nb::tuple pair(const Written& w)
    {
        return nb::make_tuple(w.slot, w.seq);
    }
}  // namespace

NB_MODULE(_core, m)
{
    m.doc() = "Durchblick's C++ core: shared-memory and CUDA rings (cpp/include/xblick/ring.hpp describes them)";
    nb::exception<xblick::RingError>(m, "RingError", PyExc_RuntimeError);
    nb::exception<xblick::rvl::Error>(m, "RvlError", PyExc_ValueError);

    m.def("host_id", &xblick::host_id, "boot id of this host; descriptors carry it");
    m.def("process_start_time", &xblick::process_start_time, "pid"_a,
          "start time of a process (/proc/<pid>/stat field 22), as in a ring's creator record");
    // the C++ rule of xblick.paths (paths.hpp), for tests/test_paths.py's comparison; Python uses its own
    m.def("cpp_cache_dir", [](const std::string& node) { return xblick::cache_dir(node).string(); });
    m.def("cpp_data_dir", [](const std::string& node) { return xblick::data_dir(node).string(); });
    m.def("cpp_config_dir", [] { return xblick::config_dir().string(); });
    m.def("ros_gid", [](const std::string& token_key) {
        const xblick::ros::Gid gid = xblick::ros::gid(token_key);
        return nb::bytes(reinterpret_cast<const char*>(gid.data()), gid.size());
    }, "token_key"_a, "rmw_zenoh's gid of a ROS entity: XXH3-128 of its liveliness token key (ros_gid.hpp)");
    m.def("rvl_encode", [](InArray pixels) {
              if (pixels.ndim() != 2 or pixels.dtype() != nb::dtype<std::uint16_t>())
              {
                  throw nb::value_error("rvl_encode takes a 2-D uint16 array");
              }
              const auto bytes = xblick::rvl::encode(pixels.data(), static_cast<std::uint32_t>(pixels.shape(0)),
                                                     static_cast<std::uint32_t>(pixels.shape(1)));
              return nb::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
          }, "pixels"_a, "a 16-bit image as ROS's compressedDepth rvl writes it (rvl.hpp)");
    m.def("rvl_shape", [](InArray data) {
              const auto [height, width] =
                  xblick::rvl::shape({static_cast<const std::uint8_t*>(data.data()), data.nbytes()});
              return std::make_tuple(height, width);
          }, "data"_a, "(height, width) of rvl_encode's bytes");
    m.def("rvl_decode", [](InArray data, HostArray out) {
              const std::span<const std::uint8_t> bytes{static_cast<const std::uint8_t*>(data.data()), data.nbytes()};
              const auto [height, width] = xblick::rvl::shape(bytes);
              if (out.dtype() != nb::dtype<std::uint16_t>() or out.nbytes() != std::size_t{height} * width * 2)
              {
                  throw nb::value_error("rvl_decode: out is not a uint16 array of rvl_shape's size");
              }
              xblick::rvl::decode(bytes, out.data());
          }, "data"_a, "out"_a, "decodes rvl_encode's bytes into out, a uint16 array of rvl_shape(data)");
    // cloud/compressed (ray_cloud.hpp): points and rays as bytes, x, y, z at their offsets in a point
    m.def("ray_cloud_ranges", [](InArray points, std::uint32_t point_step, std::uint32_t x, std::uint32_t y,
                                 std::uint32_t z, InArray rays, double range_step, HostArray steps) {
              const std::size_t count = steps.nbytes() / 2;
              if (steps.dtype() != nb::dtype<std::uint16_t>() or rays.dtype() != nb::dtype<float>()
                  or rays.nbytes() != count * 12 or points.nbytes() != count * point_step
                  or std::max({x, y, z}) + 4 > point_step)
              {
                  throw nb::value_error("ray_cloud_ranges: points, rays (float32, 3 per point) and steps (uint16) "
                                        "are not of one size");
              }
              const auto fit = xblick::ray_cloud::ranges(
                  points.data(), count, {point_step, x, y, z}, static_cast<const float*>(rays.data()), range_step,
                  static_cast<std::uint16_t*>(steps.data()));
              return std::make_tuple(fit.valid, fit.low, fit.high, fit.deviation, fit.limit);
          }, "points"_a, "point_step"_a, "x"_a, "y"_a, "z"_a, "rays"_a, "range_step"_a, "steps"_a,
          "writes the points' ranges on their rays to steps; (valid, low, high, deviation, limit)");
    m.def("ray_cloud_points", [](InArray steps, InArray rays, double range_step, HostArray points,
                                 std::uint32_t point_step, std::uint32_t x, std::uint32_t y, std::uint32_t z) {
              const std::size_t count = steps.nbytes() / 2;
              if (steps.dtype() != nb::dtype<std::uint16_t>() or rays.dtype() != nb::dtype<float>()
                  or rays.nbytes() != count * 12 or points.nbytes() != count * point_step
                  or std::max({x, y, z}) + 4 > point_step)
              {
                  throw nb::value_error("ray_cloud_points: steps (uint16), rays (float32, 3 per point) and points "
                                        "are not of one size");
              }
              xblick::ray_cloud::points(static_cast<const std::uint16_t*>(steps.data()), count,
                                        static_cast<const float*>(rays.data()), range_step,
                                        {point_step, x, y, z}, points.data());
          }, "steps"_a, "rays"_a, "range_step"_a, "points"_a, "point_step"_a, "x"_a, "y"_a, "z"_a,
          "writes x, y, z of the points of these ranges on their rays (NaN for 0)");
    // rectification (rectify.hpp): the map of a camera, as cv2.initUndistortRectifyMap gives it
    m.def("rectify_map", [](std::uint32_t width, std::uint32_t height, InArray k, const std::string& model, InArray d,
                            InArray r, InArray p, HostArray x, HostArray y) {
              const auto doubles = [](const InArray& a, std::size_t n, const char* what) {
                  if (a.dtype() != nb::dtype<double>() or (n != 0 and a.nbytes() != n * sizeof(double)))
                  {
                      throw nb::value_error((std::string("rectify_map: ") + what + " is not float64 of its size")
                                                .c_str());
                  }
                  const auto* v = static_cast<const double*>(a.data());
                  return std::vector<double>(v, v + a.nbytes() / sizeof(double));
              };
              xblick::RectifyCamera c;
              c.width = width;
              c.height = height;
              const auto kv = doubles(k, 9, "k"), rv = doubles(r, 9, "r"), pv = doubles(p, 12, "p");
              std::copy(kv.begin(), kv.end(), c.k.begin());
              std::copy(rv.begin(), rv.end(), c.r.begin());
              std::copy(pv.begin(), pv.end(), c.p.begin());
              c.distortion_model = model;
              c.d = doubles(d, 0, "d");
              const std::size_t n = std::size_t{width} * height;
              if (x.dtype() != nb::dtype<float>() or y.dtype() != nb::dtype<float>() or x.nbytes() != n * 4
                  or y.nbytes() != n * 4)
              {
                  throw nb::value_error("rectify_map: x and y are not float32 of height x width");
              }
              xblick::RectifyMap map;
              try
              {
                  map = xblick::rectify_map(c);
              }
              catch (const std::invalid_argument& e)
              {
                  throw nb::value_error(e.what());
              }
              std::memcpy(x.data(), map.x.data(), n * 4);
              std::memcpy(y.data(), map.y.data(), n * 4);
          }, "width"_a, "height"_a, "k"_a, "distortion_model"_a, "d"_a, "r"_a, "p"_a, "x"_a, "y"_a,
          "writes the rectification map of the camera (rectify.hpp) into x and y (float32, height x width)");
    // a command's attachment (command.hpp): the C++ rules, which tests compare with session.py's
    m.def("command_attachment", [](nb::bytes data, const std::string& zid, std::int64_t now_ns) {
              const xblick::CommandAttachment a =
                  xblick::parse_command_attachment(std::string_view(data.c_str(), data.size()));
              const char* kind = a.kind == xblick::CommandAttachment::Kind::none      ? "none"
                                 : a.kind == xblick::CommandAttachment::Kind::invalid ? "invalid"
                                                                                       : "ours";
              std::string receipt, effect;
              if (const auto r = xblick::refuse_on_receipt(a, zid))
              {
                  receipt = r->schema;
              }
              else if (const auto e = xblick::refuse_at_effect(a, now_ns))
              {
                  effect = e->schema;
              }
              return nb::make_tuple(std::string(kind), a.version, a.node, a.deadline_ns, a.zid, a.clock_ns, a.checked,
                                    receipt, effect);
          }, "data"_a, "zid"_a, "now_ns"_a,
          "(kind, version, node, deadline_ns, zid, clock_ns, checked, refused on receipt, refused at the effect) of "
          "a command's attachment by command.hpp's rules, for the tests");
    // pairing by stamp (pairing.hpp): history.PartnerHistory pairs with these
    m.attr("PAIR_MAX_OFFSET_NS") = xblick::kPairMaxOffsetNs;
    m.def("partner_period_ns", &xblick::partner_period_ns, "kept"_a,
          "the median step of the kept stamps, each once; None before two different stamps");
    m.def("partner_of", [](const std::vector<std::int64_t>& kept, std::int64_t stamp_ns,
                           std::optional<std::int64_t> max_offset_ns) {
              try
              {
                  return xblick::partner_of(kept, stamp_ns, max_offset_ns);
              }
              catch (const std::invalid_argument& e)
              {
                  throw nb::value_error(e.what());
              }
          }, "kept"_a, "stamp_ns"_a, "max_offset_ns"_a.none(),
          "the stamp of the partner sample of the primary frame stamped stamp_ns (pairing.hpp); None if none (yet)");
    m.def("cuda_device_count", &xblick::cuda_device_count, "CUDA devices; 0 without driver or device");
    m.def("cuda_device_uuid", &xblick::cuda_device_uuid, "ordinal"_a,
          "the UUID of the CUDA device of this ordinal (32 hex digits); empty without a driver");
    m.def("cuda_device_ordinal", &xblick::cuda_device_ordinal, "uuid"_a,
          "ordinal of the CUDA device with this UUID; -1 if this process does not see it");
    m.attr("MAX_SLOTS") = xblick::kMaxSlots;
    m.attr("__version__") = XBLICK_VERSION;

    namespace p = xblick::protocol;
    nb::module_ protocol = m.def_submodule("protocol", "the wire protocol's names and versions (protocol.hpp)");
    protocol.attr("IMAGE_RAW") = std::string(p::kImageRaw);
    protocol.attr("COMPRESSED") = std::string(p::kCompressed);
    protocol.attr("SHM") = std::string(p::kShm);
    protocol.attr("CUDA") = std::string(p::kCuda);
    protocol.attr("CAMERA_INFO") = std::string(p::kCameraInfo);
    protocol.attr("INFO") = std::string(p::kInfo);
    protocol.attr("WANT") = std::string(p::kWant);
    protocol.attr("TF_STATIC") = std::string(p::kTfStatic);
    protocol.attr("ADVANCED_PUBLISHER") = std::string(p::kAdvancedPublisher);
    protocol.attr("TF") = std::string(p::kTf);
    protocol.attr("IMU") = std::string(p::kImu);
    protocol.attr("DIAGNOSTICS") = std::string(p::kDiagnostics);
    protocol.attr("JOINT_STATES") = std::string(p::kJointStates);
    protocol.attr("ODOM") = std::string(p::kOdom);
    protocol.attr("WRENCH") = std::string(p::kWrench);
    protocol.attr("CLOUD") = std::string(p::kCloud);
    protocol.attr("CLOUD_SHM") = std::string(p::kCloudShm);
    protocol.attr("CLOUD_CUDA") = std::string(p::kCloudCuda);
    protocol.attr("CLOUD_COMPRESSED") = std::string(p::kCloudCompressed);
    protocol.attr("CLOUD_RAYS") = std::string(p::kCloudRays);
    protocol.attr("SENT") = std::string(p::kSent);
    protocol.attr("CLOCK") = std::string(p::kClock);
    protocol.attr("EVENTS") = std::string(p::kEvents);
    protocol.attr("COMMAND") = std::string(p::kCommand);
    protocol.attr("CONTROLLER") = std::string(p::kController);
    protocol.attr("RECORD") = std::string(p::kRecord);
    protocol.attr("STATUS") = std::string(p::kStatus);
    protocol.attr("TORQUE_STEP") = std::string(p::kTorqueStep);
    protocol.attr("CONTROLLER_MANAGER") = std::string(p::kControllerManager);
    protocol.attr("LIST_CONTROLLERS") = std::string(p::kListControllers);
    protocol.attr("SWITCH_CONTROLLER") = std::string(p::kSwitchController);
    protocol.attr("UNLOAD_CONTROLLER") = std::string(p::kUnloadController);
    protocol.attr("CREATE_CONTROLLER") = std::string(p::kCreateController);
    protocol.attr("GET_CONTROLLER_CONFIG") = std::string(p::kGetControllerConfig);
    protocol.attr("GET_CONFIG_FILE") = std::string(p::kGetConfigFile);
    protocol.attr("LIST_PARAMETERS") = std::string(p::kListParameters);
    protocol.attr("DESCRIBE_PARAMETERS") = std::string(p::kDescribeParameters);
    protocol.attr("GET_PARAMETERS") = std::string(p::kGetParameters);
    protocol.attr("GET_PARAMETER_TYPES") = std::string(p::kGetParameterTypes);
    protocol.attr("SET_PARAMETERS") = std::string(p::kSetParameters);
    protocol.attr("SET_PARAMETERS_ATOMICALLY") = std::string(p::kSetParametersAtomically);
    protocol.attr("PARAMETER_EVENTS") = std::string(p::kParameterEvents);
    protocol.attr("PARAMETERS_TOKEN") = std::string(p::kParametersToken);
    protocol.attr("SESSIONS") = std::string(p::kSessions);
    protocol.attr("PROGRAM") = std::string(p::kProgram);
    protocol.attr("COMMAND_FIELD") = std::string(p::kCommandField);
    protocol.attr("COMMAND_VERSION") = p::kCommandVersion;
    protocol.attr("EXPIRED") = std::string(p::kExpired);
    protocol.attr("NOT_TARGET") = std::string(p::kNotTarget);
    protocol.attr("INVALID_COMMAND") = std::string(p::kInvalidCommand);
    protocol.attr("ATTACHMENT_VERSION") = p::kAttachmentVersion;
    protocol.attr("INFO_VERSION") = p::kInfoVersion;
    protocol.attr("SENT_VERSION") = p::kSentVersion;
    protocol.attr("PROGRAM_VERSION") = p::kProgramVersion;
    protocol.attr("RING_VERSION") = xblick::kVersion;
    protocol.attr("DEFAULT_PORT") = p::kDefaultPort;
    nb::dict types;  // type name -> (schema, Zenoh encoding)
    for (const p::MessageType& t : {p::kImageType, p::kCompressedImageType, p::kCameraInfoType, p::kTFMessageType,
                                    p::kImuType, p::kDiagnosticArrayType, p::kShmImageType, p::kCudaImageType,
                                    p::kPointCloud2Type, p::kShmPointCloud2Type, p::kCudaPointCloud2Type,
                                    p::kCompressedPointCloud2Type, p::kRaysType,
                                    p::kDetection2DArrayType, p::kDetection3DArrayType, p::kClassificationType,
                                    p::kJointStateType, p::kOdometryType, p::kWrenchStampedType,
                                    p::kClockType, p::kFloat64MultiArrayType, p::kRobotStateType, p::kParameterEventType,
                                    p::kListParametersRequestType,
                                    p::kListParametersResponseType, p::kDescribeParametersRequestType,
                                    p::kDescribeParametersResponseType, p::kGetParametersRequestType,
                                    p::kGetParametersResponseType, p::kGetParameterTypesRequestType,
                                    p::kGetParameterTypesResponseType, p::kSetParametersRequestType,
                                    p::kSetParametersResponseType, p::kSetParametersAtomicallyRequestType,
                                    p::kSetParametersAtomicallyResponseType, p::kTorqueStepRequestType,
                                    p::kTorqueStepResponseType, p::kListControllersRequestType,
                                    p::kListControllersResponseType, p::kSwitchControllerRequestType,
                                    p::kSwitchControllerResponseType, p::kUnloadControllerRequestType,
                                    p::kUnloadControllerResponseType, p::kCreateControllerRequestType,
                                    p::kCreateControllerResponseType, p::kGetControllerConfigRequestType,
                                    p::kGetControllerConfigResponseType, p::kGetConfigFileRequestType,
                                    p::kGetConfigFileResponseType})
    {
        types[nb::str(std::string(t.name).c_str())] = nb::make_tuple(t.schema(), t.encoding());
    }
    protocol.attr("TYPES") = types;

    nb::class_<xblick::HostRingWriter>(m, "HostRingWriter")
        .def(nb::init<std::uint32_t, std::uint64_t>(), "slot_count"_a, "slot_size"_a)
        .def(
            "write",
            [](xblick::HostRingWriter& w, const InArray& a) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write(a.data(), nbytes_of(a));
                }
                return pair(out);
            },
            "array"_a, "Copy a C-contiguous array into the next slot; (slot, seq).")
        .def(
            "loan",
            [](nb::handle self) {
                auto& w = nb::cast<xblick::HostRingWriter&>(self);
                std::uint8_t* p = w.loan();
                return ByteView(p, {static_cast<std::size_t>(w.slot_size())}, self);
            },
            "The next slot as a writable uint8 array, to fill in place; then commit(nbytes).")
        .def("commit", [](xblick::HostRingWriter& w, std::uint64_t n) { return pair(w.commit(n)); }, "nbytes"_a)
        .def_prop_ro("name", &xblick::HostRingWriter::name)
        .def_prop_ro("slot_count", &xblick::HostRingWriter::slot_count)
        .def_prop_ro("slot_size", &xblick::HostRingWriter::slot_size);

    nb::class_<xblick::HostRingReader>(m, "HostRingReader")
        .def(nb::init<const std::string&>(), "name"_a)
        .def(
            "read",
            [](const xblick::HostRingReader& r, std::uint32_t slot, std::uint64_t seq, HostArray out) {
                nb::gil_scoped_release release;
                return r.read(slot, seq, out.data(), nbytes_of(out));
            },
            "slot"_a, "seq"_a, "out"_a, "Copy frame (slot, seq) into out; False if it was overwritten.")
        .def(
            "view",
            [](nb::handle self, std::uint32_t slot, std::uint64_t nbytes) {
                auto& r = nb::cast<const xblick::HostRingReader&>(self);
                if (nbytes > r.slot_size())
                {
                    throw xblick::RingError("view of " + std::to_string(nbytes) + " bytes, slot of " +
                                            std::to_string(r.slot_size()));
                }
                return ReadOnlyByteView(r.view(slot), {static_cast<std::size_t>(nbytes)}, self);
            },
            "slot"_a, "nbytes"_a, "The slot's bytes without a copy (read-only); check validate() after use.")
        .def("acquire", &xblick::HostRingReader::acquire, "slot"_a, "seq"_a)
        .def("validate", &xblick::HostRingReader::validate, "slot"_a, "seq"_a)
        .def("nbytes", &xblick::HostRingReader::nbytes, "slot"_a)
        .def_prop_ro("name", &xblick::HostRingReader::name)
        .def_prop_ro("slot_count", &xblick::HostRingReader::slot_count)
        .def_prop_ro("slot_size", &xblick::HostRingReader::slot_size);

    nb::class_<xblick::DeviceBuffer>(m, "DeviceBuffer")
        .def_prop_ro("ptr", &xblick::DeviceBuffer::ptr)
        .def_prop_ro("nbytes", &xblick::DeviceBuffer::nbytes)
        .def_prop_ro("device", &xblick::DeviceBuffer::device)
        .def(
            "download",
            [](const xblick::DeviceBuffer& b, HostArray out) {
                if (nbytes_of(out) < b.nbytes())
                {
                    throw xblick::RingError("download of " + std::to_string(b.nbytes()) + " bytes into " +
                                            std::to_string(nbytes_of(out)));
                }
                nb::gil_scoped_release release;
                b.download(out.data());
            },
            "out"_a)
        .def(
            "upload",
            [](xblick::DeviceBuffer& b, const InArray& a) {
                if (nbytes_of(a) != b.nbytes())
                {
                    throw xblick::RingError("upload of " + std::to_string(nbytes_of(a)) + " bytes into " +
                                            std::to_string(b.nbytes()));
                }
                nb::gil_scoped_release release;
                b.upload(a.data());
            },
            "array"_a, "Copy a C-contiguous array of nbytes bytes into the buffer.");
    m.def("make_device_buffer", &xblick::make_device_buffer, "device"_a, "nbytes"_a,
          "A device buffer of its own (e.g. for a driver that uploads frames, or tests).");

    nb::class_<xblick::CudaRingWriter>(m, "CudaRingWriter")
        .def(nb::init<int, std::uint32_t, std::uint64_t>(), "device"_a, "slot_count"_a, "slot_size"_a)
        .def(
            "write_device",
            [](xblick::CudaRingWriter& w, std::uint64_t ptr, std::uint64_t nbytes, std::uint64_t stream) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write_device(ptr, nbytes, stream);
                }
                return pair(out);
            },
            "ptr"_a, "nbytes"_a, "stream"_a = 0)
        .def(
            "wait_written",
            [](xblick::CudaRingWriter& w, std::uint32_t slot) {
                nb::gil_scoped_release release;
                w.wait_written(slot);
            },
            "slot"_a, "Wait until the frame written into slot is there (write_device's source may then be freed).")
        .def(
            "synchronize_device",
            [](xblick::CudaRingWriter& w) {
                nb::gil_scoped_release release;
                w.synchronize_device();
            },
            "Wait for all work of this process on the ring's device.")
        .def(
            "write_host",
            [](xblick::CudaRingWriter& w, const InArray& a) {
                Written out{};
                {
                    nb::gil_scoped_release release;
                    out = w.write_host(a.data(), nbytes_of(a));
                }
                return pair(out);
            },
            "array"_a)
        .def(
            "read_host",
            [](xblick::CudaRingWriter& w, std::uint32_t slot, HostArray out) {
                nb::gil_scoped_release release;
                w.read_host(slot, out.data(), nbytes_of(out));
            },
            "slot"_a, "out"_a)
        .def(
            "copy_slot",
            [](xblick::CudaRingWriter& w, std::uint32_t slot, std::uint64_t dst, std::uint64_t nbytes) {
                nb::gil_scoped_release release;
                w.copy_slot(slot, dst, nbytes);
            },
            "slot"_a, "dst"_a, "nbytes"_a, "Copy a slot to device memory of the ring's device; returns when done.")
        .def_prop_ro("endpoint", &xblick::CudaRingWriter::endpoint)
        .def_prop_ro("device_uuid", &xblick::CudaRingWriter::device_uuid)
        .def_prop_ro("device", &xblick::CudaRingWriter::device)
        .def_prop_ro("slot_count", &xblick::CudaRingWriter::slot_count)
        .def_prop_ro("slot_size", &xblick::CudaRingWriter::slot_size);

    nb::class_<xblick::CudaRingReader>(m, "CudaRingReader")
        .def(nb::init<const std::string&, int>(), "endpoint"_a, "target_device"_a = -1)
        .def(
            "copy",
            [](xblick::CudaRingReader& r, std::uint32_t slot, std::uint64_t seq) {
                nb::gil_scoped_release release;
                return r.copy(slot, seq);
            },
            "slot"_a, "seq"_a, "A DeviceBuffer with the frame, or None if it was overwritten.")
        .def("slot_ptr", &xblick::CudaRingReader::slot_ptr, "slot"_a)
        .def("acquire", &xblick::CudaRingReader::acquire, "slot"_a, "seq"_a)
        .def("validate", &xblick::CudaRingReader::validate, "slot"_a, "seq"_a)
        .def(
            "wait",
            [](const xblick::CudaRingReader& r, std::uint32_t slot, std::uint64_t stream) {
                nb::gil_scoped_release release;
                r.wait(slot, stream);
            },
            "slot"_a, "stream"_a = 0)
        .def("nbytes", &xblick::CudaRingReader::nbytes, "slot"_a)
        .def_prop_ro("device", &xblick::CudaRingReader::device)
        .def_prop_ro("ring_device", &xblick::CudaRingReader::ring_device)
        .def_prop_ro("direct", &xblick::CudaRingReader::direct)
        .def_prop_ro("slot_count", &xblick::CudaRingReader::slot_count);

    // the GPU link to ROS readers (ros_cuda.hpp)
    namespace rc = xblick::ros_cuda;
    m.attr("ROS_CUDA_HOLD_LIMIT_S") = rc::kHoldLimitS;
    m.attr("ROS_CUDA_MIN_REUSE_S") = rc::kMinReuseS;
    m.attr("ROS_CUDA_SPARE_BLOCKS") = rc::kSpareBlocks;
    m.attr("ROS_CUDA_MAX_BLOCKS") = rc::kMaxBlocks;
    m.attr("ROS_CUDA_SOCKET_DIR") = rc::kDefaultSocketDir;
    m.attr("ROS_CUDA_SHM_DIR") = rc::kDefaultShmDir;
    m.attr("ROS_CUDA_SUBSCRIPTION") = rc::kSubscription;
    // a descriptor as a dict of its fields and back ("event": 64 bytes, all zero when left out)
    const auto descriptor_of = [](const nb::dict& d) {
        rc::Descriptor descriptor;
        descriptor.size = nb::cast<std::uint64_t>(d["size"]);
        descriptor.device_id = nb::cast<std::int32_t>(d["device_id"]);
        descriptor.use_ipc = nb::cast<bool>(d["use_ipc"]);
        descriptor.pid = nb::cast<std::int32_t>(d["pid"]);
        descriptor.block_id = nb::cast<std::uint32_t>(d["block_id"]);
        descriptor.block_size = nb::cast<std::uint64_t>(d["block_size"]);
        descriptor.socket_path = nb::cast<std::string>(d["socket_path"]);
        descriptor.uid = nb::cast<std::uint64_t>(d["uid"]);
        if (d.contains("event"))  // an IPC event handle (64 bytes); without one all zero
        {
            const auto event = nb::cast<nb::bytes>(d["event"]);
            if (event.size() != descriptor.event.size())
            {
                throw nb::value_error("an IPC event handle has 64 bytes");
            }
            std::memcpy(descriptor.event.data(), event.c_str(), event.size());
        }
        return descriptor;
    };
    const auto dict_of = [](const rc::Descriptor& d) {
        nb::dict out;
        out["size"] = d.size;
        out["device_id"] = d.device_id;
        out["use_ipc"] = d.use_ipc;
        out["pid"] = d.pid;
        out["block_id"] = d.block_id;
        out["block_size"] = d.block_size;
        out["socket_path"] = d.socket_path;
        out["uid"] = d.uid;
        out["event"] = nb::bytes(reinterpret_cast<const char*>(d.event.data()), d.event.size());
        return out;
    };
    m.def("ros_cuda_image_payload",
          [descriptor_of](std::int32_t sec, std::uint32_t nanosec, const std::string& frame_id, std::uint32_t height,
                          std::uint32_t width, const std::string& encoding, std::uint8_t is_bigendian,
                          std::uint32_t step, const nb::dict& d) {
              const rc::Descriptor descriptor = descriptor_of(d);
              const auto payload = rc::image_payload({sec, nanosec, frame_id, height, width, encoding, is_bigendian,
                                                      step}, descriptor);
              return nb::bytes(reinterpret_cast<const char*>(payload.data()), payload.size());
          },
          "sec"_a, "nanosec"_a, "frame_id"_a, "height"_a, "width"_a, "encoding"_a, "is_bigendian"_a, "step"_a,
          "descriptor"_a, "sensor_msgs/Image in CDR with a CUDA buffer descriptor (a dict of its fields) as its data");
    m.def("ros_cuda_registry_name", &rc::registry_name, "the CUDA buffer backend's host registry file name");
    m.def("ros_cuda_registry_lookup",
          [](const std::string& shm_dir, const nb::bytes& gid) -> std::optional<nb::dict> {
              if (gid.size() != 16)
              {
                  throw nb::value_error("a gid has 16 bytes");
              }
              std::array<std::uint8_t, 16> key{};
              std::memcpy(key.data(), gid.c_str(), 16);
              const auto entry = rc::registry_lookup(shm_dir, key);
              if (not entry)
              {
                  return std::nullopt;
              }
              nb::dict out;
              out["pid"] = entry->pid();
              out["instance_id"] = entry->instance_id;
              out["entity_type"] = entry->entity_type;
              out["device_id"] = entry->device_id;
              out["uid"] = entry->uid;
              out["ipc_capable"] = entry->ipc_capable;
              out["name"] = entry->name;
              return out;
          },
          "shm_dir"_a, "gid"_a, "the active entry of a gid in the CUDA buffer backend's host registry, or None");
    nb::class_<rc::ExportPool>(m, "RosCudaPool")
        .def(nb::init<int, std::string, std::string, std::uint32_t, double, double>(), "device"_a,
             "socket_dir"_a = rc::kDefaultSocketDir, "shm_dir"_a = rc::kDefaultShmDir,
             "max_blocks"_a = rc::kMaxBlocks, "hold_limit_s"_a = rc::kHoldLimitS,
             "min_reuse_s"_a = rc::kMinReuseS)
        .def("acquire", &rc::ExportPool::acquire, "nbytes"_a, "reuse_frames"_a,
             "a block no reader holds or queues (its index), or -1 when there is none (counted as dropped)")
        .def("block_ptr", &rc::ExportPool::block_ptr, "index"_a)
        .def(
            "publish",
            [](rc::ExportPool& p, int index, std::uint64_t nbytes) {
                const rc::Descriptor d = p.publish(index, nbytes);
                nb::dict out;
                out["size"] = d.size;
                out["device_id"] = d.device_id;
                out["use_ipc"] = d.use_ipc;
                out["pid"] = d.pid;
                out["block_id"] = d.block_id;
                out["block_size"] = d.block_size;
                out["socket_path"] = d.socket_path;
                out["uid"] = d.uid;
                return out;
            },
            "index"_a, "nbytes"_a, "a new uid in the block's record, and its descriptor (a dict)")
        .def_prop_ro("dropped", &rc::ExportPool::dropped)
        .def_prop_ro("expired", &rc::ExportPool::expired)
        .def_prop_ro("blocks", &rc::ExportPool::blocks)
        .def_prop_ro("accepts", &rc::ExportPool::accepts)
        .def_prop_ro("refused", &rc::ExportPool::refused)
        .def_prop_ro("device", &rc::ExportPool::device)
        .def_prop_ro("socket_dir", &rc::ExportPool::socket_dir)
        .def_prop_ro("shm_dir", &rc::ExportPool::shm_dir);

    // the readers, decisions and pool of one topic (ros_cuda.hpp Link); the Zenoh part is ros_cuda.py's
    const auto gid_bytes = [](const std::array<std::uint8_t, 16>& gid) {
        return nb::bytes(reinterpret_cast<const char*>(gid.data()), gid.size());
    };
    const auto gid_of = [](const nb::bytes& gid) {
        if (gid.size() != 16)
        {
            throw nb::value_error("a gid has 16 bytes");
        }
        std::array<std::uint8_t, 16> out{};
        std::memcpy(out.data(), gid.c_str(), 16);
        return out;
    };
    m.attr("ROS_CUDA_ENABLE_ENV") = rc::kEnableEnv;
    m.attr("ROS_CUDA_SOCKETS_ENV") = rc::kSocketsEnv;
    m.attr("ROS_CUDA_SHM_ENV") = rc::kShmEnv;
    m.attr("ROS_CUDA_RECHECK_S") = rc::kRecheckS;
    m.attr("ROS_CUDA_UNKNOWN_S") = rc::kUnknownS;
    m.def("ros_cuda_enabled_by_environment", &rc::enabled_by_environment,
          "False when XBLICK_ROS_CUDA is 0, True when it is 1 or unset; ValueError otherwise");
    m.def("ros_cuda_socket_dir_from_environment", &rc::socket_dir_from_environment);
    m.def("ros_cuda_shm_dir_from_environment", &rc::shm_dir_from_environment);
    m.def(
        "ros_cuda_backends_of",
        [](const std::string& part) {
            nb::dict out;
            for (const auto& [name, meta] : rc::backends_of(part))
            {
                out[name.c_str()] = meta;
            }
            return out;
        },
        "part"_a, "the backends of a token's 14th part: backends:cpu:;cuda: -> {cpu: '', cuda: ''}");
    m.def(
        "ros_cuda_gpu_minors",
        [](const std::string& pid) {
            nb::set out;
            for (int minor : rc::gpu_minors(pid))
            {
                out.add(minor);
            }
            return out;
        },
        "pid"_a, "the GPUs (minors of /dev/nvidia<N>) a process has open ('self': this one)");
    m.def("ros_cuda_minor_of", &rc::minor_of, "uuid"_a);
    nb::class_<rc::Plan>(m, "RosCudaPlan")
        .def_prop_ro("gpu",
                     [gid_bytes](const rc::Plan& p) {
                         nb::list out;
                         for (const auto& [gid, device_id] : p.gpu)
                         {
                             out.append(nb::make_tuple(gid_bytes(gid), device_id));
                         }
                         return out;
                     })
        .def_ro("cpu_group", &rc::Plan::cpu_group)
        .def_prop_ro("cpu",
                     [gid_bytes](const rc::Plan& p) {
                         nb::list out;
                         for (const auto& gid : p.cpu)
                         {
                             out.append(gid_bytes(gid));
                         }
                         return out;
                     })
        .def_ro("depth", &rc::Plan::depth)
        .def_prop_ro("needs_cpu", &rc::Plan::needs_cpu);
    nb::class_<rc::Link>(m, "RosCudaLink")
        .def(nb::init<std::string, int, std::string, std::string, std::uint32_t, double, double>(), "topic"_a,
             "device"_a, "socket_dir"_a, "shm_dir"_a, "max_blocks"_a = rc::kMaxBlocks,
             "hold_limit_s"_a = rc::kHoldLimitS, "min_reuse_s"_a = rc::kMinReuseS)
        .def(
            "add",
            [gid_of](rc::Link& l, const std::string& key, const nb::bytes& gid, std::optional<std::string> backends,
                     bool keep_all, int depth, const std::string& node) {
                l.add(key, rc::Reader{gid_of(gid), std::move(backends), keep_all, depth, node});
            },
            "key"_a, "gid"_a, "backends"_a.none(), "keep_all"_a, "depth"_a, "node"_a,
            "a reader's token came (again: decided anew)")
        .def(
            "remove",
            [gid_bytes](rc::Link& l, const std::string& key) -> std::optional<nb::bytes> {
                const auto gid = l.remove(key);
                return gid ? std::optional<nb::bytes>(gid_bytes(*gid)) : std::nullopt;
            },
            "key"_a, "a reader's token went: its gid, or None")
        .def("plan", &rc::Link::plan, nb::call_guard<nb::gil_scoped_release>(),
             "where the next message goes (RosCudaPlan), or None: on the topic's key as before")
        .def(
            "gpu_messages",
            [gid_bytes](rc::Link& l, const rc::Plan& plan, xblick::CudaRingWriter& ring, std::uint32_t slot,
                        std::uint64_t nbytes, std::int32_t sec, std::uint32_t nanosec, const std::string& frame_id,
                        std::uint32_t height, std::uint32_t width, const std::string& encoding, std::uint32_t step) {
                std::vector<std::pair<std::array<std::uint8_t, 16>, std::vector<std::uint8_t>>> messages;
                {
                    nb::gil_scoped_release release;
                    messages = l.gpu_messages(plan, ring, slot, nbytes,
                                              {sec, nanosec, frame_id, height, width, encoding, 0, step});
                }
                nb::list out;
                for (const auto& [gid, payload] : messages)
                {
                    out.append(nb::make_tuple(gid_bytes(gid), nb::bytes(reinterpret_cast<const char*>(payload.data()),
                                                                        payload.size())));
                }
                return out;
            },
            "plan"_a, "ring"_a, "slot"_a, "nbytes"_a, "sec"_a, "nanosec"_a, "frame_id"_a, "height"_a, "width"_a,
            "encoding"_a, "step"_a,
            "the frame in a ring slot into a free block, [(gid, descriptor message)] for the plan's GPU readers")
        .def(
            "notes",
            [](rc::Link& l) {
                nb::list out;
                for (const rc::Note& n : l.notes())
                {
                    out.append(nb::make_tuple(n.warning, n.text));
                }
                return out;
            },
            "the lines to say since the last call: [(warning, text)]")
        .def("stats",
             [](const rc::Link& l) {
                 const rc::LinkStats s = l.stats();
                 nb::dict out;
                 out["sent"] = s.sent;
                 out["dropped"] = s.dropped;
                 out["expired"] = s.expired;
                 out["blocks"] = s.blocks;
                 out["accepts"] = s.accepts;
                 out["refused"] = s.refused;
                 return out;
             })
        .def(
            "importable",
            [](const rc::Link& l, const nb::dict& e) {
                rc::RegistryEntry entry;
                entry.instance_id = static_cast<std::uint64_t>(nb::cast<std::int64_t>(e["pid"])) << 32;
                entry.entity_type = nb::cast<std::uint8_t>(e["entity_type"]);
                entry.device_id = nb::cast<std::int32_t>(e["device_id"]);
                entry.uid = nb::cast<std::uint32_t>(e["uid"]);
                entry.ipc_capable = nb::cast<bool>(e["ipc_capable"]);
                entry.active = true;
                return l.importable(entry);
            },
            "entry"_a, "(whether a reader of this registry entry can import our blocks, why not)")
        .def_prop_ro("socket_dir", &rc::Link::socket_dir)
        .def_prop_ro("shm_dir", &rc::Link::shm_dir);

    // the link from ROS publishers (ros_cuda.hpp Receiver); the Zenoh part is ros_cuda.py's GpuReceiver
    m.def(
        "ros_cuda_parse_image",
        [dict_of](const nb::bytes& payload) -> std::optional<nb::dict> {
            const auto parsed = rc::parse_image_payload(reinterpret_cast<const std::uint8_t*>(payload.c_str()),
                                                        payload.size());
            if (not parsed)
            {
                return std::nullopt;
            }
            const rc::ImageHeader& h = parsed->header;
            nb::dict out;
            out["sec"] = h.sec;
            out["nanosec"] = h.nanosec;
            out["frame_id"] = h.frame_id;
            out["height"] = h.height;
            out["width"] = h.width;
            out["encoding"] = h.encoding;
            out["is_bigendian"] = h.is_bigendian;
            out["step"] = h.step;
            out["descriptor"] = dict_of(parsed->descriptor);
            return out;
        },
        "payload"_a,
        "a sensor_msgs/Image in CDR whose data is a CUDA buffer: its header fields and 'descriptor' (a dict); None for "
        "the CPU form; RingError for another backend's buffer or no Image");
    nb::class_<rc::Receiver>(m, "RosCudaReceiver")
        .def(
            "__init__",
            [gid_of](rc::Receiver* self, std::string topic, int device, std::string shm_dir, const nb::bytes& gid) {
                new (self) rc::Receiver(std::move(topic), device, std::move(shm_dir), gid_of(gid));
            },
            "topic"_a, "device"_a, "shm_dir"_a, "gid"_a)
        .def(
            "set_gid", [gid_of](rc::Receiver& r, const nb::bytes& gid) { r.set_gid(gid_of(gid)); }, "gid"_a,
            "the subscription's token changed: the old entry inactive, the new one at refresh()")
        .def(
            "add_publisher",
            [gid_of](rc::Receiver& r, const std::string& key, const nb::bytes& gid, const std::string& node) {
                r.add_publisher(key, gid_of(gid), node);
            },
            "key"_a, "gid"_a, "node"_a, "a publisher's token came (again: decided anew)")
        .def("remove_publisher", &rc::Receiver::remove_publisher, "key"_a,
             "a publisher's token went: its blocks unmapped")
        .def("enter", &rc::Receiver::enter, nb::call_guard<nb::gil_scoped_release>(),
             "before the subscription's token: the publishers known now decided, the subscription entered in the "
             "registry")
        .def("refresh", &rc::Receiver::refresh, nb::call_guard<nb::gil_scoped_release>(),
             "the undecided publishers looked up in the registry, the entry written if it is not yet, the decisions said")
        .def(
            "take",
            [descriptor_of](rc::Receiver& r, const nb::dict& d) {
                const rc::Descriptor descriptor = descriptor_of(d);
                nb::gil_scoped_release release;
                return r.take(descriptor);
            },
            "descriptor"_a, "the frame a descriptor names, a DeviceBuffer on this GPU; None when stale or refused")
        .def(
            "notes",
            [](rc::Receiver& r) {
                nb::list out;
                for (const rc::Note& n : r.notes())
                {
                    out.append(nb::make_tuple(n.warning, n.text));
                }
                return out;
            },
            "the lines to say since the last call: [(warning, text)]")
        .def("stats",
             [](const rc::Receiver& r) {
                 const rc::ReceiverStats s = r.stats();
                 nb::dict out;
                 out["copied"] = s.copied;
                 out["stale"] = s.stale;
                 out["refused"] = s.refused;
                 out["blocks"] = s.blocks;
                 out["registered"] = s.registered;
                 return out;
             })
        .def_prop_ro("device", &rc::Receiver::device)
        .def_prop_ro("shm_dir", &rc::Receiver::shm_dir);
}
