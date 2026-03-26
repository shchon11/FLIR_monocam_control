#include <Spinnaker.h>
#include <SpinGenApi/SpinnakerGenApi.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <exception>
#include <initializer_list>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "flir_spinnaker_camera/msg/flir_metadata.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace
{

using Spinnaker::CameraList;
using Spinnaker::CameraPtr;
using Spinnaker::ColorProcessingAlgorithm;
using Spinnaker::ImageProcessor;
using Spinnaker::ImagePtr;
using Spinnaker::PixelFormatEnums;
using Spinnaker::SystemPtr;
using Spinnaker::GenApi::CBooleanPtr;
using Spinnaker::GenApi::CEnumEntryPtr;
using Spinnaker::GenApi::CEnumerationPtr;
using Spinnaker::GenApi::CFloatPtr;
using Spinnaker::GenApi::CIntegerPtr;
using Spinnaker::GenApi::CNodePtr;
using Spinnaker::GenApi::CStringPtr;
using Spinnaker::GenApi::INodeMap;
using Spinnaker::GenApi::IsAvailable;
using Spinnaker::GenApi::IsReadable;
using Spinnaker::GenApi::IsWritable;
using Spinnaker::GenApi::NodeList_t;
using Spinnaker::GenApi::StringList_t;

std::string NormalizeName(std::string value)
{
  std::string normalized;
  normalized.reserve(value.size());

  for (const unsigned char character : value) {
    if (std::isalnum(character) != 0) {
      normalized.push_back(static_cast<char>(std::tolower(character)));
    }
  }

  return normalized;
}

bool IsHostBigEndian()
{
  constexpr std::uint16_t probe = 0x0102;
  return reinterpret_cast<const std::uint8_t *>(&probe)[0] == 0x01;
}

std::string SafeNodeString(INodeMap & node_map, const char * node_name)
{
  CStringPtr value_node = node_map.GetNode(node_name);
  if (!IsReadable(value_node)) {
    return "";
  }

  return value_node->GetValue().c_str();
}

bool SetEnumerationByName(INodeMap & node_map, const char * node_name, const std::string & entry_name)
{
  CEnumerationPtr enum_node = node_map.GetNode(node_name);
  if (!IsReadable(enum_node) || !IsWritable(enum_node)) {
    return false;
  }

  CEnumEntryPtr entry = enum_node->GetEntryByName(entry_name.c_str());
  if (!IsReadable(entry)) {
    return false;
  }

  enum_node->SetIntValue(entry->GetValue());
  return true;
}

bool EnumerationContains(INodeMap & node_map, const char * node_name, const std::string & entry_name)
{
  CEnumerationPtr enum_node = node_map.GetNode(node_name);
  if (!IsReadable(enum_node)) {
    return false;
  }

  CEnumEntryPtr entry = enum_node->GetEntryByName(entry_name.c_str());
  return IsReadable(entry);
}

ColorProcessingAlgorithm ParseColorProcessing(const std::string & value)
{
  const std::string normalized = NormalizeName(value);

  if (normalized == "nearestneighbor") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR;
  }

  if (normalized == "nearestneighboravg") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_NEAREST_NEIGHBOR_AVG;
  }

  if (normalized == "bilinear") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_BILINEAR;
  }

  if (normalized == "edgesensing") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_EDGE_SENSING;
  }

  if (normalized == "ipp") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_IPP;
  }

  if (normalized == "directionalfilter") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_DIRECTIONAL_FILTER;
  }

  if (normalized == "rigorous") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_RIGOROUS;
  }

  if (normalized == "weighteddirectionalfilter") {
    return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_WEIGHTED_DIRECTIONAL_FILTER;
  }

  return Spinnaker::SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR;
}

std::string NormalizePixelFormatParameter(std::string value)
{
  constexpr const char spinnaker_prefix[] = "Spinnaker::PixelFormat_";
  constexpr const char plain_prefix[] = "PixelFormat_";

  if (value.rfind(spinnaker_prefix, 0) == 0) {
    return value.substr(sizeof(spinnaker_prefix) - 1);
  }

  if (value.rfind(plain_prefix, 0) == 0) {
    return value.substr(sizeof(plain_prefix) - 1);
  }

  return value;
}

std::vector<std::string> PixelFormatParameterCandidates(const std::string & value)
{
  const std::string normalized = NormalizePixelFormatParameter(value);
  std::vector<std::string> candidates;
  candidates.push_back(normalized);

  if (normalized == "RGB8") {
    candidates.push_back("RGB8Packed");
  } else if (normalized == "RGB8Packed") {
    candidates.push_back("RGB8");
  } else if (normalized == "BGR8Packed") {
    candidates.push_back("BGR8");
  } else if (normalized == "YUV422") {
    candidates.push_back("YUV422Packed");
  }

  return candidates;
}

struct RawOutputSpec
{
  PixelFormatEnums target_pixel_format;
  std::string encoding;
  bool requires_conversion;
};

struct PreparedRawImage
{
  ImagePtr image;
  std::string encoding;
};

template<std::size_t N>
std::array<double, N> ToFixedArray(
  const std::vector<double> & values,
  const char * parameter_name)
{
  if (values.size() != N) {
    throw std::runtime_error(
      std::string("Parameter '") + parameter_name + "' must contain exactly " +
      std::to_string(N) + " values.");
  }

  std::array<double, N> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

enum class ControlMapKind
{
  Camera,
  Stream,
  TlDevice
};

enum class ControlValueKind
{
  Boolean,
  Integer,
  Float,
  Enumeration,
  String
};

struct ControlBinding
{
  ControlMapKind map_kind;
  ControlValueKind value_kind;
  std::string node_name;
};

std::optional<RawOutputSpec> RawOutputSpecForPixelFormat(PixelFormatEnums pixel_format)
{
  switch (pixel_format) {
    case Spinnaker::PixelFormat_Mono8:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono8, sensor_msgs::image_encodings::MONO8, false};
    case Spinnaker::PixelFormat_Mono16:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono16, sensor_msgs::image_encodings::MONO16, false};
    case Spinnaker::PixelFormat_Mono10:
    case Spinnaker::PixelFormat_Mono10p:
    case Spinnaker::PixelFormat_Mono10Packed:
    case Spinnaker::PixelFormat_Mono12:
    case Spinnaker::PixelFormat_Mono12p:
    case Spinnaker::PixelFormat_Mono12Packed:
    case Spinnaker::PixelFormat_Mono14:
      return RawOutputSpec{Spinnaker::PixelFormat_Mono16, sensor_msgs::image_encodings::MONO16, true};
    case Spinnaker::PixelFormat_BayerRG8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG8, sensor_msgs::image_encodings::BAYER_RGGB8, false};
    case Spinnaker::PixelFormat_BayerBG8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG8, sensor_msgs::image_encodings::BAYER_BGGR8, false};
    case Spinnaker::PixelFormat_BayerGB8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB8, sensor_msgs::image_encodings::BAYER_GBRG8, false};
    case Spinnaker::PixelFormat_BayerGR8:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR8, sensor_msgs::image_encodings::BAYER_GRBG8, false};
    case Spinnaker::PixelFormat_BayerRG16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG16, sensor_msgs::image_encodings::BAYER_RGGB16, false};
    case Spinnaker::PixelFormat_BayerBG16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG16, sensor_msgs::image_encodings::BAYER_BGGR16, false};
    case Spinnaker::PixelFormat_BayerGB16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB16, sensor_msgs::image_encodings::BAYER_GBRG16, false};
    case Spinnaker::PixelFormat_BayerGR16:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR16, sensor_msgs::image_encodings::BAYER_GRBG16, false};
    case Spinnaker::PixelFormat_BayerRG10:
    case Spinnaker::PixelFormat_BayerRG10p:
    case Spinnaker::PixelFormat_BayerRG10Packed:
    case Spinnaker::PixelFormat_BayerRG12:
    case Spinnaker::PixelFormat_BayerRG12p:
    case Spinnaker::PixelFormat_BayerRG12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerRG16, sensor_msgs::image_encodings::BAYER_RGGB16, true};
    case Spinnaker::PixelFormat_BayerBG10:
    case Spinnaker::PixelFormat_BayerBG10p:
    case Spinnaker::PixelFormat_BayerBG10Packed:
    case Spinnaker::PixelFormat_BayerBG12:
    case Spinnaker::PixelFormat_BayerBG12p:
    case Spinnaker::PixelFormat_BayerBG12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerBG16, sensor_msgs::image_encodings::BAYER_BGGR16, true};
    case Spinnaker::PixelFormat_BayerGB10:
    case Spinnaker::PixelFormat_BayerGB10p:
    case Spinnaker::PixelFormat_BayerGB10Packed:
    case Spinnaker::PixelFormat_BayerGB12:
    case Spinnaker::PixelFormat_BayerGB12p:
    case Spinnaker::PixelFormat_BayerGB12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGB16, sensor_msgs::image_encodings::BAYER_GBRG16, true};
    case Spinnaker::PixelFormat_BayerGR10:
    case Spinnaker::PixelFormat_BayerGR10p:
    case Spinnaker::PixelFormat_BayerGR10Packed:
    case Spinnaker::PixelFormat_BayerGR12:
    case Spinnaker::PixelFormat_BayerGR12p:
    case Spinnaker::PixelFormat_BayerGR12Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_BayerGR16, sensor_msgs::image_encodings::BAYER_GRBG16, true};
    case Spinnaker::PixelFormat_RGB8:
    case Spinnaker::PixelFormat_RGB8Packed:
      return RawOutputSpec{Spinnaker::PixelFormat_RGB8, sensor_msgs::image_encodings::RGB8, false};
    case Spinnaker::PixelFormat_BGR8:
      return RawOutputSpec{Spinnaker::PixelFormat_BGR8, sensor_msgs::image_encodings::BGR8, false};
    case Spinnaker::PixelFormat_YUV422Packed:
    case Spinnaker::PixelFormat_YUV422_8:
    case Spinnaker::PixelFormat_YUV422_8_UYVY:
      return RawOutputSpec{pixel_format, sensor_msgs::image_encodings::YUV422, false};
    default:
      return std::nullopt;
  }
}

std::vector<std::string> PreferredPixelFormats()
{
  return {
    "BayerRG8",
    "BayerBG8",
    "BayerGB8",
    "BayerGR8",
    "Mono8",
    "RGB8",
    "BGR8",
    "BayerRG16",
    "BayerBG16",
    "BayerGB16",
    "BayerGR16",
    "Mono16"
  };
}

}  // namespace

class FlirSpinnakerCameraNode : public rclcpp::Node
{
public:
  FlirSpinnakerCameraNode()
  : Node("flir_spinnaker_camera"),
    publish_raw_(declare_parameter<bool>("publish_raw", true)),
    publish_camera_info_(declare_parameter<bool>("publish_camera_info", true)),
    publish_metadata_(declare_parameter<bool>("publish_metadata", true)),
    publish_rgb_compressed_(declare_parameter<bool>("publish_rgb_compressed", true)),
    frame_id_(declare_parameter<std::string>("frame_id", "flir_camera_optical_frame")),
    camera_serial_(declare_parameter<std::string>("camera_serial", "")),
    camera_index_(declare_parameter<int>("camera_index", 0)),
    acquisition_timeout_ms_(declare_parameter<int>("acquisition_timeout_ms", 1000)),
    auto_pixel_format_(declare_parameter<bool>("auto_pixel_format", true)),
    pixel_format_(declare_parameter<std::string>("pixel_format", "")),
    buffer_handling_mode_(declare_parameter<std::string>("buffer_handling_mode", "NewestOnly")),
    color_processing_(declare_parameter<std::string>("color_processing", "hq_linear")),
    rgb_compression_format_(declare_parameter<std::string>("rgb_compression_format", "jpeg")),
    rgb_jpeg_quality_(declare_parameter<int>("rgb_jpeg_quality", 90)),
    rgb_png_compression_level_(declare_parameter<int>("rgb_png_compression_level", 3))
  {
    if (!publish_raw_ && !publish_camera_info_ && !publish_metadata_ && !publish_rgb_compressed_) {
      throw std::runtime_error(
        "At least one of publish_raw, publish_camera_info, publish_metadata, or publish_rgb_compressed must be true.");
    }

    const auto qos = rclcpp::SensorDataQoS();

    if (publish_raw_) {
      raw_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    }

    if (publish_camera_info_) {
      camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
    }

    if (publish_metadata_) {
      metadata_pub_ = create_publisher<flir_spinnaker_camera::msg::FlirMetadata>(
        "image_raw/metadata",
        qos);
    }

    if (publish_rgb_compressed_) {
      rgb_compressed_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>(
        "image_rgb/compressed",
        qos);
    }

    rgb_compression_format_ = NormalizeCompressionFormat(rgb_compression_format_);
    pixel_format_ = NormalizePixelFormatParameter(pixel_format_);
    image_processor_.SetColorProcessing(ParseColorProcessing(color_processing_));

    camera_info_distortion_model_ = declare_parameter<std::string>(
      "camera_info.distortion_model",
      "plumb_bob");
    camera_info_d_ = declare_parameter<std::vector<double>>(
      "camera_info.d",
      std::vector<double>{});
    camera_info_k_ = ToFixedArray<9>(
      declare_parameter<std::vector<double>>(
        "camera_info.k",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.k");
    camera_info_r_ = ToFixedArray<9>(
      declare_parameter<std::vector<double>>(
        "camera_info.r",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.r");
    camera_info_p_ = ToFixedArray<12>(
      declare_parameter<std::vector<double>>(
        "camera_info.p",
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}),
      "camera_info.p");

    const int camera_info_binning_x = declare_parameter<int>("camera_info.binning_x", 0);
    const int camera_info_binning_y = declare_parameter<int>("camera_info.binning_y", 0);
    const int camera_info_roi_x_offset = declare_parameter<int>("camera_info.roi.x_offset", 0);
    const int camera_info_roi_y_offset = declare_parameter<int>("camera_info.roi.y_offset", 0);
    const int camera_info_roi_height = declare_parameter<int>("camera_info.roi.height", 0);
    const int camera_info_roi_width = declare_parameter<int>("camera_info.roi.width", 0);
    camera_info_roi_do_rectify_ = declare_parameter<bool>("camera_info.roi.do_rectify", false);

    if (camera_info_binning_x < 0 || camera_info_binning_y < 0 ||
      camera_info_roi_x_offset < 0 || camera_info_roi_y_offset < 0 ||
      camera_info_roi_height < 0 || camera_info_roi_width < 0)
    {
      throw std::runtime_error("camera_info binning and ROI parameters must be non-negative.");
    }

    camera_info_binning_x_ = static_cast<std::uint32_t>(camera_info_binning_x);
    camera_info_binning_y_ = static_cast<std::uint32_t>(camera_info_binning_y);
    camera_info_roi_x_offset_ = static_cast<std::uint32_t>(camera_info_roi_x_offset);
    camera_info_roi_y_offset_ = static_cast<std::uint32_t>(camera_info_roi_y_offset);
    camera_info_roi_height_ = static_cast<std::uint32_t>(camera_info_roi_height);
    camera_info_roi_width_ = static_cast<std::uint32_t>(camera_info_roi_width);

    RCLCPP_INFO(
      get_logger(),
      "RGB compressed publish=%s format=%s",
      publish_rgb_compressed_ ? "true" : "false",
      rgb_compression_format_.c_str());

    try {
      InitializeCamera();
      running_.store(true);
      acquisition_thread_ = std::thread(&FlirSpinnakerCameraNode::AcquisitionLoop, this);
    } catch (...) {
      ShutdownCamera();
      throw;
    }
  }

  ~FlirSpinnakerCameraNode() override
  {
    ShutdownCamera();
  }

private:
  static std::string ControlPrefix(ControlMapKind map_kind)
  {
    switch (map_kind) {
      case ControlMapKind::Camera:
        return "camera";
      case ControlMapKind::Stream:
        return "stream";
      case ControlMapKind::TlDevice:
        return "tl_device";
    }

    return "camera";
  }

  static bool IsManagedControlNode(ControlMapKind map_kind, const std::string & node_name)
  {
    if (map_kind == ControlMapKind::Camera) {
      return node_name == "PixelFormat" || node_name == "AcquisitionMode";
    }

    if (map_kind == ControlMapKind::Stream) {
      return node_name == "StreamBufferHandlingMode";
    }

    return false;
  }

  static std::optional<ControlValueKind> ControlValueKindForNode(const CNodePtr & node)
  {
    switch (node->GetPrincipalInterfaceType()) {
      case Spinnaker::GenApi::intfIBoolean:
        return ControlValueKind::Boolean;
      case Spinnaker::GenApi::intfIInteger:
        return ControlValueKind::Integer;
      case Spinnaker::GenApi::intfIFloat:
        return ControlValueKind::Float;
      case Spinnaker::GenApi::intfIEnumeration:
        return ControlValueKind::Enumeration;
      case Spinnaker::GenApi::intfIString:
        return ControlValueKind::String;
      default:
        return std::nullopt;
    }
  }

  INodeMap & ResolveNodeMap(ControlMapKind map_kind) const
  {
    switch (map_kind) {
      case ControlMapKind::Camera:
        if (camera_node_map_ == nullptr) {
          throw std::runtime_error("Camera node map is not available.");
        }
        return *camera_node_map_;
      case ControlMapKind::Stream:
        if (stream_node_map_ == nullptr) {
          throw std::runtime_error("Stream node map is not available.");
        }
        return *stream_node_map_;
      case ControlMapKind::TlDevice:
        if (tl_device_node_map_ == nullptr) {
          throw std::runtime_error("Transport-layer device node map is not available.");
        }
        return *tl_device_node_map_;
    }

    throw std::runtime_error("Unexpected control map kind.");
  }

  std::string BuildControlParameterName(ControlMapKind map_kind, const std::string & node_name) const
  {
    return ControlPrefix(map_kind) + "." + node_name;
  }

  rcl_interfaces::msg::ParameterDescriptor BuildControlParameterDescriptor(
    const CNodePtr & node,
    ControlValueKind value_kind) const
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;

    std::ostringstream description;
    description << "GenICam node '" << node->GetName().c_str() << "'";

    const std::string display_name = node->GetDisplayName().c_str();
    if (!display_name.empty() && display_name != node->GetName().c_str()) {
      description << " (" << display_name << ")";
    }

    const std::string tooltip = node->GetToolTip().c_str();
    if (!tooltip.empty()) {
      description << ". " << tooltip;
    }

    if (value_kind == ControlValueKind::Enumeration) {
      CEnumerationPtr enum_node = static_cast<CEnumerationPtr>(node);
      if (IsReadable(enum_node)) {
        StringList_t symbolics;
        enum_node->GetSymbolics(symbolics);
        if (!symbolics.empty()) {
          description << " Valid values: ";
          for (std::size_t index = 0; index < symbolics.size(); ++index) {
            if (index > 0U) {
              description << ", ";
            }
            description << symbolics[index].c_str();
          }
          description << ".";
        }
      }
    }

    descriptor.description = description.str();
    return descriptor;
  }

  bool ShouldExposeControlNode(const CNodePtr & node, ControlMapKind map_kind) const
  {
    if (!node || !IsAvailable(node) || !IsReadable(node) || !IsWritable(node)) {
      return false;
    }

    if (node->GetVisibility() == Spinnaker::GenApi::Invisible) {
      return false;
    }

    const std::string node_name = node->GetName().c_str();
    if (node_name.empty() || IsManagedControlNode(map_kind, node_name)) {
      return false;
    }

    return ControlValueKindForNode(node).has_value();
  }

  void ApplyControlParameterValue(const ControlBinding & binding, const rclcpp::Parameter & parameter)
  {
    INodeMap & node_map = ResolveNodeMap(binding.map_kind);
    const char * node_name = binding.node_name.c_str();

    switch (binding.value_kind) {
      case ControlValueKind::Boolean:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_BOOL) {
          throw std::runtime_error("expected a bool value");
        }

        CBooleanPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_bool(), true);
        return;
      }
      case ControlValueKind::Integer:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER) {
          throw std::runtime_error("expected an integer value");
        }

        CIntegerPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_int(), true);
        return;
      }
      case ControlValueKind::Float:
      {
        double value = 0.0;
        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
          value = parameter.as_double();
        } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          value = static_cast<double>(parameter.as_int());
        } else {
          throw std::runtime_error("expected a floating-point value");
        }

        CFloatPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(value, true);
        return;
      }
      case ControlValueKind::Enumeration:
      {
        CEnumerationPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
          std::string value = parameter.as_string();
          if (binding.node_name == "PixelFormat") {
            value = NormalizePixelFormatParameter(value);
          }

          CEnumEntryPtr entry = node->GetEntryByName(value.c_str());
          if (!IsReadable(entry)) {
            throw std::runtime_error("unknown enum symbolic '" + value + "'");
          }

          node->SetIntValue(entry->GetValue(), true);
          return;
        }

        if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
          node->SetIntValue(parameter.as_int(), true);
          return;
        }

        throw std::runtime_error("expected a string or integer enum value");
      }
      case ControlValueKind::String:
      {
        if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
          throw std::runtime_error("expected a string value");
        }

        CStringPtr node = node_map.GetNode(node_name);
        if (!IsWritable(node)) {
          throw std::runtime_error("node is not writable in the current camera state");
        }

        node->SetValue(parameter.as_string().c_str(), true);
        return;
      }
    }

    throw std::runtime_error("unsupported control parameter type");
  }

  void RegisterControlParameter(
    const std::string & parameter_name,
    const ControlBinding & binding,
    const CNodePtr & node)
  {
    const auto descriptor = BuildControlParameterDescriptor(node, binding.value_kind);

    switch (binding.value_kind) {
      case ControlValueKind::Boolean:
      {
        CBooleanPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const bool current = value_node->GetValue();
        const bool configured = declare_parameter<bool>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Integer:
      {
        CIntegerPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::int64_t current = value_node->GetValue();
        const std::int64_t configured = declare_parameter<std::int64_t>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Float:
      {
        CFloatPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const double current = value_node->GetValue();
        const double configured = declare_parameter<double>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (std::abs(configured - current) > 1e-12) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::Enumeration:
      {
        CEnumerationPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::string current = value_node->ToString().c_str();
        const std::string configured = declare_parameter<std::string>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
      case ControlValueKind::String:
      {
        CStringPtr value_node = ResolveNodeMap(binding.map_kind).GetNode(binding.node_name.c_str());
        const std::string current = value_node->GetValue().c_str();
        const std::string configured = declare_parameter<std::string>(parameter_name, current, descriptor);
        control_bindings_.emplace(parameter_name, binding);
        if (configured != current) {
          pending_control_overrides_.emplace_back(parameter_name, configured);
        }
        return;
      }
    }

    throw std::runtime_error("unsupported control parameter type");
  }

  void RegisterWritableControlParameters(INodeMap & node_map, ControlMapKind map_kind)
  {
    NodeList_t nodes;
    node_map.GetNodes(nodes);

    std::size_t registered = 0U;
    for (const auto & raw_node : nodes) {
      try {
        CNodePtr node(raw_node);
        if (!ShouldExposeControlNode(node, map_kind)) {
          continue;
        }

        const auto value_kind = ControlValueKindForNode(node);
        if (!value_kind.has_value()) {
          continue;
        }

        const std::string node_name = node->GetName().c_str();
        const std::string parameter_name = BuildControlParameterName(map_kind, node_name);
        if (control_bindings_.find(parameter_name) != control_bindings_.end()) {
          continue;
        }

        RegisterControlParameter(parameter_name, ControlBinding{map_kind, *value_kind, node_name}, node);
        ++registered;
      } catch (const std::exception & exception) {
        RCLCPP_DEBUG(
          get_logger(),
          "Skipping a %s control parameter: %s",
          ControlPrefix(map_kind).c_str(),
          exception.what());
      }
    }

    RCLCPP_INFO(
      get_logger(),
      "Registered %zu writable %s control parameters.",
      registered,
      ControlPrefix(map_kind).c_str());
  }

  static int ControlOverridePriority(const ControlBinding & binding)
  {
    switch (binding.value_kind) {
      case ControlValueKind::Enumeration:
        return 0;
      case ControlValueKind::Boolean:
        return 1;
      case ControlValueKind::Integer:
        return 2;
      case ControlValueKind::Float:
        return 3;
      case ControlValueKind::String:
        return 4;
    }

    return 10;
  }

  void ApplyPendingControlOverrides()
  {
    if (pending_control_overrides_.empty()) {
      return;
    }

    auto pending = pending_control_overrides_;
    pending_control_overrides_.clear();

    std::string last_error;
    const std::size_t max_passes = std::max<std::size_t>(1U, pending.size());

    for (std::size_t pass = 0; pass < max_passes && !pending.empty(); ++pass) {
      std::sort(
        pending.begin(),
        pending.end(),
        [this](const rclcpp::Parameter & lhs, const rclcpp::Parameter & rhs) {
          const auto & lhs_binding = control_bindings_.at(lhs.get_name());
          const auto & rhs_binding = control_bindings_.at(rhs.get_name());
          const int lhs_priority = ControlOverridePriority(lhs_binding);
          const int rhs_priority = ControlOverridePriority(rhs_binding);
          if (lhs_priority != rhs_priority) {
            return lhs_priority < rhs_priority;
          }
          return lhs.get_name() < rhs.get_name();
        });

      std::vector<rclcpp::Parameter> next_pass;
      std::size_t applied = 0U;

      for (const auto & parameter : pending) {
        try {
          ApplyControlParameterValue(control_bindings_.at(parameter.get_name()), parameter);
          ++applied;
        } catch (const std::exception & exception) {
          last_error = "Failed to apply startup override '" + parameter.get_name() + "': " + exception.what();
          next_pass.push_back(parameter);
        }
      }

      if (next_pass.empty()) {
        RCLCPP_INFO(
          get_logger(),
          "Applied %zu startup camera control overrides.",
          pending.size());
        return;
      }

      if (applied == 0U) {
        throw std::runtime_error(last_error);
      }

      pending = std::move(next_pass);
    }

    throw std::runtime_error(last_error.empty() ? "Failed to apply camera control overrides." : last_error);
  }

  void InitializeControlParameters()
  {
    RegisterWritableControlParameters(*camera_node_map_, ControlMapKind::Camera);
    RegisterWritableControlParameters(*stream_node_map_, ControlMapKind::Stream);
    RegisterWritableControlParameters(*tl_device_node_map_, ControlMapKind::TlDevice);
    ApplyPendingControlOverrides();

    control_parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&FlirSpinnakerCameraNode::OnSetControlParameters, this, std::placeholders::_1));

    std::ostringstream stream;
    stream << "Camera control bridge ready. Example parameters:";
    bool appended = false;
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.AcquisitionFrameRateEnable", "camera.FrameRateEn_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.AcquisitionFrameRate", "camera.FrameRateHz_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.ExposureAuto", "camera.ExposureAuto_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {
        "camera.ExposureTime",
        "camera.ExposureTime_FloatVal",
        "camera.ExposureTime_Val",
        "camera.ExposureTimeRaw_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.GainAuto", "camera.GainAuto_Val"});
    AppendFirstAvailableExample(
      stream,
      appended,
      {"camera.Gain", "camera.GainDB_Val", "camera.Gain_Val", "camera.GainRaw_Val"});

    if (!appended) {
      stream << " use 'ros2 param list /" << get_name() << "' to inspect available camera.*, stream.*, and tl_device.* parameters.";
    }

    RCLCPP_INFO(get_logger(), "%s", stream.str().c_str());
  }

  void AppendFirstAvailableExample(
    std::ostringstream & stream,
    bool & appended,
    std::initializer_list<const char *> parameter_names) const
  {
    for (const char * parameter_name : parameter_names) {
      if (control_bindings_.find(parameter_name) == control_bindings_.end()) {
        continue;
      }

      stream << (appended ? ", " : " ") << parameter_name;
      appended = true;
      return;
    }
  }

  rcl_interfaces::msg::SetParametersResult OnSetControlParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const auto binding_it = control_bindings_.find(parameter.get_name());
      if (binding_it == control_bindings_.end()) {
        continue;
      }

      try {
        ApplyControlParameterValue(binding_it->second, parameter);
      } catch (const std::exception & exception) {
        result.successful = false;
        result.reason = "Failed to set '" + parameter.get_name() + "': " + exception.what();
        return result;
      }
    }

    return result;
  }

  void InitializeCamera()
  {
    system_ = Spinnaker::System::GetInstance();
    camera_list_ = system_->GetCameras();

    const std::size_t camera_count = camera_list_.GetSize();
    if (camera_count == 0U) {
      throw std::runtime_error("No FLIR cameras detected by Spinnaker.");
    }

    for (std::size_t index = 0; index < camera_count; ++index) {
      CameraPtr candidate = camera_list_.GetByIndex(index);
      INodeMap & tl_node_map = candidate->GetTLDeviceNodeMap();

      const std::string serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
      const std::string vendor = SafeNodeString(tl_node_map, "DeviceVendorName");
      const std::string model = SafeNodeString(tl_node_map, "DeviceModelName");

      RCLCPP_INFO(
        get_logger(),
        "Detected camera[%zu]: vendor='%s' model='%s' serial='%s'",
        index,
        vendor.c_str(),
        model.c_str(),
        serial.c_str());
    }

    camera_ = SelectCamera();
    camera_->Init();

    INodeMap & node_map = camera_->GetNodeMap();
    INodeMap & stream_node_map = camera_->GetTLStreamNodeMap();
    INodeMap & tl_node_map = camera_->GetTLDeviceNodeMap();
    camera_node_map_ = &node_map;
    stream_node_map_ = &stream_node_map;
    tl_device_node_map_ = &tl_node_map;

    ApplyBufferHandlingMode(stream_node_map);
    ApplyPixelFormat(node_map);
    SetContinuousAcquisition(node_map);
    InitializeControlParameters();

    const std::string selected_serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
    const std::string selected_model = SafeNodeString(tl_node_map, "DeviceModelName");
    RCLCPP_INFO(
      get_logger(),
      "Using camera model='%s' serial='%s'",
      selected_model.c_str(),
      selected_serial.c_str());

    camera_->BeginAcquisition();
    acquisition_started_ = true;
    RCLCPP_INFO(get_logger(), "Camera acquisition started.");
  }

  CameraPtr SelectCamera()
  {
    if (!camera_serial_.empty()) {
      for (std::size_t index = 0; index < camera_list_.GetSize(); ++index) {
        CameraPtr candidate = camera_list_.GetByIndex(index);
        INodeMap & tl_node_map = candidate->GetTLDeviceNodeMap();
        const std::string serial = SafeNodeString(tl_node_map, "DeviceSerialNumber");
        if (serial == camera_serial_) {
          return candidate;
        }
      }

      throw std::runtime_error("Requested camera_serial was not found: " + camera_serial_);
    }

    if (camera_index_ < 0 || static_cast<std::size_t>(camera_index_) >= camera_list_.GetSize()) {
      throw std::runtime_error("camera_index is out of range.");
    }

    return camera_list_.GetByIndex(static_cast<unsigned int>(camera_index_));
  }

  void ApplyBufferHandlingMode(INodeMap & stream_node_map)
  {
    if (buffer_handling_mode_.empty()) {
      return;
    }

    if (SetEnumerationByName(stream_node_map, "StreamBufferHandlingMode", buffer_handling_mode_)) {
      RCLCPP_INFO(
        get_logger(),
        "StreamBufferHandlingMode set to '%s'",
        buffer_handling_mode_.c_str());
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "Could not set StreamBufferHandlingMode to '%s'. Continuing with camera default.",
      buffer_handling_mode_.c_str());
  }

  void ApplyPixelFormat(INodeMap & node_map)
  {
    if (!pixel_format_.empty()) {
      for (const auto & candidate : PixelFormatParameterCandidates(pixel_format_)) {
        if (!SetEnumerationByName(node_map, "PixelFormat", candidate)) {
          continue;
        }

        if (candidate != pixel_format_) {
          RCLCPP_INFO(
            get_logger(),
            "PixelFormat alias '%s' resolved to '%s'",
            pixel_format_.c_str(),
            candidate.c_str());
          pixel_format_ = candidate;
        }

        RCLCPP_INFO(get_logger(), "PixelFormat set to '%s'", pixel_format_.c_str());
        return;
      }

      throw std::runtime_error("Failed to set PixelFormat to " + pixel_format_);
    }

    if (!auto_pixel_format_) {
      return;
    }

    for (const auto & candidate : PreferredPixelFormats()) {
      if (!EnumerationContains(node_map, "PixelFormat", candidate)) {
        continue;
      }

      if (SetEnumerationByName(node_map, "PixelFormat", candidate)) {
        RCLCPP_INFO(get_logger(), "PixelFormat auto-selected as '%s'", candidate.c_str());
        return;
      }
    }

    RCLCPP_WARN(
      get_logger(),
      "Could not auto-select a ROS-friendly PixelFormat. Raw publishing may be unavailable.");
  }

  void SetContinuousAcquisition(INodeMap & node_map)
  {
    if (!SetEnumerationByName(node_map, "AcquisitionMode", "Continuous")) {
      throw std::runtime_error("Failed to set AcquisitionMode to Continuous.");
    }
  }

  sensor_msgs::msg::Image BuildImageMessage(
    const ImagePtr & image,
    const std::string & encoding,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.height = static_cast<std::uint32_t>(image->GetHeight());
    msg.width = static_cast<std::uint32_t>(image->GetWidth());
    msg.encoding = encoding;
    msg.is_bigendian = IsHostBigEndian() ? 1U : 0U;

    std::size_t step = image->GetStride();
    if (step == 0U) {
      const int bits_per_channel = sensor_msgs::image_encodings::bitDepth(encoding);
      const int channels = sensor_msgs::image_encodings::numChannels(encoding);
      step = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(bits_per_channel / 8) *
        static_cast<std::size_t>(channels);
    }

    msg.step = static_cast<std::uint32_t>(step);

    const std::size_t expected_size = static_cast<std::size_t>(msg.step) * static_cast<std::size_t>(msg.height);
    const std::size_t available_size = image->GetImageSize();

    if (available_size < expected_size) {
      throw std::runtime_error("Spinnaker image buffer is smaller than ROS image dimensions require.");
    }

    msg.data.resize(expected_size);
    std::memcpy(msg.data.data(), image->GetData(), expected_size);
    return msg;
  }

  std::optional<bool> ReadCameraBooleanNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CBooleanPtr bool_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(bool_node)) {
          return bool_node->GetValue();
        }
      } catch (...) {
      }

      try {
        CIntegerPtr int_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(int_node)) {
          return int_node->GetValue() != 0;
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  std::optional<double> ReadCameraNumericNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CFloatPtr float_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(float_node)) {
          return float_node->GetValue();
        }
      } catch (...) {
      }

      try {
        CIntegerPtr int_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(int_node)) {
          return static_cast<double>(int_node->GetValue());
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  std::optional<std::string> ReadCameraTextNodeValue(std::initializer_list<const char *> node_names) const
  {
    if (camera_node_map_ == nullptr) {
      return std::nullopt;
    }

    for (const char * node_name : node_names) {
      try {
        CEnumerationPtr enum_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(enum_node)) {
          return enum_node->ToString().c_str();
        }
      } catch (...) {
      }

      try {
        CStringPtr string_node = camera_node_map_->GetNode(node_name);
        if (IsReadable(string_node)) {
          return string_node->GetValue().c_str();
        }
      } catch (...) {
      }
    }

    return std::nullopt;
  }

  PreparedRawImage PrepareRawImage(const ImagePtr & image) const
  {
    const auto raw_spec = RawOutputSpecForPixelFormat(image->GetPixelFormat());
    if (!raw_spec.has_value()) {
      throw std::runtime_error(
        "Pixel format '" + std::string(image->GetPixelFormatName().c_str()) + "' is not mapped to a ROS raw encoding.");
    }

    ImagePtr raw_image = image;
    if (raw_spec->requires_conversion) {
      raw_image = image_processor_.Convert(image, raw_spec->target_pixel_format);
    }

    return PreparedRawImage{raw_image, raw_spec->encoding};
  }

  std::string NormalizeCompressionFormat(const std::string & value) const
  {
    const std::string normalized = NormalizeName(value);
    if (normalized == "png") {
      return "png";
    }

    return "jpeg";
  }

  std::vector<int> CompressionParameters() const
  {
    if (rgb_compression_format_ == "png") {
      return {
        cv::IMWRITE_PNG_COMPRESSION,
        std::clamp(rgb_png_compression_level_, 0, 9)
      };
    }

    return {
      cv::IMWRITE_JPEG_QUALITY,
      std::clamp(rgb_jpeg_quality_, 0, 100)
    };
  }

  std::string CompressionExtension() const
  {
    return rgb_compression_format_ == "png" ? ".png" : ".jpg";
  }

  std::string CompressedFormatString() const
  {
    return rgb_compression_format_ == "png" ? "rgb8; png compressed bgr8" : "rgb8; jpeg compressed bgr8";
  }

  sensor_msgs::msg::CompressedImage BuildCompressedImageMessage(
    const ImagePtr & rgb_image,
    const rclcpp::Time & stamp) const
  {
    if (rgb_image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
      rgb_image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
    {
      throw std::runtime_error("Compressed RGB publish requires an RGB8 image.");
    }

    const int width = static_cast<int>(rgb_image->GetWidth());
    const int height = static_cast<int>(rgb_image->GetHeight());
    std::size_t step = rgb_image->GetStride();
    if (step == 0U) {
      step = static_cast<std::size_t>(width) * 3U;
    }

    cv::Mat rgb_view(
      height,
      width,
      CV_8UC3,
      rgb_image->GetData(),
      step);

    cv::Mat bgr_image;
    cv::cvtColor(rgb_view, bgr_image, cv::COLOR_RGB2BGR);

    std::vector<std::uint8_t> compressed_buffer;
    if (!cv::imencode(CompressionExtension(), bgr_image, compressed_buffer, CompressionParameters())) {
      throw std::runtime_error("OpenCV failed to encode RGB compressed image.");
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.format = CompressedFormatString();
    msg.data = std::move(compressed_buffer);
    return msg;
  }

  flir_spinnaker_camera::msg::FlirMetadata BuildMetadataMessage(
    const ImagePtr & original_image,
    const PreparedRawImage & raw_image,
    const rclcpp::Time & stamp) const
  {
    flir_spinnaker_camera::msg::FlirMetadata msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<std::uint32_t>(raw_image.image->GetWidth());
    msg.height = static_cast<std::uint32_t>(raw_image.image->GetHeight());
    msg.step = static_cast<std::uint32_t>(raw_image.image->GetStride());
    msg.encoding = raw_image.encoding;
    msg.pixel_format = original_image->GetPixelFormatName().c_str();
    msg.camera_frame_id = original_image->GetFrameID();
    msg.camera_timestamp_ns = original_image->GetTimeStamp();
    msg.acquisition_frame_rate_enable =
      ReadCameraBooleanNodeValue({"AcquisitionFrameRateEnable"}).value_or(false);
    msg.acquisition_frame_rate_hz =
      ReadCameraNumericNodeValue({"AcquisitionFrameRate", "FrameRateHz_Val"}).value_or(std::nan(""));
    msg.exposure_auto = ReadCameraTextNodeValue({"ExposureAuto"}).value_or("");
    msg.exposure_time_us =
      ReadCameraNumericNodeValue({"ExposureTime", "ExposureTime_FloatVal", "ExposureTime_Val"}).value_or(std::nan(""));
    msg.gain_auto = ReadCameraTextNodeValue({"GainAuto"}).value_or("");
    msg.gain_db = ReadCameraNumericNodeValue({"Gain", "GainDB_Val", "Gain_Val"}).value_or(std::nan(""));
    msg.black_level = ReadCameraNumericNodeValue({"BlackLevel", "BlackLevel_Val"}).value_or(std::nan(""));
    msg.gamma_enable = ReadCameraBooleanNodeValue({"GammaEnable", "GammaEnable_Val"}).value_or(false);
    msg.gamma = ReadCameraNumericNodeValue({"Gamma", "Gamma_FloatVal", "Gamma_Val"}).value_or(std::nan(""));
    msg.balance_white_auto = ReadCameraTextNodeValue({"BalanceWhiteAuto"}).value_or("");
    return msg;
  }

  sensor_msgs::msg::CameraInfo BuildCameraInfoMessage(
    const PreparedRawImage & raw_image,
    const rclcpp::Time & stamp) const
  {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id_;
    msg.width = static_cast<std::uint32_t>(raw_image.image->GetWidth());
    msg.height = static_cast<std::uint32_t>(raw_image.image->GetHeight());
    msg.distortion_model = camera_info_distortion_model_;
    msg.d = camera_info_d_;
    std::copy(camera_info_k_.begin(), camera_info_k_.end(), msg.k.begin());
    std::copy(camera_info_r_.begin(), camera_info_r_.end(), msg.r.begin());
    std::copy(camera_info_p_.begin(), camera_info_p_.end(), msg.p.begin());
    msg.binning_x = camera_info_binning_x_;
    msg.binning_y = camera_info_binning_y_;
    msg.roi.x_offset = camera_info_roi_x_offset_;
    msg.roi.y_offset = camera_info_roi_y_offset_;
    msg.roi.height = camera_info_roi_height_;
    msg.roi.width = camera_info_roi_width_;
    msg.roi.do_rectify = camera_info_roi_do_rectify_;
    return msg;
  }

  void AcquisitionLoop()
  {
    while (rclcpp::ok() && running_.load()) {
      try {
        ImagePtr image = camera_->GetNextImage(acquisition_timeout_ms_);

        if (image->IsIncomplete()) {
          RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Incomplete image received. status=%d",
            static_cast<int>(image->GetImageStatus()));
          image->Release();
          continue;
        }

        const rclcpp::Time stamp = now();

        if (publish_raw_ || publish_camera_info_ || publish_metadata_) {
          const auto raw_spec = RawOutputSpecForPixelFormat(image->GetPixelFormat());
          if (!raw_spec.has_value()) {
            const std::string pixel_format_name = image->GetPixelFormatName().c_str();
            RCLCPP_WARN_THROTTLE(
              get_logger(),
              *get_clock(),
              5000,
              "Skipping image_raw publish because pixel format '%s' is not mapped to a ROS encoding.",
              pixel_format_name.c_str());
          } else {
            const PreparedRawImage raw_image = PrepareRawImage(image);

            if (publish_raw_) {
              raw_pub_->publish(BuildImageMessage(raw_image.image, raw_image.encoding, stamp));
            }

            if (publish_camera_info_) {
              camera_info_pub_->publish(BuildCameraInfoMessage(raw_image, stamp));
            }

            if (publish_metadata_) {
              metadata_pub_->publish(BuildMetadataMessage(image, raw_image, stamp));
            }
          }
        }

        if (publish_rgb_compressed_) {
          ImagePtr rgb_image = image;
          if (image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8 &&
            image->GetPixelFormat() != Spinnaker::PixelFormat_RGB8Packed)
          {
            rgb_image = image_processor_.Convert(image, Spinnaker::PixelFormat_RGB8);
          }

          if (publish_rgb_compressed_) {
            rgb_compressed_pub_->publish(BuildCompressedImageMessage(rgb_image, stamp));
          }
        }

        image->Release();
      } catch (const Spinnaker::Exception & exception) {
        if (!running_.load()) {
          break;
        }

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Spinnaker acquisition warning: %s",
          exception.what());
      } catch (const std::exception & exception) {
        if (!running_.load()) {
          break;
        }

        RCLCPP_ERROR_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Image publish error: %s",
          exception.what());
      }
    }
  }

  void ShutdownCamera() noexcept
  {
    running_.store(false);

    if (acquisition_thread_.joinable()) {
      acquisition_thread_.join();
    }

    if (camera_) {
      try {
        if (acquisition_started_) {
          camera_->EndAcquisition();
          acquisition_started_ = false;
        }
      } catch (const Spinnaker::Exception & exception) {
        RCLCPP_WARN(get_logger(), "EndAcquisition failed during shutdown: %s", exception.what());
      }

      try {
        camera_->DeInit();
      } catch (const Spinnaker::Exception & exception) {
        RCLCPP_WARN(get_logger(), "Camera DeInit failed during shutdown: %s", exception.what());
      }

      camera_ = nullptr;
    }

    if (camera_list_.GetSize() > 0U) {
      camera_list_.Clear();
    }

    if (system_) {
      system_->ReleaseInstance();
      system_ = nullptr;
    }

    camera_node_map_ = nullptr;
    stream_node_map_ = nullptr;
    tl_device_node_map_ = nullptr;
    control_bindings_.clear();
    pending_control_overrides_.clear();
    control_parameter_callback_handle_.reset();
  }

  bool publish_raw_;
  bool publish_camera_info_;
  bool publish_metadata_;
  bool publish_rgb_compressed_;
  std::string frame_id_;
  std::string camera_serial_;
  int camera_index_;
  int acquisition_timeout_ms_;
  bool auto_pixel_format_;
  std::string pixel_format_;
  std::string buffer_handling_mode_;
  std::string color_processing_;
  std::string rgb_compression_format_;
  int rgb_jpeg_quality_;
  int rgb_png_compression_level_;
  std::string camera_info_distortion_model_;
  std::vector<double> camera_info_d_;
  std::array<double, 9> camera_info_k_{};
  std::array<double, 9> camera_info_r_{};
  std::array<double, 12> camera_info_p_{};
  std::uint32_t camera_info_binning_x_{0};
  std::uint32_t camera_info_binning_y_{0};
  std::uint32_t camera_info_roi_x_offset_{0};
  std::uint32_t camera_info_roi_y_offset_{0};
  std::uint32_t camera_info_roi_height_{0};
  std::uint32_t camera_info_roi_width_{0};
  bool camera_info_roi_do_rectify_{false};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<flir_spinnaker_camera::msg::FlirMetadata>::SharedPtr metadata_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr rgb_compressed_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr control_parameter_callback_handle_;

  SystemPtr system_;
  CameraList camera_list_;
  CameraPtr camera_;
  ImageProcessor image_processor_;
  INodeMap * camera_node_map_{nullptr};
  INodeMap * stream_node_map_{nullptr};
  INodeMap * tl_device_node_map_{nullptr};
  std::unordered_map<std::string, ControlBinding> control_bindings_;
  std::vector<rclcpp::Parameter> pending_control_overrides_;

  std::atomic<bool> running_{false};
  bool acquisition_started_{false};
  std::thread acquisition_thread_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  int exit_code = 0;

  try {
    auto node = std::make_shared<FlirSpinnakerCameraNode>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    std::cerr << "flir_spinnaker_camera_node failed: " << exception.what() << std::endl;
    exit_code = 1;
  }

  rclcpp::shutdown();
  return exit_code;
}
