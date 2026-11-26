// Image encodings (ROS sensor_msgs/image_encodings.hpp) and sensor_msgs/CompressedImage as ROS image_transport's
// compressed plugin writes it, as Python's encodings.py and codecs.py do: format "<original>; <jpeg|png> compressed
// <target>", JPEG for 8-bit colour and grey, PNG for 8-bit colour (with alpha) and 8- or 16-bit grey, lossless.
// Decoding gives back the original encoding where the file keeps it (colour order, 16-bit grey).
#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "xblick/msg.hpp"

namespace xblick
{
    class EncodingError : public std::invalid_argument
    {
    public:
        using std::invalid_argument::invalid_argument;
    };

    struct EncodingInfo
    {
        int channels;
        int bytes_per_channel;
        char kind;  // 'u' unsigned, 's' signed, 'f' float
        int pixel_bytes() const { return channels * bytes_per_channel; }
    };

    // Channels and channel type of a ROS encoding (rgb8, mono16, 16UC1, 32FC1, ...); EncodingError if unknown.
    EncodingInfo encoding_info(const std::string& encoding);

    // Pixels in host memory, row after row (rows may be padded: step >= width * pixel bytes).
    struct HostImage
    {
        const void* data = nullptr;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint32_t step = 0;  // 0: rows without padding
    };

    // A decoded image that owns its pixels (rows without padding).
    struct DecodedImage
    {
        std::vector<std::uint8_t> pixels;
        std::uint32_t height = 0;
        std::uint32_t width = 0;
        std::uint32_t step = 0;
        std::string encoding;
    };

    inline constexpr const char* kJpeg = "jpeg";
    inline constexpr const char* kPng = "png";
    // 16-bit grey and depth only, lossless, much faster than PNG: ROS's compressedDepth "rvl" (rvl.hpp); the format
    // is "<encoding>; compressedDepth rvl"
    inline constexpr const char* kRvl = "rvl";

    CompressedImage compress(const HostImage& image, const std::string& encoding, const std::string& codec,
                             int jpeg_quality, const Header& header);
    // A frame that is JPEG already (an MJPEG camera) as the message compress() would write for it: the bytes as they
    // are, the format of a JPEG of `encoding` (8-bit colour or grey). Checks the JPEG's start marker only.
    CompressedImage jpeg_message(std::span<const std::uint8_t> jpeg, const std::string& encoding,
                                 const Header& header);
    // ROS's compressedDepth plugin (image_transport's <topic>/compressedDepth): the format "<encoding>; compressedDepth
    // <png|rvl>" ("<encoding>; compressedDepth" alone: PNG, as older versions write it), the data ROS's 12-byte
    // ConfigHeader (int32 format, float32 depthParam[2]) and a 16-bit PNG or the RVL bytes (rvl.hpp). A 16-bit image
    // (16UC1, mono16) is in it as it is, a 32FC1 one as quantised inverse depth q: depth depthParam[0] / (q -
    // depthParam[1]) metres, NaN where q is 0.
    inline constexpr const char* kCompressedDepth = "compressedDepth";
    // (original encoding, png or rvl) of a format of that plugin; nullopt for another format. EncodingError for a
    // compressedDepth format with an unknown codec or an encoding the plugin does not carry.
    std::optional<std::pair<std::string, std::string>> compressed_depth(const std::string& format);
    // The pixels of a CompressedImage: of our compressed forms (JPEG, PNG, RVL), of image_transport's compressed
    // plugin, and of ROS's compressedDepth plugin (16-bit, or 32FC1 depth as float32).
    DecodedImage decompress(const CompressedImage& message);
}  // namespace xblick
