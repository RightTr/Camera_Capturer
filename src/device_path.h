#pragma once

#include <string_view>

namespace device_path {

inline constexpr const char* kLeftCamera =
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:3.4.3:1.0-video-index0";

inline constexpr const char* kRightCamera =
    "/dev/v4l/by-path/platform-3610000.usb-usb-0:3.4.2:1.0-video-index0";

inline constexpr const char* kLeftUart = "/dev/infrared_cam0";
inline constexpr const char* kRightUart = "/dev/infrared_cam1";

inline constexpr std::string_view kRealSenseSerial = "253822301280";

}  // namespace device_path
