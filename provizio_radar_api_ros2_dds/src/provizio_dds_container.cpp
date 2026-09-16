// Copyright 2025 Provizio Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <link.h>
#include <mutex>
#include <stdexcept>

#ifndef _GNU_SOURCE
// To make sure dlmopen is available
#define _GNU_SOURCE
#endif // _GNU_SOURCE
#include <dlfcn.h>

#include "provizio_radar_api_ros2/provizio_dds_contained.h"
#include "provizio_radar_api_ros2/provizio_dds_container.h"

namespace provizio
{
    namespace
    {
        constexpr const char *provizio_dds_contained_lib = "libprovizio_radar_api_ros2_provizio_dds_contained.so";

        class provizio_dds_container
        {
          public:
            provizio_dds_container();
            ~provizio_dds_container();

            static provizio_dds_container &instance();

            template <typename function_type> function_type *dlsym(const char *symbol);

            // Non-copyable
            provizio_dds_container(const provizio_dds_container &) = delete;
            provizio_dds_container(provizio_dds_container &&) = delete;
            provizio_dds_container &operator=(const provizio_dds_container &) = delete;
            provizio_dds_container &operator=(provizio_dds_container &&) = delete;

          private:
            void check_loaded();

            std::mutex dlsym_mutex;
            void *const handle = nullptr;
        };

        provizio_dds_container::provizio_dds_container()
            : handle(dlmopen(LM_ID_NEWLM, provizio_dds_contained_lib, RTLD_NOW | RTLD_LOCAL))
        {
            if (handle == nullptr)
            {
                const char *error = dlerror(); // NOLINT: used in thread safe manner here
                assert(error != nullptr);

                throw std::runtime_error{std::string{"Failed to load "} + provizio_dds_contained_lib + ": " +
                                         (error != nullptr ? error : "unkown error")};
            }
        }

        provizio_dds_container::~provizio_dds_container()
        {
            if (handle != nullptr)
            {
                dlclose(handle);
            }
        }

        provizio_dds_container &provizio_dds_container::instance()
        {
            static provizio_dds_container the_instance;
            the_instance.check_loaded();

            return the_instance;
        }

        template <typename function_type> function_type *provizio_dds_container::dlsym(const char *symbol)
        {
            std::lock_guard<std::mutex> lock{dlsym_mutex};

            dlerror(); // Clear any error there was, if any. NOLINT: thread safe due to lock_guard
            auto *result = ::dlsym(handle, symbol); // NOLINT: thread safe due to lock_guard
            auto *const error = dlerror();          // Check for error/success. NOLINT: thread safe due to lock_guard
            if (error)
            {
                throw std::runtime_error{std::string{"Error loading "} + symbol + ": " + error};
            }

            return reinterpret_cast<function_type *>(result); // NOLINT: reinterpret_cast is required here (C API)
        }

        void provizio_dds_container::check_loaded()
        {
            if (handle == nullptr)
            {
                throw std::runtime_error{std::string{"Failed to load "} + provizio_dds_contained_lib};
            }
        }
    } // namespace

#define GET_CONTAINED_FUNCTION(function_name, function_var) /*NOLINT: macro is required here*/                         \
    using function_type = decltype(function_name);                                                                     \
    static const auto function_var = provizio_dds_container::instance().dlsym<function_type>(#function_name) /*NOLINT*/

    std::shared_ptr<void> make_dds_domain_participant(const uint32_t domain_id)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_domain_participant, the_function);
        return (*the_function)(domain_id);
    }

    std::shared_ptr<void> make_dds_subscriber_pointcloud2(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_pointcloud2> on_message, on_message_context context)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_subscriber_pointcloud2, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_odometry(const std::shared_ptr<void> &domain_participant,
                                                       const std::string &topic_name,
                                                       on_message_function<provizio::contained_odometry> on_message,
                                                       on_message_context context)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_subscriber_odometry, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_image(const std::shared_ptr<void> &domain_participant,
                                                    const std::string &topic_name,
                                                    on_message_function<provizio::contained_image> on_message,
                                                    on_message_context context)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_subscriber_image, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_polygon_instance_stamped(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_polygon_instance_stamped> on_message, on_message_context context)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_subscriber_polygon_instance_stamped, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_radar_info(const std::shared_ptr<void> &domain_participant,
                                                         const std::string &topic_name,
                                                         on_message_function<provizio::contained_radar_info> on_message,
                                                         on_message_context context)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_subscriber_radar_info, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_service_client_set_radar_range(const std::shared_ptr<void> &domain_participant,
                                                                  const std::string &service_name)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_make_service_client_set_radar_range, the_function);
        return (*the_function)(domain_participant, service_name);
    }

    contained_set_radar_range_status dds_request_set_radar_range(const std::shared_ptr<void> &service_client,
                                                                 contained_set_radar_range request,
                                                                 const std::uint64_t timeout_ns,
                                                                 std::atomic<bool> *const should_stop,
                                                                 contained_set_radar_range_response &out_response)
    {
        GET_CONTAINED_FUNCTION(provizio_dds_contained_request_set_radar_range, the_function);

        // Marshal request/response as C strings + POD across the extern "C" boundary: std::string /
        // std::vector must not cross into the contained library's separate C++ runtime / heap namespace.
        std::array<char, contained_set_radar_range_error_message_capacity> error_message{};
        std::array<std::int8_t, contained_set_radar_range_max_supported_ranges> supported_ranges{};
        bool success = false;
        std::int8_t current_range = -1; // provizio_radar_api_ros2::msg::RadarInfo::UNKNOWN_RANGE
        std::size_t num_supported_ranges = 0;

        const auto status = (*the_function)(
            service_client, request.header.frame_id.c_str(), request.serial_number.c_str(), request.target_range,
            request.header.stamp.sec, request.header.stamp.nanosec, timeout_ns, should_stop, &success, &current_range,
            error_message.data(), supported_ranges.data(), &num_supported_ranges);

        if (status == contained_set_radar_range_status::ok)
        {
            // num_supported_ranges is written by the dlmopen'd contained library; clamp it to the buffer
            // capacity before using it to form an iterator, never trusting a cross-boundary count blindly.
            const std::size_t num_ranges = std::min(num_supported_ranges, supported_ranges.size());
            out_response = contained_set_radar_range_response{};
            out_response.success = success;
            out_response.current_range = current_range;
            out_response.error_message = error_message.data(); // null-terminated by the contained library
            out_response.supported_ranges.assign(supported_ranges.begin(),
                                                  supported_ranges.begin() + static_cast<std::ptrdiff_t>(num_ranges));
        }

        return status;
    }
} // namespace provizio
