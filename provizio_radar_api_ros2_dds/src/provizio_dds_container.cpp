#include <assert.h>
#include <link.h>
#include <stdexcept>

#ifndef SRC_PROVIZIO_DDS_CONTAINER
// To make sure dlmopen is available
#define SRC_PROVIZIO_DDS_CONTAINER
#endif // SRC_PROVIZIO_DDS_CONTAINER
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

          private:
            void check_loaded();

            void *handle = nullptr;
        };

        provizio_dds_container::provizio_dds_container()
        {
            handle = dlmopen(LM_ID_NEWLM, provizio_dds_contained_lib, RTLD_NOW | RTLD_LOCAL);

            if (!handle)
            {
                const char *error = dlerror();
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
            dlerror(); // Clear any error there was, if any
            auto result = ::dlsym(handle, symbol);
            const auto error = dlerror(); // Check for error/success
            if (error)
            {
                throw std::runtime_error{std::string{"Error loading "} + symbol + ": " + error};
            }

            return reinterpret_cast<function_type *>(result);
        }

        void provizio_dds_container::check_loaded()
        {
            if (handle == nullptr)
            {
                throw std::runtime_error{std::string{"Failed to load "} + provizio_dds_contained_lib};
            }
        }
    } // namespace

#define get_contained_function(function_name, function_var)                                                            \
    using function_type = decltype(function_name);                                                                     \
    static const auto function_var = provizio_dds_container::instance().dlsym<function_type>(#function_name)

    std::shared_ptr<void> make_dds_domain_participant(const uint32_t domain_id)
    {
        get_contained_function(provizio_dds_contained_make_domain_participant, the_function);
        return (*the_function)(domain_id);
    }

    std::shared_ptr<void> make_dds_subscriber_pointcloud2(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_pointcloud2> on_message, on_message_context context)
    {
        get_contained_function(provizio_dds_contained_make_subscriber_pointcloud2, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_odometry(const std::shared_ptr<void> &domain_participant,
                                                       const std::string &topic_name,
                                                       on_message_function<provizio::contained_odometry> on_message,
                                                       on_message_context context)
    {
        get_contained_function(provizio_dds_contained_make_subscriber_odometry, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_image(const std::shared_ptr<void> &domain_participant,
                                                    const std::string &topic_name,
                                                    on_message_function<provizio::contained_image> on_message,
                                                    on_message_context context)
    {
        get_contained_function(provizio_dds_contained_make_subscriber_image, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_polygon_instance_stamped(
        const std::shared_ptr<void> &domain_participant, const std::string &topic_name,
        on_message_function<provizio::contained_polygon_instance_stamped> on_message, on_message_context context)
    {
        get_contained_function(provizio_dds_contained_make_subscriber_polygon_instance_stamped, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_subscriber_radar_info(const std::shared_ptr<void> &domain_participant,
                                                         const std::string &topic_name,
                                                         on_message_function<provizio::contained_radar_info> on_message,
                                                         on_message_context context)
    {
        get_contained_function(provizio_dds_contained_make_subscriber_radar_info, the_function);
        return (*the_function)(domain_participant, topic_name, on_message, context);
    }

    std::shared_ptr<void> make_dds_publisher_set_radar_range(const std::shared_ptr<void> &domain_participant,
                                                             const std::string &topic_name)
    {
        get_contained_function(provizio_dds_contained_make_publisher_set_radar_range, the_function);
        return (*the_function)(domain_participant, topic_name);
    }

    bool dds_publish_set_radar_range(const std::shared_ptr<void> &publisher,
                                     provizio::contained_set_radar_range message)
    {
        get_contained_function(provizio_dds_contained_publish_set_radar_range, the_function);
        return (*the_function)(publisher, std::move(message));
    }
} // namespace provizio
