#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <fstream>
#include <set>
#include <thread>
#include <optional>
#include <eigen3/Eigen/Dense>

struct ObjectState
{
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Vector3f orientation;
};

struct Sphere
{
    ObjectState objectState;
    float radius;
};

struct Box
{
    ObjectState objectState;
    Eigen::Vector3f size;
};

struct Action
{
    float throttle;
    float steering;
    bool ballCamPressed;
    bool close;
};

struct State
{
    Box arena;
    Box car;
    Sphere ball;
    Eigen::Vector2f goal;
    Action action;
    bool ballCam;
};

void physics(State &state)
{
    constexpr uint FREQUENCY = 60;
    constexpr float PERIOD = 1.f / FREQUENCY;
    constexpr float MAX_ACCELERATION = 30;
    constexpr float CAR_FRICTION = 10;
    constexpr float BALL_FRICTION = 5;
    constexpr float TURN_RADIUS_MIN = 0.5;
    constexpr float TURN_RADIUS_RATE = 0.5;
    constexpr float MAX_SPEED = 50;
    constexpr float GRAVITY = 3;
    constexpr float BALL_RESTITUTION = 0.6;
    constexpr float MAX_DELTA_SPEED = MAX_ACCELERATION * PERIOD;
    constexpr float DELTA_CAR_FRICTION = CAR_FRICTION * PERIOD;
    constexpr float DELTA_BALL_FRICTION = BALL_FRICTION * PERIOD;
    constexpr float DELTA_GRAVITY = GRAVITY * PERIOD;
    while (!state.action.close)
    {
        auto start = std::chrono::high_resolution_clock::now();
        // car
        // acceleration
        Eigen::Vector3f orientationVector = {std::sin(state.car.objectState.orientation.y()), 0.0f, std::cos(state.car.objectState.orientation.y())};
        state.car.objectState.velocity += orientationVector * MAX_DELTA_SPEED * state.action.throttle;
        float speed = state.car.objectState.velocity.norm();
        if (speed > DELTA_CAR_FRICTION)
        {
            state.car.objectState.velocity -= state.car.objectState.velocity.normalized() * DELTA_CAR_FRICTION;
            speed -= DELTA_CAR_FRICTION;
            if (speed > MAX_SPEED)
            {
                state.car.objectState.velocity *= MAX_SPEED / speed;
                speed = MAX_SPEED;
            }
        }
        else
        {
            state.car.objectState.velocity.setZero();
            speed = 0;
        }
        // steering
        int backwards = state.car.objectState.velocity.dot(orientationVector) < 0 ? -1 : 1;
        state.car.objectState.orientation.y() -= backwards * state.action.steering * speed / (speed * TURN_RADIUS_RATE + TURN_RADIUS_MIN) * PERIOD;
        state.car.objectState.velocity = backwards * Eigen::Vector3f(std::sin(state.car.objectState.orientation.y()), 0.0f, std::cos(state.car.objectState.orientation.y())) * speed;
        // wall collision
        // TODO: make more efficient
        Eigen::Vector3f halfArenaSize = state.arena.size / 2.f;
        Eigen::Vector3f halfCarSize = state.car.size / 2.f;
        std::vector<Eigen::Vector2f> localCorners = {
            {-halfCarSize.x(), -halfCarSize.z()},
            {halfCarSize.x(), -halfCarSize.z()},
            {-halfCarSize.x(), halfCarSize.z()},
            {halfCarSize.x(), halfCarSize.z()}};
        for (const auto &localCorner : localCorners)
        {
            Eigen::Vector2f globalCorner = Eigen::Rotation2Df(state.car.objectState.orientation.y()).toRotationMatrix() * localCorner + Eigen::Vector2f(state.car.objectState.position.x(), state.car.objectState.position.z());
            float xDistance = std::abs(globalCorner.x()) - halfArenaSize.x();
            float zDistance = std::abs(globalCorner.y()) - halfArenaSize.z();
            if (xDistance > 0)
            {
                state.car.objectState.position.x() += (globalCorner.x() < 0 ? 1 : -1) * xDistance;
                state.car.objectState.velocity.x() = 0;
            }
            if (zDistance > 0)
            {
                state.car.objectState.position.z() += (globalCorner.y() < 0 ? 1 : -1) * zDistance;
                state.car.objectState.velocity.z() = 0;
            }
        }
        // ball
        // vertical
        if (state.ball.objectState.position.y() < state.ball.radius)
        {
            state.ball.objectState.position.y() = state.ball.radius;
            state.ball.objectState.velocity.y() *= state.ball.objectState.velocity.y() < -0.1 ? -BALL_RESTITUTION : 0;
        }
        else if (state.ball.objectState.position.y() > state.ball.radius)
        {
            state.ball.objectState.velocity.y() -= DELTA_GRAVITY;
            if (state.ball.objectState.position.y() > state.arena.size.y() - state.ball.radius)
            {
                state.ball.objectState.position.y() = state.arena.size.y() - state.ball.radius;
                state.ball.objectState.velocity.y() *= -1;
            }
        }
        // friction
        if (state.ball.objectState.position.y() == state.ball.radius)
        {
            Eigen::Vector3f &velocity = state.ball.objectState.velocity;
            float scale = std::max(1 - DELTA_BALL_FRICTION / std::hypot(velocity.x(), velocity.z()), 0.f);
            velocity.x() *= scale;
            velocity.z() *= scale;
        }
        // side walls
        if (halfArenaSize.x() - abs(state.ball.objectState.position.x()) < state.ball.radius)
        {
            state.ball.objectState.position.x() = (state.ball.objectState.position.x() < 0 ? -1 : 1) * (halfArenaSize.x() - state.ball.radius);
            state.ball.objectState.velocity.x() *= -1;
        }
        // front + back wall
        if (halfArenaSize.z() - abs(state.ball.objectState.position.z()) < state.ball.radius && (state.ball.objectState.position.y() > state.goal.y() || abs(state.ball.objectState.position.x()) > state.goal.x() / 2))
        {
            state.ball.objectState.position.z() = (state.ball.objectState.position.z() < 0 ? -1 : 1) * (halfArenaSize.z() - state.ball.radius);
            state.ball.objectState.velocity.z() *= -1;
        }
        // goal
        if (abs(state.ball.objectState.position.z()) > state.arena.size.z() / 2 + state.ball.radius)
        {
            state.ball.objectState.position.setZero();
            state.ball.objectState.velocity.setZero();
        }
        // goal posts + crossbar
        for (int i = 0; i < 2; ++i)
        {
            if ((i == 0) ? state.ball.objectState.position.y() < state.goal.y() - state.ball.radius : abs(state.ball.objectState.position.x()) < state.goal.x() / 2 - state.ball.radius)
            {
                Eigen::Vector3f difference = (i == 0) ? Eigen::Vector3f(state.goal.x() / 2 - abs(state.ball.objectState.position.x()), 0.f, state.arena.size.z() / 2 - abs(state.ball.objectState.position.z())) : Eigen::Vector3f(0.f, state.goal.y() - state.ball.objectState.position.y(), state.arena.size.z() / 2 - abs(state.ball.objectState.position.z()));
                float distance = difference.norm();
                if (distance < state.ball.radius)
                {
                    Eigen::Vector3f adjustedDifference = difference.cwiseProduct(state.ball.objectState.position.cwiseSign());
                    state.ball.objectState.position -= adjustedDifference * (state.ball.radius / distance - 1);
                    Eigen::Vector3f normal = adjustedDifference / distance;
                    state.ball.objectState.velocity -= 2 * state.ball.objectState.velocity.dot(normal) * normal;
                }
            }
        }
        // car ball collision
        Eigen::AngleAxisf::Matrix3 rotation = Eigen::AngleAxisf(state.car.objectState.orientation.y(), Eigen::Vector3f::UnitY()).toRotationMatrix();
        Eigen::Vector3f localBallPosition = rotation.transpose() * (state.ball.objectState.position - state.car.objectState.position);
        Eigen::Vector3f halfSize = state.car.size / 2.f;
        Eigen::Vector3f difference = localBallPosition.cwiseMax(-halfSize).cwiseMin(halfSize) - localBallPosition;
        float distance = difference.norm();
        if (distance < state.ball.radius)
        {
            state.ball.objectState.position -= rotation * difference * (state.ball.radius / distance - 1);
            Eigen::Vector3f collisionNormal = (state.ball.objectState.position - state.car.objectState.position).normalized();
            float velocityAlongNormal = (state.ball.objectState.velocity - state.car.objectState.velocity).dot(collisionNormal);
            if (velocityAlongNormal < 0)
            {
                state.ball.objectState.velocity -= 1 * velocityAlongNormal * collisionNormal;
            }
        }

        state.car.objectState.position += state.car.objectState.velocity * PERIOD;
        state.ball.objectState.position += state.ball.objectState.velocity * PERIOD;

        float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now() - start).count();
        if (elapsed < PERIOD)
        {
            std::this_thread::sleep_for(std::chrono::duration<float>(PERIOD - elapsed));
        }
    }
}

struct QueueFamilyIndices
{
    std::optional<uint32_t> graphicsFamily;
    std::optional<uint32_t> presentFamily;

    bool isComplete()
    {
        return graphicsFamily.has_value() && presentFamily.has_value();
    }
};

struct SwapChainSupportDetails
{
    VkSurfaceCapabilitiesKHR capabilities;
    std::vector<VkSurfaceFormatKHR> formats;
    std::vector<VkPresentModeKHR> presentModes;
};

struct Vertex
{
    glm::vec3 pos;
    glm::vec3 normal;
    static VkVertexInputBindingDescription getBindingDescription()
    {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Vertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        return bindingDescription;
    }
    static std::array<VkVertexInputAttributeDescription, 2> getAttributeDescriptions()
    {
        std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions{};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(Vertex, pos);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(Vertex, normal);

        return attributeDescriptions;
    }
};

struct UniformBufferObject
{
    alignas(16) glm::mat4 model[3];
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
};

constexpr uint32_t WIDTH = 1440;
constexpr uint32_t HEIGHT = 810;
constexpr int MAX_FRAMES_IN_FLIGHT = 2;
const std::vector<const char *> validationLayers = {"VK_LAYER_KHRONOS_validation"};
const std::vector<const char *> deviceExtensions = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
#ifdef NDEBUG
constexpr bool enableValidationLayers = false;
#else
constexpr bool enableValidationLayers = true;
#endif

class InputGraphics
{
private:
    GLFWwindow *window;

    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
    VkSurfaceKHR surface;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;

    VkQueue graphicsQueue;
    VkQueue presentQueue;

    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkFormat swapChainImageFormat;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;

    VkRenderPass renderPass;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;

    VkCommandPool commandPool;

    VkImage depthImage;
    VkDeviceMemory depthImageMemory;
    VkImageView depthImageView;

    VkBuffer vertexBuffer;
    VkDeviceMemory vertexBufferMemory;
    VkBuffer indexBuffer;
    VkDeviceMemory indexBufferMemory;

    std::vector<VkBuffer> uniformBuffers;
    std::vector<VkDeviceMemory> uniformBuffersMemory;
    std::vector<void *> uniformBuffersMapped;

    VkDescriptorPool descriptorPool;
    std::vector<VkDescriptorSet> descriptorSets;

    std::vector<VkCommandBuffer> commandBuffers;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
    uint32_t currentFrame = 0;
    std::vector<uint32_t> indicesOffsets;

    bool framebufferResized = false;

    State &state;

    static void framebufferResizeCallback(GLFWwindow *window, int width, int height)
    {
        auto inputGraphics = reinterpret_cast<InputGraphics *>(glfwGetWindowUserPointer(window));
        inputGraphics->framebufferResized = true;
    }

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageType, const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData)
    {
        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

        return VK_FALSE;
    }

    QueueFamilyIndices findQueueFamilies(VkPhysicalDevice device)
    {
        QueueFamilyIndices indices;

        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);

        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

        int i = 0;
        for (const auto &queueFamily : queueFamilies)
        {
            if (queueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT)
            {
                indices.graphicsFamily = i;
            }

            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(device, i, surface, &presentSupport);

            if (presentSupport)
            {
                indices.presentFamily = i;
            }

            if (indices.isComplete())
            {
                break;
            }

            i++;
        }

        return indices;
    }

    VkImageView createImageView(VkImage image, VkFormat format, VkImageAspectFlags aspectFlags)
    {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = image;
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = format;
        viewInfo.subresourceRange.aspectMask = aspectFlags;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        VkImageView imageView;
        if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create image view!");
        }

        return imageView;
    }

    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device)
    {
        SwapChainSupportDetails details;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);
        uint32_t formatCount;
        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
        if (formatCount != 0)
        {
            details.formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, details.formats.data());
        }
        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);
        if (presentModeCount != 0)
        {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
        }
        return details;
    }

    void createSwapChain()
    {
        SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

        VkSurfaceFormatKHR surfaceFormat = swapChainSupport.formats[0];
        for (const auto &availableFormat : swapChainSupport.formats)
        {
            if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            {
                surfaceFormat = availableFormat;
            }
        }

        VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
        for (const auto &availablePresentMode : swapChainSupport.presentModes)
        {
            if (availablePresentMode == VK_PRESENT_MODE_MAILBOX_KHR)
            {
                presentMode = availablePresentMode;
            }
        }

        VkExtent2D extent;
        if (swapChainSupport.capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
        {
            extent = swapChainSupport.capabilities.currentExtent;
        }
        else
        {
            int width, height;
            glfwGetFramebufferSize(window, &width, &height);

            VkExtent2D actualExtent = {
                static_cast<uint32_t>(width),
                static_cast<uint32_t>(height)};

            actualExtent.width = std::clamp(actualExtent.width, swapChainSupport.capabilities.minImageExtent.width, swapChainSupport.capabilities.maxImageExtent.width);
            actualExtent.height = std::clamp(actualExtent.height, swapChainSupport.capabilities.minImageExtent.height, swapChainSupport.capabilities.maxImageExtent.height);

            extent = actualExtent;
        }

        uint32_t imageCount = swapChainSupport.capabilities.minImageCount + 1;
        if (swapChainSupport.capabilities.maxImageCount > 0 && imageCount > swapChainSupport.capabilities.maxImageCount)
        {
            imageCount = swapChainSupport.capabilities.maxImageCount;
        }

        VkSwapchainCreateInfoKHR createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
        createInfo.surface = surface;

        createInfo.minImageCount = imageCount;
        createInfo.imageFormat = surfaceFormat.format;
        createInfo.imageColorSpace = surfaceFormat.colorSpace;
        createInfo.imageExtent = extent;
        createInfo.imageArrayLayers = 1;
        createInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

        QueueFamilyIndices indices = findQueueFamilies(physicalDevice);
        uint32_t queueFamilyIndices[] = {indices.graphicsFamily.value(), indices.presentFamily.value()};

        if (indices.graphicsFamily != indices.presentFamily)
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
            createInfo.queueFamilyIndexCount = 2;
            createInfo.pQueueFamilyIndices = queueFamilyIndices;
        }
        else
        {
            createInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        createInfo.preTransform = swapChainSupport.capabilities.currentTransform;
        createInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        createInfo.presentMode = presentMode;
        createInfo.clipped = VK_TRUE;

        if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create swap chain!");
        }

        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
        swapChainImages.resize(imageCount);
        vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

        swapChainImageFormat = surfaceFormat.format;
        swapChainExtent = extent;

        swapChainImageViews.resize(swapChainImages.size());

        for (uint32_t i = 0; i < swapChainImages.size(); i++)
        {
            swapChainImageViews[i] = createImageView(swapChainImages[i], swapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);
        }
    }

    VkFormat findDepthFormat()
    {
        std::vector<VkFormat> depthFormats = {
            VK_FORMAT_D32_SFLOAT,
            VK_FORMAT_D32_SFLOAT_S8_UINT,
            VK_FORMAT_D24_UNORM_S8_UINT};

        for (VkFormat format : depthFormats)
        {
            VkFormatProperties props;
            vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

            if ((props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT) == VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
            {
                return format;
            }
        }

        throw std::runtime_error("failed to find supported depth format!");
    }

    static std::vector<char> readFile(const std::string &filename)
    {
        std::ifstream file(filename, std::ios::ate | std::ios::binary);

        if (!file.is_open())
        {
            throw std::runtime_error("failed to open file!");
        }

        size_t fileSize = (size_t)file.tellg();
        std::vector<char> buffer(fileSize);

        file.seekg(0);
        file.read(buffer.data(), fileSize);

        file.close();

        return buffer;
    }

    VkShaderModule createShaderModule(const std::vector<char> &code)
    {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t *>(code.data());

        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create shader module!");
        }

        return shaderModule;
    }

    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties)
    {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
        {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
            {
                return i;
            }
        }

        throw std::runtime_error("failed to find suitable memory type!");
    }

    void createFramebuffers()
    {
        VkFormat depthFormat = findDepthFormat();

        VkImageCreateInfo imageInfo{};
        imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
        imageInfo.imageType = VK_IMAGE_TYPE_2D;
        imageInfo.extent.width = swapChainExtent.width;
        imageInfo.extent.height = swapChainExtent.height;
        imageInfo.extent.depth = 1;
        imageInfo.mipLevels = 1;
        imageInfo.arrayLayers = 1;
        imageInfo.format = depthFormat;
        imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
        imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
        imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
        imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateImage(device, &imageInfo, nullptr, &depthImage) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create image!");
        }

        VkMemoryRequirements memRequirements;
        vkGetImageMemoryRequirements(device, depthImage, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &depthImageMemory) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to allocate image memory!");
        }

        vkBindImageMemory(device, depthImage, depthImageMemory, 0);
        depthImageView = createImageView(depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);

        swapChainFramebuffers.resize(swapChainImageViews.size());

        for (size_t i = 0; i < swapChainImageViews.size(); i++)
        {
            std::array<VkImageView, 2> attachments = {
                swapChainImageViews[i],
                depthImageView};

            VkFramebufferCreateInfo framebufferInfo{};
            framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
            framebufferInfo.renderPass = renderPass;
            framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
            framebufferInfo.pAttachments = attachments.data();
            framebufferInfo.width = swapChainExtent.width;
            framebufferInfo.height = swapChainExtent.height;
            framebufferInfo.layers = 1;

            if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapChainFramebuffers[i]) != VK_SUCCESS)
            {
                throw std::runtime_error("failed to create framebuffer!");
            }
        }
    }

    void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, VkBuffer &buffer, VkDeviceMemory &bufferMemory)
    {
        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

        if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to create buffer!");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS)
        {
            throw std::runtime_error("failed to allocate buffer memory!");
        }

        vkBindBufferMemory(device, buffer, bufferMemory, 0);
    }

    void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size)
    {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        VkBufferCopy copyRegion{};
        copyRegion.size = size;
        vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    }

    void cleanupSwapChain()
    {
        vkDestroyImageView(device, depthImageView, nullptr);
        vkDestroyImage(device, depthImage, nullptr);
        vkFreeMemory(device, depthImageMemory, nullptr);

        for (auto framebuffer : swapChainFramebuffers)
        {
            vkDestroyFramebuffer(device, framebuffer, nullptr);
        }

        for (auto imageView : swapChainImageViews)
        {
            vkDestroyImageView(device, imageView, nullptr);
        }

        vkDestroySwapchainKHR(device, swapChain, nullptr);
    }

    void recreateSwapChain()
    {
        int width = 0, height = 0;
        glfwGetFramebufferSize(window, &width, &height);
        while (width == 0 || height == 0)
        {
            glfwGetFramebufferSize(window, &width, &height);
            glfwWaitEvents();
        }

        vkDeviceWaitIdle(device);

        cleanupSwapChain();

        createSwapChain();
        createFramebuffers();
    }

public:
    InputGraphics(State &state)
        : state(state) {}
    void run()
    {
        // init window
        {
            glfwInit();
            glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

            window = glfwCreateWindow(WIDTH, HEIGHT, "Universe", nullptr, nullptr);
            glfwSetWindowUserPointer(window, this);
            glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);

            std::ifstream file("gamepad.txt");
            if (!file) {
                throw std::runtime_error("error opening gamepad.txt");
            }
            std::string fileContents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
            glfwUpdateGamepadMappings(fileContents.c_str());
        }

        // init vulkan
        {
            VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo;
            // create instance
            {
                if (enableValidationLayers)
                {
                    uint32_t layerCount;
                    vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

                    std::vector<VkLayerProperties> availableLayers(layerCount);
                    vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

                    for (const char *layerName : validationLayers)
                    {
                        bool layerFound = false;

                        for (const auto &layerProperties : availableLayers)
                        {
                            if (strcmp(layerName, layerProperties.layerName) == 0)
                            {
                                layerFound = true;
                                break;
                            }
                        }

                        if (!layerFound)
                        {
                            throw std::runtime_error("validation layers requested, but not available!");
                        }
                    }
                }

                VkApplicationInfo appInfo{};
                appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
                appInfo.pApplicationName = "Universe";
                appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
                appInfo.pEngineName = "No Engine";
                appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
                appInfo.apiVersion = VK_API_VERSION_1_0;

                VkInstanceCreateInfo createInfo{};
                createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
                createInfo.pApplicationInfo = &appInfo;

                uint32_t glfwExtensionCount = 0;
                const char **glfwExtensions;
                glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
                std::vector<const char *> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);
                if (enableValidationLayers)
                {
                    extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
                }

                createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
                createInfo.ppEnabledExtensionNames = extensions.data();

                debugCreateInfo = {};
                debugCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
                debugCreateInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
                debugCreateInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
                debugCreateInfo.pfnUserCallback = debugCallback;
                if (enableValidationLayers)
                {
                    createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
                    createInfo.ppEnabledLayerNames = validationLayers.data();

                    createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT *)&debugCreateInfo;
                }
                else
                {
                    createInfo.enabledLayerCount = 0;

                    createInfo.pNext = nullptr;
                }

                if (vkCreateInstance(&createInfo, nullptr, &instance) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create instance!");
                }
            }

            // setup debug messenger
            {
                if (enableValidationLayers)
                {
                    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
                    if (func != nullptr)
                    {
                        if (func(instance, &debugCreateInfo, nullptr, &debugMessenger) != VK_SUCCESS)
                        {
                            throw std::runtime_error("failed to set up debug messenger!");
                        }
                    }
                    else
                    {
                        throw std::runtime_error("failed to set up debug messenger!");
                    }
                }
            }

            // create surface
            {
                if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create window surface!");
                }
            }

            // pick physical device
            {
                uint32_t deviceCount = 0;
                vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

                if (deviceCount == 0)
                {
                    throw std::runtime_error("failed to find GPUs with Vulkan support!");
                }

                std::vector<VkPhysicalDevice> devices(deviceCount);
                vkEnumeratePhysicalDevices(instance, &deviceCount, devices.data());

                for (const auto &device : devices)
                {
                    QueueFamilyIndices indices = findQueueFamilies(device);

                    uint32_t extensionCount;
                    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);

                    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
                    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

                    std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

                    for (const auto &extension : availableExtensions)
                    {
                        requiredExtensions.erase(extension.extensionName);
                    }

                    bool extensionsSupported = requiredExtensions.empty();

                    SwapChainSupportDetails swapChainSupport = querySwapChainSupport(device);

                    VkPhysicalDeviceFeatures supportedFeatures;
                    vkGetPhysicalDeviceFeatures(device, &supportedFeatures);

                    if (indices.isComplete() && extensionsSupported && !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty() && supportedFeatures.samplerAnisotropy)
                    {
                        physicalDevice = device;
                        break;
                    }
                }

                if (physicalDevice == VK_NULL_HANDLE)
                {
                    throw std::runtime_error("failed to find a suitable GPU!");
                }
            }

            // create logical device
            {
                QueueFamilyIndices indices = findQueueFamilies(physicalDevice);

                std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
                std::set<uint32_t> uniqueQueueFamilies = {indices.graphicsFamily.value(), indices.presentFamily.value()};

                float queuePriority = 1.0f;
                for (uint32_t queueFamily : uniqueQueueFamilies)
                {
                    VkDeviceQueueCreateInfo queueCreateInfo{};
                    queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
                    queueCreateInfo.queueFamilyIndex = queueFamily;
                    queueCreateInfo.queueCount = 1;
                    queueCreateInfo.pQueuePriorities = &queuePriority;
                    queueCreateInfos.push_back(queueCreateInfo);
                }

                VkPhysicalDeviceFeatures deviceFeatures{};
                deviceFeatures.samplerAnisotropy = VK_TRUE;

                VkDeviceCreateInfo createInfo{};
                createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;

                createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
                createInfo.pQueueCreateInfos = queueCreateInfos.data();

                createInfo.pEnabledFeatures = &deviceFeatures;

                createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
                createInfo.ppEnabledExtensionNames = deviceExtensions.data();

                if (enableValidationLayers)
                {
                    createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
                    createInfo.ppEnabledLayerNames = validationLayers.data();
                }
                else
                {
                    createInfo.enabledLayerCount = 0;
                }

                if (vkCreateDevice(physicalDevice, &createInfo, nullptr, &device) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create logical device!");
                }

                vkGetDeviceQueue(device, indices.graphicsFamily.value(), 0, &graphicsQueue);
                vkGetDeviceQueue(device, indices.presentFamily.value(), 0, &presentQueue);

                createSwapChain();
            }
            // create render pass
            {
                VkAttachmentDescription colorAttachment{};
                colorAttachment.format = swapChainImageFormat;
                colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
                colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

                VkAttachmentDescription depthAttachment{};
                depthAttachment.format = findDepthFormat();
                depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
                depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

                VkAttachmentReference colorAttachmentRef{};
                colorAttachmentRef.attachment = 0;
                colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

                VkAttachmentReference depthAttachmentRef{};
                depthAttachmentRef.attachment = 1;
                depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

                VkSubpassDescription subpass{};
                subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                subpass.colorAttachmentCount = 1;
                subpass.pColorAttachments = &colorAttachmentRef;
                subpass.pDepthStencilAttachment = &depthAttachmentRef;

                VkSubpassDependency dependency{};
                dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
                dependency.dstSubpass = 0;
                dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
                dependency.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
                dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

                std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
                VkRenderPassCreateInfo renderPassInfo{};
                renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
                renderPassInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
                renderPassInfo.pAttachments = attachments.data();
                renderPassInfo.subpassCount = 1;
                renderPassInfo.pSubpasses = &subpass;
                renderPassInfo.dependencyCount = 1;
                renderPassInfo.pDependencies = &dependency;

                if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create render pass!");
                }
            }

            // create descriptor set layout
            {
                VkDescriptorSetLayoutBinding uboLayoutBinding{};
                uboLayoutBinding.binding = 0;
                uboLayoutBinding.descriptorCount = 1;
                uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                uboLayoutBinding.pImmutableSamplers = nullptr;
                uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

                std::array<VkDescriptorSetLayoutBinding, 1> bindings = {uboLayoutBinding};
                VkDescriptorSetLayoutCreateInfo layoutInfo{};
                layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layoutInfo.bindingCount = static_cast<uint32_t>(bindings.size());
                layoutInfo.pBindings = bindings.data();

                if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create descriptor set layout!");
                }
            }

            // create graphics pipeline
            {
                auto vertShaderCode = readFile("shaders/vert.spv");
                auto fragShaderCode = readFile("shaders/frag.spv");

                VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
                VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

                VkPipelineShaderStageCreateInfo vertShaderStageInfo{};
                vertShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                vertShaderStageInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
                vertShaderStageInfo.module = vertShaderModule;
                vertShaderStageInfo.pName = "main";

                VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
                fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                fragShaderStageInfo.module = fragShaderModule;
                fragShaderStageInfo.pName = "main";

                VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageInfo, fragShaderStageInfo};

                VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
                vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;

                auto bindingDescription = Vertex::getBindingDescription();
                auto attributeDescriptions = Vertex::getAttributeDescriptions();

                vertexInputInfo.vertexBindingDescriptionCount = 1;
                vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
                vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
                vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

                VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
                inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
                inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
                inputAssembly.primitiveRestartEnable = VK_FALSE;

                VkPipelineViewportStateCreateInfo viewportState{};
                viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
                viewportState.viewportCount = 1;
                viewportState.scissorCount = 1;

                VkPipelineRasterizationStateCreateInfo rasterizer{};
                rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
                rasterizer.depthClampEnable = VK_FALSE;
                rasterizer.rasterizerDiscardEnable = VK_FALSE;
                rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
                rasterizer.lineWidth = 1.0f;
                // rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
                rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                rasterizer.depthBiasEnable = VK_FALSE;

                VkPipelineMultisampleStateCreateInfo multisampling{};
                multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                multisampling.sampleShadingEnable = VK_FALSE;
                multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

                VkPipelineDepthStencilStateCreateInfo depthStencil{};
                depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                depthStencil.depthTestEnable = VK_TRUE;
                depthStencil.depthWriteEnable = VK_TRUE;
                depthStencil.depthCompareOp = VK_COMPARE_OP_LESS;
                depthStencil.depthBoundsTestEnable = VK_FALSE;
                depthStencil.stencilTestEnable = VK_FALSE;

                VkPipelineColorBlendAttachmentState colorBlendAttachment{};
                colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
                colorBlendAttachment.blendEnable = VK_FALSE;

                VkPipelineColorBlendStateCreateInfo colorBlending{};
                colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                colorBlending.logicOpEnable = VK_FALSE;
                colorBlending.logicOp = VK_LOGIC_OP_COPY;
                colorBlending.attachmentCount = 1;
                colorBlending.pAttachments = &colorBlendAttachment;
                colorBlending.blendConstants[0] = 0.0f;
                colorBlending.blendConstants[1] = 0.0f;
                colorBlending.blendConstants[2] = 0.0f;
                colorBlending.blendConstants[3] = 0.0f;

                std::vector<VkDynamicState> dynamicStates = {
                    VK_DYNAMIC_STATE_VIEWPORT,
                    VK_DYNAMIC_STATE_SCISSOR};
                VkPipelineDynamicStateCreateInfo dynamicState{};
                dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
                dynamicState.pDynamicStates = dynamicStates.data();

                VkPushConstantRange pushConstantRange = {};
                pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT; // Stage where the push constants are used
                pushConstantRange.offset = 0;                              // Offset into the push constant data
                pushConstantRange.size = sizeof(uint);                     // Size of the push constant data

                VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
                pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipelineLayoutInfo.setLayoutCount = 1;
                pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
                pipelineLayoutInfo.pushConstantRangeCount = 1;               // Number of push constant ranges
                pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange; // Push constant ranges

                if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create pipeline layout!");
                }

                VkGraphicsPipelineCreateInfo pipelineInfo{};
                pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                pipelineInfo.stageCount = 2;
                pipelineInfo.pStages = shaderStages;
                pipelineInfo.pVertexInputState = &vertexInputInfo;
                pipelineInfo.pInputAssemblyState = &inputAssembly;
                pipelineInfo.pViewportState = &viewportState;
                pipelineInfo.pRasterizationState = &rasterizer;
                pipelineInfo.pMultisampleState = &multisampling;
                pipelineInfo.pDepthStencilState = &depthStencil;
                pipelineInfo.pColorBlendState = &colorBlending;
                pipelineInfo.pDynamicState = &dynamicState;
                pipelineInfo.layout = pipelineLayout;
                pipelineInfo.renderPass = renderPass;
                pipelineInfo.subpass = 0;
                pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;

                if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfo, nullptr, &graphicsPipeline) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create graphics pipeline!");
                }

                vkDestroyShaderModule(device, fragShaderModule, nullptr);
                vkDestroyShaderModule(device, vertShaderModule, nullptr);
            }

            // create command pool
            {
                QueueFamilyIndices queueFamilyIndices = findQueueFamilies(physicalDevice);

                VkCommandPoolCreateInfo poolInfo{};
                poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
                poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

                if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create graphics command pool!");
                }

                createFramebuffers();
            }
            {
                Eigen::Vector3f halfSize = state.car.size / 2.f;
                std::vector<glm::vec3> points = {{{-halfSize.x(), -halfSize.y(), -halfSize.z()},
                                                  {halfSize.x(), -halfSize.y(), -halfSize.z()},
                                                  {halfSize.x(), halfSize.y(), -halfSize.z()},
                                                  {-halfSize.x(), halfSize.y(), -halfSize.z()},
                                                  {-halfSize.x(), -halfSize.y(), halfSize.z()},
                                                  {halfSize.x(), -halfSize.y(), halfSize.z()},
                                                  {halfSize.x(), halfSize.y(), halfSize.z()},
                                                  {-halfSize.x(), halfSize.y(), halfSize.z()}}};
                std::vector<std::pair<std::array<int, 4>, glm::vec3>> faces = {{{{0, 1, 2, 3}, {0.0f, 0.0f, -1.0f}},
                                                                                {{4, 5, 6, 7}, {0.0f, 0.0f, 1.0f}},
                                                                                {{0, 1, 5, 4}, {0.0f, -1.0f, 0.0f}},
                                                                                {{2, 3, 7, 6}, {0.0f, 1.0f, 0.0f}},
                                                                                {{0, 3, 7, 4}, {-1.0f, 0.0f, 0.0f}},
                                                                                {{1, 2, 6, 5}, {1.0f, 0.0f, 0.0f}}}};
                std::vector<Vertex> vertices;
                for (const auto &[indices, normal] : faces)
                {
                    for (int index : indices)
                    {
                        vertices.push_back({points[index], normal});
                    }
                }
                std::vector<uint16_t> indices;
                for (uint16_t i = 0; i < faces.size() * 4; i += 4)
                {
                    indices.insert(indices.end(), {i, static_cast<uint16_t>(i + 1), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 3), i});
                }
                indicesOffsets.push_back(indices.size());

                int verticesOffset = vertices.size();
                const float PI = glm::pi<float>();
                constexpr int RESOLUTION = 32;
                for (int latitude = 0; latitude <= RESOLUTION; ++latitude)
                {
                    for (int longitude = 0; longitude <= RESOLUTION; ++longitude)
                    {
                        float theta = latitude * PI / RESOLUTION - PI / 2.f;
                        float phi = longitude * 2.f * PI / RESOLUTION;
                        glm::vec3 position = glm::vec3(cos(theta) * cos(phi), sin(theta), cos(theta) * sin(phi)) * state.ball.radius;
                        glm::vec3 normal = glm::normalize(position);
                        vertices.push_back({position, normal});

                        if (latitude < RESOLUTION && longitude < RESOLUTION)
                        {
                            uint16_t current = static_cast<uint16_t>(verticesOffset + latitude * (RESOLUTION + 1) + longitude);
                            uint16_t next = static_cast<uint16_t>(current + RESOLUTION + 1);
                            indices.insert(indices.end(), {current, next, static_cast<uint16_t>(current + 1),
                                                           static_cast<uint16_t>(current + 1), next, static_cast<uint16_t>(next + 1)});
                        }
                    }
                }
                indicesOffsets.push_back(indices.size());

                float post = 1;
                float halfGoalWidth = state.goal.x() / 2.f;
                float goalHeight = state.goal.y();
                halfSize = state.arena.size / 2.0f;
                points = {
                    {
                        {-halfSize.x(), -halfSize.y(), -halfSize.z()},
                        {halfSize.x(), -halfSize.y(), -halfSize.z()},
                        {halfSize.x(), -halfSize.y(), halfSize.z()},
                        {-halfSize.x(), -halfSize.y(), halfSize.z()},
                        {-halfSize.x(), halfSize.y(), -halfSize.z()},
                        {halfSize.x(), halfSize.y(), -halfSize.z()},
                        {halfSize.x(), halfSize.y(), halfSize.z()},
                        {-halfSize.x(), halfSize.y(), halfSize.z()},
                        {-halfSize.x() + post, -halfSize.y(), -halfSize.z()},
                        {-halfSize.x() + post, halfSize.y(), -halfSize.z()},
                        {halfSize.x() - post, -halfSize.y(), -halfSize.z()},
                        {halfSize.x() - post, halfSize.y(), -halfSize.z()},
                        {-halfSize.x() + post, -halfSize.y(), halfSize.z()},
                        {-halfSize.x() + post, halfSize.y(), halfSize.z()},
                        {halfSize.x() - post, -halfSize.y(), halfSize.z()},
                        {halfSize.x() - post, halfSize.y(), halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y(), -halfSize.z()},
                        {-halfGoalWidth, -halfSize.y(), -halfSize.z()},
                        {-halfGoalWidth, -halfSize.y() + goalHeight, -halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y() + goalHeight, -halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y(), -halfSize.z()},
                        {halfGoalWidth, -halfSize.y(), -halfSize.z()},
                        {halfGoalWidth, -halfSize.y() + goalHeight, -halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y() + goalHeight, -halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y() + goalHeight + post, -halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y() + goalHeight + post, -halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y(), halfSize.z()},
                        {-halfGoalWidth, -halfSize.y(), halfSize.z()},
                        {-halfGoalWidth, -halfSize.y() + goalHeight, halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y() + goalHeight, halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y(), halfSize.z()},
                        {halfGoalWidth, -halfSize.y(), halfSize.z()},
                        {halfGoalWidth, -halfSize.y() + goalHeight, halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y() + goalHeight, halfSize.z()},
                        {halfGoalWidth + post, -halfSize.y() + goalHeight + post, halfSize.z()},
                        {-halfGoalWidth - post, -halfSize.y() + goalHeight + post, halfSize.z()},
                    },
                };
                faces = {
                    {{0, 1, 2, 3}, {0.0f, 1.0f, 0.0f}},
                    {{4, 5, 6, 7}, {0.0f, -1.0f, 0.0f}},
                    {{0, 8, 9, 4}, {0.0f, 0.0f, 1.0f}},
                    {{1, 10, 11, 5}, {0.0f, 0.0f, 1.0f}},
                    {{3, 12, 13, 7}, {0.0f, 0.0f, -1.0f}},
                    {{2, 14, 15, 6}, {0.0f, 0.0f, -1.0f}},
                    {{16, 17, 18, 19}, {0.0f, 0.0f, 1.0f}},
                    {{20, 21, 22, 23}, {0.0f, 0.0f, 1.0f}},
                    {{19, 23, 24, 25}, {0.0f, 0.0f, 1.0f}},
                    {{26, 27, 28, 29}, {0.0f, 0.0f, -1.0f}},
                    {{30, 31, 32, 33}, {0.0f, 0.0f, -1.0f}},
                    {{29, 33, 34, 35}, {0.0f, 0.0f, -1.0f}},
                };
                verticesOffset = vertices.size();
                for (const auto &[indices, normal] : faces)
                {
                    for (int index : indices)
                    {
                        vertices.push_back({points[index], normal});
                    }
                }
                for (uint16_t i = verticesOffset; i < verticesOffset + faces.size() * 4; i += 4)
                {
                    indices.insert(indices.end(), {i, static_cast<uint16_t>(i + 1), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 3), i});
                }
                indicesOffsets.push_back(indices.size());

                VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();

                VkBuffer stagingBuffer;
                VkDeviceMemory stagingBufferMemory;
                createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

                void *data;
                vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
                memcpy(data, vertices.data(), (size_t)bufferSize);
                vkUnmapMemory(device, stagingBufferMemory);

                createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, vertexBuffer, vertexBufferMemory);

                copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

                vkDestroyBuffer(device, stagingBuffer, nullptr);
                vkFreeMemory(device, stagingBufferMemory, nullptr);

                bufferSize = sizeof(indices[0]) * indices.size();
                createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);

                vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
                memcpy(data, indices.data(), (size_t)bufferSize);
                vkUnmapMemory(device, stagingBufferMemory);

                createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, indexBuffer, indexBufferMemory);

                copyBuffer(stagingBuffer, indexBuffer, bufferSize);

                vkDestroyBuffer(device, stagingBuffer, nullptr);
                vkFreeMemory(device, stagingBufferMemory, nullptr);
            }

            // create uniform buffers
            {
                VkDeviceSize bufferSize = sizeof(UniformBufferObject);

                uniformBuffers.resize(MAX_FRAMES_IN_FLIGHT);
                uniformBuffersMemory.resize(MAX_FRAMES_IN_FLIGHT);
                uniformBuffersMapped.resize(MAX_FRAMES_IN_FLIGHT);

                for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
                {
                    createBuffer(bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, uniformBuffers[i], uniformBuffersMemory[i]);

                    vkMapMemory(device, uniformBuffersMemory[i], 0, bufferSize, 0, &uniformBuffersMapped[i]);
                }
            }

            // create descriptor pool
            {
                std::array<VkDescriptorPoolSize, 1> poolSizes{};
                poolSizes[0].type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                poolSizes[0].descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

                VkDescriptorPoolCreateInfo poolInfo{};
                poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
                poolInfo.pPoolSizes = poolSizes.data();
                poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

                if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to create descriptor pool!");
                }
            }

            // create descriptor sets
            {
                std::vector<VkDescriptorSetLayout> layouts(MAX_FRAMES_IN_FLIGHT, descriptorSetLayout);
                VkDescriptorSetAllocateInfo allocInfo{};
                allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                allocInfo.descriptorPool = descriptorPool;
                allocInfo.descriptorSetCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);
                allocInfo.pSetLayouts = layouts.data();

                descriptorSets.resize(MAX_FRAMES_IN_FLIGHT);
                if (vkAllocateDescriptorSets(device, &allocInfo, descriptorSets.data()) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to allocate descriptor sets!");
                }

                for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
                {
                    VkDescriptorBufferInfo bufferInfo{};
                    bufferInfo.buffer = uniformBuffers[i];
                    bufferInfo.offset = 0;
                    bufferInfo.range = sizeof(UniformBufferObject);

                    std::array<VkWriteDescriptorSet, 1> descriptorWrites{};

                    descriptorWrites[0].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    descriptorWrites[0].dstSet = descriptorSets[i];
                    descriptorWrites[0].dstBinding = 0;
                    descriptorWrites[0].dstArrayElement = 0;
                    descriptorWrites[0].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                    descriptorWrites[0].descriptorCount = 1;
                    descriptorWrites[0].pBufferInfo = &bufferInfo;

                    vkUpdateDescriptorSets(device, static_cast<uint32_t>(descriptorWrites.size()), descriptorWrites.data(), 0, nullptr);
                }
            }

            // create command buffers
            {
                commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

                VkCommandBufferAllocateInfo allocInfo{};
                allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                allocInfo.commandPool = commandPool;
                allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                allocInfo.commandBufferCount = (uint32_t)commandBuffers.size();

                if (vkAllocateCommandBuffers(device, &allocInfo, commandBuffers.data()) != VK_SUCCESS)
                {
                    throw std::runtime_error("failed to allocate command buffers!");
                }
            }

            // create sync objects
            {
                imageAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
                renderFinishedSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
                inFlightFences.resize(MAX_FRAMES_IN_FLIGHT);

                VkSemaphoreCreateInfo semaphoreInfo{};
                semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

                VkFenceCreateInfo fenceInfo{};
                fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

                for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
                {
                    if (vkCreateSemaphore(device, &semaphoreInfo, nullptr, &imageAvailableSemaphores[i]) != VK_SUCCESS ||
                        vkCreateSemaphore(device, &semaphoreInfo, nullptr, &renderFinishedSemaphores[i]) != VK_SUCCESS ||
                        vkCreateFence(device, &fenceInfo, nullptr, &inFlightFences[i]) != VK_SUCCESS)
                    {
                        throw std::runtime_error("failed to create synchronization objects for a frame!");
                    }
                }
            }
        }
        // main loop
        {
            while (!glfwWindowShouldClose(window))
            {
                bool gamepadExists = false;
                GLFWgamepadstate gamepadState;
                glfwPollEvents();
                if (glfwGetGamepadState(GLFW_JOYSTICK_1, &gamepadState))
                {
                    gamepadExists = true;
                    state.action = {(gamepadState.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] - gamepadState.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]) / 2, gamepadState.axes[GLFW_GAMEPAD_AXIS_LEFT_X], state.action.ballCamPressed};
                }
                if ((((gamepadExists && gamepadState.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_PRESS) || glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)) && !state.action.ballCamPressed)
                {
                    state.ballCam = !state.ballCam;
                    state.action.ballCamPressed = true;
                }
                else if ((gamepadExists && gamepadState.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_RELEASE) && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
                {
                    state.action.ballCamPressed = false;
                }
                if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
                {
                    state.action.throttle = std::min(state.action.throttle + 1.f, 1.f);
                }
                if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
                {
                    state.action.throttle = std::max(state.action.throttle - 1.f, -1.f);
                }
                if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
                {
                    state.action.steering = std::min(state.action.steering + 1.f, 1.f);
                }
                if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
                {
                    state.action.steering = std::max(state.action.steering - 1.f, -1.f);
                }
                {
                    vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

                    uint32_t imageIndex;
                    VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

                    if (result == VK_ERROR_OUT_OF_DATE_KHR)
                    {
                        recreateSwapChain();
                        continue;
                    }
                    else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
                    {
                        throw std::runtime_error("failed to acquire swap chain image!");
                    }

                    // update uniform buffer
                    {
                        constexpr float BALLCAM_RADIUS = 8;
                        UniformBufferObject ubo{};
                        glm::vec3 carPosition = glm::vec3(state.car.objectState.position.x(), state.car.objectState.position.y(), state.car.objectState.position.z());
                        glm::vec3 ballPosition = glm::vec3(state.ball.objectState.position.x(), state.ball.objectState.position.y(), state.ball.objectState.position.z());
                        glm::highp_mat4 rotation = glm::rotate(glm::mat4(1.0f), state.car.objectState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                        ubo.model[0] = glm::translate(glm::mat4(1.0f), carPosition) * rotation;
                        ubo.model[1] = glm::translate(glm::mat4(1.0f), ballPosition) * glm::rotate(glm::mat4(1.0f), state.ball.objectState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                        ubo.model[2] = glm::translate(glm::mat4(1.0f), glm::vec3(state.arena.objectState.position.x(), state.arena.objectState.position.y(), state.arena.objectState.position.z())) * glm::rotate(glm::mat4(1.0f), state.arena.objectState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                        glm::vec3 eye, center;
                        if (state.ballCam)
                        {
                            Eigen::Vector2f carPositionXZ = Eigen::Vector2f(state.car.objectState.position.x(), state.car.objectState.position.z());
                            Eigen::Vector2f ballPositionXZ = Eigen::Vector2f(state.ball.objectState.position.x(), state.ball.objectState.position.z());
                            Eigen::Vector2f eyeXZ = carPositionXZ - BALLCAM_RADIUS * (ballPositionXZ - carPositionXZ).normalized();
                            eye = glm::vec3(eyeXZ.x(), 2.0f, eyeXZ.y());
                            center = ballPosition;
                        }
                        else
                        {
                            eye = carPosition + glm::mat3(rotation) * glm::vec3(0.0f, 1.5f, -BALLCAM_RADIUS);
                            center = carPosition;
                        }
                        ubo.view = glm::lookAt(eye, center, glm::vec3(0.0f, 1.0, 0.0f));
                        ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float)swapChainExtent.height, 0.1f, 250.0f);
                        ubo.proj[1][1] *= -1;

                        memcpy(uniformBuffersMapped[currentFrame], &ubo, sizeof(ubo));
                    }

                    vkResetFences(device, 1, &inFlightFences[currentFrame]);

                    vkResetCommandBuffer(commandBuffers[currentFrame], /*VkCommandBufferResetFlagBits*/ 0);

                    // record command buffer
                    {
                        VkCommandBuffer commandBuffer = commandBuffers[currentFrame];

                        VkCommandBufferBeginInfo beginInfo{};
                        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

                        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
                        {
                            throw std::runtime_error("failed to begin recording command buffer!");
                        }

                        VkRenderPassBeginInfo renderPassInfo{};
                        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
                        renderPassInfo.renderPass = renderPass;
                        renderPassInfo.framebuffer = swapChainFramebuffers[imageIndex];
                        renderPassInfo.renderArea.offset = {0, 0};
                        renderPassInfo.renderArea.extent = swapChainExtent;

                        std::array<VkClearValue, 2> clearValues{};
                        clearValues[0].color = {{0.0f, 0.0f, 0.0f, 1.0f}};
                        clearValues[1].depthStencil = {1.0f, 0};

                        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
                        renderPassInfo.pClearValues = clearValues.data();

                        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

                        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, graphicsPipeline);

                        VkViewport viewport{};
                        viewport.x = 0.0f;
                        viewport.y = 0.0f;
                        viewport.width = (float)swapChainExtent.width;
                        viewport.height = (float)swapChainExtent.height;
                        viewport.minDepth = 0.0f;
                        viewport.maxDepth = 1.0f;
                        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

                        VkRect2D scissor{};
                        scissor.offset = {0, 0};
                        scissor.extent = swapChainExtent;
                        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

                        VkBuffer vertexBuffers[] = {vertexBuffer};
                        VkDeviceSize offsets[] = {0};
                        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);

                        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);

                        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT16);

                        uint index = 0;

                        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);

                        vkCmdDrawIndexed(commandBuffer, indicesOffsets[0], 1, 0, 0, 0);

                        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, indicesOffsets[0] * 2, VK_INDEX_TYPE_UINT16);

                        index = 1;

                        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);

                        vkCmdDrawIndexed(commandBuffer, indicesOffsets[1] - indicesOffsets[0], 1, 0, 0, 0);

                        vkCmdBindIndexBuffer(commandBuffer, indexBuffer, indicesOffsets[1] * 2, VK_INDEX_TYPE_UINT16);

                        index = 2;

                        vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);

                        vkCmdDrawIndexed(commandBuffer, indicesOffsets[2] - indicesOffsets[1], 1, 0, 0, 0);

                        vkCmdEndRenderPass(commandBuffer);

                        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
                        {
                            throw std::runtime_error("failed to record command buffer!");
                        }
                    }

                    VkSubmitInfo submitInfo{};
                    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

                    VkSemaphore waitSemaphores[] = {imageAvailableSemaphores[currentFrame]};
                    VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
                    submitInfo.waitSemaphoreCount = 1;
                    submitInfo.pWaitSemaphores = waitSemaphores;
                    submitInfo.pWaitDstStageMask = waitStages;

                    submitInfo.commandBufferCount = 1;
                    submitInfo.pCommandBuffers = &commandBuffers[currentFrame];

                    VkSemaphore signalSemaphores[] = {renderFinishedSemaphores[currentFrame]};
                    submitInfo.signalSemaphoreCount = 1;
                    submitInfo.pSignalSemaphores = signalSemaphores;

                    if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS)
                    {
                        throw std::runtime_error("failed to submit draw command buffer!");
                    }

                    VkPresentInfoKHR presentInfo{};
                    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

                    presentInfo.waitSemaphoreCount = 1;
                    presentInfo.pWaitSemaphores = signalSemaphores;

                    VkSwapchainKHR swapChains[] = {swapChain};
                    presentInfo.swapchainCount = 1;
                    presentInfo.pSwapchains = swapChains;

                    presentInfo.pImageIndices = &imageIndex;

                    result = vkQueuePresentKHR(presentQueue, &presentInfo);

                    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized)
                    {
                        framebufferResized = false;
                        recreateSwapChain();
                    }
                    else if (result != VK_SUCCESS)
                    {
                        throw std::runtime_error("failed to present swap chain image!");
                    }

                    currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
                }
            }

            vkDeviceWaitIdle(device);
        }

        // cleanup
        {
            state.action.close = true;
            cleanupSwapChain();

            vkDestroyPipeline(device, graphicsPipeline, nullptr);
            vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
            vkDestroyRenderPass(device, renderPass, nullptr);

            for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
            {
                vkDestroyBuffer(device, uniformBuffers[i], nullptr);
                vkFreeMemory(device, uniformBuffersMemory[i], nullptr);
            }

            vkDestroyDescriptorPool(device, descriptorPool, nullptr);

            vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);

            vkDestroyBuffer(device, indexBuffer, nullptr);
            vkFreeMemory(device, indexBufferMemory, nullptr);

            vkDestroyBuffer(device, vertexBuffer, nullptr);
            vkFreeMemory(device, vertexBufferMemory, nullptr);

            for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
            {
                vkDestroySemaphore(device, renderFinishedSemaphores[i], nullptr);
                vkDestroySemaphore(device, imageAvailableSemaphores[i], nullptr);
                vkDestroyFence(device, inFlightFences[i], nullptr);
            }

            vkDestroyCommandPool(device, commandPool, nullptr);

            vkDestroyDevice(device, nullptr);

            if (enableValidationLayers)
            {
                auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
                if (func != nullptr)
                {
                    func(instance, debugMessenger, nullptr);
                }
            }

            vkDestroySurfaceKHR(instance, surface, nullptr);
            vkDestroyInstance(instance, nullptr);

            glfwDestroyWindow(window);

            glfwTerminate();
        }
    }
};

int main()
{
    State state{
        {{{0.0f, 10.0f, 0.0f},
          {0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f}},
         {100.0f, 20.0f, 200.0f}},
        {{{0.0f, 0.375f, -5.0f},
          {0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f}},
         {1.25f, 0.75f, 2.f}},
        {{{0.0f, 1.0f, 0.0f},
          {0.0f, 0.0f, 0.0f},
          {0.0f, 0.0f, 0.0f}},
         1.0f},
        {20.0, 8.0},
        {0.0f, 0.0f, false, false},
        true};
    InputGraphics inputGraphics(state);
    try
    {
        std::thread inputGraphicsThread(&InputGraphics::run, &inputGraphics);
        std::thread physicsThread(&physics, std::ref(state));
        inputGraphicsThread.join();
        physicsThread.join();
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
