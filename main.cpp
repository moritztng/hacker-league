#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>

#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#include <glm/gtc/matrix_transform.hpp>

#include <iostream>
#include <iomanip>
#include <fstream>
#include <set>
#include <thread>
#include <optional>
#include <eigen3/Eigen/Dense>
#include "shaders/world/vert.spv.h"
#include "shaders/world/frag.spv.h"
#include "shaders/hud/vert.spv.h"
#include "shaders/hud/frag.spv.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <unistd.h>

#include <curl/curl.h>
#include "json.hpp"

#include "common.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "font.h"

struct Input
{
    Action action;
    bool ballCamPressed;
};

struct State
{
    Box arena;
    Eigen::Vector2f goal;
    Sphere ball;
    Eigen::Vector3f carSize;
    std::vector<Player> players;
    std::optional<std::array<uint8_t, 2>> scores;
    uint8_t playerId;
    int64_t countdown;
    int64_t transitionCountdown;
    Input input;
    bool running;
};

// TODO: lots of low hanging fruit to make it more efficient, sync with graphics
void physics(State &state, const std::vector<Player> &initialPlayers, std::optional<sockaddr_in6> &serverAddress)
{
    int udpSocket = -1;
    try
    {
        bool multiplayer = serverAddress.has_value();

        int udpSocket;
        if (multiplayer)
        {
            udpSocket = socket(AF_INET6, SOCK_DGRAM, 0);
            if (udpSocket < 0)
            {
                throw std::runtime_error("creating udp socket");
            }
            char buffer = 0;
            sendto(udpSocket, &buffer, sizeof(buffer), 0, (struct sockaddr *)&serverAddress, sizeof(serverAddress));
            int recvLength = recv(udpSocket, &buffer, sizeof(buffer), 0);
            if (recvLength == 0)
            {
                throw std::runtime_error("receiving playerId");
            }
            state.playerId = buffer;

            int flags = fcntl(udpSocket, F_GETFL, 0);
            if (flags == -1)
            {
                throw std::runtime_error("getting socket flags");
            }
            if (fcntl(udpSocket, F_SETFL, flags | O_NONBLOCK) == -1)
            {
                throw std::runtime_error("setting O_NONBLOCK flag");
            }
        }

        constexpr uint FREQUENCY = 60;
        constexpr float PERIOD = 1.f / FREQUENCY;
        constexpr size_t N_RECORDS = 5 * FREQUENCY;

        Sphere ball = state.ball;
        std::vector<Player> players = state.players;
        Player records[N_RECORDS];
        uint32_t stateId = 0;
        uint32_t statesBehind = 0;
        const auto period = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<float>(PERIOD));
        auto targetTime = std::chrono::high_resolution_clock::now();
        while (state.running)
        {
            if (statesBehind == 0)
            {
                players[state.playerId].action = state.input.action;
                if (multiplayer)
                {
                    records[stateId % N_RECORDS] = players[state.playerId];
                }
            }
            else
            {
                players[state.playerId] = records[stateId % N_RECORDS];
            }

            if (multiplayer)
            {
                if (statesBehind == 0)
                {
                    char buffer[102];
                    std::memcpy(buffer, &stateId, 4);
                    std::memcpy(buffer + 4, players[state.playerId].carState.position.data(), 12);
                    std::memcpy(buffer + 16, players[state.playerId].carState.velocity.data(), 12);
                    std::memcpy(buffer + 28, players[state.playerId].carState.orientation.data(), 12);
                    std::memcpy(buffer + 40, &players[state.playerId].action.steering, 4);
                    std::memcpy(buffer + 44, &players[state.playerId].action.throttle, 4);
                    sendto(udpSocket, buffer, sizeof(buffer), 0, (struct sockaddr *)&serverAddress, sizeof(serverAddress));

                    int recvLength = recv(udpSocket, buffer, sizeof(buffer), 0);
                    if (recvLength > 0)
                    {
                        uint32_t serverId;
                        uint8_t otherPlayer = state.playerId ^ 1;
                        std::memcpy(&serverId, buffer, 4);
                        std::memcpy(players[otherPlayer].carState.position.data(), buffer + 4, 12);
                        std::memcpy(players[otherPlayer].carState.velocity.data(), buffer + 16, 12);
                        std::memcpy(players[otherPlayer].carState.orientation.data(), buffer + 28, 12);
                        std::memcpy(&players[otherPlayer].action.steering, buffer + 40, 4);
                        std::memcpy(&players[otherPlayer].action.throttle, buffer + 44, 4);
                        std::memcpy(ball.objectState.position.data(), buffer + 48, 12);
                        std::memcpy(ball.objectState.velocity.data(), buffer + 60, 12);
                        std::memcpy(ball.objectState.orientation.data(), buffer + 72, 12);
                        std::memcpy(&state.countdown, buffer + 84, 8);
                        std::memcpy(&state.transitionCountdown, buffer + 92, 8);
                        std::memcpy(&state.scores, buffer + 100, 2);

                        // TODO: fix countdown and score latency
                        if (state.transitionCountdown > 0)
                        {
                            players[state.playerId] = initialPlayers[state.playerId];
                        }
                        else
                        {
                            players[state.playerId] = records[serverId % N_RECORDS];
                            statesBehind = stateId - serverId;
                            stateId = serverId;
                        }
                    }
                }
                else
                {
                    statesBehind--;
                }
            }

            physicsStep(state.arena.size, state.goal, ball, state.carSize, players, !multiplayer);

            if (statesBehind == 0)
            {
                state.ball = ball;
                state.players = players;
                targetTime += period;
                std::this_thread::sleep_until(targetTime);
            }
            stateId++;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << std::endl;
    }
    if (udpSocket > -1)
        close(udpSocket);
    state.running = false;
}

constexpr uint32_t WIDTH = 1280;
constexpr uint32_t HEIGHT = 720;
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
        std::vector<VkPresentModeKHR> presentModes;
    };

    struct Vertex
    {
        glm::vec3 position;
        glm::vec3 normal;
    };

    struct HudVertex
    {
        glm::vec2 position;
        glm::vec2 textureCoordinate;
    };

    struct UniformBufferObject
    {
        alignas(16) glm::mat4 model[4];
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
    };

    GLFWwindow *window;

    VkInstance instance;
    VkDebugUtilsMessengerEXT debugMessenger;
    VkSurfaceKHR surface;
    VkSurfaceFormatKHR surfaceFormat;

    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice device;

    VkQueue graphicsQueue;
    VkQueue presentQueue;

    VkSwapchainKHR swapChain;
    std::vector<VkImage> swapChainImages;
    VkExtent2D swapChainExtent;
    std::vector<VkImageView> swapChainImageViews;
    std::vector<VkFramebuffer> swapChainFramebuffers;

    VkRenderPass renderPass;
    VkDescriptorSetLayout descriptorSetLayout;
    VkPipelineLayout pipelineLayout;
    VkPipeline graphicsPipeline;

    VkCommandPool commandPool;

    VkFormat depthFormat = VK_FORMAT_UNDEFINED;
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

    VkDeviceMemory fontImageMemory;
    VkImage fontImage;
    VkImageView fontImageView;
    VkSampler fontSampler;

    VkDescriptorPool hudDescriptorPool;
    VkDescriptorSet hudDescriptorSet;
    VkDescriptorSetLayout hudDescriptorSetLayout;
    VkPipeline hudPipeline;
    VkPipelineLayout hudPipelineLayout;
    VkBuffer hudVertexBuffer;
    VkDeviceMemory hudVertexBufferMemory;
    void *hudMappedData;

    std::vector<VkCommandBuffer> commandBuffers;

    std::vector<VkSemaphore> imageAvailableSemaphores;
    std::vector<VkSemaphore> renderFinishedSemaphores;
    std::vector<VkFence> inFlightFences;
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
            throw std::runtime_error("vulkan create image view");
        }

        return imageView;
    }

    SwapChainSupportDetails querySwapChainSupport(VkPhysicalDevice device)
    {
        SwapChainSupportDetails details;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, surface, &details.capabilities);
        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, nullptr);
        if (presentModeCount != 0)
        {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(device, surface, &presentModeCount, details.presentModes.data());
        }
        return details;
    }

    void createSwapChainFramebuffers()
    {
        // swap chain
        {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(physicalDevice);

            VkExtent2D extent;
            if (swapChainSupport.capabilities.currentExtent.width != std::numeric_limits<uint32_t>::max())
            {
                extent = swapChainSupport.capabilities.currentExtent;
            }
            else
            {
                int width, height;
                glfwGetFramebufferSize(window, &width, &height);
                extent = VkExtent2D{std::clamp(static_cast<uint32_t>(width), swapChainSupport.capabilities.minImageExtent.width, swapChainSupport.capabilities.maxImageExtent.width), std::clamp(static_cast<uint32_t>(height), swapChainSupport.capabilities.minImageExtent.height, swapChainSupport.capabilities.maxImageExtent.height)};
            }

            uint32_t imageCount = swapChainSupport.capabilities.minImageCount;
            if (swapChainSupport.capabilities.maxImageCount == 0 || swapChainSupport.capabilities.minImageCount < swapChainSupport.capabilities.maxImageCount)
            {
                imageCount++;
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
            createInfo.presentMode = VK_PRESENT_MODE_FIFO_KHR;
            createInfo.clipped = VK_TRUE;

            if (vkCreateSwapchainKHR(device, &createInfo, nullptr, &swapChain) != VK_SUCCESS)
            {
                throw std::runtime_error("vulkan create swap chain");
            }

            vkGetSwapchainImagesKHR(device, swapChain, &imageCount, nullptr);
            swapChainImages.resize(imageCount);
            vkGetSwapchainImagesKHR(device, swapChain, &imageCount, swapChainImages.data());

            swapChainExtent = extent;

            swapChainImageViews.resize(swapChainImages.size());

            for (uint32_t i = 0; i < swapChainImages.size(); i++)
            {
                swapChainImageViews[i] = createImageView(swapChainImages[i], surfaceFormat.format, VK_IMAGE_ASPECT_COLOR_BIT);
            }
        }

        // frame buffers
        {
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
                throw std::runtime_error("vulkan create image");
            }

            VkMemoryRequirements memRequirements;
            vkGetImageMemoryRequirements(device, depthImage, &memRequirements);

            VkMemoryAllocateInfo allocInfo{};
            allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
            allocInfo.allocationSize = memRequirements.size;
            allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

            if (vkAllocateMemory(device, &allocInfo, nullptr, &depthImageMemory) != VK_SUCCESS)
            {
                throw std::runtime_error("vulkan allocate image memory");
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
                    throw std::runtime_error("vulkan create framebuffer");
                }
            }
        }
    }

    VkShaderModule createShaderModule(const unsigned char *code, const size_t codeSize)
    {
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = codeSize;
        createInfo.pCode = reinterpret_cast<const uint32_t *>(code);

        VkShaderModule shaderModule;
        if (vkCreateShaderModule(device, &createInfo, nullptr, &shaderModule) != VK_SUCCESS)
        {
            throw std::runtime_error("vulkan create shader module");
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

        throw std::runtime_error("vulkan find suitable memory type");
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
            throw std::runtime_error("vulkan create buffer");
        }

        VkMemoryRequirements memRequirements;
        vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS)
        {
            throw std::runtime_error("vulkan allocate buffer memory");
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
        while (width == 0 || height == 0)
        {
            glfwGetFramebufferSize(window, &width, &height);
            glfwWaitEvents();
        }

        vkDeviceWaitIdle(device);

        cleanupSwapChain();
        createSwapChainFramebuffers();
    }

    VkCommandBuffer beginSingleTimeCommands()
    {
        VkCommandBufferAllocateInfo allocInfo = {};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandPool = commandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo = {};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        return commandBuffer;
    }

    void endSingleTimeCommands(VkCommandBuffer commandBuffer)
    {
        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo = {};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        vkQueueSubmit(graphicsQueue, 1, &submitInfo, VK_NULL_HANDLE);
        vkQueueWaitIdle(graphicsQueue);

        vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    }

    void transitionImageLayout(VkImage image, VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout)
    {
        VkCommandBuffer commandBuffer = beginSingleTimeCommands();

        VkImageMemoryBarrier barrier = {};
        barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
        barrier.oldLayout = oldLayout;
        barrier.newLayout = newLayout;
        barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        barrier.image = image;
        barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        barrier.subresourceRange.baseMipLevel = 0;
        barrier.subresourceRange.levelCount = 1;
        barrier.subresourceRange.baseArrayLayer = 0;
        barrier.subresourceRange.layerCount = 1;

        VkPipelineStageFlags sourceStage;
        VkPipelineStageFlags destinationStage;
        if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED && newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL)
        {
            barrier.srcAccessMask = 0;
            barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

            sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
            destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
        }
        else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL && newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
        {
            barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
            barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

            sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
            destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        }
        else
        {
            throw std::invalid_argument("vulkan unsupported layout transition");
        }

        vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0, nullptr, 0, nullptr, 1, &barrier);

        endSingleTimeCommands(commandBuffer);
    }

public:
    InputGraphics(State &state)
        : state(state) {}
    void run()
    {
        try
        {
            // init window
            {
                glfwInit();
                glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);

                window = glfwCreateWindow(WIDTH, HEIGHT, "hacker league", nullptr, nullptr);
                glfwSetWindowUserPointer(window, this);
                glfwSetFramebufferSizeCallback(window, framebufferResizeCallback);

                // TODO: environment variable
                std::ifstream file("gamepad.txt");
                if (!file)
                {
                    throw std::runtime_error("opening gamepad.txt");
                }
                std::string fileContents((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
                glfwUpdateGamepadMappings(fileContents.c_str());
            }

            // init vulkan
            {
                VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo;
                QueueFamilyIndices queueFamilyIndices;

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
                                throw std::runtime_error("vulkan validation layers requested, but not available");
                            }
                        }
                    }

                    VkApplicationInfo appInfo{};
                    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
                    appInfo.pApplicationName = "hacker league";
                    appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
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
                        throw std::runtime_error("vulkan create instance");
                    }
                }

                // setup debug messenger
                {
                    if (enableValidationLayers)
                    {
                        auto func = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
                        const char *errorMessage = "vulkan set up debug messenger";
                        if (func != nullptr)
                        {
                            if (func(instance, &debugCreateInfo, nullptr, &debugMessenger) != VK_SUCCESS)
                            {
                                throw std::runtime_error(errorMessage);
                            }
                        }
                        else
                        {
                            throw std::runtime_error(errorMessage);
                        }
                    }
                }

                // create surface
                {
                    if (glfwCreateWindowSurface(instance, window, nullptr, &surface) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create window surface");
                    }
                }

                // pick physical device
                {
                    uint32_t deviceCount = 0;
                    vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr);

                    if (deviceCount == 0)
                    {
                        throw std::runtime_error("vulkan find GPUs with Vulkan support");
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

                        uint32_t formatCount;
                        vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, nullptr);
                        std::vector<VkSurfaceFormatKHR> formats;
                        if (formatCount != 0)
                        {
                            formats.resize(formatCount);
                            vkGetPhysicalDeviceSurfaceFormatsKHR(device, surface, &formatCount, formats.data());
                        }

                        if (indices.isComplete() && extensionsSupported && !formats.empty() && !swapChainSupport.presentModes.empty())
                        {
                            physicalDevice = device;
                            queueFamilyIndices = indices;

                            surfaceFormat = formats[0];
                            for (const auto &availableFormat : formats)
                            {
                                if (availableFormat.format == VK_FORMAT_B8G8R8A8_SRGB && availableFormat.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
                                {
                                    surfaceFormat = availableFormat;
                                }
                            }
                            break;
                        }
                    }

                    if (physicalDevice == VK_NULL_HANDLE)
                    {
                        throw std::runtime_error("vulkan find a suitable GPU");
                    }
                }

                // create logical device
                {
                    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
                    std::set<uint32_t> uniqueQueueFamilies = {queueFamilyIndices.graphicsFamily.value(), queueFamilyIndices.presentFamily.value()};

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

                    VkPhysicalDeviceFeatures deviceFeatures{}; // no textures -> samplerAnisotropy false
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
                        throw std::runtime_error("vulkan create logical device");
                    }

                    vkGetDeviceQueue(device, queueFamilyIndices.graphicsFamily.value(), 0, &graphicsQueue);
                    vkGetDeviceQueue(device, queueFamilyIndices.presentFamily.value(), 0, &presentQueue);
                }

                // find depth format
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
                            depthFormat = format;
                            break;
                        }
                    }
                    if (depthFormat == VK_FORMAT_UNDEFINED)
                    {
                        throw std::runtime_error("vulkan find supported depth format");
                    }
                }

                // create render pass
                {
                    VkAttachmentDescription colorAttachment{};
                    colorAttachment.format = surfaceFormat.format;
                    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
                    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
                    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
                    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

                    VkAttachmentDescription depthAttachment{};
                    depthAttachment.format = depthFormat;
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

                    VkSubpassDescription hudSubpass = {};
                    hudSubpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
                    hudSubpass.colorAttachmentCount = 1;
                    hudSubpass.pColorAttachments = &colorAttachmentRef;
                    hudSubpass.pDepthStencilAttachment = nullptr;

                    VkSubpassDependency dependency{};
                    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
                    dependency.dstSubpass = 0;
                    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
                    dependency.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
                    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
                    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

                    VkSubpassDependency hudDependency = {};
                    hudDependency.srcSubpass = 0;
                    hudDependency.dstSubpass = 1;
                    hudDependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                    hudDependency.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
                    hudDependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                    hudDependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT;

                    std::array<VkAttachmentDescription, 2> attachments = {colorAttachment, depthAttachment};
                    std::array<VkSubpassDescription, 2> subpasses = {subpass, hudSubpass};
                    std::array<VkSubpassDependency, 2> dependencies = {dependency, hudDependency};
                    VkRenderPassCreateInfo renderPassInfo{};
                    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
                    renderPassInfo.attachmentCount = 2;
                    renderPassInfo.pAttachments = attachments.data();
                    renderPassInfo.subpassCount = 2;
                    renderPassInfo.pSubpasses = subpasses.data();
                    renderPassInfo.dependencyCount = 2;
                    renderPassInfo.pDependencies = dependencies.data();

                    if (vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create render pass");
                    }
                }

                // create command pool
                {
                    VkCommandPoolCreateInfo poolInfo{};
                    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
                    poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

                    if (vkCreateCommandPool(device, &poolInfo, nullptr, &commandPool) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create graphics command pool");
                    }
                }

                // create graphics pipelines
                {
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
                    rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
                    rasterizer.depthBiasEnable = VK_FALSE;
                    VkPipelineMultisampleStateCreateInfo multisampling{};
                    multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
                    multisampling.sampleShadingEnable = VK_FALSE;
                    multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
                    std::vector<VkDynamicState> dynamicStates = {
                        VK_DYNAMIC_STATE_VIEWPORT,
                        VK_DYNAMIC_STATE_SCISSOR};
                    VkPipelineDynamicStateCreateInfo dynamicState{};
                    dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
                    dynamicState.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
                    dynamicState.pDynamicStates = dynamicStates.data();

                    // create world graphics pipeline
                    {
                        VkDescriptorSetLayoutBinding uboLayoutBinding{};
                        uboLayoutBinding.binding = 0;
                        uboLayoutBinding.descriptorCount = 1;
                        uboLayoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                        uboLayoutBinding.pImmutableSamplers = nullptr;
                        uboLayoutBinding.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

                        VkDescriptorSetLayoutCreateInfo layoutInfo{};
                        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                        layoutInfo.bindingCount = 1;
                        layoutInfo.pBindings = &uboLayoutBinding;

                        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &descriptorSetLayout) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan create descriptor set layout");
                        }

                        VkShaderModule vertShaderModule = createShaderModule(__shaders_world_vert_spv, __shaders_world_vert_spv_len);
                        VkShaderModule fragShaderModule = createShaderModule(__shaders_world_frag_spv, __shaders_world_frag_spv_len);

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

                        VkVertexInputBindingDescription bindingDescription{};
                        bindingDescription.binding = 0;
                        bindingDescription.stride = sizeof(Vertex);
                        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

                        std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions{};
                        attributeDescriptions[0].binding = 0;
                        attributeDescriptions[0].location = 0;
                        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
                        attributeDescriptions[0].offset = offsetof(Vertex, position);
                        attributeDescriptions[1].binding = 0;
                        attributeDescriptions[1].location = 1;
                        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
                        attributeDescriptions[1].offset = offsetof(Vertex, normal);

                        vertexInputInfo.vertexBindingDescriptionCount = 1;
                        vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
                        vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
                        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

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

                        VkPushConstantRange pushConstantRange = {};
                        pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                        pushConstantRange.offset = 0;
                        pushConstantRange.size = sizeof(uint);

                        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
                        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                        pipelineLayoutInfo.setLayoutCount = 1;
                        pipelineLayoutInfo.pSetLayouts = &descriptorSetLayout;
                        pipelineLayoutInfo.pushConstantRangeCount = 1;
                        pipelineLayoutInfo.pPushConstantRanges = &pushConstantRange;

                        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan create pipeline layout");
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
                            throw std::runtime_error("vulkan create graphics pipeline");
                        }

                        vkDestroyShaderModule(device, fragShaderModule, nullptr);
                        vkDestroyShaderModule(device, vertShaderModule, nullptr);
                    }

                    // create hud graphics pipeline
                    {
                        VkDescriptorSetLayoutBinding layoutBinding{};
                        layoutBinding.binding = 0;
                        layoutBinding.descriptorCount = 1;
                        layoutBinding.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                        layoutBinding.pImmutableSamplers = nullptr;
                        layoutBinding.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                        VkDescriptorSetLayoutCreateInfo layoutInfo{};
                        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                        layoutInfo.bindingCount = 1;
                        layoutInfo.pBindings = &layoutBinding;

                        if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &hudDescriptorSetLayout) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan create descriptor set layout");
                        }

                        VkPipelineColorBlendAttachmentState colorBlendAttachment = {};
                        colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT |
                                                              VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
                        colorBlendAttachment.blendEnable = VK_TRUE;
                        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
                        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
                        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
                        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
                        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

                        VkPipelineColorBlendStateCreateInfo colorBlending = {};
                        colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
                        colorBlending.logicOpEnable = VK_FALSE;
                        colorBlending.attachmentCount = 1;
                        colorBlending.pAttachments = &colorBlendAttachment;

                        VkVertexInputBindingDescription bindingDescription = {};
                        bindingDescription.binding = 0;
                        bindingDescription.stride = sizeof(HudVertex);
                        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

                        std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {};
                        attributeDescriptions[0].binding = 0;
                        attributeDescriptions[0].location = 0;
                        attributeDescriptions[0].format = VK_FORMAT_R32G32_SFLOAT;
                        attributeDescriptions[0].offset = offsetof(HudVertex, position);

                        attributeDescriptions[1].binding = 0;
                        attributeDescriptions[1].location = 1;
                        attributeDescriptions[1].format = VK_FORMAT_R32G32_SFLOAT;
                        attributeDescriptions[1].offset = offsetof(HudVertex, textureCoordinate);

                        VkPipelineVertexInputStateCreateInfo vertexInputInfo = {};
                        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
                        vertexInputInfo.vertexBindingDescriptionCount = 1;
                        vertexInputInfo.pVertexBindingDescriptions = &bindingDescription;
                        vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attributeDescriptions.size());
                        vertexInputInfo.pVertexAttributeDescriptions = attributeDescriptions.data();

                        VkShaderModule vertShaderModule = createShaderModule(__shaders_hud_vert_spv, __shaders_hud_vert_spv_len);
                        VkShaderModule fragShaderModule = createShaderModule(__shaders_hud_frag_spv, __shaders_hud_frag_spv_len);
                        VkPipelineShaderStageCreateInfo shaderStages[2] = {};

                        shaderStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                        shaderStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
                        shaderStages[0].module = vertShaderModule;
                        shaderStages[0].pName = "main";

                        shaderStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
                        shaderStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
                        shaderStages[1].module = fragShaderModule;
                        shaderStages[1].pName = "main";

                        VkPipelineDepthStencilStateCreateInfo depthStencil = {};
                        depthStencil.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
                        depthStencil.depthTestEnable = VK_FALSE;
                        depthStencil.depthWriteEnable = VK_FALSE;
                        depthStencil.depthCompareOp = VK_COMPARE_OP_ALWAYS;
                        depthStencil.stencilTestEnable = VK_FALSE;

                        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
                        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                        pipelineLayoutInfo.setLayoutCount = 1;
                        pipelineLayoutInfo.pSetLayouts = &hudDescriptorSetLayout;

                        if (vkCreatePipelineLayout(device, &pipelineLayoutInfo, nullptr, &hudPipelineLayout) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan create pipeline layout");
                        }

                        VkGraphicsPipelineCreateInfo pipelineInfoHUD = {};
                        pipelineInfoHUD.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
                        pipelineInfoHUD.stageCount = 2;
                        pipelineInfoHUD.pStages = shaderStages;
                        pipelineInfoHUD.pVertexInputState = &vertexInputInfo;
                        pipelineInfoHUD.pInputAssemblyState = &inputAssembly;
                        pipelineInfoHUD.pViewportState = &viewportState;
                        pipelineInfoHUD.pRasterizationState = &rasterizer;
                        pipelineInfoHUD.pMultisampleState = &multisampling;
                        pipelineInfoHUD.pDepthStencilState = &depthStencil;
                        pipelineInfoHUD.pColorBlendState = &colorBlending;
                        pipelineInfoHUD.pDynamicState = &dynamicState;
                        pipelineInfoHUD.layout = hudPipelineLayout;
                        pipelineInfoHUD.renderPass = renderPass;
                        pipelineInfoHUD.subpass = 1;

                        if (vkCreateGraphicsPipelines(device, VK_NULL_HANDLE, 1, &pipelineInfoHUD, nullptr, &hudPipeline) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan create HUD pipeline");
                        }

                        vkDestroyShaderModule(device, fragShaderModule, nullptr);
                        vkDestroyShaderModule(device, vertShaderModule, nullptr);
                    }
                }

                // create swap chain and framebuffers
                {
                    createSwapChainFramebuffers();
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
                    VkDescriptorPoolSize poolSize{};
                    poolSize.type = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                    poolSize.descriptorCount = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

                    VkDescriptorPoolCreateInfo poolInfo{};
                    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                    poolInfo.poolSizeCount = 1;
                    poolInfo.pPoolSizes = &poolSize;
                    poolInfo.maxSets = static_cast<uint32_t>(MAX_FRAMES_IN_FLIGHT);

                    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &descriptorPool) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create descriptor pool");
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
                        throw std::runtime_error("vulkan allocate descriptor sets");
                    }

                    for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
                    {
                        VkDescriptorBufferInfo bufferInfo{};
                        bufferInfo.buffer = uniformBuffers[i];
                        bufferInfo.offset = 0;
                        bufferInfo.range = sizeof(UniformBufferObject);

                        VkWriteDescriptorSet descriptorWrite{};
                        descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        descriptorWrite.dstSet = descriptorSets[i];
                        descriptorWrite.dstBinding = 0;
                        descriptorWrite.dstArrayElement = 0;
                        descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                        descriptorWrite.descriptorCount = 1;
                        descriptorWrite.pBufferInfo = &bufferInfo;

                        vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);
                    }
                }

                // create vertex and index buffer
                {
                    // TODO: fix it for rasterizer.cullMode = VK_CULL_MODE_BACK_BIT, simplify
                    std::vector<Vertex> vertices;
                    std::vector<uint16_t> indices;
                    Eigen::Vector3f halfSize;
                    std::vector<glm::vec3> points;
                    std::vector<std::pair<std::array<int, 4>, glm::vec3>> faces;
                    int verticesOffset;

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
                    for (const auto &[indices, normal] : faces)
                    {
                        for (int index : indices)
                        {
                            vertices.push_back({points[index], normal});
                        }
                    }
                    for (uint16_t i = 0; i < faces.size() * 4; i += 4)
                    {
                        indices.insert(indices.end(), {i, static_cast<uint16_t>(i + 1), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 2), static_cast<uint16_t>(i + 3), i});
                    }
                    indicesOffsets.push_back(indices.size());

                    verticesOffset = vertices.size();
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

                    verticesOffset = vertices.size();
                    halfSize = state.carSize / 2.f;
                    points = {{{-halfSize.x(), -halfSize.y(), -halfSize.z()},
                               {halfSize.x(), -halfSize.y(), -halfSize.z()},
                               {halfSize.x(), halfSize.y(), -halfSize.z()},
                               {-halfSize.x(), halfSize.y(), -halfSize.z()},
                               {-halfSize.x(), -halfSize.y(), halfSize.z()},
                               {halfSize.x(), -halfSize.y(), halfSize.z()},
                               {halfSize.x(), halfSize.y(), halfSize.z()},
                               {-halfSize.x(), halfSize.y(), halfSize.z()}}};
                    faces = {{{{0, 1, 2, 3}, {0.0f, 0.0f, -1.0f}},
                              {{4, 5, 6, 7}, {0.0f, 0.0f, 1.0f}},
                              {{0, 1, 5, 4}, {0.0f, -1.0f, 0.0f}},
                              {{2, 3, 7, 6}, {0.0f, 1.0f, 0.0f}},
                              {{0, 3, 7, 4}, {-1.0f, 0.0f, 0.0f}},
                              {{1, 2, 6, 5}, {1.0f, 0.0f, 0.0f}}}};
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

                // create hud descriptor pool
                {
                    VkDescriptorPoolSize poolSize{};
                    poolSize.type = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    poolSize.descriptorCount = 1;

                    VkDescriptorPoolCreateInfo poolInfo{};
                    poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                    poolInfo.poolSizeCount = 1;
                    poolInfo.pPoolSizes = &poolSize;
                    poolInfo.maxSets = 1;

                    if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &hudDescriptorPool) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create descriptor pool");
                    }
                }

                // create hud descriptor sets
                {
                    int textureWidth, textureHeight, textureChannels;
                    stbi_uc *data = stbi_load("font.png", &textureWidth, &textureHeight, &textureChannels, STBI_rgb_alpha);
                    VkDeviceSize imageSize = textureWidth * textureHeight * 4;

                    VkImageCreateInfo imageInfo{};
                    imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    imageInfo.imageType = VK_IMAGE_TYPE_2D;
                    imageInfo.extent.width = (uint32_t)textureWidth;
                    imageInfo.extent.height = (uint32_t)textureHeight;
                    imageInfo.extent.depth = 1;
                    imageInfo.mipLevels = 1;
                    imageInfo.arrayLayers = 1;
                    imageInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
                    imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
                    imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                    imageInfo.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                    imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
                    imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

                    if (vkCreateImage(device, &imageInfo, nullptr, &fontImage) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create image");
                    }

                    VkMemoryRequirements memRequirements;
                    vkGetImageMemoryRequirements(device, fontImage, &memRequirements);

                    VkMemoryAllocateInfo allocInfo{};
                    allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
                    allocInfo.allocationSize = memRequirements.size;
                    allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

                    if (vkAllocateMemory(device, &allocInfo, nullptr, &fontImageMemory) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan allocate image memory");
                    }

                    vkBindImageMemory(device, fontImage, fontImageMemory, 0);

                    transitionImageLayout(fontImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);

                    VkBuffer stagingBuffer;
                    VkDeviceMemory stagingBufferMemory;
                    createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, stagingBuffer, stagingBufferMemory);
                    void *mappedData;
                    vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &mappedData);
                    memcpy(mappedData, data, static_cast<size_t>(imageSize));
                    vkUnmapMemory(device, stagingBufferMemory);

                    VkCommandBuffer commandBuffer = beginSingleTimeCommands();

                    VkBufferImageCopy region{};
                    region.bufferOffset = 0;
                    region.bufferRowLength = 0;   // Zero for non-2D images
                    region.bufferImageHeight = 0; // Zero for non-2D images
                    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    region.imageSubresource.mipLevel = 0;
                    region.imageSubresource.baseArrayLayer = 0;
                    region.imageSubresource.layerCount = 1;
                    region.imageOffset = {0, 0, 0};
                    region.imageExtent = {(uint32_t)textureWidth, (uint32_t)textureHeight, 1};

                    vkCmdCopyBufferToImage(
                        commandBuffer,
                        stagingBuffer,
                        fontImage,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        1,
                        &region);

                    endSingleTimeCommands(commandBuffer);

                    transitionImageLayout(fontImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

                    vkDestroyBuffer(device, stagingBuffer, nullptr);
                    vkFreeMemory(device, stagingBufferMemory, nullptr);

                    fontImageView = createImageView(fontImage, VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_ASPECT_COLOR_BIT);

                    VkSamplerCreateInfo samplerInfo{};
                    samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
                    samplerInfo.magFilter = VK_FILTER_LINEAR;
                    samplerInfo.minFilter = VK_FILTER_LINEAR;
                    samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                    samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                    samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                    samplerInfo.anisotropyEnable = VK_TRUE;
                    samplerInfo.maxAnisotropy = 16;
                    samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
                    samplerInfo.unnormalizedCoordinates = VK_FALSE;
                    samplerInfo.compareEnable = VK_FALSE;
                    samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
                    samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
                    samplerInfo.mipLodBias = 0.0f;
                    samplerInfo.minLod = 0.0f;
                    samplerInfo.maxLod = 0.0f;

                    if (vkCreateSampler(device, &samplerInfo, nullptr, &fontSampler) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan create texture sampler");
                    }

                    VkDescriptorSetAllocateInfo descriptorAllocInfo{};
                    descriptorAllocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                    descriptorAllocInfo.descriptorPool = hudDescriptorPool;
                    descriptorAllocInfo.descriptorSetCount = 1;
                    descriptorAllocInfo.pSetLayouts = &hudDescriptorSetLayout;

                    if (vkAllocateDescriptorSets(device, &descriptorAllocInfo, &hudDescriptorSet) != VK_SUCCESS)
                    {
                        throw std::runtime_error("vulkan allocate descriptor set");
                    }

                    VkDescriptorImageInfo descriptorImageInfo{};
                    descriptorImageInfo.imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    descriptorImageInfo.imageView = fontImageView;
                    descriptorImageInfo.sampler = fontSampler;

                    VkWriteDescriptorSet descriptorWrite{};
                    descriptorWrite.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    descriptorWrite.dstSet = hudDescriptorSet;
                    descriptorWrite.dstBinding = 0;
                    descriptorWrite.dstArrayElement = 0;
                    descriptorWrite.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    descriptorWrite.descriptorCount = 1;
                    descriptorWrite.pImageInfo = &descriptorImageInfo;

                    vkUpdateDescriptorSets(device, 1, &descriptorWrite, 0, nullptr);

                    stbi_image_free(data);
                }

                // hud vertex buffer
                {
                    VkDeviceSize bufferSize = sizeof(HudVertex) * 13 * 6;

                    createBuffer(bufferSize, VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT, hudVertexBuffer, hudVertexBufferMemory);

                    vkMapMemory(device, hudVertexBufferMemory, 0, bufferSize, 0, &hudMappedData);
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
                        throw std::runtime_error("vulkan allocate command buffers");
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
                            throw std::runtime_error("vulkan create synchronization objects for a frame");
                        }
                    }
                }
            }

            // main loop
            {
                uint32_t currentFrame = 0;
                bool ballCam = true;
                std::optional<float> steeringDrift;
                std::string text{};
                while (state.running && !glfwWindowShouldClose(window))
                {
                    // actions
                    {
                        glfwPollEvents();
                        if (true) //(state.statesBehind == 0)
                        {
                            bool gamepadExists = false;
                            GLFWgamepadstate gamepadState;
                            if (glfwGetGamepadState(GLFW_JOYSTICK_1, &gamepadState))
                            {
                                gamepadExists = true;
                                if (!steeringDrift.has_value())
                                {
                                    steeringDrift = gamepadState.axes[GLFW_GAMEPAD_AXIS_LEFT_X];
                                }
                                state.input.action.throttle = (gamepadState.axes[GLFW_GAMEPAD_AXIS_RIGHT_TRIGGER] - gamepadState.axes[GLFW_GAMEPAD_AXIS_LEFT_TRIGGER]) / 2;
                                state.input.action.steering = gamepadState.axes[GLFW_GAMEPAD_AXIS_LEFT_X] - *steeringDrift;
                            }
                            else
                            {
                                state.input.action.throttle = 0.f;
                                state.input.action.steering = 0.f;
                            }
                            if ((((gamepadExists && gamepadState.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_PRESS) || glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)) && !state.input.ballCamPressed)
                            {
                                ballCam = !ballCam;
                                state.input.ballCamPressed = true;
                            }
                            else if ((!gamepadExists || gamepadState.buttons[GLFW_GAMEPAD_BUTTON_Y] == GLFW_RELEASE) && glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
                            {
                                state.input.ballCamPressed = false;
                            }
                            if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
                            {
                                state.input.action.throttle = std::min(state.input.action.throttle + 1.f, 1.f);
                            }
                            if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
                            {
                                state.input.action.throttle = std::max(state.input.action.throttle - 1.f, -1.f);
                            }
                            if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
                            {
                                state.input.action.steering = std::min(state.input.action.steering + 1.f, 1.f);
                            }
                            if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
                            {
                                state.input.action.steering = std::max(state.input.action.steering - 1.f, -1.f);
                            }
                        }
                    }
                    // present
                    {
                        vkWaitForFences(device, 1, &inFlightFences[currentFrame], VK_TRUE, UINT64_MAX);
                        vkResetFences(device, 1, &inFlightFences[currentFrame]);

                        uint32_t imageIndex;
                        VkResult result = vkAcquireNextImageKHR(device, swapChain, UINT64_MAX, imageAvailableSemaphores[currentFrame], VK_NULL_HANDLE, &imageIndex);

                        if (result == VK_ERROR_OUT_OF_DATE_KHR)
                        {
                            recreateSwapChain();
                            continue;
                        }
                        else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
                        {
                            throw std::runtime_error("vulkan acquire swap chain image");
                        }

                        // update uniform buffer
                        {
                            constexpr float BALLCAM_RADIUS = 8;
                            UniformBufferObject ubo{};
                            glm::vec3 ballPosition = glm::vec3(state.ball.objectState.position.x(), state.ball.objectState.position.y(), state.ball.objectState.position.z());
                            ubo.model[0] = glm::translate(glm::mat4(1.0f), glm::vec3(state.arena.objectState.position.x(), state.arena.objectState.position.y(), state.arena.objectState.position.z())) * glm::rotate(glm::mat4(1.0f), state.arena.objectState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                            ubo.model[1] = glm::translate(glm::mat4(1.0f), ballPosition) * glm::rotate(glm::mat4(1.0f), state.ball.objectState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                            for (int i = 0; i < state.players.size(); i++)
                            {
                                ubo.model[2 + i] = glm::translate(glm::mat4(1.0f), glm::vec3(state.players[i].carState.position.x(), state.players[i].carState.position.y(), state.players[i].carState.position.z())) * glm::rotate(glm::mat4(1.0f), state.players[i].carState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f));
                            }
                            glm::vec3 eye, center;
                            if (ballCam)
                            {
                                Eigen::Vector2f carPositionXZ = Eigen::Vector2f(state.players[state.playerId].carState.position.x(), state.players[state.playerId].carState.position.z());
                                Eigen::Vector2f ballPositionXZ = Eigen::Vector2f(state.ball.objectState.position.x(), state.ball.objectState.position.z());
                                Eigen::Vector2f eyeXZ = carPositionXZ - BALLCAM_RADIUS * (ballPositionXZ - carPositionXZ).normalized();
                                eye = glm::vec3(eyeXZ.x(), 2.0f, eyeXZ.y());
                                center = ballPosition;
                            }
                            else
                            {
                                auto carPosition = glm::vec3(state.players[state.playerId].carState.position.x(), state.players[state.playerId].carState.position.y(), state.players[state.playerId].carState.position.z());
                                eye = carPosition + glm::mat3(glm::rotate(glm::mat4(1.0f), state.players[state.playerId].carState.orientation.y(), glm::vec3(0.0f, 1.0f, 0.0f))) * glm::vec3(0.0f, 1.5f, -BALLCAM_RADIUS);
                                center = carPosition;
                            }
                            ubo.view = glm::lookAt(eye, center, glm::vec3(0.0f, 1.0, 0.0f));
                            ubo.proj = glm::perspective(glm::radians(45.0f), swapChainExtent.width / (float)swapChainExtent.height, 0.1f, 250.0f);
                            ubo.proj[1][1] *= -1;

                            memcpy(uniformBuffersMapped[currentFrame], &ubo, sizeof(ubo));
                        }

                        // hud
                        if (state.countdown != 0)
                        {
                            const uint64_t countdown = state.transitionCountdown > 0 ? state.transitionCountdown : state.countdown;
                            std::ostringstream oss;
                            oss << (uint32_t)(*state.scores)[0] << "  " << countdown / 60 << ":" << std::setw(2) << std::setfill('0') << countdown % 60 << "  " << (uint32_t)(*state.scores)[1];
                            const std::string newText = oss.str();
                            if (newText != text)
                            {
                                text = newText;

                                float textWidth = 0;
                                for (const char &c : text)
                                {
                                    textWidth += CHAR_INFO[c - FIRST_CHAR].xadvance;
                                }

                                constexpr float BOTTOM_PADDING = 20;
                                const float textWidthPadding = textWidth + 50;
                                float x = -2.f * textWidthPadding / 2 / swapChainExtent.width, y = -1, height = 2 * (BOTTOM_PADDING + FONT_ASCENT) / swapChainExtent.height, width = 2.f * textWidthPadding / swapChainExtent.width;
                                std::vector<HudVertex> vertices;
                                vertices.insert(vertices.begin(), {{{x, y + height}, {0, 0}},
                                                                   {{x, y}, {0, 0}},
                                                                   {{x + width, y}, {0, 0}},
                                                                   {{x, y + height}, {0, 0}},
                                                                   {{x + width, y}, {0, 0}},
                                                                   {{x + width, y + height}, {0, 0}}});

                                float cursor = 0;
                                for (const char &c : text)
                                {
                                    const size_t i = c - FIRST_CHAR;
                                    const auto &y0 = CHAR_INFO[i].y0;
                                    const auto &y1 = CHAR_INFO[i].y1;
                                    const auto &x0 = CHAR_INFO[i].x0;
                                    const auto &x1 = CHAR_INFO[i].x1;

                                    const float width = 2.f * (x1 - x0) / swapChainExtent.width;
                                    const float height = 2.f * (y1 - y0) / swapChainExtent.height;
                                    const float y = 2.f * (FONT_ASCENT + CHAR_INFO[i].yoff) / swapChainExtent.height - 1;
                                    const float x = 2.f * (cursor + CHAR_INFO[i].xoff - textWidth / 2) / swapChainExtent.width;

                                    const float y0Normalized = (float)y0 / ATLAS_HEIGHT;
                                    const float y1Normalized = (float)y1 / ATLAS_HEIGHT;
                                    const float x0Normalized = (float)x0 / ATLAS_WIDTH;
                                    const float x1Normalized = (float)x1 / ATLAS_WIDTH;

                                    vertices.insert(vertices.end(), {{{x, y + height}, {x0Normalized, y1Normalized}},
                                                                     {{x, y}, {x0Normalized, y0Normalized}},
                                                                     {{x + width, y}, {x1Normalized, y0Normalized}},
                                                                     {{x, y + height}, {x0Normalized, y1Normalized}},
                                                                     {{x + width, y}, {x1Normalized, y0Normalized}},
                                                                     {{x + width, y + height}, {x1Normalized, y1Normalized}}});
                                    cursor += CHAR_INFO[i].xadvance;
                                }

                                memcpy(hudMappedData, vertices.data(), (size_t)sizeof(vertices[0]) * vertices.size());
                            }
                        }
                        else
                            text = "";

                        vkResetCommandBuffer(commandBuffers[currentFrame], 0);
                        // record command buffer
                        {
                            VkCommandBuffer commandBuffer = commandBuffers[currentFrame];

                            VkCommandBufferBeginInfo beginInfo{};
                            beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

                            if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
                            {
                                throw std::runtime_error("vulkan begin recording command buffer");
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

                            VkDeviceSize offset = 0;
                            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vertexBuffer, &offset);

                            vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1, &descriptorSets[currentFrame], 0, nullptr);

                            vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT16);

                            uint index = 0;
                            vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);
                            vkCmdDrawIndexed(commandBuffer, indicesOffsets[0], 1, 0, 0, 0);

                            index = 1;
                            vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);
                            vkCmdDrawIndexed(commandBuffer, indicesOffsets[1] - indicesOffsets[0], 1, indicesOffsets[0], 0, 0);

                            for (int i = 0; i < state.players.size(); i++)
                            {
                                index = 2 + i;
                                vkCmdPushConstants(commandBuffer, pipelineLayout, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(index), &index);
                                vkCmdDrawIndexed(commandBuffer, indicesOffsets[2] - indicesOffsets[1], 1, indicesOffsets[1], 0, 0);
                            }

                            vkCmdNextSubpass(commandBuffer, VK_SUBPASS_CONTENTS_INLINE);

                            vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, hudPipeline);

                            offset = 0;
                            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &hudVertexBuffer, &offset);

                            vkCmdBindDescriptorSets(
                                commandBuffer,
                                VK_PIPELINE_BIND_POINT_GRAPHICS,
                                hudPipelineLayout,
                                0,
                                1,
                                &hudDescriptorSet,
                                0,
                                nullptr);

                            if (!text.empty())
                            {
                                for (int i = 0; i < text.size() + 1; i++)
                                {
                                    vkCmdDraw(commandBuffer, 6, 1, i * 6, 0);
                                }
                            }

                            vkCmdEndRenderPass(commandBuffer);

                            if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
                            {
                                throw std::runtime_error("vulkan record command buffer");
                            }
                        }

                        VkSubmitInfo submitInfo{};
                        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

                        VkPipelineStageFlags waitStage = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
                        submitInfo.waitSemaphoreCount = 1;
                        submitInfo.pWaitSemaphores = &imageAvailableSemaphores[currentFrame];
                        submitInfo.pWaitDstStageMask = &waitStage;

                        submitInfo.commandBufferCount = 1;
                        submitInfo.pCommandBuffers = &commandBuffers[currentFrame];

                        submitInfo.signalSemaphoreCount = 1;
                        submitInfo.pSignalSemaphores = &renderFinishedSemaphores[currentFrame];

                        if (vkQueueSubmit(graphicsQueue, 1, &submitInfo, inFlightFences[currentFrame]) != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan submit draw command buffer");
                        }

                        VkPresentInfoKHR presentInfo{};
                        presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
                        presentInfo.waitSemaphoreCount = 1;
                        presentInfo.pWaitSemaphores = &renderFinishedSemaphores[currentFrame];
                        presentInfo.swapchainCount = 1;
                        presentInfo.pSwapchains = &swapChain;
                        presentInfo.pImageIndices = &imageIndex;

                        result = vkQueuePresentKHR(presentQueue, &presentInfo);
                        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || framebufferResized)
                        {
                            framebufferResized = false;
                            recreateSwapChain();
                        }
                        else if (result != VK_SUCCESS)
                        {
                            throw std::runtime_error("vulkan present swap chain image");
                        }

                        currentFrame = (currentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
                    }
                }

                vkDeviceWaitIdle(device);
            }

            // cleanup
            {
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

                vkDestroySampler(device, fontSampler, nullptr);
                vkFreeMemory(device, fontImageMemory, nullptr);
                vkDestroyImage(device, fontImage, nullptr);
                vkDestroyImageView(device, fontImageView, nullptr);
                vkDestroyDescriptorPool(device, hudDescriptorPool, nullptr);
                vkDestroyDescriptorSetLayout(device, hudDescriptorSetLayout, nullptr);
                vkDestroyPipeline(device, hudPipeline, nullptr);
                vkDestroyPipelineLayout(device, hudPipelineLayout, nullptr);
                vkDestroyBuffer(device, hudVertexBuffer, nullptr);
                vkFreeMemory(device, hudVertexBufferMemory, nullptr);

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
        catch (const std::exception &e)
        {
            std::cerr << "error: " << e.what() << std::endl;
        }
        state.running = false;
    }
};

static size_t writeCallback(void *contents, size_t size, size_t nmemb, void *userp)
{
    ((std::string *)userp)->append((char *)contents, size * nmemb);
    return size * nmemb;
}

int main(int argc, char *argv[])
{
    try
    {
        std::cout << "Usage\nSingleplayer: " << argv[0] << "\nMultiplayer: " << argv[0] << " servers or " << argv[0] << " <IP Address> <Port>\n"
                  << std::endl;

        State state = {
            .arena = {.objectState = {.position = {0.0f, 10.0f, 0.0f},
                                      .velocity = {},
                                      .orientation = {}},
                      .size = arenaSize},
            .goal = goal,
            .ball = initialBall,
            .carSize = carSize,
            .players = {initialPlayers[0]},
            .countdown = 0,
            .transitionCountdown = 0,
            .input = {.action = {.throttle = 0.0f, .steering = 0.0f}, .ballCamPressed = false},
            .running = true,
        };
        std::optional<sockaddr_in6> serverAddress;

        if (argc == 2 && std::string(argv[1]) == "servers" || argc == 3)
        {

            std::string address, port;
            if (argc == 3)
            {
                address = argv[1];
                port = argv[2];
            }
            else if (argc == 2)
            {
                CURL *curl;
                curl_global_init(CURL_GLOBAL_DEFAULT);
                curl = curl_easy_init();
                if (curl)
                {
                    std::string response;
                    curl_easy_setopt(curl, CURLOPT_URL, "http://hacker-league.molyz.app:8080/servers");
                    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
                    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

                    CURLcode code = curl_easy_perform(curl);
                    if (code != CURLE_OK)
                    {
                        throw std::runtime_error("curl: " + std::string(curl_easy_strerror(code)));
                    }

                    nlohmann::json servers = nlohmann::json::parse(response);

                    std::cout << "Servers (Address, Port, Current Number of Players)" << std::endl;
                    for (int i = 0; i < servers.size(); i++)
                    {
                        std::cout << i + 1 << ") " << std::string(servers[i]["address"]) << ", " << std::string(servers[i]["port"]) << ", " << servers[i]["nPlayers"] << std::endl;
                    }
                    std::cout << "\nEnter the number of the server you want to choose: ";
                    int choice;
                    std::cin >> choice;

                    if (choice < 1 || choice > servers.size())
                    {
                        throw std::runtime_error("invalid server choice");
                    }

                    if (servers[choice - 1]["nPlayers"] > 1)
                    {
                        throw std::runtime_error("server is full. already 2 players on server.");
                    }

                    address = servers[choice - 1]["address"];
                    port = servers[choice - 1]["port"];

                    curl_easy_cleanup(curl);
                }
                curl_global_cleanup();
            }

            serverAddress = sockaddr_in6{};
            serverAddress->sin6_family = AF_INET6;
            if (inet_pton(AF_INET6, address.c_str(), &serverAddress->sin6_addr) < 0)
            {
                throw std::runtime_error("invalid ipv6 address or unsupported address family");
            }
            serverAddress->sin6_port = htons(std::stoi(port));
            state.players.push_back(initialPlayers[1]);
            state.scores = {};
        }
        else if (argc != 1)
        {
            throw std::runtime_error("wrong number of arguments");
        }
        std::vector<Player> initialPlayers = state.players;

        InputGraphics inputGraphics(state);

        std::thread inputGraphicsThread(&InputGraphics::run, &inputGraphics);
        std::thread physicsThread(&physics, std::ref(state), std::ref(initialPlayers), std::ref(serverAddress));
        inputGraphicsThread.join();
        physicsThread.join();
    }
    catch (const std::exception &e)
    {
        std::cerr << "error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
