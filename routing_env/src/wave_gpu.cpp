// In-process GPU wavefront with dlopen'd Vulkan (no link dependency).
// See wave_gpu.hpp. Kernel: tools/wave_step_map.comp, embedded at build time
// as wave_spv.h (ROUTING_HAVE_WAVE_GPU is defined only when that succeeded).
#include "routing/wave_gpu.hpp"

#include <atomic>

namespace routing {

namespace {
std::atomic<bool> g_gpu_enabled{true};
}
void set_wave_gpu_enabled(bool on) { g_gpu_enabled.store(on); }
bool wave_gpu_enabled() { return g_gpu_enabled.load(); }

} // namespace routing

#if !defined(ROUTING_HAVE_WAVE_GPU)

namespace routing {
bool wave_gpu_available() { return false; }
bool wave_gpu_map(int, int, int, const uint32_t*, const uint32_t*,
                  const uint32_t*, uint16_t*) { return false; }
bool gpu_cost_field(int, int, int, const uint32_t*, const uint8_t*, uint32_t,
                    const int64_t*, std::size_t, uint64_t*,
                    const uint32_t*) { return false; }
bool gpu_cost_field_warm(int, int, int, uint32_t, const int64_t*, std::size_t,
                         uint64_t*) { return false; }
bool gpu_field_labels(int, int, int, uint32_t, const int64_t*, const int32_t*,
                      std::size_t, uint32_t*, uint32_t*) { return false; }
bool gpu_bake_cost(int, int, int, int32_t, uint32_t, uint32_t, const int32_t*,
                   const int32_t*, const int32_t*, const int32_t*,
                   const uint32_t*, const uint32_t*, uint32_t*) { return false; }
bool gpu_bake_prealloc(int, int, int) { return false; }
bool gpu_boundary_pairs(int, int, int, uint32_t,
                        std::vector<std::array<uint32_t, 5>>&) { return false; }
bool gpu_walk_edges(int, int, int,
                    const std::vector<std::pair<uint32_t, uint32_t>>&,
                    std::vector<uint32_t>&, std::vector<uint32_t>&) {
    return false;
}

bool gpu_field_available() { return false; }
} // namespace routing

#else

#include <vulkan/vulkan.h>
#include <dlfcn.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <stdexcept>
#include <vector>

#include "wave_spv.h"   // generated: unsigned char g_wave_spv[]; unsigned g_wave_spv_len;
#include "field_spv.h"  // generated: unsigned char g_field_spv[]; unsigned g_field_spv_len;
#include "bake_spv.h"   // generated: unsigned char g_bake_spv[]; unsigned g_bake_spv_len;

namespace routing {
namespace {

// ---- dynamically loaded entry points --------------------------------------
#define VK_FNS(X)                                                              \
    X(vkCreateInstance) X(vkEnumeratePhysicalDevices)                          \
    X(vkGetPhysicalDeviceProperties) X(vkGetPhysicalDeviceQueueFamilyProperties)\
    X(vkCreateDevice) X(vkGetDeviceQueue) X(vkGetPhysicalDeviceMemoryProperties)\
    X(vkGetPhysicalDeviceFeatures)                                             \
    X(vkCreateBuffer) X(vkGetBufferMemoryRequirements) X(vkAllocateMemory)     \
    X(vkBindBufferMemory) X(vkMapMemory) X(vkCreateShaderModule)               \
    X(vkCreateDescriptorSetLayout) X(vkCreatePipelineLayout)                   \
    X(vkCreateComputePipelines) X(vkCreateDescriptorPool)                      \
    X(vkAllocateDescriptorSets) X(vkUpdateDescriptorSets)                      \
    X(vkCreateCommandPool) X(vkAllocateCommandBuffers) X(vkCreateFence)        \
    X(vkBeginCommandBuffer) X(vkCmdFillBuffer) X(vkCmdPipelineBarrier)         \
    X(vkCmdBindPipeline) X(vkCmdPushConstants) X(vkCmdBindDescriptorSets)      \
    X(vkCmdDispatch) X(vkCmdCopyBuffer) X(vkEndCommandBuffer) X(vkQueueSubmit) \
    X(vkWaitForFences) X(vkResetFences) X(vkDestroyBuffer) X(vkFreeMemory)

#define DECLARE(fn) PFN_##fn p_##fn = nullptr;
VK_FNS(DECLARE)
#undef DECLARE

struct VkErr : std::runtime_error {
    explicit VkErr(int r) : std::runtime_error("vk error " + std::to_string(r)) {}
};
inline void ck(VkResult r) { if (r != VK_SUCCESS) throw VkErr((int)r); }

struct Gpu {
    bool ok = false;
    VkInstance inst{};
    VkPhysicalDevice phys{};
    VkDevice dev{};
    VkQueue queue{};
    uint32_t qfi = 0;
    VkPhysicalDeviceMemoryProperties mp{};
    VkDescriptorSetLayout dsl{};
    VkPipelineLayout pl{};
    VkPipeline pipe{};
    VkDescriptorPool dpool{};
    VkDescriptorSet ds[2]{};
    VkCommandPool cpool{};
    VkCommandBuffer cb{};
    VkFence fence{};
    // field router (exact weighted Dijkstra field; requires shaderInt64)
    bool has_int64 = false;
    VkDescriptorSetLayout dslF{};
    VkPipelineLayout plF{};
    VkPipeline pipeF{};
    VkDescriptorSet dsF{};
    VkDescriptorSetLayout dslB{};
    VkPipelineLayout plB{};
    VkPipeline pipeB{};
    VkDescriptorSet dsB{};
    struct Buf { VkBuffer b{}; VkDeviceMemory m{}; void* map = nullptr; size_t sz = 0; };
    Buf ent, via, cur, nxt, vis, flag, wmap, stage, stageFlag;
    Buf fCost, fVia, fDist, fPredO, fPredJ, fLab;
    Buf bOwn, bClz, bPin, bPad, bCng, bSlf, fLine, fAux;
    std::mutex mtx;

    uint32_t mem_type(uint32_t bits, VkMemoryPropertyFlags want) {
        for (uint32_t i = 0; i < mp.memoryTypeCount; ++i)
            if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & want) == want)
                return i;
        throw VkErr(-99);
    }
    void free_buf(Buf& b) {
        if (b.b) p_vkDestroyBuffer(dev, b.b, nullptr);
        if (b.m) p_vkFreeMemory(dev, b.m, nullptr);
        b = Buf{};
    }
    void ensure_buf(Buf& b, size_t sz, bool host) {
        if (b.sz >= sz) return;
        free_buf(b);
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = sz;
        bci.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                    VK_BUFFER_USAGE_TRANSFER_DST_BIT;
        ck(p_vkCreateBuffer(dev, &bci, nullptr, &b.b));
        VkMemoryRequirements mr;
        p_vkGetBufferMemoryRequirements(dev, b.b, &mr);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize = mr.size;
        mai.memoryTypeIndex = mem_type(
            mr.memoryTypeBits,
            host ? (VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                    VK_MEMORY_PROPERTY_HOST_COHERENT_BIT)
                 : VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        ck(p_vkAllocateMemory(dev, &mai, nullptr, &b.m));
        ck(p_vkBindBufferMemory(dev, b.b, b.m, 0));
        if (host) ck(p_vkMapMemory(dev, b.m, 0, sz, 0, &b.map));
        b.sz = sz;
    }

    bool init() {
        void* lib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
        if (!lib) lib = dlopen("libvulkan.so", RTLD_NOW | RTLD_LOCAL);
        if (!lib) return false;
#define LOAD(fn)                                                               \
        p_##fn = (PFN_##fn)dlsym(lib, #fn);                                    \
        if (!p_##fn) return false;
        VK_FNS(LOAD)
#undef LOAD
        try {
            VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO};
            ai.pApplicationName = "routing_wave_gpu";
            ai.apiVersion = VK_API_VERSION_1_0;
            VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
            ici.pApplicationInfo = &ai;
            ck(p_vkCreateInstance(&ici, nullptr, &inst));
            uint32_t nd = 0;
            ck(p_vkEnumeratePhysicalDevices(inst, &nd, nullptr));
            if (!nd) return false;
            std::vector<VkPhysicalDevice> devs(nd);
            ck(p_vkEnumeratePhysicalDevices(inst, &nd, devs.data()));
            phys = devs[0];
            for (auto d : devs) {
                VkPhysicalDeviceProperties p;
                p_vkGetPhysicalDeviceProperties(d, &p);
                if (p.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
                    phys = d;
                    break;
                }
            }
            uint32_t nqf = 0;
            p_vkGetPhysicalDeviceQueueFamilyProperties(phys, &nqf, nullptr);
            std::vector<VkQueueFamilyProperties> qf(nqf);
            p_vkGetPhysicalDeviceQueueFamilyProperties(phys, &nqf, qf.data());
            qfi = ~0u;
            for (uint32_t i = 0; i < nqf; ++i)
                if (qf[i].queueFlags & VK_QUEUE_COMPUTE_BIT) { qfi = i; break; }
            if (qfi == ~0u) return false;
            float prio = 1.0f;
            VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
            qci.queueFamilyIndex = qfi;
            qci.queueCount = 1;
            qci.pQueuePriorities = &prio;
            VkPhysicalDeviceFeatures sup{};
            p_vkGetPhysicalDeviceFeatures(phys, &sup);
            VkPhysicalDeviceFeatures want{};
            want.shaderInt64 = sup.shaderInt64;
            has_int64 = sup.shaderInt64 == VK_TRUE;
            VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
            dci.queueCreateInfoCount = 1;
            dci.pQueueCreateInfos = &qci;
            dci.pEnabledFeatures = &want;
            ck(p_vkCreateDevice(phys, &dci, nullptr, &dev));
            p_vkGetDeviceQueue(dev, qfi, 0, &queue);
            p_vkGetPhysicalDeviceMemoryProperties(phys, &mp);

            VkShaderModuleCreateInfo smci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
            smci.codeSize = g_wave_spv_len;
            smci.pCode = (const uint32_t*)g_wave_spv;
            VkShaderModule sm;
            ck(p_vkCreateShaderModule(dev, &smci, nullptr, &sm));
            VkDescriptorSetLayoutBinding binds[7]{};
            for (uint32_t i = 0; i < 7; ++i)
                binds[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1,
                            VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
            VkDescriptorSetLayoutCreateInfo dslci{
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
            dslci.bindingCount = 7;
            dslci.pBindings = binds;
            ck(p_vkCreateDescriptorSetLayout(dev, &dslci, nullptr, &dsl));
            VkPushConstantRange pcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, 20};
            VkPipelineLayoutCreateInfo plci{
                VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
            plci.setLayoutCount = 1;
            plci.pSetLayouts = &dsl;
            plci.pushConstantRangeCount = 1;
            plci.pPushConstantRanges = &pcr;
            ck(p_vkCreatePipelineLayout(dev, &plci, nullptr, &pl));
            VkComputePipelineCreateInfo cpi{
                VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
            cpi.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                         nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, sm, "main",
                         nullptr};
            cpi.layout = pl;
            ck(p_vkCreateComputePipelines(dev, VK_NULL_HANDLE, 1, &cpi, nullptr,
                                          &pipe));
            if (has_int64) {   // field-router pipeline (4 bindings, 24B push)
                VkShaderModuleCreateInfo fsm{
                    VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
                fsm.codeSize = g_field_spv_len;
                fsm.pCode = (const uint32_t*)g_field_spv;
                VkShaderModule smF;
                ck(p_vkCreateShaderModule(dev, &fsm, nullptr, &smF));
                VkDescriptorSetLayoutBinding fb[9]{};
                for (uint32_t i = 0; i < 9; ++i)
                    fb[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1,
                             VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                VkDescriptorSetLayoutCreateInfo fdsl{
                    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
                fdsl.bindingCount = 9;
                fdsl.pBindings = fb;
                ck(p_vkCreateDescriptorSetLayout(dev, &fdsl, nullptr, &dslF));
                VkPushConstantRange fpcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, 24};
                VkPipelineLayoutCreateInfo fplci{
                    VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
                fplci.setLayoutCount = 1;
                fplci.pSetLayouts = &dslF;
                fplci.pushConstantRangeCount = 1;
                fplci.pPushConstantRanges = &fpcr;
                ck(p_vkCreatePipelineLayout(dev, &fplci, nullptr, &plF));
                VkComputePipelineCreateInfo fcpi{
                    VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
                fcpi.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                              nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, smF,
                              "main", nullptr};
                fcpi.layout = plF;
                ck(p_vkCreateComputePipelines(dev, VK_NULL_HANDLE, 1, &fcpi,
                                              nullptr, &pipeF));
                VkShaderModuleCreateInfo bsm{
                    VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
                bsm.codeSize = g_bake_spv_len;
                bsm.pCode = (const uint32_t*)g_bake_spv;
                VkShaderModule smB;
                ck(p_vkCreateShaderModule(dev, &bsm, nullptr, &smB));
                VkDescriptorSetLayoutBinding bb[7]{};
                for (uint32_t i = 0; i < 7; ++i)
                    bb[i] = {i, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1,
                             VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                VkDescriptorSetLayoutCreateInfo bdsl{
                    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO};
                bdsl.bindingCount = 7;
                bdsl.pBindings = bb;
                ck(p_vkCreateDescriptorSetLayout(dev, &bdsl, nullptr, &dslB));
                VkPushConstantRange bpcr{VK_SHADER_STAGE_COMPUTE_BIT, 0, 24};
                VkPipelineLayoutCreateInfo bplci{
                    VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
                bplci.setLayoutCount = 1;
                bplci.pSetLayouts = &dslB;
                bplci.pushConstantRangeCount = 1;
                bplci.pPushConstantRanges = &bpcr;
                ck(p_vkCreatePipelineLayout(dev, &bplci, nullptr, &plB));
                VkComputePipelineCreateInfo bcpi{
                    VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO};
                bcpi.stage = {VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO,
                              nullptr, 0, VK_SHADER_STAGE_COMPUTE_BIT, smB,
                              "main", nullptr};
                bcpi.layout = plB;
                ck(p_vkCreateComputePipelines(dev, VK_NULL_HANDLE, 1, &bcpi,
                                              nullptr, &pipeB));
            }
            VkDescriptorPoolSize dps{VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 32};
            VkDescriptorPoolCreateInfo dpci{
                VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO};
            dpci.maxSets = 4;
            dpci.poolSizeCount = 1;
            dpci.pPoolSizes = &dps;
            ck(p_vkCreateDescriptorPool(dev, &dpci, nullptr, &dpool));
            VkDescriptorSetLayout ls[2] = {dsl, dsl};
            VkDescriptorSetAllocateInfo dsai{
                VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
            dsai.descriptorPool = dpool;
            dsai.descriptorSetCount = 2;
            dsai.pSetLayouts = ls;
            ck(p_vkAllocateDescriptorSets(dev, &dsai, ds));
            if (has_int64) {
                VkDescriptorSetAllocateInfo fai{
                    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                fai.descriptorPool = dpool;
                fai.descriptorSetCount = 1;
                fai.pSetLayouts = &dslF;
                ck(p_vkAllocateDescriptorSets(dev, &fai, &dsF));
                VkDescriptorSetAllocateInfo bai{
                    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO};
                bai.descriptorPool = dpool;
                bai.descriptorSetCount = 1;
                bai.pSetLayouts = &dslB;
                ck(p_vkAllocateDescriptorSets(dev, &bai, &dsB));
            }
            VkCommandPoolCreateInfo cpci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
            cpci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
            cpci.queueFamilyIndex = qfi;
            ck(p_vkCreateCommandPool(dev, &cpci, nullptr, &cpool));
            VkCommandBufferAllocateInfo cbai{
                VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
            cbai.commandPool = cpool;
            cbai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
            cbai.commandBufferCount = 1;
            ck(p_vkAllocateCommandBuffers(dev, &cbai, &cb));
            VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
            ck(p_vkCreateFence(dev, &fci, nullptr, &fence));
            ok = true;
            return true;
        } catch (...) {
            return false;
        }
    }
};

Gpu& gpu() {
    static Gpu g;
    static std::once_flag once;
    std::call_once(once, [] { g.init(); });   // NOT gpu(): reentrancy deadlocks
    return g;
}

} // namespace

bool wave_gpu_available() { return gpu().ok; }

bool wave_gpu_map(int W, int H, int L, const uint32_t* ent,
                  const uint32_t* via, const uint32_t* seeds,
                  uint16_t* map_out) {
    Gpu& G = gpu();
    if (!G.ok || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const int S = (W + 31) >> 5;
        const size_t plane = (size_t)H * S, total = (size_t)L * plane;
        const size_t bytes = total * 4;
        const size_t cells = (size_t)L * H * W;
        G.ensure_buf(G.ent, bytes, false);
        G.ensure_buf(G.via, plane * 4, false);
        G.ensure_buf(G.cur, bytes, false);
        G.ensure_buf(G.nxt, bytes, false);
        G.ensure_buf(G.vis, bytes, false);
        G.ensure_buf(G.flag, 4, false);
        G.ensure_buf(G.wmap, cells * 4, false);
        G.ensure_buf(G.stage, std::max(bytes, cells * 4), true);
        G.ensure_buf(G.stageFlag, 4, true);
        // (re)bind descriptor sets to current buffers (cheap, safe every call)
        auto write_set = [&](VkDescriptorSet s, VkBuffer fin, VkBuffer fout) {
            VkBuffer bufs[7] = {G.ent.b, G.via.b, fin, fout,
                                G.vis.b, G.flag.b, G.wmap.b};
            VkDescriptorBufferInfo dbi[7];
            VkWriteDescriptorSet wds[7]{};
            for (uint32_t i = 0; i < 7; ++i) {
                dbi[i] = {bufs[i], 0, VK_WHOLE_SIZE};
                wds[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr, s, i,
                          0, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, nullptr,
                          &dbi[i], nullptr};
            }
            p_vkUpdateDescriptorSets(G.dev, 7, wds, 0, nullptr);
        };
        write_set(G.ds[0], G.cur.b, G.nxt.b);
        write_set(G.ds[1], G.nxt.b, G.cur.b);

        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        auto upload = [&](Gpu::Buf& dst, const void* src, size_t sz) {
            std::memcpy(G.stage.map, src, sz);
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, sz};
            p_vkCmdCopyBuffer(G.cb, G.stage.b, dst.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        };
        upload(G.ent, ent, bytes);
        upload(G.via, via, plane * 4);
        upload(G.cur, seeds, bytes);
        upload(G.vis, seeds, bytes);
        {   // zero nxt + map + flag, all device-side
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.nxt.b, 0, G.nxt.sz, 0);
            p_vkCmdFillBuffer(G.cb, G.wmap.b, 0, G.wmap.sz, 0);
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }

        const uint32_t groups = (uint32_t)((total + 255) / 256);
        const int K = 128;
        int wave = 1;                      // seeds are wave 1; first ring = 2
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_SHADER_WRITE_BIT,
                             VK_ACCESS_TRANSFER_READ_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        int guard = 0;
        while (true) {
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipe);
            for (int k = 0; k < K; ++k) {
                const int32_t pcv[5] = {W, H, L, S, ++wave};
                p_vkCmdPushConstants(G.cb, G.pl, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                     20, pcv);
                p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                          G.pl, 0, 1, &G.ds[k & 1], 0, nullptr);
                p_vkCmdDispatch(G.cb, groups, 1, 1);
                p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                       1, &mb, 0, nullptr, 0, nullptr);
            }
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy fc{0, 0, 4};
            p_vkCmdCopyBuffer(G.cb, G.flag.b, G.stageFlag.b, 1, &fc);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            if (*(uint32_t*)G.stageFlag.map == 0) break;
            if (++guard > 2000) return false;      // >256k waves: give up
        }
        {   // read back the map
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, cells * 4};
            p_vkCmdCopyBuffer(G.cb, G.wmap.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        const uint32_t* m32 = (const uint32_t*)G.stage.map;
        for (size_t i = 0; i < cells; ++i)
            map_out[i] = (uint16_t)(m32[i] > 0xFFFFu ? 0xFFFFu : m32[i]);
        // seeds are wave 1 (kernel writes >= 2)
        for (size_t w = 0; w < total; ++w) {
            uint32_t bits = seeds[w];
            if (!bits) continue;
            const size_t l = w / plane;
            const size_t rem = w % plane;
            const size_t y = rem / (size_t)S, wx = rem % (size_t)S;
            const size_t cbase = ((l * H) + y) * (size_t)W + (wx << 5);
            while (bits) {
                const int b = __builtin_ctz(bits);
                bits &= bits - 1;
                map_out[cbase + b] = 1;
            }
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_boundary_pairs(int W, int H, int L, uint32_t via_q,
                        std::vector<std::array<uint32_t, 5>>& out) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        constexpr uint32_t NSLOT = 131072;
        if (G.fAux.sz < (size_t)(NSLOT * 6 + 2) * 4) return false;
        G.ensure_buf(G.stage, (size_t)(NSLOT * 6 + 2) * 4, true);
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        {   // init table: key = 0, mins = 0xFFFFFFFF
            uint32_t* st = (uint32_t*)G.stage.map;
            for (uint32_t i = 0; i < NSLOT; ++i)
                for (int f = 0; f < 6; ++f)
                    st[i * 6 + f] = (f == 0 || f == 5) ? 0u : 0xFFFFFFFFu;
            st[NSLOT * 6] = 0;       // path counter
            st[NSLOT * 6 + 1] = 0;   // overflow flag
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, (VkDeviceSize)(NSLOT * 6 + 2) * 4};
            p_vkCmdCopyBuffer(G.cb, G.stage.b, G.fAux.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_SHADER_WRITE_BIT,
                             VK_ACCESS_TRANSFER_READ_BIT};
        {   // 4 reduction passes
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            for (int pass = 0; pass < 4; ++pass) {
                const int32_t pcv[6] = {W, H, L, 5, pass, (int32_t)via_q};
                p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT,
                                     0, 24, pcv);
                p_vkCmdDispatch(G.cb, (uint32_t)((cells + 255) / 256), 1, 1);
                p_vkCmdPipelineBarrier(G.cb,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                       1, &mb, 0, nullptr, 0, nullptr);
            }
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy c{0, 0, (VkDeviceSize)(NSLOT * 6 + 2) * 4};
            p_vkCmdCopyBuffer(G.cb, G.fAux.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        const uint32_t* t = (const uint32_t*)G.stage.map;
        if (t[NSLOT * 6 + 1] != 0) return false;   // table overflow: CPU path
        out.clear();
        for (uint32_t i = 0; i < NSLOT; ++i) {
            if (t[i * 6] == 0) continue;
            out.push_back({t[i * 6] - 1u, t[i * 6 + 1], t[i * 6 + 2],
                           t[i * 6 + 3], t[i * 6 + 4]});
        }
        return true;
    } catch (const std::exception& e) {
        if (getenv("ROUTING_FIELD_DEBUG"))
            fprintf(stderr, "[gpu] boundary exception: %s\n", e.what());
        return false;
    }
}

bool gpu_walk_edges(int W, int H, int L,
                    const std::vector<std::pair<uint32_t, uint32_t>>& edges,
                    std::vector<uint32_t>& path_cells,
                    std::vector<uint32_t>& edge_off) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        constexpr uint32_t NSLOT = 131072;
        const size_t maxpath = (G.fAux.sz / 4) - (NSLOT * 6 + 2);
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        {   // clear the in-tree mask (predJ reuse) + path counter
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.fPredJ.b, 0, (VkDeviceSize)cells * 4, 0);
            p_vkCmdFillBuffer(G.cb, G.fAux.b, (VkDeviceSize)NSLOT * 6 * 4, 8,
                              0);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        edge_off.assign(1, 0);
        for (auto& e : edges) {
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, 6, (int32_t)e.first,
                                    (int32_t)e.second};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, 1, 1, 1);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT |
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   0, 1, &mb, 0, nullptr, 0, nullptr);
            VkBufferCopy c{(VkDeviceSize)NSLOT * 6 * 4, 0, 4};
            p_vkCmdCopyBuffer(G.cb, G.fAux.b, G.stageFlag.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            edge_off.push_back(*(uint32_t*)G.stageFlag.map);
            if (edge_off.back() > maxpath) return false;   // overflow
        }
        const uint32_t total = edge_off.back();
        path_cells.resize(total);
        if (total) {
            G.ensure_buf(G.stage, (size_t)total * 4, true);
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{(VkDeviceSize)(NSLOT * 6 + 2) * 4, 0,
                           (VkDeviceSize)total * 4};
            p_vkCmdCopyBuffer(G.cb, G.fAux.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            std::memcpy(path_cells.data(), G.stage.map, (size_t)total * 4);
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_field_available() {
    Gpu& G = gpu();
    return G.ok && G.has_int64;
}

bool gpu_field_labels(int W, int H, int L, uint32_t via_q,
                      const int64_t* seeds, const int32_t* seed_labels,
                      std::size_t n_seeds, uint32_t* labels_out,
                      uint32_t* pred_out) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        if (G.fDist.sz < cells * 8 || G.fPredO.sz < cells * 4) return false;
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        {   // labels = UNSET everywhere, then per-seed label stamps
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.fLab.b, 0, G.fLab.sz, 0xFFFFFFFFu);
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            uint32_t* st = (uint32_t*)G.stage.map;
            std::vector<VkBufferCopy> zc(n_seeds);
            for (size_t s = 0; s < n_seeds; ++s) {
                st[s] = (uint32_t)seed_labels[s];
                zc[s] = {(VkDeviceSize)(s * 4), (VkDeviceSize)seeds[s] * 4, 4};
            }
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            if (!zc.empty())
                p_vkCmdCopyBuffer(G.cb, G.stage.b, G.fLab.b,
                                  (uint32_t)zc.size(), zc.data());
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        auto groups = [](size_t n) { return (uint32_t)((n + 255) / 256); };
        const uint32_t gCells = groups(cells);
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_SHADER_WRITE_BIT,
                             VK_ACCESS_TRANSFER_READ_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        {   // predecessor pass (writes predO and predJ)
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, 2, 0, (int32_t)via_q};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, gCells, 1, 1);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        int guard = 0;
        while (true) {          // pointer-jumping label propagation (log depth)
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            for (int it = 0; it < 4; ++it) {
                const int32_t pcv[6] = {W, H, L, 3, 0, (int32_t)via_q};
                p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT,
                                     0, 24, pcv);
                p_vkCmdDispatch(G.cb, gCells, 1, 1);
                p_vkCmdPipelineBarrier(G.cb,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                       1, &mb, 0, nullptr, 0, nullptr);
            }
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy fc{0, 0, 4};
            p_vkCmdCopyBuffer(G.cb, G.flag.b, G.stageFlag.b, 1, &fc);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            if (*(uint32_t*)G.stageFlag.map == 0) break;
            if (++guard > 256) return false;
        }
        if (labels_out && pred_out) {   // nullptr = leave resident
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, cells * 4};
            p_vkCmdCopyBuffer(G.cb, G.fLab.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            std::memcpy(labels_out, G.stage.map, cells * 4);
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdCopyBuffer(G.cb, G.fPredO.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            std::memcpy(pred_out, G.stage.map, cells * 4);
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_bake_prealloc(int W, int H, int L) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        const size_t plane = (size_t)H * W;
        G.ensure_buf(G.bOwn, cells * 4, false);
        G.ensure_buf(G.bClz, cells * 4, false);
        G.ensure_buf(G.bPin, cells * 4, false);
        G.ensure_buf(G.bPad, cells * 4, false);
        G.ensure_buf(G.bCng, plane * 4, false);
        G.ensure_buf(G.bSlf, plane * 4, false);
        G.ensure_buf(G.fCost, cells * 4, false);
        G.ensure_buf(G.stage, cells * 8, true);
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_bake_cost(int W, int H, int L, int32_t tok, uint32_t wq,
                   uint32_t base_q20, const int32_t* own, const int32_t* clz,
                   const int32_t* pin, const int32_t* pad,
                   const uint32_t* cong, const uint32_t* selfq,
                   uint32_t* cost_out) {
    // Bakes the windowed cost grid ON DEVICE into fCost (the field pass can
    // then run with cost == nullptr to reuse it) and reads a copy back for
    // the CPU boundary/corner checks.
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        const size_t plane = (size_t)H * W;
        G.ensure_buf(G.bOwn, cells * 4, false);
        G.ensure_buf(G.bClz, cells * 4, false);
        G.ensure_buf(G.bPin, cells * 4, false);
        G.ensure_buf(G.bPad, cells * 4, false);
        G.ensure_buf(G.bCng, plane * 4, false);
        G.ensure_buf(G.bSlf, plane * 4, false);
        G.ensure_buf(G.fCost, cells * 4, false);
        G.ensure_buf(G.stage, cells * 8, true);
        {
            VkBuffer bufs[7] = {G.bOwn.b, G.bClz.b, G.bPin.b, G.bPad.b,
                                G.bCng.b, G.bSlf.b, G.fCost.b};
            VkDescriptorBufferInfo dbi[7];
            VkWriteDescriptorSet wds[7]{};
            for (uint32_t i = 0; i < 7; ++i) {
                dbi[i] = {bufs[i], 0, VK_WHOLE_SIZE};
                wds[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
                          G.dsB, i, 0, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                          nullptr, &dbi[i], nullptr};
            }
            p_vkUpdateDescriptorSets(G.dev, 7, wds, 0, nullptr);
        }
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        auto upload = [&](Gpu::Buf& dst, const void* src, size_t sz) {
            std::memcpy(G.stage.map, src, sz);
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, sz};
            p_vkCmdCopyBuffer(G.cb, G.stage.b, dst.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        };
        upload(G.bOwn, own, cells * 4);
        upload(G.bClz, clz, cells * 4);
        upload(G.bPin, pin, cells * 4);
        upload(G.bPad, pad, cells * 4);
        upload(G.bCng, cong, plane * 4);
        upload(G.bSlf, selfq, plane * 4);
        {
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                                 VK_ACCESS_TRANSFER_WRITE_BIT,
                                 VK_ACCESS_SHADER_READ_BIT};
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeB);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plB, 0, 1, &G.dsB, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, tok, (int32_t)wq,
                                    (int32_t)base_q20};
            p_vkCmdPushConstants(G.cb, G.plB, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, (uint32_t)((cells + 255) / 256), 1, 1);
            VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                                 VK_ACCESS_SHADER_WRITE_BIT,
                                 VK_ACCESS_TRANSFER_READ_BIT};
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy c{0, 0, cells * 4};
            p_vkCmdCopyBuffer(G.cb, G.fCost.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        std::memcpy(cost_out, G.stage.map, cells * 4);
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_cost_field(int W, int H, int L, const uint32_t* cost,
                    const uint8_t* via_ok, uint32_t via_q,
                    const int64_t* seeds, std::size_t n_seeds,
                    uint64_t* dist_out, const uint32_t* corridor_bits) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        G.ensure_buf(G.fCost, cells * 4, false);
        G.ensure_buf(G.fVia, cells * 4, false);
        G.ensure_buf(G.fDist, cells * 8, false);
        G.ensure_buf(G.flag, 4, false);
        G.ensure_buf(G.stage, cells * 8, true);
        G.ensure_buf(G.stageFlag, 4, true);
        G.ensure_buf(G.fPredO, cells * 4, false);
        G.ensure_buf(G.fPredJ, cells * 4, false);
        G.ensure_buf(G.fLab, cells * 4, false);
        const size_t LPH = (size_t)L * (H + W + 2 * (W + H - 1));
        G.ensure_buf(G.fLine, 2 * LPH * 4, false);
        G.ensure_buf(G.fAux, (size_t)(131072 * 6 + 2 + 4 * 1000 * 1000) * 4,
                     false);
        {   // bind field descriptor set (all 9 bindings)
            VkBuffer bufs[9] = {G.fCost.b, G.fVia.b, G.fDist.b, G.flag.b,
                                G.fPredO.b, G.fPredJ.b, G.fLab.b, G.fLine.b,
                                G.fAux.b};
            VkDescriptorBufferInfo dbi[9];
            VkWriteDescriptorSet wds[9]{};
            for (uint32_t i = 0; i < 9; ++i) {
                dbi[i] = {bufs[i], 0, VK_WHOLE_SIZE};
                wds[i] = {VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET, nullptr,
                          G.dsF, i, 0, 1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                          nullptr, &dbi[i], nullptr};
            }
            p_vkUpdateDescriptorSets(G.dev, 9, wds, 0, nullptr);
        }
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        auto upload = [&](Gpu::Buf& dst, const void* src, size_t sz) {
            std::memcpy(G.stage.map, src, sz);
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, sz};
            p_vkCmdCopyBuffer(G.cb, G.stage.b, dst.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        };
        if (cost) upload(G.fCost, cost, cells * 4);  // nullptr = resident (gpu_bake_cost)
        if (corridor_bits) {   // mask -> aux path region, then mode-7 apply
            constexpr uint32_t NSLOT = 131072;
            const size_t nwords = (cells + 31) / 32;
            if (G.fAux.sz < (size_t)(NSLOT * 6 + 2 + nwords) * 4) return false;
            std::memcpy(G.stage.map, corridor_bits, nwords * 4);
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, (VkDeviceSize)(NSLOT * 6 + 2) * 4, nwords * 4};
            p_vkCmdCopyBuffer(G.cb, G.stage.b, G.fAux.b, 1, &c);
            VkMemoryBarrier xw{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                               VK_ACCESS_TRANSFER_WRITE_BIT,
                               VK_ACCESS_SHADER_READ_BIT |
                                   VK_ACCESS_SHADER_WRITE_BIT};
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &xw, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, 7, 0, 0};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, (uint32_t)((cells + 255) / 256), 1, 1);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        {   // via mask u8 -> u32
            std::vector<uint32_t> v32(cells);
            for (size_t i = 0; i < cells; ++i) v32[i] = via_ok[i];
            upload(G.fVia, v32.data(), cells * 4);
        }
        {   // dist = INF everywhere, then 0 at seeds
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.fDist.b, 0, G.fDist.sz, 0xFFFFFFFFu);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            std::memset(G.stage.map, 0, 8);
            std::vector<VkBufferCopy> zc(n_seeds);
            for (size_t s = 0; s < n_seeds; ++s)
                zc[s] = {0, (VkDeviceSize)seeds[s] * 8, 8};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            if (!zc.empty())
                p_vkCmdCopyBuffer(G.cb, G.stage.b, G.fDist.b,
                                  (uint32_t)zc.size(), zc.data());
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdFillBuffer(G.cb, G.fLine.b, 0, 2 * LPH * 4, 0);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }

        auto groups = [](size_t n) { return (uint32_t)((n + 255) / 256); };
        const uint32_t gCells = groups(cells);
        const uint32_t gLines[8] = {
            groups((size_t)L * H), groups((size_t)L * H),
            groups((size_t)L * W), groups((size_t)L * W),
            groups((size_t)L * (W + H - 1)), groups((size_t)L * (W + H - 1)),
            groups((size_t)L * (W + H - 1)), groups((size_t)L * (W + H - 1))};
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_SHADER_WRITE_BIT,
                             VK_ACCESS_TRANSFER_READ_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        {   // mark seed lines into half 0 (mode 4, parity bit set to 0)
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, 4, 0, (int32_t)via_q};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, (uint32_t)((cells + 255) / 256), 1, 1);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        const int B = 4;                       // iterations per submit
        int guard = 0;
        int itg = 0;                           // global iteration (parity)
        while (true) {
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            for (int it = 0; it < B; ++it, ++itg) {
                const int32_t par = itg & 1;
                // clear the OUT half for this iteration's marks
                p_vkCmdPipelineBarrier(G.cb,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1,
                                       &to_x, 0, nullptr, 0, nullptr);
                p_vkCmdFillBuffer(G.cb, G.fLine.b,
                                  (VkDeviceSize)(1 - par) * LPH * 4, LPH * 4,
                                  0);
                p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                       1, &x_to, 0, nullptr, 0, nullptr);
                for (int dir = 0; dir < 8; ++dir) {
                    const int32_t pcv[6] = {W, H, L, 0, dir | (par << 8),
                                            (int32_t)via_q};
                    p_vkCmdPushConstants(G.cb, G.plF,
                                         VK_SHADER_STAGE_COMPUTE_BIT, 0, 24,
                                         pcv);
                    p_vkCmdDispatch(G.cb, gLines[dir], 1, 1);
                    p_vkCmdPipelineBarrier(
                        G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                        VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &mb, 0,
                        nullptr, 0, nullptr);
                }
                const int32_t pcv[6] = {W, H, L, 1, par << 8, (int32_t)via_q};
                p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT,
                                     0, 24, pcv);
                p_vkCmdDispatch(G.cb, gCells, 1, 1);
                p_vkCmdPipelineBarrier(G.cb,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0,
                                       1, &mb, 0, nullptr, 0, nullptr);
            }
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy fc{0, 0, 4};
            p_vkCmdCopyBuffer(G.cb, G.flag.b, G.stageFlag.b, 1, &fc);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            if (*(uint32_t*)G.stageFlag.map == 0) break;
            if (++guard > 4096) return false;
        }
        if (dist_out) {   // nullptr = leave resident (on-device tree path)
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, cells * 8};
            p_vkCmdCopyBuffer(G.cb, G.fDist.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            std::memcpy(dist_out, G.stage.map, cells * 8);
        }
        return true;
    } catch (...) {
        return false;
    }
}

bool gpu_cost_field_warm(int W, int H, int L, uint32_t via_q,
                         const int64_t* seeds, std::size_t n_seeds,
                         uint64_t* dist_out) {
    Gpu& G = gpu();
    if (!G.ok || !G.has_int64 || !wave_gpu_enabled()) return false;
    std::lock_guard<std::mutex> lk(G.mtx);
    try {
        const size_t cells = (size_t)L * H * W;
        if (G.fDist.sz < cells * 8 || G.fCost.sz < cells * 4) return false;
        auto submit = [&]() {
            VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
            si.commandBufferCount = 1;
            si.pCommandBuffers = &G.cb;
            ck(p_vkQueueSubmit(G.queue, 1, &si, G.fence));
            ck(p_vkWaitForFences(G.dev, 1, &G.fence, VK_TRUE, ~0ULL));
            ck(p_vkResetFences(G.dev, 1, &G.fence));
        };
        const size_t LPH = (size_t)L * (H + W + 2 * (W + H - 1));
        if (G.fLine.sz < 2 * LPH * 4) return false;
        {   // stamp new seeds only + reset line flags + mark seed lines
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            std::memset(G.stage.map, 0, 8);
            std::vector<VkBufferCopy> zc(n_seeds);
            for (size_t s = 0; s < n_seeds; ++s)
                zc[s] = {0, (VkDeviceSize)seeds[s] * 8, 8};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            if (!zc.empty())
                p_vkCmdCopyBuffer(G.cb, G.stage.b, G.fDist.b,
                                  (uint32_t)zc.size(), zc.data());
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdFillBuffer(G.cb, G.fLine.b, 0, 2 * LPH * 4, 0);
            VkMemoryBarrier xw{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                               VK_ACCESS_TRANSFER_WRITE_BIT,
                               VK_ACCESS_SHADER_READ_BIT |
                                   VK_ACCESS_SHADER_WRITE_BIT};
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &xw, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            const int32_t pcv[6] = {W, H, L, 4, 0, (int32_t)via_q};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, (uint32_t)((cells + 255) / 256), 1, 1);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        auto groups = [](size_t n) { return (uint32_t)((n + 255) / 256); };
        const uint32_t gCells = groups(cells);
        const uint32_t gLines[8] = {
            groups((size_t)L * H), groups((size_t)L * H),
            groups((size_t)L * W), groups((size_t)L * W),
            groups((size_t)L * (W + H - 1)), groups((size_t)L * (W + H - 1)),
            groups((size_t)L * (W + H - 1)), groups((size_t)L * (W + H - 1))};
        VkMemoryBarrier mb{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                           VK_ACCESS_SHADER_WRITE_BIT,
                           VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        VkMemoryBarrier to_x{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_SHADER_WRITE_BIT,
                             VK_ACCESS_TRANSFER_READ_BIT};
        VkMemoryBarrier x_to{VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr,
                             VK_ACCESS_TRANSFER_WRITE_BIT,
                             VK_ACCESS_SHADER_READ_BIT | VK_ACCESS_SHADER_WRITE_BIT};
        int guard = 0;
        int itg = 0;
        while (true) {
            const int32_t par = itg & 1;
            ++itg;
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            p_vkCmdFillBuffer(G.cb, G.flag.b, 0, 4, 0);
            p_vkCmdFillBuffer(G.cb, G.fLine.b,
                              (VkDeviceSize)(1 - par) * LPH * 4, LPH * 4, 0);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_TRANSFER_BIT,
                                   VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1,
                                   &x_to, 0, nullptr, 0, nullptr);
            p_vkCmdBindPipeline(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE, G.pipeF);
            p_vkCmdBindDescriptorSets(G.cb, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      G.plF, 0, 1, &G.dsF, 0, nullptr);
            for (int dir = 0; dir < 8; ++dir) {
                const int32_t pcv[6] = {W, H, L, 0, dir | (par << 8),
                                        (int32_t)via_q};
                p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT,
                                     0, 24, pcv);
                p_vkCmdDispatch(G.cb, gLines[dir], 1, 1);
                p_vkCmdPipelineBarrier(G.cb,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                       0, 1, &mb, 0, nullptr, 0, nullptr);
            }
            const int32_t pcv[6] = {W, H, L, 1, par << 8, (int32_t)via_q};
            p_vkCmdPushConstants(G.cb, G.plF, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                 24, pcv);
            p_vkCmdDispatch(G.cb, gCells, 1, 1);
            p_vkCmdPipelineBarrier(G.cb, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 1, &to_x,
                                   0, nullptr, 0, nullptr);
            VkBufferCopy fc{0, 0, 4};
            p_vkCmdCopyBuffer(G.cb, G.flag.b, G.stageFlag.b, 1, &fc);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
            if (*(uint32_t*)G.stageFlag.map == 0) break;
            if (++guard > 4096) return false;
        }
        {   // read back
            VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
            ck(p_vkBeginCommandBuffer(G.cb, &bi));
            VkBufferCopy c{0, 0, cells * 8};
            p_vkCmdCopyBuffer(G.cb, G.fDist.b, G.stage.b, 1, &c);
            ck(p_vkEndCommandBuffer(G.cb));
            submit();
        }
        std::memcpy(dist_out, G.stage.map, cells * 8);
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace routing
#endif
