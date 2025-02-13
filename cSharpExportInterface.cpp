// ReSharper disable CppTooWideScopeInitStatement
#include "openVCB.h"

#if (defined __x86_64__ || defined __i386__) && 0
# include <tbb/spin_mutex.h>
using MyMutex = tbb::spin_mutex;
#else
using MyMutex = std::mutex;
#endif

#ifdef _WIN32
# define EXPORT_API extern "C" __declspec(dllexport)
#else
# define EXPORT_API extern "C"
#endif


EXPORT_API char const *const *openVCB_CompileAndRun(size_t *numErrors, int *stateSize);

EXPORT_API int32_t   openVCB_InitProject(void);
EXPORT_API int64_t   openVCB_GetLineNumber(int addr);
EXPORT_API size_t    openVCB_GetSymbol(char const *buf, int size);
EXPORT_API uint64_t  openVCB_GetNumTicks(void) noexcept;
EXPORT_API uintptr_t openVCB_GetVMemAddress(void) noexcept;
EXPORT_API float     openVCB_GetMaxTPS(void) noexcept;

EXPORT_API void openVCB_SetTickRate(float tps) noexcept;
EXPORT_API void openVCB_Tick(int tick);
EXPORT_API void openVCB_ToggleLatch(int x, int y);
EXPORT_API void openVCB_ToggleLatchIndex(int idx);
EXPORT_API void openVCB_AddBreakpoint(int gid);
EXPORT_API void openVCB_RemoveBreakpoint(int gid);
EXPORT_API bool openVCB_PollBreakpoint(void) noexcept;
EXPORT_API void openVCB_SetClockPeriod(uint32_t high, uint32_t low) noexcept;
EXPORT_API void openVCB_SetTimerPeriod(uint32_t period) noexcept;
EXPORT_API void openVCB_NewProject(int64_t seed, bool vmemIsBytes);
EXPORT_API void openVCB_InitVMem(char const *assembly, int aSize, char *err, int errSize);
EXPORT_API void openVCB_DeleteProject(void);
EXPORT_API void openVCB_AddInstrumentBuffer(openVCB::InkState *buf, int bufSize, int idx);
EXPORT_API void openVCB_SetStateMemory(int *data, int size);
EXPORT_API void openVCB_SetVMemMemory(void *data, int size);
EXPORT_API void openVCB_SetIndicesMemory(int *data, int size);
EXPORT_API void openVCB_SetImageMemory(void *data, int width, int height);
EXPORT_API void openVCB_SetDecoMemory(int *__restrict indices, int indLen, int const *__restrict col, int colLen);
EXPORT_API void openVCB_GetGroupStats(int *numGroups, int *numConnections) noexcept;
EXPORT_API void openVCB_SetInterface(openVCB::LatchInterface const *__restrict addr, openVCB::LatchInterface const *__restrict data);
EXPORT_API void openVCB_FreeErrorArray(char **arr, size_t numStrings) noexcept;

namespace util = openVCB::util;


/****************************************************************************************/
namespace {

using openVCB::Ink;

constexpr float TARGET_DT = 1.0f / 60.0f;
constexpr float MIN_DT    = 1.0f / 30.0f;

openVCB::Project *proj      = nullptr;
std::thread      *simThread = nullptr;
MyMutex           simLock;

float targetTPS     = 0.0f;
float maxTPS        = 0.0f;
bool  simRun        = true;
bool  simBreakpoint = false;


[[gnu::hot]]
void simFunc()
{
    using std::chrono::duration, std::chrono::milliseconds;
    using clock = std::chrono::high_resolution_clock;

    float tpsEst       = 2.0f / TARGET_DT;
    float desiredTicks = 0.0f;
    auto  lastTime     = clock::now();

    while (simRun) {
        auto curTime = clock::now();
        auto diff    = duration_cast<duration<float>>(curTime - lastTime).count();
        lastTime     = curTime;
        desiredTicks = glm::min(desiredTicks + diff * targetTPS, tpsEst * MIN_DT);

        // Find max tick amount we can do
        if (desiredTicks >= 1.0f) {
            float maxTickAmount = glm::max(tpsEst * TARGET_DT, 1.0f);
            float tickAmount    = glm::min(desiredTicks, maxTickAmount);

            // Aquire lock, simulate, and time
            simLock.lock();
            auto t1  = clock::now();
            auto res = proj->tick(static_cast<int32_t>(tickAmount));
            auto t2  = clock::now();
            simLock.unlock();

            // Use timings to estimate max possible tps
            desiredTicks = desiredTicks - static_cast<float>(res.numTicksProcessed);
            maxTPS       = static_cast<float>(res.numTicksProcessed) /
                           duration_cast<duration<float>>(t2 - t1).count();

            if (std::isfinite(maxTPS))
                tpsEst = glm::clamp(glm::mix(maxTPS, tpsEst, 0.95f), 1.0f, 1e8f);
            if (res.breakpoint) {
                targetTPS     = 0.0f;
                desiredTicks  = 0.0f;
                simBreakpoint = true;
            }
        }

        // Check how much time I got left and sleep until the next check
        curTime = clock::now();
        diff    = duration_cast<duration<float>>(curTime - lastTime).count();
        if (float ms = 1000.0f * (TARGET_DT - diff); ms > 1.05f)
            std::this_thread::sleep_for(milliseconds(static_cast<int>(ms)));
    }
}

} // namespace
/****************************************************************************************/


/*
 * Functions to control openVCB simulations
 */

EXPORT_API int64_t
openVCB_GetLineNumber(int addr)
{
    auto itr = proj->lineNumbers.find(addr);
    if (itr != proj->lineNumbers.end())
        return itr->second;
    return 0;
}

EXPORT_API size_t
openVCB_GetSymbol(char const *buf, int size)
{
    auto itr = proj->assemblySymbols.find({buf, static_cast<size_t>(size)});
    if (itr != proj->assemblySymbols.end())
        return itr->second;
    return 0;
}

EXPORT_API uint64_t
openVCB_GetNumTicks() noexcept
{
    return proj->tickNum;
}

EXPORT_API float
openVCB_GetMaxTPS() noexcept
{
    return static_cast<float>(maxTPS);
}

EXPORT_API uintptr_t
openVCB_GetVMemAddress() noexcept
{
#ifdef OVCB_BYTE_ORIENTED_VMEM 
    if (proj->vmemIsBytes)
        return proj->lastVMemAddr / 4U;
    else
        return proj->lastVMemAddr;
#else
    return proj->lastVMemAddr;
#endif
}

EXPORT_API void
openVCB_SetTickRate(float tps) noexcept
{
    targetTPS = static_cast<float>(glm::max(0.0f, tps));
}

EXPORT_API void
openVCB_Tick(int tick)
{
    if (!proj)
        return;
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->tick(tick);
    targetTPS = tps;
}

EXPORT_API void
openVCB_ToggleLatch(int x, int y)
{
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->toggleLatch(glm::ivec2(x, y));
    targetTPS = tps;
}

EXPORT_API void
openVCB_ToggleLatchIndex(int idx)
{
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->toggleLatch(idx);
    targetTPS = tps;
}

EXPORT_API void
openVCB_AddBreakpoint(int gid)
{
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->addBreakpoint(gid);
    targetTPS = tps;
}

EXPORT_API void
openVCB_RemoveBreakpoint(int gid)
{
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->removeBreakpoint(gid);
    targetTPS = tps;
}

EXPORT_API bool
openVCB_PollBreakpoint() noexcept
{
    return std::exchange(simBreakpoint, false);
}

EXPORT_API void
openVCB_SetClockPeriod(uint32_t high, uint32_t low) noexcept
{
    proj->tickClock.set_period(low, high);
}

EXPORT_API void
openVCB_SetTimerPeriod(uint32_t period) noexcept
{
    proj->realtimeClock.set_period(period);
}


/*--------------------------------------------------------------------------------------*/
/*
 * Functions to initialize openVCB
 */

EXPORT_API void
openVCB_NewProject(int64_t seed, bool vmemIsBytes)
{
    std::lock_guard lock(simLock);
    proj = new openVCB::Project(seed, vmemIsBytes);
}

EXPORT_API int
openVCB_InitProject()
{
    std::lock_guard lock(simLock);

    proj->preprocess();

    // Start sim thread paused
    simRun    = true;
    simThread = new std::thread(simFunc);

    return proj->numGroups;
}

EXPORT_API void
openVCB_InitVMem(char const *assembly, int aSize, char *err, int errSize)
{
    std::lock_guard lock(simLock);
    proj->assembly = std::string(assembly, aSize);
    proj->assembleVmem(err, errSize);
}

EXPORT_API void
openVCB_DeleteProject()
{
    simRun = false;
    std::lock_guard lock(simLock);

    if (simThread) {
        simThread->join();
        delete simThread;
        simThread = nullptr;
    }

    if (proj) {
        // These should be managed.
        proj->states = nullptr;
        proj->vmem   = nullptr;
        proj->image  = nullptr;

        delete proj;
        proj = nullptr;
    }
}

/*--------------------------------------------------------------------------------------*/
/*
 * Functions to replace openVCB buffers with managed ones
 */

EXPORT_API void
openVCB_AddInstrumentBuffer(openVCB::InkState *buf, int bufSize, int idx)
{
    std::lock_guard lock(simLock);

    float tps = targetTPS;
    targetTPS  = 0.0f;
    proj->instrumentBuffers.emplace_back(buf, bufSize, idx);
    targetTPS = tps;
}

EXPORT_API void
openVCB_SetStateMemory(int *data, int size)
{
    std::lock_guard lock(simLock);

    if (proj->states_is_native) {
        memcpy(data, proj->states, sizeof(*proj->states) * size);
        delete[] proj->states;
        proj->states_is_native = false;
        proj->states           = reinterpret_cast<openVCB::InkState *>(data);
    } else {
        throw std::runtime_error("Cannot replace states matrix twice!");
    }
}

EXPORT_API void
openVCB_SetVMemMemory(void *data, int size)
{
    std::lock_guard lock(simLock);
    proj->vmem     = static_cast<uint32_t *>(data);
    proj->vmemSize = size;
}

EXPORT_API void
openVCB_SetIndicesMemory(int *data, int size)
{
    std::lock_guard lock(simLock);
    memcpy(data, proj->indexImage, sizeof(*proj->indexImage) * size);
}

EXPORT_API void
openVCB_SetImageMemory(void *data, int width, int height)
{
    std::lock_guard lock(simLock);
    proj->width  = width;
    proj->height = height;
    proj->image  = static_cast<openVCB::InkPixel *>(data);
}

EXPORT_API void
openVCB_SetDecoMemory(int       *__restrict indices, UU int indLen,
                      int const *__restrict col,     UU int colLen)
{
    std::lock_guard lock(simLock);

    auto queue   = std::queue<glm::ivec3>{};
    auto visited = std::vector(size_t(proj->width) * size_t(proj->height), false);

    for (int32_t y = 0; y < proj->height; ++y) {
        for (int32_t x = 0; x < proj->width; ++x) {
            int32_t idx  = x + y * proj->width;
            indices[idx] = -1;

            if (static_cast<uint32_t>(col[idx]) != UINT32_MAX)
                continue;

            auto ink = SetOff(proj->image[idx].ink);
            if (util::eq_any(ink, Ink::LatchOff, Ink::LedOff)) {
                queue.emplace(x, y, proj->indexImage[idx]);
                visited[idx] = true;
            }
        }
    }

    while (!queue.empty()) {
        auto pos = queue.front();
        queue.pop();

        indices[pos.x + pos.y * proj->width] = pos.z;

        for (auto const &neighbor : openVCB::fourNeighbors) {
            glm::ivec2 np = static_cast<glm::ivec2>(pos) + neighbor;
            if (np.x < 0 || np.x >= proj->width || np.y < 0 || np.y >= proj->height)
                continue;

            int nidx = np.x + np.y * proj->width;
            if (visited[nidx])
                continue;

            visited[nidx] = true;

            if (static_cast<uint32_t>(col[nidx]) != UINT32_MAX)
                continue;
            queue.emplace(np.x, np.y, pos.z);
        }
    }
}

/*--------------------------------------------------------------------------------------*/
/*
 * Functions to configure openVCB
 */

EXPORT_API void
openVCB_GetGroupStats(int *numGroups, int *numConnections) noexcept
{
    //std::lock_guard lock(simLock);
    *numGroups      = proj->numGroups;
    *numConnections = proj->writeMap.nnz;
}

EXPORT_API void
openVCB_SetInterface(openVCB::LatchInterface const *__restrict addr,
             openVCB::LatchInterface const *__restrict data)
{
    std::lock_guard lock(simLock);
    proj->vmAddr = *addr;
    proj->vmData = *data;
}

/*--------------------------------------------------------------------------------------*/

EXPORT_API char const *const *
openVCB_CompileAndRun(size_t *numErrors, int *stateSize)
{
    std::lock_guard lock(simLock);

    // This processes the logic data to a form that can be simulated efficiently. It
    // assumes without checking that the project structure is correctly initialized
    // with all required data.
    proj->preprocess();

    if (!proj->error_messages->empty()) {
        *numErrors = proj->error_messages->size();
        *stateSize = 0;

        char const *const *errors    = proj->error_messages->data();
        proj->error_messages->data() = nullptr;
        proj->error_messages         = nullptr;

        delete proj;
        proj = nullptr;
        return errors;
    }
    *stateSize = proj->numGroups;

    delete proj->error_messages;
    proj->error_messages = nullptr;

    // Start sim thread paused
    simRun    = true;
    simThread = new std::thread(simFunc);
    return nullptr;
}

EXPORT_API void 
openVCB_FreeErrorArray(char **arr, size_t numStrings) noexcept
{
    if (arr) {
        for (size_t i = 0; i < numStrings; ++i)
            delete[] arr[i];
        delete[] arr;
    }
}
