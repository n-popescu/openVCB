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

namespace util = openVCB::util;


/****************************************************************************************/
namespace {

using openVCB::Ink;

constexpr double TARGET_DT = 1.0 / 60.0;
constexpr double MIN_DT    = 1.0 / 30.0;

openVCB::Project *proj      = nullptr;
std::thread      *simThread = nullptr;
MyMutex           simLock;

double targetTPS     = 0.0;
double maxTPS        = 0.0;
bool  simRun        = true;
bool  simBreakpoint = false;


[[gnu::hot]]
void simFunc()
{
      using std::chrono::duration, std::chrono::milliseconds;
      using clock = std::chrono::high_resolution_clock;

      double tpsEst       = 2.0 / TARGET_DT;
      double desiredTicks = 0.0;
      auto   lastTime     = clock::now();

      while (simRun) {
            auto curTime = clock::now();
            auto diff    = duration_cast<duration<double>>(curTime - lastTime).count();
            lastTime     = curTime;
            desiredTicks = glm::min(desiredTicks + diff * targetTPS, tpsEst * MIN_DT);

            // Find max tick amount we can do
            if (desiredTicks >= 1.0) {
                  double maxTickAmount = glm::max(tpsEst * TARGET_DT, 1.0);
                  double tickAmount    = glm::min(desiredTicks, maxTickAmount);

                  // Aquire lock, simulate, and time
                  //std::unique_lock lock(simLock);
                  simLock.lock();
                  auto t1  = clock::now();
                  auto res = proj->tick(static_cast<int32_t>(tickAmount));
                  auto t2  = clock::now();
                  simLock.unlock();

                  // Use timings to estimate max possible tps
                  desiredTicks = desiredTicks - static_cast<double>(res.numTicksProcessed);
                  maxTPS = static_cast<double>(res.numTicksProcessed) /
                           duration_cast<duration<double>>(t2 - t1).count();

                  if (std::isfinite(maxTPS))
                        tpsEst = glm::clamp(glm::mix(maxTPS, tpsEst, 0.95), 1.0, 1e8);
                  if (res.breakpoint) {
                        targetTPS     = 0.0;
                        desiredTicks  = 0.0;
                        simBreakpoint = true;
                  }
            }

            // Check how much time I got left and sleep until the next check
            curTime = clock::now();
            diff    = duration_cast<duration<double>>(curTime - lastTime).count();
            if (double ms = 1000.0 * (TARGET_DT - diff); ms > 1.05)
                  std::this_thread::sleep_for(milliseconds(static_cast<int64_t>(ms)));
      }
}

} // namespace
/****************************************************************************************/


/*
 * Functions to control openVCB simulations
 */

EXPORT_API int64_t
getLineNumber(int const addr)
{
      auto const itr = proj->lineNumbers.find(addr);
      if (itr != proj->lineNumbers.end())
            return itr->second;
      return 0;
}

EXPORT_API size_t
getSymbol(char const *buf, int const size)
{
      auto const itr = proj->assemblySymbols.find({buf, static_cast<size_t>(size)});
      if (itr != proj->assemblySymbols.end())
            return itr->second;
      return 0;
}

EXPORT_API uint64_t
getNumTicks()
{
      return proj->tickNum;
}

EXPORT_API float
getMaxTPS()
{
      return (float)maxTPS;
}

EXPORT_API uintptr_t
getVMemAddress()
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
setTickRate(float const tps)
{
      targetTPS = (double)std::max(0.0f, tps);
}

EXPORT_API void
tick(int const tick)
{
      if (!proj)
            return;
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS       = 0;
      proj->tick(tick);
      targetTPS = tps;
}

EXPORT_API void
toggleLatch(int const x, int const y)
{
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS       = 0;
      proj->toggleLatch(glm::ivec2(x, y));
      targetTPS = tps;
}

EXPORT_API void
toggleLatchIndex(int const idx)
{
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS       = 0;
      proj->toggleLatch(idx);
      targetTPS = tps;
}

EXPORT_API void
addBreakpoint(int const gid)
{
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS       = 0;
      proj->addBreakpoint(gid);
      targetTPS = tps;
}

EXPORT_API void
removeBreakpoint(int const gid)
{
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS       = 0;
      proj->removeBreakpoint(gid);
      targetTPS = tps;
}

EXPORT_API int
pollBreakpoint()
{
      bool const res = simBreakpoint;
      simBreakpoint  = false;
      return res;
}

EXPORT_API void
openVCB_SetClockPeriod(uint const high, uint const low)
{
      //std::lock_guard lock(simLock);
      proj->tickClock.set_period(low, high);
}

EXPORT_API void
openVCB_SetTimerPeriod(uint32_t const period)
{
      //std::lock_guard lock(simLock);
      proj->realtimeClock.set_period(period);
}


/*--------------------------------------------------------------------------------------*/
/*
 * Functions to initialize openVCB
 */

EXPORT_API void
newProject(int64_t const seed, bool const vmemIsBytes)
{
      proj = new openVCB::Project(seed, vmemIsBytes);
}

EXPORT_API int
initProject()
{
      std::lock_guard lock(simLock);

      proj->preprocess();

      // Start sim thread paused
      simRun    = true;
      simThread = new std::thread(simFunc);

      return proj->numGroups;
}

EXPORT_API void
initVMem(char const *assembly, int const aSize, char *err, int const errSize)
{
      proj->assembly = std::string(assembly, aSize);
      proj->assembleVmem(err, errSize);
}

EXPORT_API void
deleteProject()
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
addInstrumentBuffer(openVCB::InkState *buf, int const bufSize, int const idx)
{
      std::lock_guard lock(simLock);

      double const tps = targetTPS;
      targetTPS        = 0;
      proj->instrumentBuffers.emplace_back(buf, bufSize, idx);
      targetTPS = tps;
}

EXPORT_API void
setStateMemory(int *data, int const size) noexcept(false)
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
setVMemMemory(intptr_t data, int const size)
{
      std::lock_guard lock(simLock);
      proj->vmem     = reinterpret_cast<int *>(data);
      proj->vmemSize = size;
}

EXPORT_API void
setIndicesMemory(int *data, int const size)
{
      std::lock_guard lock(simLock);
      memcpy(data, proj->indexImage, sizeof(*proj->indexImage) * size);

      {
          FILE *dump = nullptr;
          _wfopen_s(&dump, L"indexImage2.txt", L"w");
          if (dump) {
              for (int i = 0; i < proj->width * proj->height; ++i) {
                  if (proj->indexImage[i] >= 0) {
                      int         ink  = static_cast<int>(SetOff(proj->image[i].ink));
                      char const *name = ink > 0 && ink < std::size(openVCB::inkNames) ? openVCB::inkNames[static_cast<size_t>(ink)].data() : "UNKNOWN";
                      fwprintf(dump, L"%-7d : %7d, %3d, %hs\n",
                               i, proj->indexImage[i], proj->image[i].ink, name);
                  }
              }
          }
          fclose(dump);
      }
}

EXPORT_API void
setImageMemory(intptr_t data, int const width, int const height)
{
      std::lock_guard lock(simLock);
      proj->width  = width;
      proj->height = height;
      proj->image  = reinterpret_cast<openVCB::InkPixel *>(data);
}

EXPORT_API void
setDecoMemory(int       *__restrict indices, UU int indLen,
              int const *__restrict col,     UU int colLen)
{
      std::lock_guard lock(simLock);

      std::queue<glm::ivec3> queue;
      std::vector            visited(size_t(proj->width * proj->height), false);

      for (int32_t y = 0; y < proj->height; ++y) {
            for (int32_t x = 0; x < proj->width; ++x) {
                  int32_t const idx = x + y * proj->width;
                  indices[idx]      = -1;

                  if (uint32_t(col[idx]) != UINT32_MAX)
                        continue;

                  auto const ink = SetOff(proj->image[idx].ink);
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
                  glm::ivec2 const np = static_cast<glm::ivec2>(pos) + neighbor;
                  if (np.x < 0 || np.x >= proj->width || np.y < 0 || np.y >= proj->height)
                        continue;

                  int const nidx = np.x + np.y * proj->width;
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
getGroupStats(int *numGroups, int *numConnections)
{
      //std::lock_guard lock(simLock);
      *numGroups      = proj->numGroups;
      *numConnections = proj->writeMap.nnz;
}

EXPORT_API void
setInterface(openVCB::LatchInterface const *__restrict addr,
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

            char const *const *errors = proj->error_messages->data();
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
      simRun     = true;
      simThread  = new std::thread(simFunc);
      return nullptr;
}

EXPORT_API void 
openVCB_FreeErrorArray(char **arr, size_t numStrings)
{
      if (arr) {
            for (size_t i = 0; i < numStrings; ++i)
                  delete[] arr[i];
            delete[] arr;
      }
}
