/*
 * Code for the simulation.
 */

// ReSharper disable CppTooWideScope
// ReSharper disable CppTooWideScopeInitStatement
// ReSharper disable CppUseStructuredBinding

#include "openVCB.h"

#if defined __GNUC__
# define ASSUME(cond) __attribute__((__assume__(cond)))
#elif defined _MSC_VER
# define ASSUME(cond) __assume(cond)
#elif (defined __cplusplus && __cplusplus >= 202302L) || (defined _MSVC_LANG && _MSVC_LANG >= 202302L)
# define ASSUME(cond) [[assume(cond)]]
#else
# define ASSUME(cond) 
#endif

namespace openVCB {
/*======================================================================================*/


[[gnu::hot]]
SimulationResult Project::tick(int32_t numTicks, int64_t maxEvents)
{
    SimulationResult res = {0, false};
    int64_t totalEvents = 0;
    bool bp = false;

    for (; !bp && res.numTicksProcessed < numTicks && totalEvents <= maxEvents; ++res.numTicksProcessed)
    {
        for (auto const &[buffer, bufferSize, idx] : instrumentBuffers)
            buffer[tickNum % bufferSize] = states[idx];
        ++tickNum;
      
        // VMem implementation.
        if (vmem) {
#ifdef OVCB_BYTE_ORIENTED_VMEM
            if (vmemIsBytes)
                handleByteVMemTick();
            else
                handleWordVMemTick();
#else
            handleWordVMemTick();
#endif
        }

        if (tickClock.tick())
            for (auto gid : tickClock.GIDs)
                if (!states[gid].visited)
                    updateQ[0][qSize++] = gid;
        if (!realtimeClock.GIDs.empty() && realtimeClock.tick()) [[unlikely]]
            for (auto gid : realtimeClock.GIDs)
                if (!states[gid].visited)
                    updateQ[0][qSize++] = gid;

        for (int traceUpdate = 0; traceUpdate < 2; ++traceUpdate) {
            // We update twice per tick
            // Remember stuff
            uint numEvents = qSize;
            qSize          = 0;
            totalEvents += numEvents;

            // Copy over the current number of active inputs
            for (uint i = 0; i < numEvents; ++i) {
                int   gid = updateQ[0][i];
                Logic ink = states[gid].logic;

                // Reset visited flag
                states[gid].visited = false;

                // Copy over last active inputs
                lastActiveInputs[i] = states[gid].activeInputs;
                if (SetOff(ink) == Logic::Latch)
                    states[gid].activeInputs = 0;
            }

            // Main update loop
            for (uint i = 0; i < numEvents; ++i) {
                int  gid        = updateQ[0][i];
                auto curInk     = states[gid];
                bool lastActive = IsOn(curInk.logic);
                bool nextActive = resolve_state(res, curInk, lastActiveInputs[i], lastActive);

                // Short circuit if the state didn't change
                if (lastActive == nextActive)
                    continue;

                // Update the state
                states[gid].logic = SetOn(curInk.logic, nextActive);

                // Loop over neighbors
                int     delta = nextActive ? 1 : -1;
                int32_t end   = writeMap.ptr[gid + 1];

                for (int n = writeMap.ptr[gid]; n < end; ++n) {
                    auto  nxtId  = writeMap.rows[n];
                    Logic nxtInk = SetOff(states[nxtId].logic);

                    // Ignore falling edge for latches
                    if (!nextActive && nxtInk == Logic::Latch)
                        continue;

                    // Update actives
                    int lastNxtInput           = states[nxtId].activeInputs;
                    states[nxtId].activeInputs = int16_t(lastNxtInput + delta);

                    // Inks have convenient "critical points"
                    // We can skip any updates that do not hover around 0 with a few exceptions.
                    if (lastNxtInput == 0 || lastNxtInput + delta == 0 || nxtInk == Logic::Xor || nxtInk == Logic::Xnor)
                        tryEmit(nxtId);
                }
            }

            if (res.breakpoint) // Stop early
                bp = true;

            // Swap buffer
            std::swap(updateQ[0], updateQ[1]);
        }
    }

    return res;
}


[[gnu::hot]]
bool Project::resolve_state(SimulationResult &res, InkState curInk, int lastInputs, bool lastActive)
{
    // NOLINTNEXTLINE(clang-diagnostic-switch-enum)
    switch (SetOff(curInk.logic)) {
    case Logic::NonZero: return lastInputs;
    case Logic::Zero:    return !lastInputs;
    case Logic::Xor:     return lastInputs & 1;
    case Logic::Xnor:    return !(lastInputs & 1);
    case Logic::Latch:   return lastActive ^ (lastInputs & 1);
    case Logic::Random:  return lastInputs && (lastActive || GetRandomBit());
    case Logic::Clock:   return tickClock.is_zero()     ? !lastActive : lastActive;
    case Logic::Timer:   return realtimeClock.is_zero() ? !lastActive : lastActive;
    case Logic::Breakpoint: {
        bool ret = lastInputs > 0;
        if (ret)
            res.breakpoint = true;
        return ret;
    }
    default:
        return false;
    }
}


[[gnu::hot]]
bool Project::tryEmit(int32_t gid)
{
    // Check if this event is already in queue.
    if (states[gid].visited)
        return false;
    states[gid].visited = true;
    updateQ[1][qSize++] = gid;
    return true;
}


void Project::handleWordVMemTick()
{
    unsigned addrBits = static_cast<unsigned>(vmAddr.numBits);
    unsigned dataBits = static_cast<unsigned>(vmData.numBits);
    ASSUME(addrBits <= 32 && dataBits <= 32);
    if (addrBits > 32 || dataBits > 32)
        throw std::runtime_error("Disaster");

#if 0
    static constexpr uint32_t addrMasks[] = {
        0x00000001, 0x00000002, 0x00000004, 0x00000008,
        0x00000010, 0x00000020, 0x00000040, 0x00000080,
        0x00000100, 0x00000200, 0x00000400, 0x00000800,
        0x00001000, 0x00002000, 0x00004000, 0x00008000,
        0x00010000, 0x00020000, 0x00040000, 0x00080000,
        0x00100000, 0x00200000, 0x00400000, 0x00800000,
        0x01000000, 0x02000000, 0x04000000, 0x08000000,
        0x10000000, 0x20000000, 0x40000000, 0x80000000,
    };

    // Get current address
    uint32_t addr = 0;
    for (unsigned k = 0; k < addrBits; ++k)
        addr |= IsOn(states[vmAddr.gids[k]].logic) ? addrMasks[k] : 0;
#else
    // Get current address
    uint32_t addr = 0;
    for (unsigned k = 0; k < addrBits; ++k)
        addr |= static_cast<uint32_t>(IsOn(states[vmAddr.gids[k]].logic)) << k;
#endif

    if (addr != lastVMemAddr) {
        // Load address
        lastVMemAddr  = addr;
        uint32_t data = vmem.i[addr];

        // Turn on those latches
        for (unsigned k = 0; k < dataBits; ++k) {
            auto &state = states[vmData.gids[k]];
            if (((data >> k) & 1) != IsOn(state.logic)) {
                state.activeInputs = 1;
                if (state.visited)
                    continue;
                state.visited       = true;
                updateQ[0][qSize++] = vmData.gids[k];
            }
        }

        // Forcibly ignore further address updates
        for (unsigned k = 0; k < addrBits; ++k)
            states[vmAddr.gids[k]].activeInputs = 0;
    } else {
        // Write address
        uint32_t data = 0;
        for (unsigned k = 0; k < dataBits; ++k)
            data |= static_cast<uint32_t>(IsOn(states[vmData.gids[k]].logic)) << k;

        vmem.i[addr] = data;
    }
}


#ifdef OVCB_BYTE_ORIENTED_VMEM
void Project::handleByteVMemTick()
{
    unsigned addrBits = static_cast<unsigned>(vmAddr.numBits);
    unsigned dataBits = static_cast<unsigned>(vmData.numBits);
    ASSUME(addrBits <= 32 && dataBits <= 32);
    if (addrBits > 32 || dataBits > 32)
        throw std::runtime_error("Disaster");

    // Get current address
    uint32_t addr = 0;
    for (unsigned k = 0; k < addrBits; ++k)
        addr |= static_cast<uint32_t>(IsOn(states[vmAddr.gids[k]].logic)) << k;

    if (addr != lastVMemAddr) {
        // Load address
        uint32_t data = 0;
        lastVMemAddr  = addr;
        memcpy(&data, vmem.b + addr, sizeof data);

        // Turn on those latches
        for (unsigned k = 0; k < dataBits; ++k) {
            auto &state = states[vmData.gids[k]];
            if (((data >> k) & 1) != IsOn(state.logic)) {
                state.activeInputs = 1;
                if (state.visited)
                    continue;
                state.visited       = true;
                updateQ[0][qSize++] = vmData.gids[k];
            }
        }

        // Forcibly ignore further address updates
        for (unsigned k = 0; k < addrBits; ++k)
            states[vmAddr.gids[k]].activeInputs = 0;
    } else {
        // Write address
        uint32_t data = 0;
        for (unsigned k = 0; k < dataBits; ++k)
            data |= static_cast<uint32_t>(IsOn(states[vmData.gids[k]].logic)) << k;

        uint32_t tmp;
        memcpy(&tmp, vmem.b + addr, sizeof data);
        tmp = tmp >> dataBits;
        tmp = static_cast<uint32_t>(static_cast<uint64_t>(tmp) << dataBits);
        data |= tmp;
        memcpy(vmem.b + addr, &data, sizeof data);
    }
}
#endif


/*======================================================================================*/
} // namespace openVCB
