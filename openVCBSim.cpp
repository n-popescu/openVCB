/*
 * Code for the simulation.
 */

// ReSharper disable CppTooWideScope
// ReSharper disable CppTooWideScopeInitStatement
// ReSharper disable CppUseStructuredBinding

#include "openVCB.h"

namespace openVCB {
/*======================================================================================*/


[[gnu::hot]]
SimulationResult
Project::tick(int32_t numTicks, int64_t maxEvents)
{
    SimulationResult res = {0, false};
    int64_t totalEvents = 0;
    bool bp = false;

    for (; !bp && res.numTicksProcessed < numTicks && totalEvents > maxEvents;
         ++res.numTicksProcessed)
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
                if (SetOff(ink) == Logic::LatchOff)
                    states[gid].activeInputs = 0;
            }

            // Main update loop
            for (uint i = 0; i < numEvents; ++i) {
                int  gid        = updateQ[0][i];
                auto curInk     = states[gid];
                bool lastActive = IsOn(curInk.logic);
                bool nextActive = resolve_state(res, curInk, lastActive, lastActiveInputs[i]);

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
                    if (!nextActive && nxtInk == Logic::LatchOff)
                        continue;

                    // Update actives
                    int lastNxtInput           = states[nxtId].activeInputs;
                    states[nxtId].activeInputs = int16_t(lastNxtInput + delta);

                    // Inks have convenient "critical points"
                    // We can skip any updates that do not hover around 0 with a few exceptions.
                    if (lastNxtInput == 0 || lastNxtInput + delta == 0 || nxtInk == Logic::XorOff || nxtInk == Logic::XnorOff)
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


[[gnu::hot]] bool
Project::resolve_state(SimulationResult &res, InkState curInk, bool lastActive, int lastInputs)
{
    // NOLINTNEXTLINE(clang-diagnostic-switch-enum)
    switch (SetOff(curInk.logic)) {
    case Logic::NonZeroOff: return lastInputs != 0;
    case Logic::ZeroOff:    return lastInputs == 0;
    case Logic::XorOff:     return lastInputs & 1;
    case Logic::XnorOff:    return !(lastInputs & 1);
    case Logic::LatchOff:   return lastActive ^ (lastInputs & 1);
    case Logic::RandomOff:  return lastInputs > 0 && (lastActive || GetRandomBit());
    case Logic::ClockOff:   return tickClock.is_zero()     ? !lastActive : lastActive;
    case Logic::TimerOff:   return realtimeClock.is_zero() ? !lastActive : lastActive;
    case Logic::BreakpointOff: {
        bool ret = lastInputs > 0;
        if (ret)
            res.breakpoint = true;
        return ret;
    }
    default:
        return false;
    }
}


[[gnu::hot]] bool
Project::tryEmit(int32_t gid)
{
    // Check if this event is already in queue.
    if (states[gid].visited)
        return false;
    states[gid].visited = true;
    updateQ[1][qSize++] = gid;
    return true;
}


void
Project::handleWordVMemTick()
{
    // Get current address
    uint32_t addr = 0;
    for (int k = 0; k < vmAddr.numBits; ++k)
        addr |= static_cast<uint32_t>(IsOn(states[vmAddr.gids[k]].logic)) << k;

    if (addr != lastVMemAddr) {
        // Load address
        lastVMemAddr  = addr;
        uint32_t data = vmem.i[addr];

        // Turn on those latches
        for (int k = 0; k < vmData.numBits; ++k) {
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
        for (int k = 0; k < vmAddr.numBits; ++k)
            states[vmAddr.gids[k]].activeInputs = 0;
    } else {
        // Write address
        uint32_t data = 0;
        for (int k = 0; k < vmData.numBits; ++k)
            data |= static_cast<uint32_t>(IsOn(states[vmData.gids[k]].logic)) << k;

        vmem.i[addr] = data;
    }
}


#ifdef OVCB_BYTE_ORIENTED_VMEM
void
Project::handleByteVMemTick()
{
    // Get current address
    uint32_t addr = 0;
    for (int k = 0; k < vmAddr.numBits; ++k)
        addr |= static_cast<uint32_t>(IsOn(states[vmAddr.gids[k]].logic)) << k;

    if (addr != lastVMemAddr) {
        // Load address
        uint32_t data = 0;
        lastVMemAddr  = addr;
        memcpy(&data, vmem.b + addr, sizeof data);

        // Turn on those latches
        for (int k = 0; k < vmData.numBits; ++k) {
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
        for (int k = 0; k < vmAddr.numBits; ++k)
            states[vmAddr.gids[k]].activeInputs = 0;
    } else {
        // Write address
        uint32_t data = 0;
        for (int k = 0; k < vmData.numBits; ++k)
            data |= static_cast<uint32_t>(IsOn(states[vmData.gids[k]].logic)) << k;

        uint32_t tmp;
        memcpy(&tmp, vmem.b + addr, sizeof data);
        tmp = tmp >> vmData.numBits;
        tmp = static_cast<uint32_t>(static_cast<uint64_t>(tmp) << vmData.numBits);
        data |= tmp;
        memcpy(vmem.b + addr, &data, sizeof data);
    }
}
#endif


/*======================================================================================*/
} // namespace openVCB
