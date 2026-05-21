/*
 * Copyright (c) MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RVI_PLIC_REGS_H_
#define _RVI_PLIC_REGS_H_

#include <stdint.h>

#include <algorithm>
#include <vector>

#include <scc/register.h>
#include <scc/tlm_target.h>
#include <scc/utilities.h>
#include <util/bit_field.h>

namespace vpvper {
namespace rvi {

class plic_regs : public sc_core::sc_module, public scc::resetable {
   public:
    using Uint32Vec = std::vector<uint32_t>;
    using Reg32 = scc::sc_register<uint32_t>;
    using Reg32Ptr = std::unique_ptr<Reg32>;
    using Reg32PtrVec = std::vector<Reg32Ptr>;

    // Storage declarations
    Uint32Vec r_priority;
    Uint32Vec r_pending;
    std::vector<Uint32Vec> r_enabled;
    Uint32Vec r_threshold;
    Uint32Vec r_claim_complete;

    // Register declarations
    Reg32PtrVec priority;
    Reg32PtrVec pending;
    std::vector<Reg32PtrVec> enabled;
    Reg32PtrVec threshold;
    Reg32PtrVec claim_complete;

    plic_regs(sc_core::sc_module_name nm, uint32_t num_sources, uint32_t num_contexts);

    template <unsigned BUSWIDTH = 32>
    void registerResources(scc::tlm_target<BUSWIDTH>& target);
};

//////////////////////////////////////////////////////////////////////////////
// member functions
//////////////////////////////////////////////////////////////////////////////

plic_regs::plic_regs(sc_core::sc_module_name nm, uint32_t num_sources, uint32_t num_contexts) : sc_core::sc_module(nm) {
    const uint32_t num_pending = (num_sources + 31) / 32;
    const uint32_t num_enabled = num_pending;

    // Create the storage for the register values
    // Source IDs are 1-based (0 is reserved), so r_priority must be indexed 0..num_sources.
    r_priority.resize(num_sources + 1, 0);
    r_pending.resize(num_pending, 0);
    r_enabled.resize(num_contexts);
    for (uint32_t ctx = 0; ctx < num_contexts; ++ctx) {
        r_enabled[ctx].resize(num_enabled, 0);
    }
    r_threshold.resize(num_contexts, 0);
    r_claim_complete.resize(num_contexts, 0);

    // Create the registers
    // Source 0 is reserved but its priority register slot must exist at address 0
    // so that source ID s maps to address s*4 and r_priority[s] is always in bounds.
    for (uint32_t src = 0UL; src <= num_sources; ++src) {
        // Priority registers are read/write
        const auto nm_str = "priority_" + std::to_string(src);
        priority.emplace_back(std::make_unique<Reg32>(nm_str.c_str(), r_priority[src], 0, *this));
    }
    for (uint32_t pnd = 0UL; pnd < num_pending; ++pnd) {
        // Pending registers are read-only
        const auto nm_str = "pending_" + std::to_string(pnd);
        pending.emplace_back(std::make_unique<Reg32>(nm_str.c_str(), r_pending[pnd], 0, *this, UINT32_MAX, 0));
    }
    enabled.resize(num_contexts);
    for (uint32_t ctx = 0; ctx < num_contexts; ++ctx) {
        // Enable registers are read/write
        for (uint32_t en = 0; en < num_enabled; ++en) {
            const auto nm_str = "enabled_" + std::to_string(ctx) + "_" + std::to_string(en);
            enabled[ctx].emplace_back(std::make_unique<Reg32>(nm_str.c_str(), r_enabled[ctx][en], 0, *this));
        }
        // Threshold registers are read/write
        auto nm_str = "threshold_" + std::to_string(ctx);
        threshold.emplace_back(std::make_unique<Reg32>(nm_str.c_str(), r_threshold[ctx], 0, *this));
        // Claim/complete registers are read/write
        nm_str = "claim_complete_" + std::to_string(ctx);
        claim_complete.emplace_back(std::make_unique<Reg32>(nm_str.c_str(), r_claim_complete[ctx], 0, *this));
    }
}

template <unsigned BUSWIDTH>
inline void plic_regs::registerResources(scc::tlm_target<BUSWIDTH>& target) {
    static constexpr uint32_t priority_base = 0x0UL;
    for (uint32_t i = 0; i < priority.size(); ++i) {
        target.addResource(*priority[i], priority_base + (sizeof(uint32_t) * i));
    }
    static constexpr uint32_t pending_base = 0x1000UL;
    for (uint32_t i = 0; i < pending.size(); ++i) {
        target.addResource(*pending[i], pending_base + (sizeof(uint32_t) * i));
    }
    static constexpr uint32_t enable_base = 0x2000UL;
    static constexpr uint32_t enable_stride = 0x80UL;
    static constexpr uint32_t threshold_base = 0x200000UL;
    static constexpr uint32_t threshold_stride = 0x1000UL;
    static constexpr uint32_t claim_complete_base = 0x200004UL;
    static constexpr uint32_t claim_complete_stride = threshold_stride;
    for (uint32_t i = 0; i < enabled.size(); ++i) {
        for (uint32_t j = 0; j < enabled[i].size(); ++j) {
            target.addResource(*enabled[i][j], enable_base + (enable_stride * i) + (sizeof(uint32_t) * j));
        }
        target.addResource(*threshold[i], threshold_base + (threshold_stride * i));
        target.addResource(*claim_complete[i], claim_complete_base + (claim_complete_stride * i));
    }
}

}  // namespace rvi
}  // namespace vpvper

#endif  // _RVI_PLIC_REGS_H_
