/*
 * Copyright (c) MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "plic.h"

#include <algorithm>
#include <memory>
#include <vector>

#include <scc/register.h>
#include <scc/tlm_target.h>
#ifndef SC_SIGNAL_IF
#include <tlm/scc/signal_initiator_mixin.h>
#include <tlm/scc/signal_target_mixin.h>
#endif

#include "gen/plic_regs.h"

namespace vpvper {
namespace rvi {

plic::plic(sc_core::sc_module_name nm, uint32_t num_sources, uint32_t num_contexts)
    : sc_core::sc_module(nm),
      tlm_target<>(clk),
      interrupts_i("interrupts_i", num_sources),
      interrupts_o("interrupts_o", num_contexts),
      regs(std::make_unique<plic_regs>("regs", num_sources, num_contexts)),
      src_irq_cleared(num_sources, true)

{
    regs->registerResources(*this);
    // Register per-context claim/complete write callbacks
    for (uint32_t i = 0; i < num_contexts; ++i) {
        regs->claim_complete[i]->set_read_cb(
            [this, i](const scc::sc_register<uint32_t>& reg, uint32_t& v, sc_core::sc_time& d) -> bool {
                claim_complete_read_cb(i, v);
                return true;
            });
        regs->claim_complete[i]->set_write_cb(
            [this, i](scc::sc_register<uint32_t>& reg, const uint32_t& v, sc_core::sc_time& d) -> bool {
                claim_complete_write_cb(i, v);
                return true;
            });
    }

    // Port callbacks
#ifdef SC_SIGNAL_IF
    SC_METHOD(source_irq_cb);
    for (auto& irq : interrupts_i) sensitive << irq.pos();
    dont_initialize();
#else
    for (uint32_t i = 0; i < interrupts_i.size(); ++i) {
        interrupts_i[i].register_nb_transport(
            [this, i](tlm::scc::tlm_signal_gp<bool>& gp, tlm::tlm_phase& p, sc_core::sc_time& t) {
                this->source_irq_cb(i, gp.get_value());
                return tlm::TLM_COMPLETED;
            });
    }
#endif

    // register event callbacks
    SC_METHOD(clock_cb);
    sensitive << clk_i;
    SC_METHOD(reset_cb);
    sensitive << rst_i;
    dont_initialize();
}

plic::~plic() {}  // NOLINT

void plic::clock_cb() { this->clk = clk_i.read(); }

void plic::reset_cb() {
    if (rst_i.read())
        regs->reset_start();
    else
        regs->reset_stop();
}

#ifdef SC_SIGNAL_IF
void plic::source_irq_cb() {
    auto handle_pending = false;
    for (auto i = 0UL; i < interrupts_i.size(); ++i) {
        if (src_irq_cleared[i] && (interrupts_i[i].read() == 1)) {
            const auto src_id = get_source_id(i);
            const auto bit_ofs = src_id & 0x1F;
            get_pending(src_id) |= (0x1UL << bit_ofs);
            src_irq_cleared[i] = false;
            handle_pending = true;
            SCCDEBUG(this->name()) << "Setting interrupt " << src_id << " as pending";
        }
    }

    if (handle_pending) {
        for (auto i = 0UL; i < interrupts_o.size(); ++i) {
            handle_pending_irq(i);
        }
    }
}
#else
void plic::source_irq_cb(uint32_t irq_idx, uint32_t value) {
    if (!src_irq_cleared[irq_idx] || !value) {
        // If the source interrupt has not been cleared or the interrupt isn't high, do nothing
        return;
    }

    // Set the source's pending interrupt bit
    const auto src_id = get_source_id(irq_idx);
    const auto bit_ofs = src_id & 0x1F;
    get_pending(src_id) |= (0x1UL << bit_ofs);
    src_irq_cleared[irq_idx] = false;
    SCCDEBUG(this->name()) << "Setting interrupt " << src_id << " as pending";

    // Check if any contexts need interrupts
    for (auto i = 0UL; i < interrupts_o.size(); ++i) {
        handle_pending_irq(i);
    }
}
#endif

void plic::handle_pending_irq(uint32_t ctx) {
    if (get_source_irq(ctx, false, false)) {
        // If any pending interrupt passes the threshold test and an interrupt is not already active, trigger one
        SCCDEBUG(this->name()) << "Context " << ctx << " sending interrupt";
        write_irq(ctx, true);
    } else {
        // If there are no pending source interrupts that pass the threshold test, the interrupt can be deactivated
        write_irq(ctx, false);
        SCCDEBUG(this->name()) << "Context " << ctx << " has no further pending interrupts";
    }
}

void plic::claim_complete_read_cb(uint32_t ctx, uint32_t& src_id) {
    // Reading the claim/complete register returns the ID of the highest priority pending interrupt and clears that
    // source's interrupt pending bit. The claim operation is not affected by the threshold setting.
    src_id = get_source_irq(ctx, true, false);
    regs->r_claim_complete[ctx] = src_id;
    const auto bit_ofs = src_id & 0x1F;
    get_pending(src_id) &= ~(0x1u << bit_ofs);
    SCCTRACE(this->name()) << "Context " << ctx << " claimed interrupt " << src_id;

    // After clearing the highest priority interrupt's pending bit, handle any remaining pending interrupts.
    // This might cause the output interrupt to de-assert if there are no pending interrupts that are serviceable,
    // or the output interrupt might remain asserted if there are.
    handle_pending_irq(ctx);
}

void plic::claim_complete_write_cb(uint32_t ctx, uint32_t src_id) {
    if (src_id == 0) {
        // Interrupt 0 is unused/reserved
        SCCTRACE(this->name()) << "Context " << ctx << " ignoring claim completion for interrupt 0";
        return;
    }

    // If the source interrupt is disabled, ignore the claim completion
    const auto reg_idx = src_id >> 5;
    const auto bit_ofs = src_id & 0x1F;
    const bool enabled = (regs->r_enabled[ctx][reg_idx] >> bit_ofs) & 1u;
    if (!enabled) {
        SCCTRACE(this->name()) << "Context " << ctx << " ignoring claim completion for disabled source " << src_id;
        return;
    }

    // Writing the claim/complete register signals completion of the interrupt ID written.
    // The source interrupt can now be triggered again.
    SCCTRACE(this->name()) << "Context " << ctx << " claim complete for interrupt " << src_id;
    src_irq_cleared[src_id - 1] = true;  // Source cleared array does not include an entry for unused source 0
}

void plic::write_irq(uint32_t ctx, bool value) {
#ifdef SC_SIGNAL_IF
    interrupts_o[ctx].write(value);
#else
    sc_core::sc_time t;
    tlm::tlm_phase p{tlm::BEGIN_REQ};
    tlm::scc::tlm_signal_gp<bool> gp;
    gp.set_value(value);
    interrupts_o[ctx]->nb_transport_fw(gp, p, t);
#endif
}

uint32_t plic::get_source_irq(uint32_t ctx, bool find_top_priority, bool ignore_threshold) const {
    auto prio_irq = 0U;
    auto top_prio = 0U;
    const auto thold = regs->r_threshold[ctx];

    // Walk through each interrupt source and check for pending interrupts
    for (auto i = 0UL; i < interrupts_i.size(); ++i) {
        const auto src_id = get_source_id(i);
        const auto reg_idx = src_id >> 5;
        const auto bit_ofs = src_id & 0x1F;
        const bool pending = (get_pending(src_id) >> bit_ofs) & 1u;
        const bool enabled = (regs->r_enabled[ctx][reg_idx] >> bit_ofs) & 1u;
        const auto prio = regs->r_priority[src_id];

        if (pending && enabled && (ignore_threshold || (prio > thold))) {
            // Lowest source ID wins in case of equal priority
            if (prio > top_prio) {
                top_prio = prio;
                prio_irq = src_id;
                if (!find_top_priority) {
                    // If we don't need the top priority interrupt, we can stop here
                    SCCDEBUG(this->name()) << "Context " << ctx << " found pending interrupt " << src_id;
                    break;
                }
            }
        }
    }

    if (find_top_priority) {
        SCCDEBUG(this->name()) << "Context " << ctx << " top priority pending interrupt is " << prio_irq;
    }

    return prio_irq;
}

uint32_t& plic::get_pending(uint32_t src_id) const {
    const auto reg_idx = src_id >> 5;
    return regs->r_pending[reg_idx];
}

uint32_t plic::get_source_id(uint32_t src_idx) const {
    // Source interrupt 0 is not valid on the PLIC, so index 0 of the source interrupt vector corresponds to ID 1
    return src_idx + 1;
}

}  // namespace rvi
}  // namespace vpvper
