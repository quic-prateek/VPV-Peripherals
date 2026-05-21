/*
 * Copyright (c) MINRES Technologies GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _RVI_PLIC_H_
#define _RVI_PLIC_H_

#include <memory>
#include <vector>

#include <scc/tlm_target.h>
#ifndef SC_SIGNAL_IF
#include <tlm/scc/signal_initiator_mixin.h>
#include <tlm/scc/signal_target_mixin.h>
#endif

namespace vpvper {
namespace rvi {

class plic_regs;

/**
 * @brief RISC-V Platform-Level Interrupt Controller (PLIC) SystemC/TLM model.
 *
 * Implements the RISC-V PLIC specification as a synthesizable SystemC module.
 * Accepts up to num_sources interrupt inputs and drives one interrupt
 * output per context (hart). Interrupt arbitration follows priority ordering
 * with per-context threshold filtering; the lowest source ID wins ties.
 *
 * The standard claim/complete handshake is modeled via register callbacks:
 * reading the claim/complete register returns and atomically clears the
 * highest-priority pending interrupt; writing it signals completion and
 * allows the source to fire again.
 */
class plic : public sc_core::sc_module, public scc::tlm_target<> {
   public:
    SC_HAS_PROCESS(plic);  // NOLINT

    sc_core::sc_in<sc_core::sc_time> clk_i{"clk_i"};
    sc_core::sc_in<bool> rst_i{"rst_i"};

#ifdef SC_SIGNAL_IF
    sc_core::sc_vector<sc_core::sc_in<bool>> interrupts_i;
    sc_core::sc_vector<sc_core::sc_out<bool>> interrupts_o;
#else
    sc_core::sc_vector<tlm::scc::tlm_signal_bool_opt_in> interrupts_i;
    sc_core::sc_vector<tlm::scc::tlm_signal_bool_out> interrupts_o;
#endif

    plic(sc_core::sc_module_name nm, uint32_t num_sources, uint32_t num_contexts);
    ~plic() override;

   protected:
    /**
     * @brief SC_METHOD sensitive to clk_i; captures the current clock period.
     *
     * Stores the value read from clk_i into the internal @ref clk member so
     * that the TLM target base class always has an up-to-date clock reference.
     */
    void clock_cb();

    /**
     * @brief SC_METHOD sensitive to rst_i; drives the register block reset.
     *
     * Calls regs->reset_start() while rst_i is asserted and regs->reset_stop()
     * once it is de-asserted.
     */
    void reset_cb();

#ifdef SC_SIGNAL_IF
    /**
     * @brief SC_METHOD triggered on any rising edge of interrupts_i (sc_signal build).
     *
     * Walks all interrupt sources, sets the pending bit for every source that
     * has transitioned high and has been cleared, then calls handle_pending_irq()
     * for every context.
     */
    void source_irq_cb();
#else
    /**
     * @brief TLM nb_transport callback invoked when a source interrupt changes (TLM build).
     *
     * Sets the pending bit for the specified source when it asserts and has
     * been cleared, then calls handle_pending_irq() for every context.
     *
     * @param irq_idx Source interrupt index (0 corresponds to source ID 1).
     * @param value   New value driven by the source (non-zero = asserted).
     */
    void source_irq_cb(uint32_t irq_idx, uint32_t value);
#endif

    /**
     * @brief Evaluate pending interrupts for a context and assert the core IRQ if needed.
     *
     * Checks whether any enabled, pending source interrupt above the context
     * threshold exists. If one is found, the interrupt output is asserted.
     * If not, the interrupt output is de-asserted.
     *
     * @param ctx Context index to evaluate (0-based).
     */
    void handle_pending_irq(uint32_t ctx);

    /**
     * @brief Read callback for the claim/complete register of a context.
     *
     * Finds the highest-priority pending interrupt ID for @p ctx, respecting
     * the context threshold, and clears its pending bit. The found source ID
     * is also written back to the claim/complete register. After the pending
     * bit is cleared, handle_pending_irq() is called to re-evaluate and
     * update the interrupt output for @p ctx.
     *
     * @param ctx         Context index performing the claim (0-based).
     * @param[out] src_id Filled with the claimed interrupt source ID, or 0 if none.
     */
    void claim_complete_read_cb(uint32_t ctx, uint32_t& src_id);

    /**
     * @brief Write callback for the claim/complete register of a context.
     *
     * Signals that the interrupt identified by @p src_id has been serviced and
     * marks the source as cleared so it can fire again. Writes for source ID 0
     * (reserved) and for disabled sources are silently ignored.
     *
     * @param ctx Context index completing the interrupt (0-based).
     * @param src_id Interrupt source ID being completed.
     */
    void claim_complete_write_cb(uint32_t ctx, uint32_t src_id);

    /**
     * @brief Drive the core interrupt output for a context.
     *
     * Abstracts the sc_signal / TLM signal interface difference and writes
     * @p value to interrupts_o[ctx].
     *
     * @param ctx   Context index whose interrupt output is to be driven (0-based).
     * @param value Value to drive: true = assert, false = de-assert.
     */
    void write_irq(uint32_t ctx, bool value);

    /**
     * @brief Find a candidate pending interrupt for a context.
     *
     * Scans all interrupt sources for one that is pending, enabled for @p ctx,
     * and has priority strictly above the context threshold.
     *
     * When @p find_top_priority is true the full source list is scanned and
     * the highest-priority (lowest source ID on a tie) interrupt ID is returned.
     * When false the scan stops at the first qualifying source, which is
     * sufficient for deciding whether an interrupt should be raised.
     *
     * @param ctx               Context index to evaluate (0-based).
     * @param find_top_priority True to return the highest-priority source ID;
     *                          false to return any qualifying source ID.
     * @param ignore_threshold  True to ignore the threshold value during the search;
     *                          false to take threshold into account.
     * @return Source interrupt ID of the selected interrupt, or 0 if none qualifies.
     */
    uint32_t get_source_irq(uint32_t ctx, bool find_top_priority, bool ignore_threshold) const;

    /**
     * @brief Return a reference to the pending register value that contains @p src_id.
     *
     * The PLIC pending state is stored as a flat array of 32-bit words; this
     * helper returns the word for the given interrupt source ID so callers can
     * test or modify individual bits.
     *
     * @param src_id Interrupt source ID whose pending word is required.
     * @return       Reference to the 32-bit pending register containing the bit for @p src_id.
     */
    uint32_t& get_pending(uint32_t src_id) const;

    /**
     * @brief Convert a zero-based source interrupt vector index to a PLIC source ID.
     *
     * The PLIC reserves source ID 0 as "no interrupt", so the first real interrupt
     * source (index 0 in the interrupt vector) maps to PLIC source ID 1.  This
     * helper encapsulates that offset so callers never hard-code the adjustment.
     *
     * @param src_idx Zero-based index into the source interrupt vector.
     * @return        PLIC source ID corresponding to @p src_idx (always >= 1).
     */
    uint32_t get_source_id(uint32_t src_idx) const;

    sc_core::sc_time clk;
    std::unique_ptr<plic_regs> regs;
    std::vector<bool> src_irq_cleared;
};

}  // namespace rvi
}  // namespace vpvper

#endif  // _RVI_PLIC_H_
