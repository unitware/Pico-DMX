
/*
 * Copyright (c) 2021 Jostein Løwer
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "DmxOutput.h"
#include "DmxOutput.pio.h"

#if defined(ARDUINO_ARCH_MBED)
  #include <clocks.h>
  #include <irq.h>
#else
  #include "hardware/clocks.h"
  #include "hardware/timer.h"
  #include "hardware/irq.h"
  #include "pico/time.h"
#endif


#include <stdio.h>

volatile struct {
    void (*done_cb)(void * arg);
    void * arg;
} tx_done_callbacks[2][4];

void dmxoutput_irq_handler()
{
    for (uint8_t i=0; i<4; i++)
    {
        if (pio_interrupt_get(pio0, i))
        {
            if (tx_done_callbacks[0][i].done_cb)
            {
                pio_interrupt_clear(pio0, i);
                tx_done_callbacks[0][i].done_cb(tx_done_callbacks[0][i].arg);
            }
        }

        if (pio_interrupt_get(pio1, i))
        {
            if (tx_done_callbacks[1][i].done_cb)
            {
                pio_interrupt_clear(pio1, i);
                tx_done_callbacks[1][i].done_cb(tx_done_callbacks[1][i].arg);
            }
        }
    }
}

DmxOutput::return_code DmxOutput::begin(uint pin, PIO pio)
{
    // Attempt to load the DMX PIO assembly program into the PIO program memory
    if (!pio_can_add_program(pio, &DmxOutput_program))
    {
        return ERR_INSUFFICIENT_PRGM_MEM;
    }
    uint prgm_offset = pio_add_program(pio, &DmxOutput_program);

    // Attempt to claim an unused State Machine into the PIO program memory
    int sm = pio_claim_unused_sm(pio, false);
    if (sm == -1)
    {
        return ERR_NO_SM_AVAILABLE;
    }

    // Set this pin's GPIO function (connect PIO to the pad)
    pio_sm_set_pins_with_mask(pio, sm, 1u << pin, 1u << pin);
    pio_sm_set_pindirs_with_mask(pio, sm, 1u << pin, 1u << pin);
    pio_gpio_init(pio, pin);

    // Generate the default PIO state machine config provided by pioasm
    pio_sm_config sm_conf = DmxOutput_program_get_default_config(prgm_offset);

    // Setup the side-set pins for the PIO state machine
    sm_config_set_out_pins(&sm_conf, pin, 1);
    sm_config_set_sideset_pins(&sm_conf, pin);

    // Setup the clock divider to run the state machine at exactly 1MHz
    uint clk_div = clock_get_hz(clk_sys) / DMX_SM_FREQ;
    sm_config_set_clkdiv(&sm_conf, clk_div);

    // Load our configuration, jump to the start of the program and run the State Machine
    pio_sm_init(pio, sm, prgm_offset, &sm_conf);
    pio_sm_set_enabled(pio, sm, true);

    // Claim an unused DMA channel.
    // The channel is kept througout the lifetime of the DMX source
    int dma = dma_claim_unused_channel(false);

    if (dma == -1)
        return ERR_NO_DMA_AVAILABLE;

    // Get the default DMA config for our claimed channel
    dma_channel_config dma_conf = dma_channel_get_default_config(dma);

    // Set the DMA to move one byte per DREQ signal
    channel_config_set_transfer_data_size(&dma_conf, DMA_SIZE_8);

    // Setup the DREQ so that the DMA only moves data when there
    // is available room in the TXF buffer of our PIO state machine
    channel_config_set_dreq(&dma_conf, pio_get_dreq(pio, sm, true));

    // Setup the DMA to write to the TXF buffer of the PIO state machine
    dma_channel_set_write_addr(dma, &pio->txf[sm], false);

    // Apply the config
    dma_channel_set_config(dma, &dma_conf, false);

    // Set member values of C++ class
    _prgm_offset = prgm_offset;
    _pio = pio;
    _sm = sm;
    _pin = pin;
    _dma = dma;

    uint pio_irq = (_pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    if (irq_get_exclusive_handler(pio_irq)) {
        pio_irq++;
        if (irq_get_exclusive_handler(pio_irq)) {
            panic("All IRQs are in use");
        }
    }

    const uint irq_index = pio_irq - ((_pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0); // Get index of the IRQ
    irq_add_shared_handler(pio_irq, dmxoutput_irq_handler, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);

    irq_set_enabled(pio_irq, true); // Enable the IRQ
    pio_set_irqn_source_enabled(_pio, irq_index, static_cast<pio_interrupt_source>(pis_interrupt0 + _sm), true); // Set pio to tell us when the FIFO is NOT empty

    return SUCCESS;
}


void DmxOutput::write(uint8_t *universe, uint length, bool send_mab,
                void (*done_cb)(void * arg), void * done_cb_arg)
{
    uint8_t pio_index = (_pio == pio0) ? 0 : 1;
    tx_done_callbacks[pio_index][_sm].done_cb = done_cb;
    tx_done_callbacks[pio_index][_sm].arg     = done_cb_arg;

    // Temporarily disable the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Reset the PIO state machine to a consistent state. Clear the buffers and registers
    pio_sm_restart(_pio, _sm);

    pio_sm_put_blocking(_pio, _sm, length);

    // Start the DMX PIO program from the beginning
    pio_sm_exec(_pio, _sm, pio_encode_jmp(_prgm_offset + (send_mab ? 0 : DmxOutput_offset_start_tx)));

    // Restart the PIO state machinge
    pio_sm_set_enabled(_pio, _sm, true);

    // Start the DMA transfer
    dma_channel_transfer_from_buffer_now(_dma, universe, length+1);
}


bool DmxOutput::busy()
{
    if (dma_channel_is_busy(_dma))
        return true;

    return !pio_sm_is_tx_fifo_empty(_pio, _sm);
}


// bool DmxOutput::await(uint timeout_us)
// {
//     uint64_t end_time_us = 0xffffffffffffffff;
//     if (timeout_us)
//     {
//         end_time_us = time_us_64() + timeout_us + 500;
//     }

//     while (!pio_interrupt_get(_pio, 0) || !pio_sm_is_tx_fifo_empty(_pio, _sm))
//     {
//         if (end_time_us < time_us_64())
//         {
//             return false;
//         }
//     }
//     return true;
// }

void DmxOutput::end()
{
    // Stop the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Remove the PIO DMX program from the PIO program memory
    pio_remove_program(_pio, &DmxOutput_program, _prgm_offset);

    // Unclaim the DMA channel
    dma_channel_unclaim(_dma);

    // Unclaim the sm
    pio_sm_unclaim(_pio, _sm);
}
