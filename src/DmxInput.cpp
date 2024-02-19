/*
 * Copyright (c) 2021 Jostein Løwer
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#include "DmxInput.h"
#include "DmxInput.pio.h"
#include "DmxInputInverted.pio.h"

#if defined(ARDUINO_ARCH_MBED)
  #include <clocks.h>
  #include <irq.h>
  #include <Arduino.h> // REMOVE ME
#else
  #include "pico/time.h"
  #include "hardware/clocks.h"
  #include "hardware/irq.h"
  #include "hardware/dma.h"
#endif

#include <stdio.h>
#include <memory.h>

#define HAL_DBG0      0  // rx task
#define HAL_DBG1      1  // brk detect
#define HAL_DBG2      2  // dmxinput_dma_handler
#define HAL_DBG3      3  // rdp


bool prgm_loaded[] = {false,false};
bool prgm_bd_loaded[] = {false,false};
volatile uint prgm_offsets[] = {0,0};
volatile uint prgm_bd_offsets[] = {0,0};

/*
This array tells the interrupt handler which instance has interrupted.
The interrupt handler has only the ints0 register to go on, so this array needs as many spots as there are DMA channels.
*/
#define NUM_DMA_CHANS 12
DmxInput *active_inputs[NUM_DMA_CHANS] = {nullptr};
#define RDM_MINIMAL_HEADER_SIZE  3

/* RDM START CODE (Slot 0) */
#ifndef E120_SC_RDM
#define E120_SC_RDM 0xCC
#endif

/* RDM Protocol Data Structure ID's (Slot 1) */
#ifndef E120_SC_SUB_MESSAGE
#define E120_SC_SUB_MESSAGE 0x01
#endif


void dmxinput_break_detect_handler()
{
    for(int i=0;i<NUM_DMA_CHANS;i++)
    {
        DmxInput * me = (DmxInput*)active_inputs[i];

        if(me != nullptr)
        {
            if (pio_interrupt_get(me->_pio, me->_sm_bd))
            {
                pio_interrupt_clear(me->_pio, me->_sm_bd);
                gpio_put(HAL_DBG1, true);
                me->call_cb_and_restart_rx(true);
                gpio_put(HAL_DBG1, false);
            }
        }
    }
}

DmxInput::return_code DmxInput::begin(uint pin, uint start_channel, uint num_channels, PIO pio, bool inverted)
{
    uint pio_ind = pio_get_index(pio);

    if(!prgm_loaded[pio_ind])
    {
        // Attempt to load the DMX PIO assembly program into the PIO program memory
        if(!inverted)
        {
            if (!pio_can_add_program(pio, &DmxInput_program))
            {
                return ERR_INSUFFICIENT_PRGM_MEM;
            }
            prgm_offsets[pio_ind] = pio_add_program(pio, &DmxInput_program);
        }
        else
        {
            if (!pio_can_add_program(pio, &DmxInputInverted_program))
            {
                return ERR_INSUFFICIENT_PRGM_MEM;
            }
            prgm_offsets[pio_ind] = pio_add_program(pio, &DmxInputInverted_program);
        }

        prgm_loaded[pio_ind] = true;
    }

    if(!prgm_bd_loaded[pio_ind])
    {
        // Attempt to load the DMX PIO assembly program into the PIO program memory
        if (!pio_can_add_program(pio, &DmxBreakDetect_program))
        {
            return ERR_INSUFFICIENT_PRGM_MEM;
        }
        prgm_bd_offsets[pio_ind] = pio_add_program(pio, &DmxBreakDetect_program);
        prgm_bd_loaded[pio_ind] = true;
    }

    // Attempt to claim an unused State Machine into the PIO program memory
    int sm = pio_claim_unused_sm(pio, false);
    if (sm == -1)
    {
        return ERR_NO_SM_AVAILABLE;
    }

    int sm_bd = pio_claim_unused_sm(pio, false);
    if (sm_bd == -1)
    {
        return ERR_NO_SM_AVAILABLE;
    }

    // Set this pin's GPIO function (connect PIO to the pad)
    pio_sm_set_consecutive_pindirs(pio, sm,    pin, 1, false);
    pio_sm_set_consecutive_pindirs(pio, sm_bd, pin, 1, false);
    pio_gpio_init(pio, pin);
    gpio_pull_up(pin);

    // Generate the default PIO state machine config provided by pioasm
    pio_sm_config sm_conf;
    pio_sm_config sm_conf_bd;

    if(!inverted)
    {
        sm_conf = DmxInput_program_get_default_config(prgm_offsets[pio_ind]);
    }
    else
    {
        sm_conf = DmxInputInverted_program_get_default_config(prgm_offsets[pio_ind]);
    }

    sm_conf_bd = DmxBreakDetect_program_get_default_config(prgm_bd_offsets[pio_ind]);

    sm_config_set_in_pins(&sm_conf,    pin); // for WAIT, IN
    sm_config_set_in_pins(&sm_conf_bd, pin); // for WAIT, IN
    sm_config_set_jmp_pin(&sm_conf,    pin); // for JMP
    sm_config_set_jmp_pin(&sm_conf_bd, pin); // for JMP

    // Setup the side-set pins for the PIO state machine
    // Shift to right, autopush disabled
    sm_config_set_in_shift(&sm_conf,    true, false, 8);
    // Deeper FIFO as we're not doing any TX
    sm_config_set_fifo_join(&sm_conf, PIO_FIFO_JOIN_RX);

    // Setup the clock divider to run the state machine at exactly 1MHz
    uint clk_div = clock_get_hz(clk_sys) / DMX_SM_FREQ;
    sm_config_set_clkdiv(&sm_conf, clk_div);
    sm_config_set_clkdiv(&sm_conf_bd, clk_div);

    // Load our configuration, jump to the start of the program and run the State Machine
    pio_sm_init(pio, sm,    prgm_offsets[pio_ind],    &sm_conf);
    pio_sm_init(pio, sm_bd, prgm_bd_offsets[pio_ind], &sm_conf_bd);
    //sm_config_set_in_shift(&c, true, false, n_bits)

    //pio_sm_put_blocking(pio, sm, (start_channel + num_channels) - 1);

    _pio = pio;
    _sm = sm;
    _sm_bd = sm_bd;
    _pin = pin;
    _start_channel = start_channel;
    _num_channels = num_channels;
    _buf = nullptr;
    _cb = nullptr;

    _dma_chan = dma_claim_unused_channel(true);


    if (active_inputs[_dma_chan] != nullptr)
    {
        return ERR_NO_SM_AVAILABLE;
    }
    active_inputs[_dma_chan] = this;


    uint pio_irq = (_pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0;
    if (irq_get_exclusive_handler(pio_irq)) {
        pio_irq++;
        if (irq_get_exclusive_handler(pio_irq)) {
            panic("All IRQs are in use");
        }
    }

    const uint irq_index = pio_irq - ((_pio == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0);
    irq_set_exclusive_handler(pio_irq, dmxinput_break_detect_handler); //, PICO_SHARED_IRQ_HANDLER_HIGHEST_ORDER_PRIORITY);

    irq_set_enabled(pio_irq, true);
    pio_set_irqn_source_enabled(_pio, irq_index, static_cast<pio_interrupt_source>(pis_interrupt0 + _sm_bd), true);

    return SUCCESS;
}

void DmxInput::read(uint8_t *buffer)
{
    if(_rx_buf==nullptr)
    {
        read_async(buffer);
    }
    unsigned long start = _last_packet_timestamp;
    while(_last_packet_timestamp == start)
    {
        tight_loop_contents();
    }
}

void dmxinput_dma_handler()
{
    gpio_put(HAL_DBG2, true);
    for(int i=0;i<NUM_DMA_CHANS;i++)
    {
        if(active_inputs[i]!=nullptr && (dma_hw->ints0 & (1u<<i)))
        {
            dma_hw->ints0 = 1u << i;
            DmxInput *instance = active_inputs[i];

            if (instance->read_header)
            {
                instance->read_header = false;
                if (   (instance->_rx_buf[0] == E120_SC_RDM)
                    && (instance->_rx_buf[1] == E120_SC_SUB_MESSAGE))
                {
                    gpio_put(HAL_DBG3, true);
                    // rdp_frame = true;
                    dma_channel_set_trans_count(i, instance->_rx_buf[2] - 1, false);
                    dma_channel_set_write_addr(i, instance->_rx_buf + RDM_MINIMAL_HEADER_SIZE, true);
                }
                else
                {
                    dma_channel_set_trans_count(i, DMXINPUT_BUFFER_SIZE(_start_channel, instance->_num_channels) - RDM_MINIMAL_HEADER_SIZE, false);
                    dma_channel_set_write_addr(i, instance->_rx_buf + RDM_MINIMAL_HEADER_SIZE, true);
                }
                instance->data_is_reported = false;
            }
            else
            {
                gpio_put(HAL_DBG2, false);
                instance->call_cb_and_restart_rx(false);
            }
        }
    }
    gpio_put(HAL_DBG2, false);
    gpio_put(HAL_DBG3, false);
}

void DmxInput::call_cb_and_restart_rx(bool break_detected_now)
{
    if (break_detected_now)
    {
        //interslot timeout
        // pio_sm_set_enabled(_pio, _sm_bd, false);
        // uint16_t remaining = dma_channel_hw_addr(_dma_chan)->transfer_count;
        read_header = true;
        rdp_frame = false;
        dma_channel_set_irq0_enabled(_dma_chan, false);
        dma_channel_abort(_dma_chan);
        dma_channel_set_trans_count(_dma_chan, RDM_MINIMAL_HEADER_SIZE, false);
        dma_channel_set_write_addr(_dma_chan, _rx_buf, true);
        pio_sm_exec(_pio, _sm, pio_encode_jmp(prgm_offsets[pio_get_index(_pio)] + (break_detected_now ? DmxInput_offset_start_in_break : 0)));
        pio_sm_clear_fifos(_pio, _sm);
        dma_channel_acknowledge_irq0(_dma_chan);
        dma_channel_set_irq0_enabled(_dma_chan, true);
    }

    pio_sm_set_enabled(_pio, _sm, break_detected_now);

    memcpy(_buf, _rx_buf, DMXINPUT_BUFFER_SIZE(_start_channel, _num_channels));
    // memset(_rx_buf, 0, sizeof(_rx_buf));

    #ifdef ARDUINO
        _last_packet_timestamp = millis();
    #else
        _last_packet_timestamp = to_ms_since_boot(get_absolute_time());
    #endif

    // Trigger the callback if we have one
    if (!data_is_reported && _cb != nullptr)
    {
        data_is_reported = true;
        (*(_cb))(this);
    }
}

void DmxInput::stop_rx()
{
    // pio_sm_set_enabled(_pio, _sm_bd, false);
}

void DmxInput::read_async(uint8_t *buffer, void (*inputUpdatedCallback)(DmxInput*))
{
    _buf = buffer;
    if (inputUpdatedCallback!=nullptr)
    {
        _cb = inputUpdatedCallback;
    }

    // do not callbakc on first break
    data_is_reported = true;

    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_set_enabled(_pio, _sm_bd, false);

    // Reset the PIO state machine to a consistent state. Clear the buffers and registers
    pio_sm_restart(_pio, _sm);
    pio_sm_restart(_pio, _sm_bd);

    //setup dma
    dma_channel_config cfg = dma_channel_get_default_config(_dma_chan);

    // Reading from constant address, writing to incrementing byte addresses
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_read_increment(&cfg, false);
    channel_config_set_write_increment(&cfg, true);

    // Pace transfers based on DREQ_PIO0_RX0 (or whichever pio and sm we are using)
    channel_config_set_dreq(&cfg, pio_get_dreq(_pio, _sm, false));

    //channel_config_set_ring(&cfg, true, 5);
    dma_channel_configure(
        _dma_chan,
        &cfg,
        NULL,    // dst
        &_pio->rxf[_sm],  // src
        RDM_MINIMAL_HEADER_SIZE,  // transfer count,
        false
    );

    dma_channel_set_irq0_enabled(_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dmxinput_dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);


    //aaand start!
    read_header = true;
    dma_channel_set_write_addr(_dma_chan, buffer, false);
    pio_sm_exec(_pio, _sm, pio_encode_jmp(prgm_offsets[pio_get_index(_pio)]));
    pio_sm_clear_fifos(_pio, _sm);
    #ifdef ARDUINO
      _last_packet_timestamp = millis();
    #else
      _last_packet_timestamp = to_ms_since_boot(get_absolute_time());
    #endif

    // pio_sm_set_enabled(_pio, _sm, true);

    pio_sm_exec(_pio, _sm_bd, pio_encode_jmp(prgm_bd_offsets[pio_get_index(_pio)]));
    pio_sm_set_enabled(_pio, _sm_bd, true);
}

unsigned long DmxInput::latest_packet_timestamp()
{
    return _last_packet_timestamp;
}

uint DmxInput::pin()
{
    return _pin;
}

void DmxInput::end()
{
    // Stop the PIO state machine
    pio_sm_set_enabled(_pio, _sm, false);

    // Remove the PIO DMX program from the PIO program memory
    uint pio_id = pio_get_index(_pio);
    bool inuse = false;
    for(uint i=0;i<NUM_DMA_CHANS;i++)
    {
        if(i==_dma_chan)
        {
            continue;
        }
        if(pio_id == pio_get_index(active_inputs[i]->_pio))
        {
            inuse = true;
            break;
        }
    }

    if(!inuse)
    {
        prgm_loaded[pio_id] = false;
        pio_remove_program(_pio, &DmxInput_program, prgm_offsets[pio_id]);
        prgm_offsets[pio_id]=0;

        prgm_bd_loaded[pio_id] = false;
        pio_remove_program(_pio, &DmxBreakDetect_program, prgm_bd_offsets[pio_id]);
        prgm_bd_offsets[pio_id]=0;
    }

    // Unclaim the sm
    pio_sm_unclaim(_pio, _sm);
    pio_sm_unclaim(_pio, _sm_bd);

    dma_channel_unclaim(_dma_chan);
    active_inputs[_dma_chan] = nullptr;

    _buf = nullptr;
}
