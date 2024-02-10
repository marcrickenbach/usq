void update_leds(bool channel, 
                 uint16_t step, 
                 uint16_t offset, 
                 uint16_t value)
{
    // for (int i = 0; i < 16; i++) {
    //     if (led_armed[i]) {
    //         led_buffer[i] = 28;
    //     } else {
    //         led_buffer[i] = 0;
    //     }
    // }

    int index = (channel * offset) + step;
    index = led_to_out_map[index];
    /*  Make sure we're only packing data into the appropriate channel.
        We also account for moving offsets, which would change
        the start index for channel 1. Finally, before packing the data,
        we add our 12-bit value to the led_buffer.
    */

    int byte_offset = (offset * 3) / 2;
    int led_offset = (offset % 2 == 0) ? byte_offset * 2 / 3 : byte_offset * 2 / 3 + 1;

    int start_idx, end_idx, start_led, end_led;
    if (channel == 1) {
        start_idx = 0;
        end_idx = byte_offset - 1;
        start_led = 0;
        end_led = led_offset - 1;
        led_buffer[index] = value; 
    } else { // Assuming channel is 1
        start_idx = byte_offset;
        end_idx = 23;
        start_led = led_offset;
        end_led = 15;
        led_buffer[index] = value; 
    }

    int j = start_idx; // Start index for led_data
    for (int i = start_led; i <= end_led; i += 2) {
        led_data[j] = led_buffer[i] >> 4;
        led_data[j+1] = (led_buffer[i] << 4) | (i + 1 <= end_led ? led_buffer[i + 1] >> 8 : 0);
        led_data[j+2] = (i + 1 <= end_led) ? led_buffer[i + 1] & 0xFF : 0;
        j += 3;
    }


    int ret = spi_write(spi_dev, &spi_cfg, &tx);
    if (ret != 0) {
        LOG_ERR("Could not write to SPI device");
        return;
    }

    led_latch_enable_pulse(true);
}






void configure_USB_PLLSAI_CLK(void) {
    

    // RCC_CR &= ~(1U << 28); // Clear PLLSAION to turn off PLLSAI
    /* Clear PLLSAICFGR Register relevant fields */
    // Clear PLLSAIM, PLLSAIN, PLLSAIP, and PLLSAIQ fields    

    // RCC_PLLSAICFGR &= ~((0x3F << 0) | (0x1FF << 6) | (0x3 << 16) | (0xF << 24));
    /* Set PLLSAICFGR Register */
    // PLLSAIM: Division of PLLSAI input clock
    // RCC_PLLSAICFGR |= (8 << 0); // HSE Divosr 8 (1MHz input clock for PLLSAI)
    // PLLSAIN: PLLSAI Multiplication
    // RCC_PLLSAICFGR |= (192 << 6); // Multiply by 192 (192 MHz clock for PLLSAI)
    // PLLSAIP: PLLSAI division by 4 for 48 MHz clock (USB)
    // RCC_PLLSAICFGR |= (0x1 << 16); // Set PLLSAIP to 4, for 48MHz clock
    // // PLLSAIQ: PLLSAI division factor for SAIs clock
    // RCC_PLLSAICFGR |= (0x2 << 24); // Set PLLSAIQ to 2, if needed for SAI or other peripherals
    /* Configure CLK48 source to be PLLSAIP */
    RCC_DCKCFGR2 &= ~((1U << 27) | (1U << 28)); // Clear bits for CK48MSEL and potentially SDIOSEL if needed
    RCC_DCKCFGR2 |= (1U << 27); // Set CK48MSEL to use PLLSAIP as CLK48 source
    RCC_DCKCFGR2 |= (1U << 28); // Clear SDIOSEL to use PLLSAIP as SDIO source
    /* Enable PLLSAI */
    // RCC_CR |= (1U << 28); // Set PLLSAION to turn on PLLSAI
    // RCC_AHB2ENR |= (1U << 7); // Enable OTG FS clock



}
