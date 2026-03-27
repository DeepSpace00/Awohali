/* ============================================================================
 * STM32 HAL Platform Implementation Example
 * ============================================================================ */

#ifdef STM32_HAL_PLATFORM

// #include "stm32f4xx_hal.h"
#include "lan8742_platform.h"

/* External ETH handle - should be defined in main.c */
extern ETH_HandleTypeDef heth;

void lan8742_platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

void lan8742_platform_delay_us(uint32_t us)
{
    /* STM32 doesn't have HAL microsecond delay, use busy wait */
    uint32_t cycles = us * (SystemCoreClock / 1000000U);
    while (cycles--) {
        __NOP();
    }
}

bool lan8742_platform_mdio_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t* data)
{
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, reg_addr, (uint32_t*)data) == HAL_OK) {
        return true;
    }
    return false;
}

bool lan8742_platform_mdio_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    uint32_t temp_data = data;
    if (HAL_ETH_WritePHYRegister(&heth, phy_addr, reg_addr, temp_data) == HAL_OK) {
        return true;
    }
    return false;
}

bool lan8742_platform_mmd_read(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data)
{
    /* Step 1: Write MMD address register */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    /* Step 2: Write MMD register address */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    /* Step 3: Set data mode (no post increment) */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    /* Step 4: Read data */
    return lan8742_platform_mdio_read(phy_addr, 0x0E, data);
}

bool lan8742_platform_mmd_write(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data)
{
    /* Step 1: Write MMD address register */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    /* Step 2: Write MMD register address */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    /* Step 3: Set data mode (no post increment) */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    /* Step 4: Write data */
    return lan8742_platform_mdio_write(phy_addr, 0x0E, data);
}

void lan8742_platform_debug_print(const char* format, ...)
{
    /* Implementation depends on your debug output method */
    /* Could use ITM, UART, RTT, etc. */
    #ifdef DEBUG
    va_list args;
    va_start(args, format);
    vprintf(format, args);
    va_end(args);
    #endif
}

#endif /* STM32_HAL_PLATFORM */

/* ============================================================================
 * Arduino Platform Implementation Example
 * ============================================================================ */

#ifdef ARDUINO_PLATFORM

#include <Arduino.h>
#include <SPI.h>
#include "lan8742_platform.h"

/* Arduino with W5500 Ethernet shield example */
/* Note: This is conceptual - real implementation would depend on specific hardware */

void lan8742_platform_delay_ms(uint32_t ms)
{
    delay(ms);
}

void lan8742_platform_delay_us(uint32_t us)
{
    delayMicroseconds(us);
}

bool lan8742_platform_mdio_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t* data)
{
    /* Implementation would depend on how MDIO is connected to Arduino */
    /* This could be bit-banged GPIO or through an Ethernet controller */
    
    /* Example bit-bang implementation */
    #define MDIO_PIN 2
    #define MDC_PIN  3
    
    // Implement MDIO read protocol here
    // This is a simplified placeholder
    
    *data = 0x1234; // Placeholder
    return true;
}

bool lan8742_platform_mdio_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    /* Implementation would depend on how MDIO is connected to Arduino */
    /* This could be bit-banged GPIO or through an Ethernet controller */
    
    // Implement MDIO write protocol here
    // This is a simplified placeholder
    
    return true;
}

bool lan8742_platform_mmd_read(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data)
{
    /* Same MMD access sequence as STM32 example */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    return lan8742_platform_mdio_read(phy_addr, 0x0E, data);
}

bool lan8742_platform_mmd_write(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data)
{
    /* Same MMD access sequence as STM32 example */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    return lan8742_platform_mdio_write(phy_addr, 0x0E, data);
}

void lan8742_platform_debug_print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    Serial.print(buffer);
    
    va_end(args);
}

#endif /* ARDUINO_PLATFORM */

/* ============================================================================
 * Zephyr RTOS Platform Implementation Example
 * ============================================================================ */

#ifdef ZEPHYR_PLATFORM

#include <zephyr/kernel.h>
#include <zephyr/drivers/mdio.h>
#include <zephyr/logging/log.h>
#include "lan8742_platform.h"

LOG_MODULE_REGISTER(lan8742_platform);

/* Device tree reference to MDIO device */
static const struct device *mdio_dev = DEVICE_DT_GET(DT_NODELABEL(mdio));

void lan8742_platform_delay_ms(uint32_t ms)
{
    k_msleep(ms);
}

void lan8742_platform_delay_us(uint32_t us)
{
    k_usleep(us);
}

bool lan8742_platform_mdio_read(uint8_t phy_addr, uint8_t reg_addr, uint16_t* data)
{
    if (!device_is_ready(mdio_dev)) {
        return false;
    }
    
    int ret = mdio_read(mdio_dev, phy_addr, reg_addr, data);
    return (ret == 0);
}

bool lan8742_platform_mdio_write(uint8_t phy_addr, uint8_t reg_addr, uint16_t data)
{
    if (!device_is_ready(mdio_dev)) {
        return false;
    }
    
    int ret = mdio_write(mdio_dev, phy_addr, reg_addr, data);
    return (ret == 0);
}

bool lan8742_platform_mmd_read(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data)
{
    /* Use Zephyr's MDIO MMD functions if available, otherwise implement manually */
    #ifdef CONFIG_MDIO_MMD
    if (!device_is_ready(mdio_dev)) {
        return false;
    }
    
    int ret = mdio_read_c45(mdio_dev, phy_addr, mmd_addr, reg_addr, data);
    return (ret == 0);
    #else
    /* Manual MMD access */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    return lan8742_platform_mdio_read(phy_addr, 0x0E, data);
    #endif
}

bool lan8742_platform_mmd_write(uint8_t phy_addr, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data)
{
    #ifdef CONFIG_MDIO_MMD
    if (!device_is_ready(mdio_dev)) {
        return false;
    }
    
    int ret = mdio_write_c45(mdio_dev, phy_addr, mmd_addr, reg_addr, data);
    return (ret == 0);
    #else
    /* Manual MMD access */
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, mmd_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0E, reg_addr)) {
        return false;
    }
    
    if (!lan8742_platform_mdio_write(phy_addr, 0x0D, 0x4000 | mmd_addr)) {
        return false;
    }
    
    return lan8742_platform_mdio_write(phy_addr, 0x0E, data);
    #endif
}

void lan8742_platform_debug_print(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    
    char buffer[256];
    vsnprintf(buffer, sizeof(buffer), format, args);
    LOG_DBG("%s", buffer);
    
    va_end(args);
}

#endif /* ZEPHYR_PLATFORM */

/* ============================================================================
 * Usage Examples
 * ============================================================================ */

/**
 * @brief Basic initialization and link status check example
 */
void example_basic_usage(void)
{
    lan8742_handle_t phy_handle;
    lan8742_status_t status;
    
    /* Initialize PHY at address 0 */
    status = lan8742_init(&phy_handle, 0);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("PHY initialization failed: %d\n", status);
        return;
    }
    
    lan8742_platform_debug_print("PHY initialized successfully\n");
    lan8742_platform_debug_print("PHY ID1: 0x%04X, PHY ID2: 0x%04X\n", 
                                 phy_handle.phy_id1, phy_handle.phy_id2);
    
    /* Configure auto-negotiation */
    uint16_t advertise = LAN8742_ANAR_100BASE_TX_FD | LAN8742_ANAR_100BASE_TX |
                        LAN8742_ANAR_10BASE_T_FD | LAN8742_ANAR_10BASE_T |
                        LAN8742_ANAR_SELECTOR_IEEE8023;
    
    status = lan8742_config_autoneg(&phy_handle, true, advertise);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("Auto-negotiation config failed: %d\n", status);
        return;
    }
    
    /* Restart auto-negotiation */
    status = lan8742_restart_autoneg(&phy_handle);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("Auto-negotiation restart failed: %d\n", status);
        return;
    }
    
    /* Wait for link to establish */
    lan8742_link_status_t link_status;
    int retry_count = 0;
    const int max_retries = 50; /* 5 seconds */
    
    do {
        lan8742_platform_delay_ms(100);
        status = lan8742_get_link_status(&phy_handle, &link_status);
        retry_count++;
    } while (status == LAN8742_STATUS_OK && !link_status.link_up && retry_count < max_retries);
    
    if (link_status.link_up) {
        lan8742_platform_debug_print("Link established!\n");
        lan8742_platform_debug_print("Speed: %s, Duplex: %s\n",
                                     link_status.speed == LAN8742_SPEED_100M ? "100Mbps" : "10Mbps",
                                     link_status.duplex == LAN8742_DUPLEX_FULL ? "Full" : "Half");
        lan8742_platform_debug_print("Auto-negotiation: %s\n",
                                     link_status.autoneg_complete ? "Complete" : "In progress");
    } else {
        lan8742_platform_debug_print("Link not established after timeout\n");
    }
}

/**
 * @brief Wake-on-LAN configuration example
 */
void example_wol_configuration(void)
{
    lan8742_handle_t phy_handle;
    lan8742_status_t status;
    
    /* Initialize PHY */
    status = lan8742_init(&phy_handle, 0);
    if (status != LAN8742_STATUS_OK) {
        return;
    }
    
    /* Configure basic WoL settings */
    status = lan8742_config_wol(&phy_handle, true, true, true, false);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("WoL configuration failed: %d\n", status);
        return;
    }
    
    /* Set MAC address for WoL */
    uint8_t mac_addr[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    status = lan8742_set_wol_mac_addr(&phy_handle, mac_addr);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("WoL MAC address setting failed: %d\n", status);
        return;
    }
    
    /* Configure custom WoL filter */
    lan8742_wol_filter_t wol_filter = {
        .enable = true,
        .filter_triggered = false,
        .addr_match_enable = true,
        .multicast_enable = false,
        .broadcast_enable = true,
        .pattern_offset = 0,
        .filter_crc = 0x1234,
        .byte_mask = {0xFF, 0xFF, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00,
                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
    };
    
    status = lan8742_config_wol_filter(&phy_handle, &wol_filter);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("WoL filter configuration failed: %d\n", status);
        return;
    }
    
    lan8742_platform_debug_print("Wake-on-LAN configured successfully\n");
}

/**
 * @brief Cable diagnostics example
 */
void example_cable_diagnostics(void)
{
    lan8742_handle_t phy_handle;
    lan8742_status_t status;
    
    /* Initialize PHY */
    status = lan8742_init(&phy_handle, 0);
    if (status != LAN8742_STATUS_OK) {
        return;
    }
    
    lan8742_platform_debug_print("Starting cable diagnostics...\n");
    
    /* Start TDR test */
    status = lan8742_start_tdr_test(&phy_handle);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("TDR test start failed: %d\n", status);
        return;
    }
    
    /* Wait for test to complete */
    lan8742_tdr_result_t tdr_result;
    int retry_count = 0;
    const int max_retries = 100; /* 10 seconds */
    
    do {
        lan8742_platform_delay_ms(100);
        status = lan8742_get_tdr_result(&phy_handle, &tdr_result);
        retry_count++;
    } while (status == LAN8742_STATUS_OK && !tdr_result.test_complete && retry_count < max_retries);
    
    if (tdr_result.test_complete) {
        lan8742_platform_debug_print("TDR Test Results:\n");
        
        const char* cable_type_str;
        switch (tdr_result.cable_type) {
            case LAN8742_TDR_CABLE_MATCH:
                cable_type_str = "Matched (Good)";
                break;
            case LAN8742_TDR_CABLE_SHORT:
                cable_type_str = "Short Circuit";
                break;
            case LAN8742_TDR_CABLE_OPEN:
                cable_type_str = "Open Circuit";
                break;
            default:
                cable_type_str = "Unknown";
                break;
        }
        
        lan8742_platform_debug_print("Cable Type: %s\n", cable_type_str);
        lan8742_platform_debug_print("Cable Length: %d meters\n", tdr_result.cable_length);
        
        /* Get additional cable length measurement for matched cables */
        if (tdr_result.cable_type == LAN8742_TDR_CABLE_MATCH) {
            uint8_t cable_length;
            status = lan8742_get_cable_length(&phy_handle, &cable_length);
            if (status == LAN8742_STATUS_OK) {
                lan8742_platform_debug_print("Precise Cable Length: %d meters\n", cable_length);
            }
        }
    } else {
        lan8742_platform_debug_print("TDR test did not complete within timeout\n");
    }
}

/**
 * @brief Interrupt handling example
 */
void example_interrupt_handling(void)
{
    lan8742_handle_t phy_handle;
    lan8742_status_t status;
    
    /* Initialize PHY */
    status = lan8742_init(&phy_handle, 0);
    if (status != LAN8742_STATUS_OK) {
        return;
    }
    
    /* Configure interrupt mask to enable link status change interrupts */
    uint16_t int_mask = LAN8742_ISR_LINK_DOWN | LAN8742_ISR_AUTONEG_COMPLETE;
    status = lan8742_set_interrupt_mask(&phy_handle, int_mask);
    if (status != LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("Interrupt mask configuration failed: %d\n", status);
        return;
    }
    
    lan8742_platform_debug_print("Interrupt monitoring enabled\n");
    
    /* Example interrupt service routine (to be called from actual ISR) */
    /* This would typically be called from a GPIO interrupt handler */
    /*
    void lan8742_isr_handler(void) {
        lan8742_interrupt_status_t int_status;
        
        if (lan8742_get_interrupt_status(&phy_handle, &int_status) == LAN8742_STATUS_OK) {
            if (int_status.link_down) {
                lan8742_platform_debug_print("Link down interrupt\n");
            }
            if (int_status.autoneg_complete) {
                lan8742_platform_debug_print("Auto-negotiation complete interrupt\n");
            }
            if (int_status.wol_event) {
                lan8742_platform_debug_print("Wake-on-LAN event interrupt\n");
            }
        }
    }
    */
}

/**
 * @brief Power management example
 */
void example_power_management(void)
{
    lan8742_handle_t phy_handle;
    lan8742_status_t status;
    
    /* Initialize PHY */
    status = lan8742_init(&phy_handle, 0);
    if (status != LAN8742_STATUS_OK) {
        return;
    }
    
    lan8742_platform_debug_print("Configuring power management features...\n");
    
    /* Enable Energy Detect Power-Down (EDPD) */
    status = lan8742_set_edpd(&phy_handle, true);
    if (status == LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("EDPD enabled\n");
    }
    
    /* Example: Put PHY to sleep when not needed */
    lan8742_platform_debug_print("Entering power down mode...\n");
    status = lan8742_set_power_down(&phy_handle, true);
    if (status == LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("PHY in power down mode\n");
    }
    
    /* Wait some time */
    lan8742_platform_delay_ms(5000);
    
    /* Wake up PHY */
    lan8742_platform_debug_print("Waking up PHY...\n");
    status = lan8742_set_power_down(&phy_handle, false);
    if (status == LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("PHY awakened\n");
    }
    
    /* Wait for PHY to stabilize */
    lan8742_platform_delay_ms(1000);
    
    /* Check link status after wake-up */
    lan8742_link_status_t link_status;
    status = lan8742_get_link_status(&phy_handle, &link_status);
    if (status == LAN8742_STATUS_OK) {
        lan8742_platform_debug_print("Link status after wake-up: %s\n",
                                     link_status.link_up ? "UP" : "DOWN");
    }
}