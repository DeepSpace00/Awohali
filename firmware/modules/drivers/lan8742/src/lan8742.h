#ifndef LAN8742_H
#define LAN8742_H

#include <stdint.h>
#include <stdbool.h>
#include "lan8742_platform.h"

// Driver version
#define LAN8742_DRIVER_VERSION_MAJOR    1
#define LAN8742_DRIVER_VERSION_MINOR    0
#define LAN8742_DRIVER_VERSION_PATCH    0

// PHY Identifier values
#define LAN8742_PHY_ID1                 0x0007
#define LAN8742_PHY_ID2_BASE            0xC130
#define LAN8742_PHY_ID2_MASK            0xFFF0

// Register addresses (IEEE 802.3 standard)
#define LAN8742_REG_BASIC_CONTROL       0x00
#define LAN8742_REG_BASIC_STATUS        0x01
#define LAN8742_REG_PHY_ID1             0x02
#define LAN8742_REG_PHY_ID2             0x03
#define LAN8742_REG_AUTONEG_ADV         0x04
#define LAN8742_REG_AUTONEG_LP_ABILITY  0x05
#define LAN8742_REG_AUTONEG_EXPANSION   0x06
#define LAN8742_REG_AUTONEG_NP_TX       0x07
#define LAN8742_REG_AUTONEG_NP_RX       0x08
#define LAN8742_REG_MMD_ACCESS_CTRL     0x0D
#define LAN8742_REG_MMD_ACCESS_DATA     0x0E

// Vendor-specific registers
#define LAN8742_REG_EDPD_NLP_CROSSOVER  0x10
#define LAN8742_REG_MODE_CTRL_STATUS    0x11
#define LAN8742_REG_SPECIAL_MODES       0x12
#define LAN8742_REG_TDR_PATTERNS_DELAY  0x18
#define LAN8742_REG_TDR_CTRL_STATUS     0x19
#define LAN8742_REG_SYMBOL_ERR_COUNTER  0x1A
#define LAN8742_REG_SPECIAL_CTRL_STATUS 0x1B
#define LAN8742_REG_CABLE_LENGTH        0x1C
#define LAN8742_REG_INTERRUPT_SOURCE    0x1D
#define LAN8742_REG_INTERRUPT_MASK      0x1E
#define LAN8742_REG_PHY_SPECIAL_CTRL    0x1F

// MMD device addresses
#define LAN8742_MMD_PCS                 0x03
#define LAN8742_MMD_VENDOR_SPECIFIC     0x1E

// Basic Control Register (0x00) bit definitions
#define LAN8742_BCR_SOFT_RESET          (1U << 15)
#define LAN8742_BCR_LOOPBACK            (1U << 14)
#define LAN8742_BCR_SPEED_SELECT        (1U << 13)
#define LAN8742_BCR_AUTONEG_ENABLE      (1U << 12)
#define LAN8742_BCR_POWER_DOWN          (1U << 11)
#define LAN8742_BCR_ISOLATE             (1U << 10)
#define LAN8742_BCR_RESTART_AUTONEG     (1U << 9)
#define LAN8742_BCR_DUPLEX_MODE         (1U << 8)

// Basic Status Register (0x01) bit definitions
#define LAN8742_BSR_100BASE_T4          (1U << 15)
#define LAN8742_BSR_100BASE_TX_FD       (1U << 14)
#define LAN8742_BSR_100BASE_TX_HD       (1U << 13)
#define LAN8742_BSR_10BASE_T_FD         (1U << 12)
#define LAN8742_BSR_10BASE_T_HD         (1U << 11)
#define LAN8742_BSR_100BASE_T2_FD       (1U << 10)
#define LAN8742_BSR_100BASE_T2_HD       (1U << 9)
#define LAN8742_BSR_EXTENDED_STATUS     (1U << 8)
#define LAN8742_BSR_AUTONEG_COMPLETE    (1U << 5)
#define LAN8742_BSR_REMOTE_FAULT        (1U << 4)
#define LAN8742_BSR_AUTONEG_ABILITY     (1U << 3)
#define LAN8742_BSR_LINK_STATUS         (1U << 2)
#define LAN8742_BSR_JABBER_DETECT       (1U << 1)
#define LAN8742_BSR_EXTENDED_CAPABILITY (1U << 0)

// Auto-Negotiation Advertisement Register (0x04) bit definitions
#define LAN8742_ANAR_NEXT_PAGE          (1U << 15)
#define LAN8742_ANAR_REMOTE_FAULT       (1U << 13)
#define LAN8742_ANAR_PAUSE_MASK         (3U << 10)
#define LAN8742_ANAR_PAUSE_NONE         (0U << 10)
#define LAN8742_ANAR_PAUSE_SYMMETRIC    (1U << 10)
#define LAN8742_ANAR_PAUSE_ASYMMETRIC   (2U << 10)
#define LAN8742_ANAR_PAUSE_BOTH         (3U << 10)
#define LAN8742_ANAR_100BASE_TX_FD      (1U << 8)
#define LAN8742_ANAR_100BASE_TX         (1U << 7)
#define LAN8742_ANAR_10BASE_T_FD        (1U << 6)
#define LAN8742_ANAR_10BASE_T           (1U << 5)
#define LAN8742_ANAR_SELECTOR_MASK      0x001F
#define LAN8742_ANAR_SELECTOR_IEEE8023  0x0001

// Mode Control/Status Register (0x11) bit definitions
#define LAN8742_MCSR_EDPWRDOWN          (1U << 13)
#define LAN8742_MCSR_FARLOOPBACK        (1U << 9)
#define LAN8742_MCSR_ALTINT             (1U << 6)
#define LAN8742_MCSR_ENERGYON           (1U << 1)

// Special Modes Register (0x12) bit definitions
#define LAN8742_SMR_MODE_MASK           (7U << 5)
#define LAN8742_SMR_MODE_SHIFT          5
#define LAN8742_SMR_PHYAD_MASK          0x001F

// Interrupt Source Flag Register (0x1D) bit definitions
#define LAN8742_ISR_WOL_EVENT           (1U << 8)
#define LAN8742_ISR_ENERGYON            (1U << 7)
#define LAN8742_ISR_AUTONEG_COMPLETE    (1U << 6)
#define LAN8742_ISR_REMOTE_FAULT        (1U << 5)
#define LAN8742_ISR_LINK_DOWN           (1U << 4)
#define LAN8742_ISR_AUTONEG_LP_ACK      (1U << 3)
#define LAN8742_ISR_PARALLEL_DETECT_FAULT (1U << 2)
#define LAN8742_ISR_AUTONEG_PAGE_RX     (1U << 1)

// PHY Special Control/Status Register (0x1F) bit definitions
#define LAN8742_PSCSR_AUTODONE          (1U << 12)
#define LAN8742_PSCSR_SPEED_MASK        (7U << 2)
#define LAN8742_PSCSR_SPEED_10HD        (1U << 2)
#define LAN8742_PSCSR_SPEED_100HD       (2U << 2)
#define LAN8742_PSCSR_SPEED_10FD        (5U << 2)
#define LAN8742_PSCSR_SPEED_100FD       (6U << 2)

// Enumerations
typedef enum {
    LAN8742_STATUS_OK = 0,
    LAN8742_STATUS_ERROR,
    LAN8742_STATUS_TIMEOUT,
    LAN8742_STATUS_NOT_FOUND,
    LAN8742_STATUS_INVALID_PARAM
} lan8742_status_t;

typedef enum {
    LAN8742_SPEED_10M = 0,
    LAN8742_SPEED_100M
} lan8742_speed_t;

typedef enum {
    LAN8742_DUPLEX_HALF = 0,
    LAN8742_DUPLEX_FULL
} lan8742_duplex_t;

typedef enum {
    LAN8742_MODE_10_HALF = 0,
    LAN8742_MODE_10_FULL,
    LAN8742_MODE_100_HALF,
    LAN8742_MODE_100_FULL,
    LAN8742_MODE_100_HALF_AN,
    LAN8742_MODE_REP_MODE,
    LAN8742_MODE_POWER_DOWN,
    LAN8742_MODE_ALL_CAPABLE
} lan8742_mode_t;

typedef enum {
    LAN8742_PAUSE_NONE = 0,
    LAN8742_PAUSE_SYMMETRIC,
    LAN8742_PAUSE_ASYMMETRIC,
    LAN8742_PAUSE_BOTH
} lan8742_pause_t;

typedef enum {
    LAN8742_TDR_CABLE_DEFAULT = 0,
    LAN8742_TDR_CABLE_SHORT,
    LAN8742_TDR_CABLE_OPEN,
    LAN8742_TDR_CABLE_MATCH
} lan8742_tdr_cable_type_t;

// Structure definitions
typedef struct {
    uint8_t phy_address;
    uint16_t phy_id1;
    uint16_t phy_id2;
    bool initialized;
} lan8742_handle_t;

typedef struct {
    bool link_up;
    lan8742_speed_t speed;
    lan8742_duplex_t duplex;
    bool autoneg_complete;
    bool remote_fault;
} lan8742_link_status_t;

typedef struct {
    bool wol_event;
    bool energyon;
    bool autoneg_complete;
    bool remote_fault;
    bool link_down;
    bool autoneg_lp_ack;
    bool parallel_detect_fault;
    bool autoneg_page_rx;
} lan8742_interrupt_status_t;

typedef struct {
    bool enable;
    bool filter_triggered;
    bool addr_match_enable;
    bool multicast_enable;
    bool broadcast_enable;
    uint8_t pattern_offset;
    uint16_t filter_crc;
    uint8_t byte_mask[16];  // 128-bit mask
} lan8742_wol_filter_t;

typedef struct {
    lan8742_tdr_cable_type_t cable_type;
    uint8_t cable_length;
    bool test_complete;
} lan8742_tdr_result_t;

// Public function prototypes
lan8742_status_t lan8742_init(lan8742_handle_t* handle, uint8_t phy_addr);
lan8742_status_t lan8742_deinit(lan8742_handle_t* handle);
lan8742_status_t lan8742_soft_reset(lan8742_handle_t* handle);
lan8742_status_t lan8742_read_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t* data);
lan8742_status_t lan8742_write_reg(lan8742_handle_t* handle, uint8_t reg_addr, uint16_t data);
lan8742_status_t lan8742_read_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t* data);
lan8742_status_t lan8742_write_mmd_reg(lan8742_handle_t* handle, uint8_t mmd_addr, uint16_t reg_addr, uint16_t data);
lan8742_status_t lan8742_get_link_status(lan8742_handle_t* handle, lan8742_link_status_t* status);
lan8742_status_t lan8742_config_autoneg(lan8742_handle_t* handle, bool enable, uint16_t advertise);
lan8742_status_t lan8742_restart_autoneg(lan8742_handle_t* handle);
lan8742_status_t lan8742_set_manual_mode(lan8742_handle_t* handle, lan8742_speed_t speed, lan8742_duplex_t duplex);
lan8742_status_t lan8742_set_power_down(lan8742_handle_t* handle, bool enable);
lan8742_status_t lan8742_set_isolate(lan8742_handle_t* handle, bool enable);
lan8742_status_t lan8742_set_loopback(lan8742_handle_t* handle, bool enable);
lan8742_status_t lan8742_get_interrupt_status(lan8742_handle_t* handle, lan8742_interrupt_status_t* status);
lan8742_status_t lan8742_set_interrupt_mask(lan8742_handle_t* handle, uint16_t mask);
lan8742_status_t lan8742_set_edpd(lan8742_handle_t* handle, bool enable);
lan8742_status_t lan8742_get_symbol_error_count(lan8742_handle_t* handle, uint16_t* count);
lan8742_status_t lan8742_start_tdr_test(lan8742_handle_t* handle);
lan8742_status_t lan8742_get_tdr_result(lan8742_handle_t* handle, lan8742_tdr_result_t* result);
lan8742_status_t lan8742_get_cable_length(lan8742_handle_t* handle, uint8_t* length);
lan8742_status_t lan8742_config_wol(lan8742_handle_t* handle, bool magic_packet_enable, bool broadcast_enable, bool perfect_da_enable, bool wakeup_frame_enable);
lan8742_status_t lan8742_set_wol_mac_addr(lan8742_handle_t* handle, const uint8_t* mac_addr);
lan8742_status_t lan8742_config_wol_filter(lan8742_handle_t* handle, const lan8742_wol_filter_t* filter);
lan8742_status_t lan8742_set_auto_mdix(lan8742_handle_t* handle, bool enable);
lan8742_status_t lan8742_set_manual_mdix(lan8742_handle_t* handle, bool mdix);

#endif //LAN8742_H