#if defined(CONFIG_USB)
#include <drivers/uart.h>
#include <usb/usb_device.h>

static void status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
    switch(status)
    {
        case USB_DC_CONNECTED:
            LOG_INF("Connected.");
            break;
        case USB_DC_DISCONNECTED:
            LOG_INF("Disconnected.");
            break;
        default:
            LOG_DBG("status %u unhandled", status);
            break;
    }
}
#endif

void main(void)
{
    int err = 0;

    LOG_INF("USB Detect Test");
    
}