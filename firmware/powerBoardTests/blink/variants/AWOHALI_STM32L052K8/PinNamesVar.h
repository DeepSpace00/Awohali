/* SYS_WKUP */
#ifdef PWR_WAKEUP_PIN1
  SYS_WKUP1 = PA_0,
#endif

/* USB */
#ifdef USBCON
  USB_DM  = PA_11,
  USB_DP  = PA_12,
  USB_NOE = PA_13,
#endif