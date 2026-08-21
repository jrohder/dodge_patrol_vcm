/**
 * @file usb_console.h
 * @brief Line-oriented commands on USB CDC / UART0 for bench calibration.
 *
 * Type `help` then Enter. Used so the VCM can be calibrated over the
 * CH343 UART without joining the Wi-Fi AP.
 */
#pragma once

namespace vcm {

void pollUsbConsole();

}  // namespace vcm
