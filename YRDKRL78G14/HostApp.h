#ifndef HOST_APP_H_
#define HOST_APP_H_

#include <drv\SPI.h>

#define VERSION_MAJOR       2
#define VERSION_MINOR       0
#define VERSION_TEXT        "1.0"

/* Set the UART rate to the Console: */
#define GAINSPAN_CONSOLE_BAUD        115200
#define GAINSPAN_SERIAL_PORT_BAUD    9600

//#define ATLIBGS_DEBUG_ENABLE       // output information on the serial port to PC

// Choose one of the following:  SPI or UART communications
// NOTE that the GainSpan module requires the correct firmware to be loaded.
#define  ATLIBGS_INTERFACE_SPI      /* SPI interface is used for GS1011 S2w App communication */
//#define  ATLIBGS_INTERFACE_UART   /* UART interface is used for GS1011 S2w App communication */

#define GAINSPAN_SPI_CHANNEL      SPI_APPHEADER_CHANNEL  /* Use this line if you connect to Application Header (J6) */
//#define GAINSPAN_SPI_CHANNEL    SPI_PMOD1_CHANNEL   /* Use this line if you connect to PMOD1 (J11) */
//#define GAINSPAN_SPI_CHANNEL    SPI_PMOD2_CHANNEL   /* Use this line if you connect to PMOD1 (J11) */

/* Set the SPI rate to the GainSpan module: */
#define GAINSPAN_SPI_RATE            11500 // 11500 312500   // Max 857142

/* Set the UART rate to the GainSpan module: */
#define GAINSPAN_UART_BAUD           9600

/* MAC Address of the S2W Node  */
#define ATLIBGS_GS_NODE_MAC_ID      "00:1D:C9:01:01:D0"

#define ATLIBGS_AP_SSID             ""
#define ATLIBGS_AP_CHANNEL          NULL //  "6"

//#define ATLIBGS_LIMITED_PROVISION_AP_SSID   "ProvisionAP"
//#define ATLIBGS_LIMITED_DOWNLOADAP_SSID     "DownloadAP"
#define ATLIBGS_LIMITED_AP_CHANNEL   9  // use 0 for any
#define ATLIBGS_LIMITED_AP_IP       "192.168.240.1"
#define ATLIBGS_LIMITED_AP_MASK     "255.255.255.0"
#define ATLIBGS_LIMITED_AP_GATEWAY  "192.168.240.1"

#define ATLIBGS_LIMITED_AP_SSID     "RDK_"
#define ATLIBGS_ADK_CHANNEL         6
#define ATLIBGS_ADK_IP              "192.168.240.1"
#define ATLIBGS_ADK_MASK            "255.255.255.0"
#define ATLIBGS_ADK_GATEWAY         "192.168.240.1"

#define ATLIBGS_ADK_MDNS_SERVER          "Renesas1"      // "NXP_WPG_1"
#define ATLIBGS_ADK_MDNS_SERVER_RPOV     "provisioning"
#define ATLIBGS_ADK_MDNS_SERVER_OTAFU    "ota-fwu"

#define ATLIBGS_FWUPGRADE_SERVER    "192.168.0.101"

/* Access Point (AP) channel here. For auto channel, AP channel is NULL  */
#define HOST_APP_AP_CHANNEL          NULL

// Enable one of the securities below 
//-----------------------------------------------------------------------------
/*  demo WPA security  */
#define HOST_APP_WPA2

void ConsolePrintf(const char *format, ...);

#endif

