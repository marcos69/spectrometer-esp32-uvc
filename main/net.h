#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Start the network: NVS creds -> STA connect (20 s timeout) -> AP fallback.
 * Non-blocking: returns immediately, AP/IP state is polled by the web API. */
esp_err_t net_start(void);

/* "sta", "sta-failed", "ap" */
const char *net_mode_str(void);
/* IPv4 string of the active interface ("-") */
const char *net_ip_str(void);

/* Save credentials to NVS and (re)connect STA. Call from web /api/net. */
void net_set_credentials(const char *ssid, const char *pass);
void net_reconnect_sta(void);

#ifdef __cplusplus
}
#endif
