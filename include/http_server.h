#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────────────────────
 *  VARIABLES EXTERNAS
 * ────────────────────────────────────────────────────────────── */

extern float ultimo_angulo;
extern int contador_peticiones;

/* ──────────────────────────────────────────────────────────────
 *  FUNCIONES
 * ────────────────────────────────────────────────────────────── */

httpd_handle_t start_webserver(void);
void stop_webserver(httpd_handle_t server);
float get_ultimo_angulo(void);
int get_contador_peticiones(void);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_SERVER_H */