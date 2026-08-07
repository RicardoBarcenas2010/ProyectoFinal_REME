#ifndef VISION_H
#define VISION_H

#ifdef __cplusplus
extern "C" {
#endif

void init_vision(void);
void vision_task(void *pvParameters);
float vision_obtener_ultimo_angulo(void);
int vision_obtener_contador(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_H */