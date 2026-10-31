#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct {
    int x1;
    int y1;
    int x2;
    int y2;
} face_detection_box_t;

#ifdef __cplusplus
extern "C" {
#endif

void *face_detection_create(void);
size_t face_detection_run(void *detector, const uint16_t *pixels,
                          uint16_t width, uint16_t height,
                          face_detection_box_t *boxes, size_t box_capacity);

#ifdef __cplusplus
}
#endif
