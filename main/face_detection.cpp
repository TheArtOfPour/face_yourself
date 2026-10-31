#include <new>

#include "dl_image_define.hpp"
#include "face_detection.h"
#include "human_face_detect.hpp"

extern "C" void *face_detection_create(void)
{
    auto *detector = new (std::nothrow) HumanFaceDetect(
        HumanFaceDetect::ESPDET_PICO_224_224_FACE, false);
    if (detector != nullptr) {
        detector->set_score_thr(0.30f);
    }
    return detector;
}

extern "C" size_t face_detection_run(void *detector, const uint16_t *pixels,
                                      uint16_t width, uint16_t height,
                                      face_detection_box_t *boxes, size_t box_capacity)
{
    if (detector == nullptr || pixels == nullptr || boxes == nullptr || box_capacity == 0) {
        return 0;
    }

    dl::image::img_t image = {};
    image.data = const_cast<uint16_t *>(pixels);
    image.width = width;
    image.height = height;
    image.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB565BE;

    auto *face_detector = static_cast<HumanFaceDetect *>(detector);
    const auto &results = face_detector->run(image);
    size_t box_count = 0;
    for (const auto &result : results) {
        if (result.box.size() < 4 || box_count >= box_capacity) {
            continue;
        }
        boxes[box_count] = {
            .x1 = result.box[0],
            .y1 = result.box[1],
            .x2 = result.box[2],
            .y2 = result.box[3],
        };
        box_count++;
    }
    return box_count;
}
