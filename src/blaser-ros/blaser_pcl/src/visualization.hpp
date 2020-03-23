#ifndef __VISUALIZATION__HPP__
#define __VISUALIZATION__HPP__

#include <opencv2/opencv.hpp>

#include "configuration.hpp"

/* Debug configurations and macros. */
#define COLLAGE_VIS_OFFSET_CORNER (10)

#define COLLAGE_VIS_WS_ROWS (2)

#define COLLAGE_VIS_WS_COLS (2)

// OpenCV putText method specifies bottom-left of text drawn.
#define COLLAGE_VIS_TXTPOS_OFFSET_C(WSC, W) (W * WSC + \
    COLLAGE_VIS_OFFSET_CORNER)

#define COLLAGE_VIS_TXTPOS_OFFSET_R(WSR, H) (H * (WSR + 1) - \
    COLLAGE_VIS_OFFSET_CORNER)

#define COLLAGE_VIS_TXTPOS_OFFSET_PT(WSR, WSC, H, W) (cv::Point(\
    COLLAGE_VIS_TXTPOS_OFFSET_C(WSC, W), COLLAGE_VIS_TXTPOS_OFFSET_R(WSR, H)))

#define COLLAGE_VIS_WS_OFFSET_R(WSR, H) (WSR * H)

#define COLLAGE_VIS_WS_OFFSET_C(WSC, W) (WSC * W)

#define COLLAGE_TXT_COLOR (cv::Scalar(255,255,255))

class CollageManager {
    public:
        /* Initializes collage manager. */
        CollageManager(int rows, int cols, int height, int width);

        /* Register a frame to be drawn onto collage at given location. */
        int registerFrameAtWorkspace(int r, int c, const cv::Mat& frame);

        /* Unregisters frame from collage, returns non-zero on failure. */
        int unregisterFrameAtWorkspace(int r, int c);

        /* Displays given text at workspace (bottom-left corner) */
        int displayTextAtWorkspace(int r, int c, const std::string text);

        /* Renders all registered frame to collage buffer. */
        cv::Mat& resolve();

    private:
        int rows;
        int cols;
        int height;
        int width;
};

#endif /* __VISUALIZATION__HPP__ */