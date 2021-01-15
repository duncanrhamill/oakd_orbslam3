/**
 * @file util.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief General utilities for the experiments.
 * @version 0.1
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) Duncan Hamill 2021
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <opencv2/opencv.hpp>

#include "depthai/depthai.hpp"

/* -------------------------------------------------------------------------
 * FUNCTIONS
 * ------------------------------------------------------------------------- */

cv::Mat imgframe_to_mat(
    std::shared_ptr<dai::ImgFrame> frame, 
    int data_type=CV_8UC1
) {
    return cv::Mat(
        frame->getHeight(), 
        frame->getWidth(), 
        data_type, 
        frame->getData().data()
    );
}


