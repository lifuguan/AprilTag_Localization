#include <iostream>
#include <omp.h>
#include "opencv2/opencv.hpp"

extern "C"
{
#include "apriltag_pose.h"
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
}

static omp_lock_t lock;

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    omp_init_lock(&lock);

    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tag36h11", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");

    if (!getopt_parse(getopt, argc, argv, 1) ||
        getopt_get_bool(getopt, "help"))
    {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    // Initialize tag detector with options
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11"))
    {
        tf = tag36h11_create();
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tf = tag25h9_create();
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        tf = tag16h5_create();
    }
    else if (!strcmp(famname, "tagCircle21h7"))
    {
        tf = tagCircle21h7_create();
    }
    else if (!strcmp(famname, "tagCircle49h12"))
    {
        tf = tagCircle49h12_create();
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        tf = tagStandard41h12_create();
    }
    else if (!strcmp(famname, "tagStandard52h13"))
    {
        tf = tagStandard52h13_create();
    }
    else if (!strcmp(famname, "tagCustom48h12"))
    {
        tf = tagCustom48h12_create();
    }
    else
    {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    apriltag_detector_t *td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    apriltag_detection_info_t info;
    info.tagsize = 0.1; //打印的qrcode尺寸大小
    info.fx = 594.295;
    info.fy = 584.958;
    info.cx = 310.907;
    info.cy = 117.003;

    Mat frame, gray;

    int cnt = 0;
    // Initialize camera
    VideoCapture capture;
    capture.open("formal3.mp4");
    if (!capture.isOpened())
    {
        cerr << "Couldn't open video capture device . " << endl;
        return -1;
    }
    while (true)
    {
        capture.read(frame);
        cv::resize(frame, frame, Size(640, 480));
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        cv::imshow("frame", frame);
        // Make an image_u8_t header for the Mat data
        image_u8_t im = {.width = gray.cols,
                         .height = gray.rows,
                         .stride = gray.cols,
                         .buf = gray.data};

        zarray_t *detections = apriltag_detector_detect(td, &im);
        cout << zarray_size(detections) << " tags detected" << endl;

        // Draw detection outlines
        int i = 0;
        int n = zarray_size(detections);
#pragma omp parallel shared(n) private(i)
        {
#pragma omp for
            for (i = 0; i < n; i++)
            {
                cout << "n =  " << n << ". In thread " << omp_get_thread_num() << ". Execute loop " << i << ". " << endl;
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);

                info.det = det;
                apriltag_pose_t pose;
                double err = estimate_tag_pose(&info, &pose);
                cout << "3D pose: x = " << pose.t->data[0] << "; y = " << pose.t->data[1] << "; z = " << pose.t->data[2] << endl;
                cout << pose.R->nrows << " " << pose.R->ncols << endl;

                stringstream s_;
                s_ << setprecision(2) << pose.t->data[0] << ", " << setprecision(2) << pose.t->data[1] << ", " << setprecision(2) << pose.t->data[2];
                putText(frame, s_.str(), Point(det->p[0][0], det->p[0][1]), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);
                
                line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[1][0], det->p[1][1]),
                     Scalar(0, 0xff, 0), 2);
                line(frame, Point(det->p[0][0], det->p[0][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0, 0, 0xff), 2);
                line(frame, Point(det->p[1][0], det->p[1][1]),
                     Point(det->p[2][0], det->p[2][1]),
                     Scalar(0xff, 0, 0), 2);
                line(frame, Point(det->p[2][0], det->p[2][1]),
                     Point(det->p[3][0], det->p[3][1]),
                     Scalar(0xff, 0, 0), 2);

                stringstream ss;
                ss << det->id;
                String text = ss.str();
                int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
                double fontscale = 1.0;
                int baseline;
                Size textsize = getTextSize(text, fontface, fontscale, 2,
                                            &baseline);
                putText(frame, text, Point(det->c[0] - textsize.width / 2, det->c[1] + textsize.height / 2),
                        fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
            }
        }
        cv::imshow("frame", frame);
        cnt++;
        cout << "cnt : " << cnt << endl;
        apriltag_detections_destroy(detections);
        if (waitKey(50) == 'q')
        {
            break;
        }
    }
    apriltag_detector_destroy(td);
    if (!strcmp(famname, "tag36h11"))
    {
        tag36h11_destroy(tf);
    }
    else if (!strcmp(famname, "tag25h9"))
    {
        tag25h9_destroy(tf);
    }
    else if (!strcmp(famname, "tag16h5"))
    {
        tag16h5_destroy(tf);
    }
    else if (!strcmp(famname, "tagCircle21h7"))
    {
        tagCircle21h7_destroy(tf);
    }
    else if (!strcmp(famname, "tagCircle49h12"))
    {
        tagCircle49h12_destroy(tf);
    }
    else if (!strcmp(famname, "tagStandard41h12"))
    {
        tagStandard41h12_destroy(tf);
    }
    else if (!strcmp(famname, "tagStandard52h13"))
    {
        tagStandard52h13_destroy(tf);
    }
    else if (!strcmp(famname, "tagCustom48h12"))
    {
        tagCustom48h12_destroy(tf);
    }

    getopt_destroy(getopt);

    return 0;
}
