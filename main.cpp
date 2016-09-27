#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ctime>

using namespace cv;
using namespace std;

Mat src, src_gray, imgHSV, yellow_img, green_img, blue_img, brown_img, red_img, head_img, tail_img;
Mat yellow_img_gray, green_img_gray, blue_img_gray, brown_img_gray, red_img_gray, head_img_gray, tail_img_gray;
Mat yellow_img_poly, green_img_poly, blue_img_poly, brown_img_poly, red_img_poly, head_img_poly, tail_img_poly;
Mat drawing;

int binary_thresh = 100;

RNG rng(12345);

struct hsv_trackbar {
    int h_low;
    int h_high;
    int s_low;
    int s_high;
    int v_low;
    int v_high;
} yellow, green, blue, brown, red, head, tail;

/* @function thresh_callback */
void thresh_callback(int, void *);

void init_trackbars() {
    namedWindow("Bot Trackbars", 1);
    namedWindow("Arena Trackbars", 1);
    createTrackbar("H1 Head", "Bot Trackbars", &head.h_low, 256);
    createTrackbar("H2 Head", "Bot Trackbars", &head.h_high, 256);
    createTrackbar("S1 Head", "Bot Trackbars", &head.s_low, 256);
    createTrackbar("S2 Head", "Bot Trackbars", &head.s_high, 256);
    createTrackbar("V1 Head", "Bot Trackbars", &head.v_low, 256);
    createTrackbar("V2 Head", "Bot Trackbars", &head.v_high, 256);
    createTrackbar("H1 Tail", "Bot Trackbars", &tail.h_low, 256);
    createTrackbar("H2 Tail", "Bot Trackbars", &tail.h_high, 256);
    createTrackbar("S1 Tail", "Bot Trackbars", &tail.s_low, 256);
    createTrackbar("S2 Tail", "Bot Trackbars", &tail.s_high, 256);
    createTrackbar("V1 Tail", "Bot Trackbars", &tail.v_low, 256);
    createTrackbar("V2 Tail", "Bot Trackbars", &tail.v_high, 256);
    createTrackbar("H1 Yellow", "Arena Trackbars", &yellow.h_low, 256);
    createTrackbar("H2 Yellow", "Arena Trackbars", &yellow.h_high, 256);
    createTrackbar("S1 Yellow", "Arena Trackbars", &yellow.s_low, 256);
    createTrackbar("S2 Yellow", "Arena Trackbars", &yellow.s_high, 256);
    createTrackbar("V1 Yellow", "Arena Trackbars", &yellow.v_low, 256);
    createTrackbar("V2 Yellow", "Arena Trackbars", &yellow.v_high, 256);
    createTrackbar("H1 Green", "Arena Trackbars", &green.h_low, 256);
    createTrackbar("H2 Green", "Arena Trackbars", &green.h_high, 256);
    createTrackbar("S1 Green", "Arena Trackbars", &green.s_low, 256);
    createTrackbar("S2 Green", "Arena Trackbars", &green.s_high, 256);
    createTrackbar("V1 Green", "Arena Trackbars", &green.v_low, 256);
    createTrackbar("V2 Green", "Arena Trackbars", &green.v_high, 256);
    createTrackbar("H1 Blue", "Arena Trackbars", &blue.h_low, 256);
    createTrackbar("H2 Blue", "Arena Trackbars", &blue.h_high, 256);
    createTrackbar("S1 Blue", "Arena Trackbars", &blue.s_low, 256);
    createTrackbar("S2 Blue", "Arena Trackbars", &blue.s_high, 256);
    createTrackbar("V1 Blue", "Arena Trackbars", &blue.v_low, 256);
    createTrackbar("V2 Blue", "Arena Trackbars", &blue.v_high, 256);
    createTrackbar("H1 Brown", "Arena Trackbars", &brown.h_low, 256);
    createTrackbar("H2 Brown", "Arena Trackbars", &brown.h_high, 256);
    createTrackbar("S1 Brown", "Arena Trackbars", &brown.s_low, 256);
    createTrackbar("S2 Brown", "Arena Trackbars", &brown.s_high, 256);
    createTrackbar("V1 Brown", "Arena Trackbars", &brown.v_low, 256);
    createTrackbar("V2 Brown", "Arena Trackbars", &brown.v_high, 256);
    createTrackbar("H1 Red", "Arena Trackbars", &red.h_low, 256);
    createTrackbar("H2 Red", "Arena Trackbars", &red.h_high, 256);
    createTrackbar("S1 Red", "Arena Trackbars", &red.s_low, 256);
    createTrackbar("S2 Red", "Arena Trackbars", &red.s_high, 256);
    createTrackbar("V1 Red", "Arena Trackbars", &red.v_low, 256);
    createTrackbar("V2 Red", "Arena Trackbars", &red.v_high, 256);
}

void init_hsvcolor() {
    yellow.h_low = 0;
    yellow.h_high = 0;
    yellow.s_low = 0;
    yellow.s_high = 0;
    yellow.v_low = 0;
    yellow.v_high = 0;
    green.h_low = 0;
    green.h_high = 0;
    green.s_low = 0;
    green.s_high = 0;
    green.v_low = 0;
    green.v_high = 0;
    brown.h_low = 0;
    brown.h_high = 0;
    brown.s_low = 0;
    brown.s_high = 0;
    red.h_low = 0;
    red.h_high = 0;
    red.s_low = 0;
    red.s_high = 0;
    red.v_low = 0;
    red.v_high = 0;
    head.h_low = 0;
    head.h_high = 0;
    head.s_low = 0;
    head.s_high = 0;
    head.v_low = 0;
    head.v_high = 0;
    tail.h_low = 0;
    tail.h_high = 0;
    tail.s_low = 0;
    tail.s_high = 0;
    tail.v_low = 0;
    tail.v_high = 0;
}

int main(int argc, const char **argv) {
    VideoCapture cap(1);

    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

    init_trackbars();
    init_hsvcolor();

    while(1) {
        cap >> src;
        cvtColor(src, imgHSV, CV_BGR2HSV);
        inRange(imgHSV, Scalar(yellow.h_low, yellow.s_low, yellow.v_low),
                Scalar(yellow.h_high, yellow.s_high, yellow.v_high), yellow_img);
        inRange(imgHSV, Scalar(green.h_low, green.s_low, green.v_low),
                Scalar(green.h_high, green.s_high, green.v_high), green_img);
        inRange(imgHSV, Scalar(blue.h_low, blue.s_low, blue.v_low),
                Scalar(blue.h_high, blue.s_high, blue.v_high), blue_img);
        inRange(imgHSV, Scalar(brown.h_low, brown.s_low, brown.v_low),
                Scalar(brown.h_high, brown.s_high, brown.v_high), brown_img);
        inRange(imgHSV, Scalar(red.h_low, red.s_low, red.v_low),
                Scalar(red.h_high, red.s_high, red.v_high), red_img);
        inRange(imgHSV, Scalar(head.h_low, head.s_low, head.v_low),
                Scalar(head.h_high, head.s_high, head.v_high), head_img);
        inRange(imgHSV, Scalar(tail.h_low, tail.s_low, tail.v_low),
                Scalar(tail.h_high, tail.s_high, tail.v_high), tail_img);

        imshow("Source", src);
        imshow("Head", head_img);
        imshow("Tail", tail_img);
        imshow("Yellow", yellow_img);
        imshow("Green", green_img);
        imshow("Blue", blue_img);
        imshow("Brown", brown_img);
        imshow("Red", red_img);

        createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        thresh_callback(0, 0);
    }

    return 0;
}

void thresh_callback(int, void *) {
    Mat threshold_output;
    vector<vector<Point> > head_contours, tail_contours, yellow_contours, green_contours, blue_contours, brown_contours, red_contours;
    vector<Vec4i> head_hierarchy, tail_hierarchy, yellow_hierarchy, green_hierarchy, blue_hierarchy, brown_hierarchy, red_hierarchy;

    /* Detect edges in all using Threshold */
    threshold(head_img, head_img_gray, 100, 255, THRESH_BINARY);
    threshold(tail_img, tail_img_gray, 100, 255, THRESH_BINARY);
    threshold(yellow_img, yellow_img_gray, 100, 255, THRESH_BINARY);
    threshold(green_img, green_img_gray, 100, 255, THRESH_BINARY);
    threshold(brown_img, brown_img_gray, 100, 255, THRESH_BINARY);
    threshold(red_img, red_img_gray, 100, 255, THRESH_BINARY);

    /* Find contours */
    findContours(head_img_gray, head_contours, head_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(tail_img_gray, tail_contours, tail_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(yellow_img_gray, yellow_contours, yellow_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(green_img_gray, green_contours, green_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(brown_img_gray, brown_contours, brown_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(red_img_gray, red_contours, red_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    vector<vector<Point> > head_contours_poly(head_contours.size());
    vector<vector<Point> > tail_contours_poly(tail_contours.size());
    vector<vector<Point> > yellow_contours_poly(yellow_contours.size());
    vector<vector<Point> > green_contours_poly(green_contours.size());
    vector<vector<Point> > brown_contours_poly(brown_contours.size());
    vector<vector<Point> > red_contours_poly(red_contours.size());

    vector<Rect> headBoundRect(head_contours.size());
    vector<Rect> tailBoundRect(tail_contours.size());
    vector<Rect> yellowBoundRect(yellow_contours.size());
    vector<Rect> greenBoundRect(green_contours.size());
    vector<Rect> brownBoundRect(brown_contours.size());
    vector<Rect> redBoundRect(red_contours.size());

    for (int i = 0; i < head_contours.size(); i++) {
        approxPolyDP(Mat(head_contours[i]), head_contours_poly[i], 3, true);
        headBoundRect[i] = boundingRect(Mat(head_contours_poly[i]));
    }

    for (int i = 0; i < tail_contours.size(); i++) {
        approxPolyDP(Mat(tail_contours[i]), tail_contours_poly[i], 3, true);
        tailBoundRect[i] = boundingRect(Mat(tail_contours_poly[i]));
    }

    for (int i = 0; i < yellow_contours.size(); i++) {
        approxPolyDP(Mat(yellow_contours[i]), yellow_contours_poly[i], 3, true);
        yellowBoundRect[i] = boundingRect(Mat(yellow_contours_poly[i]));
    }

    for (int i = 0; i < green_contours.size(); i++) {
        approxPolyDP(Mat(green_contours[i]), green_contours_poly[i], 3, true);
        greenBoundRect[i] = boundingRect(Mat(green_contours_poly[i]));
    }

    for (int i = 0; i < brown_contours.size(); i++) {
        approxPolyDP(Mat(brown_contours[i]), brown_contours_poly[i], 3, true);
        brownBoundRect[i] = boundingRect(Mat(brown_contours_poly[i]));
    }

    for (int i = 0; i < red_contours.size(); i++) {
        approxPolyDP(Mat(red_contours[i]), red_contours_poly[i], 3, true);
        redBoundRect[i] = boundingRect(Mat(red_contours_poly[i]));
    }

    drawing = Mat::zeros(src_gray.size(), CV_8UC3);
    for (int i = 0; i < head_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, head_contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        rectangle(drawing, headBoundRect[i].tl(), headBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < tail_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, tail_contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        rectangle(drawing, tailBoundRect[i].tl(), tailBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < yellow_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        drawContours(drawing, yellow_contours_poly, i, color, 1, 8, vector<Vec4i>(), 0, Point());
        rectangle(drawing, yellowBoundRect[i].tl(), yellowBoundRect[i].br(), color, 2, 8, 0);
    }

    namedWindow("Contours", CV_WINDOW_AUTOSIZE);
    imshow("Contours", drawing);
}
