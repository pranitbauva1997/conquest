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
Mat yellow_drawing, green_drawing, blue_drawing, brown_drawing, red_drawing, head_drawing, tail_drawing;

int binary_thresh = 100;

/* Path planning variables */

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
    namedWindow("Yellow Trackbars", 1);
    namedWindow("Green Trackbars", 1);
    namedWindow("Blue Trackbars", 1);
    namedWindow("Brown Trackbars", 1);
    namedWindow("Red Trackbars", 1);
    createTrackbar("H1 Head", "Bot Trackbars", &head.h_low, 255);
    createTrackbar("H2 Head", "Bot Trackbars", &head.h_high, 255);
    createTrackbar("S1 Head", "Bot Trackbars", &head.s_low, 255);
    createTrackbar("S2 Head", "Bot Trackbars", &head.s_high, 255);
    createTrackbar("V1 Head", "Bot Trackbars", &head.v_low, 255);
    createTrackbar("V2 Head", "Bot Trackbars", &head.v_high, 255);
    createTrackbar("H1 Tail", "Bot Trackbars", &tail.h_low, 255);
    createTrackbar("H2 Tail", "Bot Trackbars", &tail.h_high, 255);
    createTrackbar("S1 Tail", "Bot Trackbars", &tail.s_low, 255);
    createTrackbar("S2 Tail", "Bot Trackbars", &tail.s_high, 255);
    createTrackbar("V1 Tail", "Bot Trackbars", &tail.v_low, 255);
    createTrackbar("V2 Tail", "Bot Trackbars", &tail.v_high, 255);
    createTrackbar("H1 Yellow", "Yellow Trackbars", &yellow.h_low, 255);
    createTrackbar("H2 Yellow", "Yellow Trackbars", &yellow.h_high, 255);
    createTrackbar("S1 Yellow", "Yellow Trackbars", &yellow.s_low, 255);
    createTrackbar("S2 Yellow", "Yellow Trackbars", &yellow.s_high, 255);
    createTrackbar("V1 Yellow", "Yellow Trackbars", &yellow.v_low, 255);
    createTrackbar("V2 Yellow", "Yellow Trackbars", &yellow.v_high, 255);
    createTrackbar("H1 Green", "Green Trackbars", &green.h_low, 255);
    createTrackbar("H2 Green", "Green Trackbars", &green.h_high, 255);
    createTrackbar("S1 Green", "Green Trackbars", &green.s_low, 255);
    createTrackbar("S2 Green", "Green Trackbars", &green.s_high, 255);
    createTrackbar("V1 Green", "Green Trackbars", &green.v_low, 255);
    createTrackbar("V2 Green", "Green Trackbars", &green.v_high, 255);
    createTrackbar("H1 Blue", "Blue Trackbars", &blue.h_low, 255);
    createTrackbar("H2 Blue", "Blue Trackbars", &blue.h_high, 255);
    createTrackbar("S1 Blue", "Blue Trackbars", &blue.s_low, 255);
    createTrackbar("S2 Blue", "Blue Trackbars", &blue.s_high, 255);
    createTrackbar("V1 Blue", "Blue Trackbars", &blue.v_low, 255);
    createTrackbar("V2 Blue", "Blue Trackbars", &blue.v_high, 255);
    createTrackbar("H1 Brown", "Brown Trackbars", &brown.h_low, 255);
    createTrackbar("H2 Brown", "Brown Trackbars", &brown.h_high, 255);
    createTrackbar("S1 Brown", "Brown Trackbars", &brown.s_low, 255);
    createTrackbar("S2 Brown", "Brown Trackbars", &brown.s_high, 255);
    createTrackbar("V1 Brown", "Brown Trackbars", &brown.v_low, 255);
    createTrackbar("V2 Brown", "Brown Trackbars", &brown.v_high, 255);
    createTrackbar("H1 Red", "Red Trackbars", &red.h_low, 255);
    createTrackbar("H2 Red", "Red Trackbars", &red.h_high, 255);
    createTrackbar("S1 Red", "Red Trackbars", &red.s_low, 255);
    createTrackbar("S2 Red", "Red Trackbars", &red.s_high, 255);
    createTrackbar("V1 Red", "Red Trackbars", &red.v_low, 255);
    createTrackbar("V2 Red", "Red Trackbars", &red.v_high, 255);
}

void init_hsvcolor() {
    yellow.h_low = 4;
    yellow.h_high = 58;
    yellow.s_low = 97;
    yellow.s_high = 255;
    yellow.v_low = 0;
    yellow.v_high = 255;
    green.h_low = 0;
    green.h_high = 0;
    green.s_low = 0;
    green.s_high = 0;
    green.v_low = 0;
    green.v_high = 0;
    brown.h_low = 0;
    brown.h_high = 17;
    brown.s_low = 43;
    brown.s_high = 94;
    brown.v_low = 188;
    brown.v_high = 217;
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
    blue.h_low = 47;
    blue.h_high = 100;
    blue.s_low = 0;
    blue.s_high = 18;
    blue.v_low = 235;
    blue.v_high = 255;
}

int main(int argc, const char **argv) {
    VideoCapture cap(0);

    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

    init_hsvcolor();
    init_trackbars();

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

        // createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        // thresh_callback(0, 0);
        waitKey(70);
    }

    return 0;
}

void thresh_callback(int, void *) {
}
