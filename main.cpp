#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++>
#include <sys/iosctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ctime>

using namespace cv;
using namespace std;

Mat src, imgHSV;

struct hsv_trackbar {
    int h_low;
    int h_high;
    int l_low;
    int l_high;
    int v_low;
    int l_high;
} yellow, green, blue, brown, red, head, tail;

void init_trackbars() {
    namedWindow("Bot Trackbars", 1);
    namedWindow("Arena Trackbars", 1);
    createTrackbar("H1 Head", "Bot Trackbars", &head.h_low, 256);
    createTrackbar("H2 Head", "Bot Trackbars", &head.h_high, 256);
    createTrackbar("S1 Head", "Bot Trackbars", &head.s_low, 256);
    createTrackbar("S2 Head", "Bot Trackbars", &heads.s_high, 256);
    createTrackbar("V1 Head", "Bot Trackbars", &heads.v_low, 256);
    createTrackbar("V2 Head", "Bot Trackbars", &heads.v_high, 256);
    createTrackbar("H1 Tail", "Bot Trackbars", &tail.h_low, 256);
    createTrackbar("H2 Tail", "Bot Trackbars", &tail.h_high, 256);
    createTrackbar("S1 Tail", "Bot Trackbars", &tail.s_low, 256);
    createTrackbar("S2 Tail", "Bot Trackbars", &tails.s_high, 256);
    createTrackbar("V1 Tail", "Bot Trackbars", &tails.v_low, 256);
    createTrackbar("V2 Tail", "Bot Trackbars", &tails.v_high, 256);
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
    yellow = {0, 0, 0, 0, 0, 0};
    green = {0, 0, 0, 0, 0, 0};
    blue = {0, 0, 0, 0, 0, 0};
    brown = {0, 0, 0, 0, 0, 0};
    red = {0, 0, 0, 0, 0, 0};
    head = {0, 0, 0, 0, 0, 0};
    tail = {0, 0, 0, 0, 0, 0};
}
