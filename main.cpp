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
Mat yellow_img_poly, green_img_poly, blue_img_poly, brown_img_poly, red_img_poly, head_img_poly, tail_img_poly;
Mat yellow_drawing, green_drawing, blue_drawing, brown_drawing, red_drawing, head_drawing, tail_drawing;
Mat yellow_output, blue_output, head_output, tail_output, brown_output;
vector<vector<Point> > head_contours, tail_contours, yellow_contours, green_contours, blue_contours, brown_contours, red_contours;
vector<Vec4i> head_hierarchy, tail_hierarchy, yellow_hierarchy, green_hierarchy, blue_hierarchy, brown_hierarchy, red_hierarchy;
vector<vector<Point> > head_contours_poly;
vector<vector<Point> > tail_contours_poly;
vector<vector<Point> > yellow_contours_poly;
vector<vector<Point> > green_contours_poly;
vector<vector<Point> > blue_contours_poly;
vector<vector<Point> > brown_contours_poly;
vector<vector<Point> > red_contours_poly;
vector<Rect> headBoundRect;
vector<Rect> tailBoundRect;
vector<Rect> yellowBoundRect;
vector<Rect> greenBoundRect;
vector<Rect> blueBoundRect;
vector<Rect> brownBoundRect;
vector<Rect> redBoundRect;

int binary_thresh = 100;
int initial = 0;

/* Path planning variables */

enum state {
    START_POINT = 1,
    IN_BETWEEN_PATH,
    END_POINT,
    BLINK_LED
};

vector<Point> target_resources;
Point town_centre, bot_position;
Point start_point, end_point, current_point;

void get_path();
void move_bot();

Mat path_img;

RNG rng(12345);

/* Arduino code */
int arduino = -1;
void sendCommand(const char *command);
void init_arduino(const char *file);

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
    namedWindow("Blue Trackbars", 1);
    namedWindow("Brown Trackbars", 1);
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
}

void init_hsvcolor() {
    yellow.h_low = 27;
    yellow.h_high = 39;
    yellow.s_low = 60;
    yellow.s_high = 255;
    yellow.v_low = 234;
    yellow.v_high = 255;
    green.h_low = 0;
    green.h_high = 0;
    green.s_low = 0;
    green.s_high = 0;
    green.v_low = 0;
    green.v_high = 0;
    brown.h_low = 10;
    brown.h_high = 16;
    brown.s_low = 55;
    brown.s_high = 85;
    brown.v_low = 188;
    brown.v_high = 210;
    red.h_low = 0;
    red.h_high = 0;
    red.s_low = 0;
    red.s_high = 0;
    red.v_low = 0;
    red.v_high = 0;
    head.h_low = 109;
    head.h_high = 139;
    head.s_low = 66;
    head.s_high = 97;
    head.v_low = 183;
    head.v_high = 255;
    tail.h_low = 0;
    tail.h_high = 13;
    tail.s_low = 0;
    tail.s_high = 182;
    tail.v_low = 219;
    tail.v_high = 255;
    blue.h_low = 75;
    blue.h_high = 85;
    blue.s_low = 0;
    blue.s_high = 18;
    blue.v_low = 235;
    blue.v_high = 255;
}

int main(int argc, const char **argv) {
    VideoCapture cap(1);

    if (argc < 2) {
        fprintf(stderr, "Please enter the ardunio dev file\n");
        return -1;
    }
    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

    init_arduino(argv[0]);
    init_hsvcolor();
    //init_trackbars();

    state status = START_POINT;

    while(1) {
        cap >> src;
        if (status == START_POINT)
            path_img = Mat::zeros(src.size(), CV_8UC3);
        cvtColor(src, imgHSV, CV_BGR2HSV);
        inRange(imgHSV, Scalar(yellow.h_low, yellow.s_low, yellow.v_low),
                Scalar(yellow.h_high, yellow.s_high, yellow.v_high), yellow_img);
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
        //imshow("Head", head_img);
        //imshow("Tail", tail_img);
        //imshow("Yellow", yellow_img);
        //imshow("Green", green_img);
        //imshow("Blue", blue_img);
        //imshow("Brown", brown_img);
        //imshow("Red", red_img);

        /* Compute Path and move bot's ass
        if (status == IN_BETWEEN_PATH) {
            continue_moving();
        } else if (status == START_POINT) {
            get_path_to_start_point();
            status == IN_BETWEEN_PATH;
        } else if (status == RESOURCE) {
            get_path_to_end_point();
            status = IN_BETWEEN_PATH;
        }

        check_status(); */

        createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        thresh_callback(0, 0);
        if (status == START_POINT) {
            get_path();
            move_bot();
        }
        imshow("RRT Path", path_img);
        waitKey(30);
    }

    return 0;
}

void get_path() {

}

void move_bot() {

}

void sendCommand(const char *command) {
    write(arduino, command, 1);
    printf("sending: %s\n", command);
}

void init_arduino(const char *file) {
    arduino = open(file, O_RDWR || O_NOCTTY);
    if (arduino)
        exit(255);
}

/*
void check_status() {
    if (distance_(&bot_position, &current_target) < 20) {
        status = BLINK_LED;
        blink_led();
    }
}*/

/*
void continue_moving() {
    int x = current_target.x - bot_position.x;
    int y = current_target.y - bot_position.y;
    if (x > 0 && y > 0)
        go_left();
    if (x > 0 && y < 0)
        go_right();
}*/

void thresh_callback(int, void *) {

    threshold(yellow_img, yellow_output, binary_thresh, 255, THRESH_BINARY);
    threshold(blue_img, blue_output, binary_thresh, 255, THRESH_BINARY);
    threshold(head_img, head_output, binary_thresh, 255, THRESH_BINARY);
    threshold(tail_img, tail_output, binary_thresh, 255, THRESH_BINARY);
    threshold(brown_img, brown_output, binary_thresh, 255, THRESH_BINARY);

    /* Find contours */
    findContours(head_output, head_contours, head_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(tail_output, tail_contours, tail_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(yellow_output, yellow_contours, yellow_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(brown_output, brown_contours, brown_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(blue_output, blue_contours, blue_hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    head_contours_poly.resize(head_contours.size());
    tail_contours_poly.resize(tail_contours.size());
    yellow_contours_poly.resize(yellow_contours.size());
    blue_contours_poly.resize(blue_contours.size());
    brown_contours_poly.resize(brown_contours.size());

    for (int i = 0; i < head_contours.size(); i++) {
        approxPolyDP(Mat(head_contours[i]), head_contours_poly[i], 3, true);
        if (contourArea(head_contours[i]) > 100)
            headBoundRect.push_back(boundingRect(Mat(head_contours_poly[i])));
    }

    for (int i = 0; i < tail_contours.size(); i++) {
        approxPolyDP(Mat(tail_contours[i]), tail_contours_poly[i], 3, true);
        if (contourArea(tail_contours[i]) > 500)
            tailBoundRect.push_back(boundingRect(Mat(tail_contours_poly[i])));
    }

    for (int i = 0; i < yellow_contours.size(); i++) {
        approxPolyDP(Mat(yellow_contours[i]), yellow_contours_poly[i], 3, true);
        if (contourArea(yellow_contours[i]) > 100)
            yellowBoundRect.push_back(boundingRect(Mat(yellow_contours_poly[i])));
    }

    for (int i = 0; i < green_contours.size(); i++) {
        approxPolyDP(Mat(green_contours[i]), green_contours_poly[i], 3, true);
        if (contourArea(green_contours[i]) > 100)
            greenBoundRect.push_back(boundingRect(Mat(green_contours_poly[i])));
    }

    for (int i = 0; i < blue_contours.size(); i++) {
        approxPolyDP(Mat(blue_contours[i]), blue_contours_poly[i], 3, true);
        if (contourArea(blue_contours[i]) > 100)
            blueBoundRect.push_back(boundingRect(Mat(blue_contours_poly[i])));
    }

    for (int i = 0; i < brown_contours.size(); i++) {
        approxPolyDP(Mat(brown_contours[i]), brown_contours_poly[i], 3, true);
        if (contourArea(brown_contours[i]) > 500)
            brownBoundRect.push_back(boundingRect(Mat(brown_contours_poly[i])));
    }

    head_drawing = Mat::zeros(src.size(), CV_8UC3);
    tail_drawing = Mat::zeros(src.size(), CV_8UC3);
    green_drawing = Mat::zeros(src.size(), CV_8UC3);
    blue_drawing = Mat::zeros(src.size(), CV_8UC3);
    brown_drawing = Mat::zeros(src.size(), CV_8UC3);
    yellow_drawing = Mat::zeros(src.size(), CV_8UC3);

    for (int i = 0; i < head_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(head_drawing, headBoundRect[i].tl(), headBoundRect[i].br(), color, 2, 8, 0);
        break;
    }
    for (int i = 0; i < tail_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(tail_drawing, tailBoundRect[i].tl(), tailBoundRect[i].br(), color, 2, 8, 0);
        break;
    }
    for (int i = 0; i < yellow_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(yellow_drawing, yellowBoundRect[i].tl(), yellowBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (yellowBoundRect[i].tl() + yellowBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (!initial)
            target_resources.push_back(centre_rect);
    }
    for (int i = 0; i < blue_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(blue_drawing, blueBoundRect[i].tl(), blueBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < brown_contours.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(brown_drawing, brownBoundRect[i].tl(), brownBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (brownBoundRect[i].tl() + brownBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (centre_rect.x != 0 && centre_rect.y != 0) {
            town_centre = brownBoundRect[i].tl();
            break;
        }
    }
    initial++;
    end_point = target_resources[0];
    for (int i = -5; i < 5; i++) {
        for (int j = -5; j < 5; j++) {
            path_img.at<Vec3b>(i + town_centre.x, j + town_centre.y) = {255, 255, 255};
        }
    }

    imshow("Head Contours", head_drawing);
    imshow("Tail Contours", tail_drawing);
    imshow("Yellow Contours", yellow_drawing);
    imshow("Blue Contours", blue_drawing);
    imshow("Brown Contours", brown_drawing);
}
