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

VideoCapture cap(0);

int binary_thresh = 100;
int initial = 0;

/* Path planning variables */

enum state {
    START_POINT = 1,
    IN_BETWEEN_PATH,
    END_POINT,
    BLINK_LED
} status = START_POINT;

vector<Point> target_resources;
Point town_centre, bot_position;
Point start_point, end_point, current_point;
Point head_point, tail_point;

void get_path();
void move_bot();

Mat path_img;

RNG rng(12345);

/* Arduino code */
int arduino = -1;
void sendCommand(const char *command);
void init_arduino(const char *file);
int dist(Point a, Point b);
float angle_between_4(Point a, Point b, Point c, Point d);
float angle_between_3(Point a, Point b, Point c);

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
    yellow.h_low = 16;
    yellow.h_high = 34;
    yellow.s_low = 188;
    yellow.s_high = 255;
    yellow.v_low = 233;
    yellow.v_high = 255;
    brown.h_low = 7;
    brown.h_high = 17;
    brown.s_low = 102;
    brown.s_high = 153;
    brown.v_low = 87;
    brown.v_high = 255;
    head.h_low = 117;
    head.h_high = 140;
    head.s_low = 41;
    head.s_high = 102;
    head.v_low = 74;
    head.v_high = 189;
    tail.h_low = 4;
    tail.h_high = 50;
    tail.s_low = 8;
    tail.s_high = 112;
    tail.v_low = 183;
    tail.v_high = 255;
    blue.h_low = 44;
    blue.h_high = 85;
    blue.s_low = 0;
    blue.s_high = 18;
    blue.v_low = 235;
    blue.v_high = 255;
}

int main(int argc, const char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Please enter the ardunio dev file\n");
        return -1;
    }
    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

    init_arduino(argv[1]);
    init_hsvcolor();
    // init_trackbars();

    while(1) {
        createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        thresh_callback(0, 0);
        if (status == START_POINT) {
            get_path();
            move_bot();
        }
        waitKey(10);
    }

    return 0;
}

void get_path() {
    line(path_img, town_centre, end_point, Scalar(255, 255, 255), 2, 8);
}

int dist(Point a, Point b) {
    return (int) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

float angle_between_4(Point a, Point b, Point c, Point d) {
    float slope1, slope2;
    slope1 = (float) (a.y - b.y) / (a.x - b.x);
    slope2 = (float) (c.y - d.y) / (c.x - d.x);

    float inter_angle = (atan(slope2) - atan(slope1)) * 180 / 3.14;
    /*
     * If positive, then bot has to take a left. Similarly,
     * if negative, then bot has to take a right.
     */
    return inter_angle;
}

float angle_between_3(Point a, Point b, Point c) {
    float slope1, slope2;
    slope1 = (float) (a.y - b.y) / (a.x - b.x);
    slope2 = (float) (c.y - b.y) / (c.x - b.x);

    float inter_angle = (atan(slope2) - atan(slope1)) * 180 / 3.14;

    return inter_angle;
}

void move_bot() {
    status = IN_BETWEEN_PATH;
    char previous;
    float d, angle4, angle3;
    do {
        d = dist(current_point, end_point);
        angle4 = angle_between_4(end_point, town_centre, head_point, tail_point);
        angle3 = angle_between_3(current_point, town_centre, end_point);
        //printf("Angle4: %f\n", angle4);
        //printf("Angle3: %f\n", angle3);
        //printf("Town Centre: (%d, %d)\n", town_centre.x, town_centre.y);
        //printf("Current Point: (%d, %d)\n", current_point.x, current_point.y);
        //printf("End Point: (%d, %d)\n", end_point.x, end_point.y);
        if (angle3 < -30) {
            if (previous != 'A') {
                previous = 'A';
                //sendCommand("A");
                //printf("A\n");
            }
            continue;
        }
        if (angle3 > 30) {
            if (previous != 'D') {
                previous = 'D';
                //sendCommand("D");
                //printf("D\n");
            }
            continue;
        }
        if (angle4 > 30 && (angle3 <= 30 && angle3 >= -30)) {
            if (previous != 'A') {
                previous = 'A';
                //sendCommand("A");
                //printf("A\n");
            }
        }
        if (angle4 < -30 && (angle3 <= 30 && angle3 >= -30)) {
            if (previous != 'D') {
                previous = 'D';
                //sendCommand("D");
                //printf("D\n");
            }
        }
        if (angle4 <= 30 && angle4 >= -30 && angle3 <= 30 && angle3 >= -30) {
            if (previous != 'W') {
                previous = 'W';
                //sendCommand("W");
                //printf("W\n");
            }
        }
        thresh_callback(0, 0);
    } while (d > 80);
    sendCommand("S");
    //printf("S\n");
}

void sendCommand(const char *command) {
    write(arduino, command, 1);
    //printf("sending: %s\n", command);
}

void init_arduino(const char *file) {
    arduino = open(file, O_RDWR || O_NOCTTY);
    if (arduino < 0) {
        fprintf(stderr, "Couldn't access the arduino device\n");
        exit(255);
    }
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

	Mat src, src_gray, imgHSV, yellow_img, green_img, blue_img, brown_img, red_img, head_img, tail_img;
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

    cap >> src;
    cvtColor(src, imgHSV, CV_BGR2HSV);
    imshow("Source", src);

    path_img = Mat::zeros(src.size(), CV_8UC3);

    inRange(imgHSV, Scalar(yellow.h_low, yellow.s_low, yellow.v_low),
            Scalar(yellow.h_high, yellow.s_high, yellow.v_high), yellow_img);
    inRange(imgHSV, Scalar(blue.h_low, blue.s_low, blue.v_low),
            Scalar(blue.h_high, blue.s_high, blue.v_high), blue_img);
    inRange(imgHSV, Scalar(brown.h_low, brown.s_low, brown.v_low),
            Scalar(brown.h_high, brown.s_high, brown.v_high), brown_img);
    inRange(imgHSV, Scalar(head.h_low, head.s_low, head.v_low),
            Scalar(head.h_high, head.s_high, head.v_high), head_img);
    inRange(imgHSV, Scalar(tail.h_low, tail.s_low, tail.v_low),
            Scalar(tail.h_high, tail.s_high, tail.v_high), tail_img);

    //imshow("Head", head_img);
    //imshow("Tail", tail_img);
    //imshow("Yellow", yellow_img);
    //imshow("Blue", blue_img);
    //imshow("Brown", brown_img);

    threshold(yellow_img, yellow_output, binary_thresh, 255, THRESH_BINARY);
    threshold(blue_img, blue_output, binary_thresh, 255, THRESH_BINARY);
    threshold(head_img, head_output, binary_thresh, 255, THRESH_BINARY);
    threshold(tail_img, tail_output, binary_thresh, 255, THRESH_BINARY);
    threshold(brown_img, brown_output, binary_thresh, 255, THRESH_BINARY);

    return ;
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
        if (contourArea(tail_contours[i]) > 100)
            tailBoundRect.push_back(boundingRect(Mat(tail_contours_poly[i])));
    }

    for (int i = 0; i < yellow_contours.size(); i++) {
        approxPolyDP(Mat(yellow_contours[i]), yellow_contours_poly[i], 3, true);
        if (contourArea(yellow_contours[i]) > 100)
            yellowBoundRect.push_back(boundingRect(Mat(yellow_contours_poly[i])));
    }

    for (int i = 0; i < blue_contours.size(); i++) {
        approxPolyDP(Mat(blue_contours[i]), blue_contours_poly[i], 3, true);
        if (contourArea(blue_contours[i]) > 100)
            blueBoundRect.push_back(boundingRect(Mat(blue_contours_poly[i])));
    }

    for (int i = 0; i < brown_contours.size(); i++) {
        approxPolyDP(Mat(brown_contours[i]), brown_contours_poly[i], 3, true);
        if (contourArea(brown_contours[i]) > 100)
            brownBoundRect.push_back(boundingRect(Mat(brown_contours_poly[i])));
    }

    head_drawing = Mat::zeros(src.size(), CV_8UC3);
    tail_drawing = Mat::zeros(src.size(), CV_8UC3);
    blue_drawing = Mat::zeros(src.size(), CV_8UC3);
    brown_drawing = Mat::zeros(src.size(), CV_8UC3);
    yellow_drawing = Mat::zeros(src.size(), CV_8UC3);

    for (int i = 0; i < head_contours_poly.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(head_drawing, headBoundRect[i].tl(), headBoundRect[i].br(), color, 2, 8, 0);
        head_point = (headBoundRect[i].tl() + headBoundRect[i].br());
        break;
    }
    for (int i = 0; i < tail_contours_poly.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(tail_drawing, tailBoundRect[i].tl(), tailBoundRect[i].br(), color, 2, 8, 0);
        tail_point = (tailBoundRect[i].tl() + tailBoundRect[i].br());
        break;
    }
    for (int i = 0; i < yellow_contours_poly.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(yellow_drawing, yellowBoundRect[i].tl(), yellowBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (yellowBoundRect[i].tl() + yellowBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (!initial)
            target_resources.push_back(centre_rect);
    }
    for (int i = 0; i < blue_contours_poly.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(blue_drawing, blueBoundRect[i].tl(), blueBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < brown_contours_poly.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(brown_drawing, brownBoundRect[i].tl(), brownBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (brownBoundRect[i].tl() + brownBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (centre_rect.x != 0 && centre_rect.y != 0) {
            town_centre = centre_rect;
            break;
        }
    }
    end_point = target_resources[1];
    for (int i = -5; i < 5; i++) {
        for (int j = -5; j < 5; j++) {
            path_img.at<Vec3b>(i + town_centre.y, j + town_centre.x) = {255, 255, 255};
        }
    }

    for (int i = -5; i < 5; i++) {
        for (int j = -5; j < 5; j++) {
            path_img.at<Vec3b>(i + end_point.y, j + end_point.x) = {255, 255, 255};
        }
    }
    initial++;

    // Update the bot values
    current_point.x = (head_point.x + tail_point.x) / 4;
    current_point.y = (head_point.y + tail_point.y) / 4;

    line(path_img, tail_point, head_point, Scalar(255, 255, 255), 2, 8);
    get_path();
    line(path_img, current_point, town_centre, Scalar(255, 255, 255), 2, 8);

    for (int i = -2; i < 2; i++) {
        for (int j = -2; j < 2; j++) {
            path_img.at<Vec3b>(i + current_point.y, j + current_point.x) = {255, 255, 255};
        }
    }

    imshow("Head Contours", head_drawing);
    imshow("Tail Contours", tail_drawing);
    imshow("Yellow Contours", yellow_drawing);
    imshow("Blue Contours", blue_drawing);
    imshow("Brown Contours", brown_drawing);
    imshow("Path img", path_img);
}
