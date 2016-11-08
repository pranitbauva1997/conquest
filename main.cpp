#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <bits/stdc++.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <ctime>
#include <math.h>

using namespace cv;
using namespace std;

VideoCapture cap(0);

int binary_thresh = 100;
int initial = 0;
int debug = 0;
int front = 1;
int resource = 0;

/* Path planning variables */

enum state {
    START_POINT = 1,
    IN_BETWEEN_PATH,
    END_POINT,
    BLINK_LED,
    REVERSE_MOVE,
    END_POINT_TC
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
    yellow.h_low = 26;
    yellow.h_high = 41;
    yellow.s_low = 78;
    yellow.s_high = 255;
    yellow.v_low = 0;
    yellow.v_high = 255;
    brown.h_low = 4;
    brown.h_high = 22;
    brown.s_low = 77;
    brown.s_high = 111;
    brown.v_low = 195;
    brown.v_high = 255;
    head.h_low = 117;
    head.h_high = 131;
    head.s_low = 33;
    head.s_high = 74;
    head.v_low = 162;
    head.v_high = 229;
    tail.h_low = 18;
    tail.h_high = 56;
    tail.s_low = 0;
    tail.s_high = 32;
    tail.v_low = 232;
    tail.v_high = 255;
    blue.h_low = 55;
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

    if (debug)
        init_trackbars();


    while(1) {
        createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        thresh_callback(0, 0);
        if (!debug)
            move_bot();
        waitKey(10);
    }

    return 0;
}

void get_path() {
    line(path_img, town_centre, end_point, Scalar(255, 255, 255), 2, 8);
}

double dist(Point a, Point b) {
    return (double) sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

double angle_between(Point a, Point b, Point c) {
    double slope1 = (double) (a.y - b.y) / (a.x - b.x);
    double slope2 = (double) (c.y - b.y) / (c.x - b.x);

    double inter_angle = (double) (atan((slope1 - slope2) / (1 + (slope1 * slope2)))) * 180 / 3.14;

    return inter_angle;
}

void blink_led(int seconds) {
    int i;
    for (i = 0; i < seconds; i++) {
        sendCommand("B");
    }
}

void move_bot() {
    printf("End Point: (%d, %d)\n", end_point.x, end_point.y);
    if (status == START_POINT){
        status = IN_BETWEEN_PATH;
    }

    if (status == BLINK_LED) {
        printf("Blinking LED\n");
        blink_led(1);
        printf("now status = REVERSE_MOVE\n");
        status = REVERSE_MOVE;
        move_bot();
    }

    if (status == REVERSE_MOVE) {
        status = IN_BETWEEN_PATH;
        if (end_point.x == town_centre.x && end_point.y == town_centre.y) {
            printf("Taking Reverse\n");
            sendCommand("V");
            printf("Finished reverse turn\n");
            sleep(3);
            end_point = target_resources[++resource];
        }
        else {
            printf("Taking Reverse\n");
            sendCommand("V");
            sleep(3);
            printf("Finished reverse turn\n");
            end_point = town_centre;
        }
        move_bot();
    }

    if (status == IN_BETWEEN_PATH) {
        printf("status = IN_BETWEEN_PATH\n");
        char previous;
        double d, d1, d2, angle;
        do {
            d1 = dist(head_point, end_point);
            d2 = dist(tail_point, end_point);
            d = d1 >= d2 ? d1 : d2;
            angle = angle_between(head_point, end_point, tail_point);
            printf("Head Point: (%d, %d)\n", head_point.x, head_point.y);
            printf("Tail Point: (%d, %d)\n", tail_point.x, tail_point.y);
            printf("End Point: (%d, %d)\n", end_point.x, end_point.y);
            printf("Angle: %f\n", angle);
            printf("Distance: %f\n", d);
            if (angle <= 10 && angle >= -10) {
                if (previous != 'W') {
                    previous = 'W';
                    sendCommand("W");
                    printf("W\n");
                }
            }
            if (front) {
                if (angle < -10) {
                    if (previous != 'A') {
                        previous = 'A';
                        sendCommand("A");
                        printf("A\n");
                    }
                } else if (angle > 10) {
                    if (previous != 'D') {
                        previous = 'D';
                        sendCommand("D");
                        printf("D\n");
                    }
                }
            } else {
                if (angle < -10) {
                    if (previous != 'D') {
                        previous = 'A';
                        sendCommand("A");
                        printf("A\n");
                    }
                } else if (angle > 10) {
                    if (previous != 'A') {
                        sendCommand("D");
                        printf("D");
                    }
                }
            }
            thresh_callback(0, 0);
        } while (d > 60);
        sendCommand("S");
        printf("S\n");
        printf("status = BLINK_LED\n");
        status = BLINK_LED;
        move_bot();
    }
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

    if (!initial || debug) {
        inRange(imgHSV, Scalar(yellow.h_low, yellow.s_low, yellow.v_low),
                Scalar(yellow.h_high, yellow.s_high, yellow.v_high), yellow_img);
        inRange(imgHSV, Scalar(blue.h_low, blue.s_low, blue.v_low),
                Scalar(blue.h_high, blue.s_high, blue.v_high), blue_img);
        inRange(imgHSV, Scalar(brown.h_low, brown.s_low, brown.v_low),
                Scalar(brown.h_high, brown.s_high, brown.v_high), brown_img);
    }
    inRange(imgHSV, Scalar(head.h_low, head.s_low, head.v_low),
            Scalar(head.h_high, head.s_high, head.v_high), head_img);
    inRange(imgHSV, Scalar(tail.h_low, tail.s_low, tail.v_low),
            Scalar(tail.h_high, tail.s_high, tail.v_high), tail_img);

    if (debug) {
        imshow("Head", head_img);
        imshow("Tail", tail_img);
        imshow("Yellow", yellow_img);
        imshow("Blue", blue_img);
        imshow("Brown", brown_img);
    }

    if (!initial || debug) {
        threshold(yellow_img, yellow_output, binary_thresh, 255, THRESH_BINARY);
        threshold(blue_img, blue_output, binary_thresh, 255, THRESH_BINARY);
        threshold(brown_img, brown_output, binary_thresh, 255, THRESH_BINARY);
    }
    threshold(head_img, head_output, binary_thresh, 255, THRESH_BINARY);
    threshold(tail_img, tail_output, binary_thresh, 255, THRESH_BINARY);

    /* Find contours */
    findContours(head_output, head_contours, head_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    findContours(tail_output, tail_contours, tail_hierarchy, CV_RETR_TREE,
                 CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    if (!initial || debug) {
        findContours(yellow_output, yellow_contours, yellow_hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(brown_output, brown_contours, brown_hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
        findContours(blue_output, blue_contours, blue_hierarchy,
                     CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    }

    if (debug)
        return ;

    head_contours_poly.resize(head_contours.size());
    tail_contours_poly.resize(tail_contours.size());

    if (!initial || debug) {
        yellow_contours_poly.resize(yellow_contours.size());
        blue_contours_poly.resize(blue_contours.size());
        brown_contours_poly.resize(brown_contours.size());
    }

    for (int i = 0; i < head_contours.size(); i++) {
        approxPolyDP(Mat(head_contours[i]), head_contours_poly[i], 3, true);
        if (contourArea(head_contours[i]) > 500)
            headBoundRect.push_back(boundingRect(Mat(head_contours_poly[i])));
    }

    for (int i = 0; i < tail_contours.size(); i++) {
        approxPolyDP(Mat(tail_contours[i]), tail_contours_poly[i], 3, true);
        if (contourArea(tail_contours[i]) > 1000)
            tailBoundRect.push_back(boundingRect(Mat(tail_contours_poly[i])));
    }

    if (!initial || debug) {
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
    }

    head_drawing = Mat::zeros(src.size(), CV_8UC3);
    tail_drawing = Mat::zeros(src.size(), CV_8UC3);

    if (!initial || debug) {
        blue_drawing = Mat::zeros(src.size(), CV_8UC3);
        brown_drawing = Mat::zeros(src.size(), CV_8UC3);
        yellow_drawing = Mat::zeros(src.size(), CV_8UC3);
    }

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
    if (!initial || debug) {
        for (int i = 0; i < yellow_contours_poly.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            rectangle(yellow_drawing, yellowBoundRect[i].tl(), yellowBoundRect[i].br(), color, 2, 8, 0);
            Point centre_rect = (yellowBoundRect[i].tl() + yellowBoundRect[i].br());
            centre_rect.x /= 2;
            centre_rect.y /= 2;
            target_resources.push_back(centre_rect);
        }

        for (int i = 0; i < blue_contours_poly.size(); i++) {
            Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
            rectangle(blue_drawing, blueBoundRect[i].tl(), blueBoundRect[i].br(), color, 2, 8, 0);
            Point centre_rect = (blueBoundRect[i].tl() + blueBoundRect[i].br());
            centre_rect.x /= 2;
            centre_rect.y /= 2;
            target_resources.push_back(centre_rect);
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
    }
    if (!initial)
        end_point = target_resources[0];
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

    if (!initial)
        initial++;

    // Update the bot values
    head_point.x /= 2;
    head_point.y /= 2;
    tail_point.x /= 2;
    tail_point.y /= 2;
    current_point.x = (head_point.x + tail_point.x) / 2;
    current_point.y = (head_point.y + tail_point.y) / 2;

    get_path();
    line(path_img, current_point, town_centre, Scalar(255, 255, 255), 2, 8);

    for (int i = -2; i < 2; i++) {
        for (int j = -2; j < 2; j++) {
            path_img.at<Vec3b>(i + current_point.y, j + current_point.x) = {255, 255, 255};
        }
    }

    if (debug) {
        imshow("Head Contours", head_drawing);
        imshow("Tail Contours", tail_drawing);
        imshow("Yellow Contours", yellow_drawing);
        imshow("Blue Contours", blue_drawing);
        imshow("Brown Contours", brown_drawing);
        imshow("Path img", path_img);
    }
}
