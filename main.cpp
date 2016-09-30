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
enum start_point {
    TOWN_CENTRE = 1,
    RESOURCE
};

enum status {
    START_POINT = 1,
    IN_BETWEEN_PATH,
    END_POINT,
    BLINK_LED
};

struct Node {
    vector<Node *> children;
    Node *parent;
    Point position;
};

Node start_node;
Node end_node;

Node* nodes[5000];
int totnodes = 0;
int reached = 0;
int step_size = 20;
int iter = 0;

vector<Point> target_resources;
Point town_centre, end_target;
Point bot_position, current_target, start_point;

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

        threshold_color_images();

        // createTrackbar("Threshold: ", "Source", &binary_thresh, 255, thresh_callback);
        // thresh_callback(0, 0);
        merge_obstacle_image();

        /* Compute Path and move bot's ass */
        if (status == IN_BETWEEN_PATH) {
            continue_moving();
        } else if (status == START_POINT) {
            get_path_to_start_point();
            status == IN_BETWEEN_PATH;
        } else if (status == RESOURCE) {
            get_path_to_end_point();
            status = IN_BETWEEN_PATH;
        }

        check_status();
        waitKey(70);
    }

    return 0;
}

void init() {
    start_node.position.x = town_centre.x;
    start_node.position.y = town_centre.y;
    nodes[totnodes++] = &start_node;
    end_node.position.x = end_target.x;
    end_node.position.y = end_target.y;
    srand(time(NULL));
}

int near_node(Node rnode) {
    float min_dist = 999.0;
    int dis = distance(start_node.position, rnode.position);
    int lnode = 0;
    for (int i = 0; i < totnodes; i++) {
        dist = distance(nodes[i]->position, rnode.position);
        if (dist < min_dist) {
            min_dist = dist;
            lnode = i;
        }
    }

    return lnode;
}

Point stepping(Point nnode, Point rnode) {
    Point interm, step;
    float magn = 0.0, x = 0.0, y = 0.0;
    interm.x = rnode.x - nnode.x;
    interm.y = rnode.y - nnode.y;
    magn = sqrt(pow(interm.x, 2) + pow(interm.y, 2));
    x = interm.x / magn;
    y = interm.y / magn;
    step.x = (int) (nnode.x + step_size * x);
    step.y = (int) (nnode.y + step_size * y);
    return step;
}

check_validity_1(Point p, Point q) {
    Point large, small;
    int i = 0, j1 = 0, j2 = 0;
    double slope;
    if (q.x < p.x) {
        small  = q;
        large = p;
    } else {
        small = p;
        large = q;
    }
    if (large.x = small.x)
        return 0;

    slope = ((double) large.y - small.y) / ((double) large.x - small.x);
    for (i = small.x + 1; i < large.x; i++) {
        j1 = (int) ((i * slope) - (small.x) * (slope) + small.y);
        j2 = j1 + 1;
        if (i < 0 || j1 < 0 || j2 < 0 || i > src.rows || j1 > src.cols || j2 > src.cols)
            continue;
        if (blue_img_gray.at<uchar>(i, j1) == 255)
            return 0;
        if (blue_img_gray.at<uchar>(i, j2) == 255)
            return 0;
    }

    return 1;
}

int check_validity_2(Point p, Point q) {
    Point large, small;
    int i = 0, j1 = 0, j2 = 0;
    double slope;
    if (q.y < p.y) {
        small = q;
        large = p;
    } else {
        small = p;
        large = q;
    }
    if (large.x = small.x)
        return 0;
    slope = ((double) large.y - small.y) / ((double) large.x - small.x);
    for (i = small.y + i; i < large.y; i++) {
        j1 = (int) (((i - small.y) / slope) + small.x);
        j2 = j1 + 1;
        if (i < 0 || j1 < 0 || j2 < 0 || i > src.rows || j1 > src.cols || j2 > src.cols)
            continue;
        if (blue_img_gray.at<uchar>(i, j1) == 255)
            return 0;
        if (blue_img_gray.at<uchar>(i, j2) == 255)
            return 0;
    }

    return 1;
}

void draw_path() {
    Node up, down;
    int breaking = 0;
    down = end_node;
    up = *(end_node.parent);
    while (1) {
        line(path_img, Point(up.position.y, up.position.x), Point(down.position.y, down.position.x), Scalar(0, 255, 0), 2, 8);
        if (up.parent == NULL)
            break;
        up = *(up.parent);
        down = *(down.parent);
    }
}

void rrt() {
    int flag1 = 0, index = 0, flag2 = 0;
    Node *rnode = new Node;
    Node *stepnode = new Node;
    (rnode->position).x = rand() % 400 + 1;
    (rnode->position).y = rand() % 400 + 1;
    index = near_node(*rnode);
    if ((distance(rnode->position, nodes[index]->position)) < step_size)
        return;
    else
        stepnode->position = stepping(nodes[index]->position, rnode->position);
    flag1 = check_validity_1(nodes[index]->position, stepnode->position);
    if ((flag == 1) && (flag2 == 1)) {
        nodes[totnodes++] = stepnode;
        stepnode->parent = nodes[index];
        (nodes[index]->children).push_back(stepnode);
        line(path_img, Point((stepnode->position).y, (stepnode->position).x), Point(nodes[index]->position.y, nodes[index]-position.x), Scalar(0, 255, 255), 2, 8);
        for (int i = stepnode->position.x - 2; i < stepnode->position.x + 2; i++) {
            for (int j = stepnode->position.y - 2; j < stepnode->position.y + 2; j++) {
                if ((i < 0 || j < 0 || i > src.rows || j > src.cols))
                    continue;

                path_img.at<Vec3b>(i, j)[0] = 0;
                path_img.at<Vec3b>(i, j)[1] = 255;
                path_img.at<Vec3b>(i, j)[2] = 0;
            }
        }
        if ((check_validity_1(stepnode->position, end_node.position)) && (check_validity_2(stepnode->position, end_node.position)) && (distance(stepnode->position, end_node.position) < step_size)) {
            reached = 1;
            nodes[totnodes++] = &end_node;
            end_node.parent = stepnode;
            (nodes[totnodes - 1]->children).push_back(&end_node);
            draw_path();
        }
    }
    iter++;
}

void check_status() {
    if (distance(&bot_position, &current_target) < 20) {
        status = BLINK_LED;
        blink_led();
    }
}

int distance distance(Point a, Point b) {
    Point temp;
    temp.x = a.x - b.x;
    temp.y = a.y - b.y;
    return (int) sqrt(pow(temp.x, 2) + pow(temp.y, 2));
}

void continue_moving() {
    int x = current_target.x - bot_position.x;
    int y = current_target.y - bot_position.y;
    if (x > 0 && y > 0)
        go_left();
    if (x > 0 && y < 0)
        go_right();
}

void threshold_color_images() {
    threshold_yellow_image();
    threshold_green_image();
    threshold_blue_image();
    threshold_red_image();
    threshold_brown_image();
}

void threshold_blue_image() {
    for (int i = 0; i < blue_img.rows; i++) {
        if (int j = 0; j < blue_img.cols; j++) {
            if (blue_img.at<Vec3b>(i, j)[2] > 100)
                blue_img_gray.at<uchar> = 255;
            else
                blue_image_gray.at<uchar> = 0;
        }
    }
}

void threshold_green_image() {
    for (int i = 0; i < green_img.rows; i++) {
        for (int j = 0; j < green_img.cols; j++) {
            if (green_img.at<Vec3b>(i, j)[2] > 100)
                green_img_gray.at<uchar> = 255;
            else
                green_img_gray.at<uchar> = 0;
        }
    }
}

void threshold_brown_image() {
    for (int i = 0; i < brown_image.rows; i++) {
        for (int j = 0; j < brown_image.cols; j++) {
            if (brown_image.at<Vec3b>(i, j)[2] > 100)
                brown_image_gray.at<uchar> = 255;
            else
                brown_image_gray.at<uchar> = 0;
        }
    }
}

void threshold_red_image() {
    for (int i = 0; i < red_image.rows; i++) {
        for (int j = 0; j < red_image.cols; j++) {
            if (red_image.at<Vec3b>(i, j)[2] > 100)
                red_image_gray.at<uchar> = 255;
            else
                red_image_gray.at<uchar> = 0;
        }
    }
}

void merge_obstacle_image() {
    for (int i = 0; i < blue_img.rows; i++)
        for (int j = 0; j < blue_img.cols; j++)
            if (blue_img_gray.at<uchar>(i, j) == 255)
                brown_img.at<Vec3b>(i, j) = 255;
}

void thresh_callback(int, void *) {
}
