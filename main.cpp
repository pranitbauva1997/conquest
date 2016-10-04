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

int binary_thresh = 100;
int initial = 0;

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

typedef struct {
    int x;
    int y;
} coordi;

struct Node {
    vector<Node *> children;
    Node *parent;
    coordi position;
};

Node start_node;
Node end_node;

Node *nodes[5000];
int totnodes = 0;
int reached = 0;
int step_size = 10;
int iter = 0;
int path = 0;

vector<Point> target_resources;
Point town_centre, end_target;
Point bot_position, current_target, start_point;

Mat path_img;

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
void rrt();
void draw_path();
int check_validity_2(coordi a, coordi b);
int check_validity_1(coordi a, coordi b);
coordi stepping(coordi nnode, coordi rnode);
int near_node(Node rnode);
float node_dist(coordi a, coordi b);
void init();
int dist(coordi a, coordi b);


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

    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR!\n");
        return -1;
    }

    init_hsvcolor();
    //init_trackbars();

    while(1) {
        cap >> src;
        if (!path)
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
        if (!path) {
            init();
            while ((reached == 0))
                rrt();
            path++;
        }
        imshow("RRT Path", path_img);
        waitKey(5);
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
    int dis = dist(start_node.position, rnode.position);
    int lnode = 0;
    for (int i = 0; i < totnodes; i++) {
        dis = dist(nodes[i]->position, rnode.position);
        if (dis < min_dist) {
            min_dist = dis;
            lnode = i;
        }
    }

    return lnode;
}

coordi stepping(coordi nnode, coordi rnode) {
    coordi interm, step;
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

int check_validity_1(coordi p, coordi q) {
    coordi large, small;
    int i = 0, j1 = 0, j2 = 0;
    double slope;
    if (q.x < p.x) {
        small  = q;
        large = p;
    } else {
        small = p;
        large = q;
    }
    if (large.x == small.x)
        return 0;

    slope = ((double) large.y - small.y) / ((double) large.x - small.x);
    for (i = small.x + 1; i < large.x; i++) {
        j1 = (int) ((i * slope) - (small.x) * (slope) + small.y);
        j2 = j1 + 1;
        if (i < 0 || j1 < 0 || j2 < 0 || i > src.rows || j1 > src.cols || j2 > src.cols)
            continue;
        if (((int)blue_img.at<Vec3b>(i, j1)[0] > 50) || ((int)blue_img.at<Vec3b>(i, j1)[1] > 50) || ((int)blue_img.at<Vec3b>(i, j1)[2] > 50))
            return 0;
        if (((int)blue_img.at<Vec3b>(i, j2)[0] > 50) || ((int)blue_img.at<Vec3b>(i, j2)[1] > 50) || ((int)blue_img.at<Vec3b>(i, j2)[2] > 50))
            return 0;
    }

    return 1;
}

int check_validity_2(coordi p, coordi q) {
    coordi large, small;
    int i = 0, j1 = 0, j2 = 0;
    double slope;
    if (q.y < p.y) {
        small = q;
        large = p;
    } else {
        small = p;
        large = q;
    }
    if (large.x == small.x)
        return 0;
    slope = ((double) large.y - small.y) / ((double) large.x - small.x);
    for (i = small.y + i; i < large.y; i++) {
        j1 = (int) (((i - small.y) / slope) + small.x);
        j2 = j1 + 1;
        if (i < 0 || j1 < 0 || j2 < 0 || i > src.rows || j1 > src.cols || j2 > src.cols)
            continue;
        if (((int)blue_img.at<Vec3b>(i, j1)[0] > 50) || ((int)blue_img.at<Vec3b>(i, j1)[1] > 50) || ((int)blue_img.at<Vec3b>(i, j1)[2] > 50))
            return 0;
        if (((int)blue_img.at<Vec3b>(i, j2)[0] > 50) || ((int)blue_img.at<Vec3b>(i, j2)[1] > 50) || ((int)blue_img.at<Vec3b>(i, j2)[2] > 50))
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
    (rnode->position).x = rand() % src.rows + 1;
    (rnode->position).y = rand() % src.cols + 1;
    index = near_node(*rnode);
    if ((dist(rnode->position, nodes[index]->position)) < step_size)
        return;
    else
        stepnode->position = stepping(nodes[index]->position, rnode->position);
    flag1 = check_validity_1(nodes[index]->position, stepnode->position);
    flag2 = check_validity_2(nodes[index]->position, stepnode->position);
    if ((flag1 == 1) && (flag2 == 1)) {
        nodes[totnodes++] = stepnode;
        stepnode->parent = nodes[index];
        (nodes[index]->children).push_back(stepnode);
        line(path_img, Point((stepnode->position).y, (stepnode->position).x), Point(nodes[index]->position.y, nodes[index]->position.x), Scalar(0, 255, 255), 2, 8);
        for (int i = stepnode->position.x - 2; i < stepnode->position.x + 2; i++) {
            for (int j = stepnode->position.y - 2; j < stepnode->position.y + 2; j++) {
                if ((i < 0 || j < 0 || i > src.rows || j > src.cols))
                    continue;

                path_img.at<Vec3b>(i, j)[0] = 0;
                path_img.at<Vec3b>(i, j)[1] = 255;
                path_img.at<Vec3b>(i, j)[2] = 0;
            }
        }
        if ((check_validity_1(stepnode->position, end_node.position)) && (check_validity_2(stepnode->position, end_node.position)) && (dist(stepnode->position, end_node.position) < step_size)) {
            reached = 1;
            nodes[totnodes++] = &end_node;
            end_node.parent = stepnode;
            (nodes[totnodes - 1]->children).push_back(&end_node);
            draw_path();
        }
    }
    iter++;
}

/*
void check_status() {
    if (distance_(&bot_position, &current_target) < 20) {
        status = BLINK_LED;
        blink_led();
    }
}*/

int dist(coordi a, coordi b) {
    coordi temp;
    temp.x = a.x - b.x;
    temp.y = a.y - b.y;
    return (int) sqrt(pow(temp.x, 2) + pow(temp.y, 2));
}
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
    Mat yellow_output, blue_output, head_output, tail_output, brown_output;
    vector<vector<Point> > head_contours, tail_contours, yellow_contours, green_contours, blue_contours, brown_contours, red_contours;
    vector<Vec4i> head_hierarchy, tail_hierarchy, yellow_hierarchy, green_hierarchy, blue_hierarchy, brown_hierarchy, red_hierarchy;

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

    vector<vector<Point> > head_contours_poly(head_contours.size());
    vector<vector<Point> > tail_contours_poly(tail_contours.size());
    vector<vector<Point> > yellow_contours_poly(yellow_contours.size());
    vector<vector<Point> > green_contours_poly(green_contours.size());
    vector<vector<Point> > blue_contours_poly(blue_contours.size());
    vector<vector<Point> > brown_contours_poly(brown_contours.size());
    vector<vector<Point> > red_contours_poly(red_contours.size());

    vector<Rect> headBoundRect(head_contours.size());
    vector<Rect> tailBoundRect(tail_contours.size());
    vector<Rect> yellowBoundRect(yellow_contours.size());
    vector<Rect> greenBoundRect(green_contours.size());
    vector<Rect> blueBoundRect(blue_contours.size());
    vector<Rect> brownBoundRect(brown_contours.size());
    vector<Rect> redBoundRect(red_contours.size());

    for (int i = 0; i < head_contours.size(); i++) {
        approxPolyDP(Mat(head_contours[i]), head_contours_poly[i], 3, true);
        if (contourArea(head_contours[i]) > 100)
            headBoundRect[i] = boundingRect(Mat(head_contours_poly[i]));
    }

    for (int i = 0; i < tail_contours.size(); i++) {
        approxPolyDP(Mat(tail_contours[i]), tail_contours_poly[i], 3, true);
        if (contourArea(tail_contours[i]) > 500)
            tailBoundRect[i] = boundingRect(Mat(tail_contours_poly[i]));
    }

    for (int i = 0; i < yellow_contours.size(); i++) {
        approxPolyDP(Mat(yellow_contours[i]), yellow_contours_poly[i], 3, true);
        if (contourArea(yellow_contours[i]) > 100)
            yellowBoundRect[i] = boundingRect(Mat(yellow_contours_poly[i]));
    }

    for (int i = 0; i < green_contours.size(); i++) {
        approxPolyDP(Mat(green_contours[i]), green_contours_poly[i], 3, true);
        if (contourArea(green_contours[i]) > 100)
            greenBoundRect[i] = boundingRect(Mat(green_contours_poly[i]));
    }

    for (int i = 0; i < blue_contours.size(); i++) {
        approxPolyDP(Mat(blue_contours[i]), blue_contours_poly[i], 3, true);
        if (contourArea(blue_contours[i]) > 100)
            blueBoundRect[i] = boundingRect(Mat(blue_contours_poly[i]));
    }

    for (int i = 0; i < brown_contours.size(); i++) {
        approxPolyDP(Mat(brown_contours[i]), brown_contours_poly[i], 3, true);
        if (contourArea(brown_contours[i]) > 500)
            brownBoundRect[i] = boundingRect(Mat(brown_contours_poly[i]));
    }

    head_drawing = Mat::zeros(src.size(), CV_8UC3);
    tail_drawing = Mat::zeros(src.size(), CV_8UC3);
    green_drawing = Mat::zeros(src.size(), CV_8UC3);
    blue_drawing = Mat::zeros(src.size(), CV_8UC3);
    brown_drawing = Mat::zeros(src.size(), CV_8UC3);
    yellow_drawing = Mat::zeros(src.size(), CV_8UC3);

    for (int i = 0; i < headBoundRect.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(head_drawing, headBoundRect[i].tl(), headBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < tailBoundRect.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(tail_drawing, tailBoundRect[i].tl(), tailBoundRect[i].br(), color, 2, 8, 0);
    }
    for (int i = 0; i < yellowBoundRect.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(yellow_drawing, yellowBoundRect[i].tl(), yellowBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (yellowBoundRect[i].tl() + yellowBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (!initial)
            target_resources.push_back(centre_rect);
    }
    for (int i = 0; i < blueBoundRect.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(blue_drawing, blueBoundRect[i].tl(), blueBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (blueBoundRect[i].tl() + blueBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (!initial)
            target_resources.push_back(centre_rect);
    }
    for (int i = 0; i < brownBoundRect.size(); i++) {
        Scalar color = Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        rectangle(brown_drawing, brownBoundRect[i].tl(), brownBoundRect[i].br(), color, 2, 8, 0);
        Point centre_rect = (brownBoundRect[i].tl() + brownBoundRect[i].br());
        centre_rect.x /= 2;
        centre_rect.y /= 2;
        if (centre_rect.x != 0 && centre_rect.y != 0)
            town_centre = centre_rect;
    }
    initial++;
    end_target = target_resources[0];

    imshow("Head Contours", head_drawing);
    imshow("Tail Contours", tail_drawing);
    imshow("Yellow Contours", yellow_drawing);
    imshow("Blue Contours", blue_drawing);
    imshow("Brown Contours", brown_drawing);
}
