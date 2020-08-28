#include "ros/ros.h"
#include "ros_tutorials_topic/MsgTutorial.h"

#include "opencv2/opencv.hpp"
#include <iostream>
#include <cmath>

using namespace std;
using namespace cv;

void find_HSign();

int main(int argc, char **argv)
{
	ros::init(argc, argv, "topic_publisher");
	ros::NodeHandle nh;

	ros::Publisher target_topic_pub = nh.advertise<targetDetection_topic::TargetPosition>("Target_coord_msg", 100);
	ros::Rate loop_rate(10);

	targetDetection_topic::TargetPosition msg;

	VideoCapture cap(0);
	Mat templ = imread("LandingTarget.jpg", IMREAD_GRAYSCALE);

	if (!cap.isOpened() || templ.empty()) {
		cerr << "Camera or Template image open failed" << endl;
		return;
	}

	cout << "Frame width: " << cvRound(cap.get(CAP_PROP_FRAME_WIDTH)) << endl;
	cout << "Frame height: " << cvRound(cap.get(CAP_PROP_FRAME_HEIGHT)) << endl;
	cout << "Template Size: " << templ.size << endl;


	int w = cvRound(cap.get(CAP_PROP_FRAME_WIDTH));
	int h = cvRound(cap.get(CAP_PROP_FRAME_HEIGHT));
	double fps = cap.get(CAP_PROP_FPS);

	int fourcc = VideoWriter::fourcc('D', 'X', '5', '0');
	int delay = cvRound(1000 / fps);

	// ������ ���� ����

	VideoWriter writer;
	writer.open("test.mpeg", fourcc, fps, Size(w + templ.cols, h));

	Ptr<Feature2D> orb = ORB::create();
	vector<KeyPoint> keypoints_templ; // template�� Ư¡���� ����� ���� ���
	Mat desc_templ;
	orb->detectAndCompute(templ, Mat(), keypoints_templ, desc_templ);

	Mat img, img_color;
	while (ros::ok()) {
		cap >> img_color;
		if (img_color.empty()) break;
		cvtColor(img_color, img, COLOR_BGR2GRAY);

		vector<KeyPoint> keypoints_img; // �� ������(img) ���� Ư¡���� ����� ���
		Mat desc_img;
		orb->detectAndCompute(img, Mat(), keypoints_img, desc_img);

		Ptr<DescriptorMatcher> matcher = BFMatcher::create(NORM_HAMMING);

		vector<DMatch> matches;
		matcher->match(desc_templ, desc_img, matches);

		std::sort(matches.begin(), matches.end());
		vector<DMatch> good_matches(matches.begin(), matches.begin() + 50);

		Mat dst;
		drawMatches(templ, keypoints_templ, img, keypoints_img, good_matches, dst, Scalar::all(-1), Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		
		// Whether the target is detected in Camera
		bool is_detected;
		if (good_matches.size() < 50) {
			is_detected = false;
		}
		else {
			is_detected = true;
		}

		vector<Point2f> pts1, pts2;
		for (size_t i = 0; i < good_matches.size(); i++) {
			pts1.push_back(keypoints_templ[good_matches[i].queryIdx].pt); //keyoints_templ���� query Ư¡���� ����Ǿ� �ִ�. �� Ư¡���� pts1�� �����Ѵ�.
			pts2.push_back(keypoints_img[good_matches[i].trainIdx].pt);
		}

		Mat H = findHomography(pts1, pts2, RANSAC);

		vector<Point2f> corners1, corners2;
		corners1.push_back(Point2f(0, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, 0));
		corners1.push_back(Point2f(templ.cols - 1.f, templ.rows - 1.f));
		corners1.push_back(Point2f(0, templ.rows - 1.f));
		perspectiveTransform(corners1, corners2, H);

		vector<Point> corners_dst;
		float del_east = 0.0;
		float del_north = 0.0;

		int midPointX = 0;
		int midPointY = 0;

		for (Point2f pt : corners2) {
			corners_dst.push_back(Point(cvRound(pt.x + templ.cols), cvRound(pt.y)));
			midPointX += cvRound(pt.x);
			midPointY += cvRound(pt.y);
		}
		// midPoint of H_sign
		midPointX = midPointX / 4;
		midPointY = midPointY / 4;

		///////////////////////////////////
		///// Find del_east, del_north ////
		///////////////////////////////////
		
		// H_sign's height = 1m
		Point2f pt1 = corners_dst[0];
		Point2f pt4 = corners_dst[3];
		
		float pixel_dist = sqrt(pow(pt1.x - pt4.x, 2) + pow(pt1.y - pt4.y, 2));
		float scale = 1.0 / pixel_dist;  //scale: 1 pixel = ? meter

		// Considering Camera fronts North
		del_east = (midPointX - w / 2) * scale;
		del_north = (h / 2 - midPointY) * scale;

		// Rotate the coordinates
		Point2f n_dir = pt1 - pt4;
		float theta = acos(-n_dir.y / sqrt(pow(n_dir.x, 2) + pow(n_dir.y, 2)));
		
		del_east = cos(theta) * del_east + sin(theta) * del_north;
		del_north = -sin(theta) * del_east + cos(theta) * del_north;

		msg.del_east = del_east;
		msg.del_north = del_north;
		msg.is_detected = is_detected;

		target_topic_pub.publish(msg);

		polylines(dst, corners_dst, true, Scalar(0, 255, 0), 2, LINE_AA);

		writer << dst;

		imshow("dst", dst);

		if (waitKey(10) == 27) break;
	}

	loop_rate.sleep();

	return 0;
}
