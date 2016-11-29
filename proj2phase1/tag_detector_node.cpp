#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Odometry.h>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigen>
#include <Eigen/SVD>

//EIgen SVD libnary, may help you solve SVD
//JacobiSVD<MatrixXd> svd(A, ComputeThinU | ComputeThinV);

using namespace cv;
using namespace aruco;
using namespace Eigen;

//global varialbles for aruco detector
aruco::CameraParameters CamParam;
MarkerDetector MDetector;
vector<Marker> Markers;
float MarkerSize = 0.20 / 1.5 * 1.524;
float MarkerWithMargin = MarkerSize * 1.2;
BoardConfiguration TheBoardConfig;
BoardDetector TheBoardDetector;
Board TheBoardDetected;
ros::Publisher pub_odom_yourwork;
ros::Publisher pub_odom_ref;
cv::Mat K, D;
double rms = 0.0;

// test function, can be used to verify your estimation
void calculateReprojectionError(const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const cv::Mat R, const cv::Mat t)
{
    puts("calculateReprojectionError begins");
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);
    for (unsigned int i = 0; i < pts_3.size(); i++)
    {
        cv::Mat p_mat(3, 1, CV_64FC1);
        p_mat.at<double>(0, 0) = pts_3[i].x;
        p_mat.at<double>(1, 0) = pts_3[i].y;
        p_mat.at<double>(2, 0) = pts_3[i].z;
        cv::Mat p = (R * p_mat + t);
        printf("(%f, %f, %f) -> (%f, %f) and (%f, %f)\n",
               pts_3[i].x, pts_3[i].y, pts_3[i].z,
               un_pts_2[i].x, un_pts_2[i].y,
               p.at<double>(0) / p.at<double>(2), p.at<double>(1) / p.at<double>(2));
    }
    puts("calculateReprojectionError ends");
}

// the main function you need to work with
// pts_id: id of each point
// pts_3: 3D position (x, y, z) in world frame
// pts_2: 2D position (u, v) in image frame
void process(const vector<int> &pts_id, const vector<cv::Point3f> &pts_3, const vector<cv::Point2f> &pts_2, const ros::Time& frame_time, double& rms)
{
    //version 1, as reference
    cv::Mat r, rvec, t;
    cv::solvePnP(pts_3, pts_2, K, D, rvec, t);
//  bool solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs,
//  OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )
    cv::Rodrigues(rvec, r);
    Matrix3d R_ref;
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
        {
            R_ref(i,j) = r.at<double>(i, j);
        }
    Quaterniond Q_ref;
    Q_ref = R_ref;
    nav_msgs::Odometry odom_ref;
    odom_ref.header.stamp = frame_time;
    odom_ref.header.frame_id = "world";
    odom_ref.pose.pose.position.x = t.at<double>(0, 0);
    odom_ref.pose.pose.position.y = t.at<double>(1, 0);
    odom_ref.pose.pose.position.z = t.at<double>(2, 0);
    odom_ref.pose.pose.orientation.w = Q_ref.w();
    odom_ref.pose.pose.orientation.x = Q_ref.x();
    odom_ref.pose.pose.orientation.y = Q_ref.y();
    odom_ref.pose.pose.orientation.z = Q_ref.z();
    pub_odom_ref.publish(odom_ref);

    // version 2, your work
    Matrix3d R;
    Vector3d T;
    R.setIdentity();
    T.setZero();
    //ROS_INFO("write your code here!");

	//cout<<"time"<<frame_time<<endl;
    vector<cv::Point2f> un_pts_2;
    cv::undistortPoints(pts_2, un_pts_2, K, D);

    MatrixXd A(2*pts_3.size(),9);
    for(unsigned int i=0;i<pts_3.size();i++)
    {
		A(2*i,0)=pts_3[i].x;
		A(2*i,1)=pts_3[i].y;
		A(2*i,2)=1;
		A(2*i,3)=0;
		A(2*i,4)=0;
		A(2*i,5)=0;
		A(2*i,6)=-pts_3[i].x*un_pts_2[i].x;
		A(2*i,7)=-pts_3[i].y*un_pts_2[i].x;
		A(2*i,8)=-un_pts_2[i].x;

		A(2*i+1,0)=0;
		A(2*i+1,1)=0;
		A(2*i+1,2)=0;
		A(2*i+1,3)=pts_3[i].x;
		A(2*i+1,4)=pts_3[i].y;
		A(2*i+1,5)=1;
		A(2*i+1,6)=-pts_3[i].x*un_pts_2[i].y;
		A(2*i+1,7)=-pts_3[i].y*un_pts_2[i].y;
		A(2*i+1,8)=-un_pts_2[i].y;
    }

    JacobiSVD<MatrixXd> svd1(A, ComputeFullU | ComputeFullV);
    VectorXd h(9);
	h=svd1.matrixV().col(8);

    Matrix3d H;
    for(unsigned int i=0;i<3;i++)
		for(unsigned int j=0;j<3;j++)
       	    H(i,j)=h[3*i+j];

    Matrix3d K_eigen;
    for(int i = 0; i < 3; i ++)
		for(int j = 0; j < 3; j ++)
	    	K_eigen(i,j) = K.at<double>(i,j);

    MatrixXd H2=H;
    H = H2;
    
    Vector3d h1 = H.col(0);
    Vector3d h2 = H.col(1);
    Vector3d h3=h1.cross(h2);
    H<<h1, h2, h3;

    JacobiSVD<MatrixXd> svd(H, ComputeThinU | ComputeThinV);
    R=svd.matrixU()*svd.matrixV().transpose();
    T=H2.col(2)/h1.norm();

    Quaterniond Q_yourwork;

    nav_msgs::Odometry odom_yourwork;
    odom_yourwork.header.stamp = frame_time;
    odom_yourwork.header.frame_id = "world";
	if(T(2)<0)
	{
		T*=-1;
		R.col(0) = -R.col(0);
		R.col(1) = -R.col(1);
	}
    Q_yourwork = R;
    odom_yourwork.pose.pose.position.x = T(0);
    odom_yourwork.pose.pose.position.y = T(1);
    odom_yourwork.pose.pose.position.z = T(2);
    odom_yourwork.pose.pose.orientation.w = Q_yourwork.w();
    odom_yourwork.pose.pose.orientation.x = Q_yourwork.x();
    odom_yourwork.pose.pose.orientation.y = Q_yourwork.y();
    odom_yourwork.pose.pose.orientation.z = Q_yourwork.z();
    pub_odom_yourwork.publish(odom_yourwork);

    rms = sqrt(rms*rms + (T(0)-t.at<double>(0, 0))*(T(0)-t.at<double>(0, 0))+(T(1)-t.at<double>(1, 0))*(T(1)-t.at<double>(1, 0))+(T(2)-t.at<double>(2, 0))*(T(2)-t.at<double>(2, 0)));
    cout<<"rms = "<<rms<<endl;
}

cv::Point3f getPositionFromIndex(int idx, int nth)
{
    int idx_x = idx % 6, idx_y = idx / 6;
    double p_x = idx_x * MarkerWithMargin - (3 + 2.5 * 0.2) * MarkerSize;
    double p_y = idx_y * MarkerWithMargin - (12 + 11.5 * 0.2) * MarkerSize;
    return cv::Point3f(p_x + (nth == 1 || nth == 2) * MarkerSize, p_y + (nth == 2 || nth == 3) * MarkerSize, 0.0);
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    double t = clock();
    cv_bridge::CvImagePtr bridge_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    MDetector.detect(bridge_ptr->image, Markers);
    float probDetect = TheBoardDetector.detect(Markers, TheBoardConfig, TheBoardDetected, CamParam, MarkerSize);
    ROS_DEBUG("p: %f, time cost: %f\n", probDetect, (clock() - t) / CLOCKS_PER_SEC);

    vector<int> pts_id;
    vector<cv::Point3f> pts_3;
    vector<cv::Point2f> pts_2;
    for (unsigned int i = 0; i < Markers.size(); i++)
    {
        int idx = TheBoardConfig.getIndexOfMarkerId(Markers[i].id);

        char str[100];
        sprintf(str, "%d", idx);
        cv::putText(bridge_ptr->image, str, Markers[i].getCenter(), CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        for (unsigned int j = 0; j < 4; j++)
        {
            sprintf(str, "%d", j);
            cv::putText(bridge_ptr->image, str, Markers[i][j], CV_FONT_HERSHEY_COMPLEX, 0.4, cv::Scalar(-1));
        }

        for (unsigned int j = 0; j < 4; j++)
        {
            pts_id.push_back(Markers[i].id * 4 + j);
            pts_3.push_back(getPositionFromIndex(idx, j));
            pts_2.push_back(Markers[i][j]);
        }
    }

    //begin your function
    if (pts_id.size() > 5)
        process(pts_id, pts_3, pts_2, img_msg->header.stamp, rms);

    cv::imshow("in", bridge_ptr->image);
    cv::waitKey(10);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tag_detector");
    ros::NodeHandle n("~");

    ros::Subscriber sub_img = n.subscribe("image_raw", 100, img_callback);
    pub_odom_yourwork = n.advertise<nav_msgs::Odometry>("odom_yourwork",10);
    pub_odom_ref = n.advertise<nav_msgs::Odometry>("odom_ref",10);
    //init aruco detector
    string cam_cal, board_config;
    n.getParam("cam_cal_file", cam_cal);
    n.getParam("board_config_file", board_config);
    CamParam.readFromXMLFile(cam_cal);
    TheBoardConfig.readFromFile(board_config);

    //init intrinsic parameters
    cv::FileStorage param_reader(cam_cal, cv::FileStorage::READ);
    param_reader["camera_matrix"] >> K;
    param_reader["distortion_coefficients"] >> D;

    //init window for visualization
    cv::namedWindow("in", 1);

    ros::spin();
}
