/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <deque>
#include <map>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

std::mutex m_buf;
sensor_msgs::msg::Imu::ConstPtr latest_measured_msg = nullptr;
sensor_msgs::msg::Imu::ConstPtr prev_measured_msg = nullptr;

deque<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;

bool is_first_imu = true;
double prev_t = 0.0;            // 이전 메시지 시각 [sec]
double last_update_t = 0.0;     // 5ms 마다 입력 전달 시각 [sec]
double update_interval = 0.005; // 업데이트 주기 [sec]
double input_t = 0.0;           // inputIMU 함수 호출 시각 [sec]

// Kalman filter 관련 변수 선언 및 초기화
double accl_x_est_ = 0.0; double accl_y_est_ = 0.0; double accl_z_est_ = 0.0;
double gyro_x_est_ = 0.0; double gyro_y_est_ = 0.0; double gyro_z_est_ = 0.0;
double accl_x_P_ = 1.0; double accl_y_P_ = 1.0; double accl_z_P_ = 1.0;
double gyro_x_P_ = 1.0; double gyro_y_P_ = 1.0; double gyro_z_P_ = 1.0;
double accl_Q_ = 0.00000009; double gyro_Q_ = 0.0001;       
double accl_R_ = 0.000064; double gyro_R_ = 0.01;
bool kf_accel_updated_ = false;
bool kf_gyro_updated_ = false;

// header: 1403715278
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    m_buf.lock();
    // std::cout << "Right: " << img_msg->header.stamp.sec << "." << img_msg->header.stamp.nanosec << endl;
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr img_msg)
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::msg::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();
                    //printf("find img0 and img1\n");

                    // std::cout << std::fixed << img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9) << std::endl;
                    // assert(0);
                    
                }
            }
            m_buf.unlock();
            if(!image0.empty())
                estimator.inputImage(time, image0, image1);
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


sensor_msgs::msg::Imu CreateInterpolatedImu(const sensor_msgs::msg::Imu::SharedPtr first_lattest_msg, const sensor_msgs::msg::Imu::SharedPtr second_lattest_msg, const double alpha)
{
    sensor_msgs::msg::Imu imu_msg;
    return imu_msg;
}

void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    // --- 이상치 주기(ex. 2, 3, 10ms)로 들어올시 kalman filter 이용한 반복적 예측을 위해 마지막 output 저장 ---
    static Vector3d last_pred_acc;
    static Vector3d last_pred_gyr;
    // --- 이상치 주기(ex. 2, 3, 10ms)로 들어올시 처음에 측정된 IMU 이용해서 예측값 산출하기 위해 저장 ---
    sensor_msgs::msg::Imu::ConstPtr kf_input_msg = nullptr;

    // 최신 메시지 저장
    latest_measured_msg = imu_msg;

    // --- 타임스탬프 계산 ---
    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * 1e-9;
    double diff_measured_t = (prev_t > 0.0) ? t - prev_t : 0.0;
    double diff_measured_t_rounded = std::round(diff_measured_t * 1000.0) / 1000.0;
    
    // 디버그 출력
    std::cout << std::fixed << "prev_t: " << prev_t << std::endl;
    std::cout << std::fixed << "t: " << t << std::endl;
    std::cout << "diff_measured_t_rounded: " << diff_measured_t_rounded << std::endl;

    // 현재 타임스탬프를 저장
    prev_t = t;
    
    // --- inputIMU 함수에 전달할 입력 변수들 ---
    Vector3d input_acc;
    Vector3d input_gyr;

    // Case 1. 첫 번째 메세지인 경우: 일단 전달
    if (is_first_imu)
    {
        // inputIMU 함수 전달 인자 초기화
        input_t = t;
        input_acc = Vector3d(imu_msg->linear_acceleration.x,
                             imu_msg->linear_acceleration.y,
                             imu_msg->linear_acceleration.z);
        input_gyr = Vector3d(imu_msg->angular_velocity.x,
                             imu_msg->angular_velocity.y,
                             imu_msg->angular_velocity.z);
        
        //마지막 inputIMU 전달인자 업데이트
        last_pred_acc = input_acc;
        last_pred_gyr = input_gyr;
        // 5ms 마다 입력 전달 시각 업데이트
        last_update_t = input_t;

        // estimator.inputIMU(input_t, input_acc, input_gyr);
        std::cout << "input_t: " << input_t << std::endl;
        std::cout << "t - last_update_t: " << t - last_update_t << std::endl;
        std::cout << "-------------------------------------------" << std::endl;

        is_first_imu = false;
    }
    else
    {
        // Case 2. 첫번째 이후, 구독 주기가 4ms 이상 6ms 이하인 경우
        if (diff_measured_t_rounded >= 0.004 && diff_measured_t_rounded <= 0.006)
        {   
            // 5ms 마다 subscribe한 IMU 토픽 데이터를 inputIMU 함수에 전달
            input_t = last_update_t + update_interval;
            input_acc = Vector3d(latest_measured_msg->linear_acceleration.x,
                                latest_measured_msg->linear_acceleration.y,
                                latest_measured_msg->linear_acceleration.z);
            input_gyr = Vector3d(latest_measured_msg->angular_velocity.x,
                                latest_measured_msg->angular_velocity.y,
                                latest_measured_msg->angular_velocity.z);

            // estimator.inputIMU(input_t, input_acc, input_gyr);

            //마지막 inputIMU 전달인자 업데이트
            last_pred_acc = input_acc;
            last_pred_gyr = input_gyr;
            // 5ms 마다 입력 전달 시각 업데이트
            last_update_t = input_t;

            std::cout << "input_t: " << input_t << std::endl;
            std::cout << "t - last_update_t: " << t - last_update_t << std::endl;
            std::cout << "-------------------------------------------" << std::endl;

        }
        // Case 3. 이상치 주기(ex. 2, 3, 10ms)로 다음 IMU 토픽 subscribe한 경우
        else
        {
            // 현재 타임스탬프에서 5ms 만큼 더한 시각을 input_t로 설정
            input_t = last_update_t + update_interval;

            // 현재 시각과 마지막 업데이트 시각의 차이 계산
            double dt = t - input_t;
            double dt_rounded = std::round(dt * 1000.0) / 1000.0;
            std::cout << "dt_rounded: " << dt_rounded << std::endl;
            std :: cout << "t: " << t << std::endl;
            std :: cout << "input_t: " << input_t << std::endl;
            while ((std::abs(dt_rounded) >= 0.005) && (t > input_t))
            {   
                
                std::cout << "루프 안" << std::endl;
                // // 업데이트 시각 결정 (마지막 업데이트 시각 + 5ms)
                // input_t = last_update_t + update_interval;

                if (!kf_input_msg)
                {
                    std::cout << "여기서 kalman filter 적용 (새로운 메시지 사용)" << std::endl;
                    kf_input_msg = prev_measured_msg;

                    // ====== 아래에서는 12ms 시점 측정된 IMU 이용해서 17ms 시점 예측값을 input으로 활용 ========
                    // kf_input_acc = Vector3d(kf_input_msg->linear_acceleration.x,
                    //                     kf_input_msg->linear_acceleration.y,
                    //                     kf_input_msg->linear_acceleration.z);
                    // kf_input_gyr = Vector3d(kf_input_msg->angular_velocity.x,
                    //                     kf_input_msg->angular_velocity.y,
                    //                     kf_input_msg->angular_velocity.z);
                    // input_acc = kalman_filter(kf_input_acc);
                    // input_gyr = kalman_filter(kf_input_gyr);
                    input_acc = Vector3d(kf_input_msg->linear_acceleration.x,
                                        kf_input_msg->linear_acceleration.y,
                                        kf_input_msg->linear_acceleration.z);
                    input_gyr = Vector3d(kf_input_msg->angular_velocity.x,
                                        kf_input_msg->angular_velocity.y,
                                        kf_input_msg->angular_velocity.z);
                }
                else
                {
                    // ====== 아래에서는 17ms 시점 예측된 IMU 이용해서 22ms 시점 예측값을 input으로 활용 ========
                    std::cout << "여기서 kalman filter 적용 (이전 예측값 사용)" << std::endl;
                    // // 이전 예측값을 그대로 사용하여 예측을 반복.
                    // // 여기서 Kalman filter를 적용하여 input_t 시점의 예측값 산출 (예: 예측 결과가 input_acc, input_gyr에 저장)
                    // input_acc = kalman_filter(last_pred_acc);
                    // input_gyr = kalman_filter(last_pred_gyr);
                    input_acc = last_pred_acc;
                    input_gyr = last_pred_gyr;
                }
                
                // 5ms 간격마다 estimator.inputIMU() 호출
                // estimator.inputIMU(input_t, input_acc, input_gyr);

                // 다음 시점의 KF 입력으로 사용하기 위해 갱신
                last_pred_acc = input_acc;
                last_pred_gyr = input_gyr;
                last_update_t = input_t;
                
                input_t = last_update_t + update_interval;

                // 남은 시간 재계산
                dt = t - input_t;
                dt_rounded = std::round(dt * 1000.0) / 1000.0;
            }
            std::cout << "루프 밖" << std::endl;
            if (std::abs(dt_rounded) < 0.005)
            // if (!((std::abs(dt_rounded) >= 0.005) && (t > input_t)))
            {
                
                std::cout << "루프 밖 조건 만족" << std::endl;
                // input_acc = Vector3d(latest_measured_msg->linear_acceleration.x,
                //                     latest_measured_msg->linear_acceleration.y,
                //                     latest_measured_msg->linear_acceleration.z);
                // input_gyr = Vector3d(latest_measured_msg->angular_velocity.x,
                //                     latest_measured_msg->angular_velocity.y,
                //                     latest_measured_msg->angular_velocity.z);

                // estimator.inputIMU(input_t, input_acc, input_gyr);

                //마지막 inputIMU 전달인자 업데이트
                last_pred_acc = input_acc;
                last_pred_gyr = input_gyr;
                // 5ms 마다 입력 전달 시각 업데이트
                last_update_t = input_t;

                std::cout << "input_t: " << input_t << std::endl;
                std::cout << "t - last_update_t: " << t - last_update_t << std::endl;
                std::cout << "-------------------------------------------" << std::endl;
            }
        }
    }

    prev_measured_msg = latest_measured_msg;  

    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    std::cout << "feature cb" << std::endl;
    std::cout << "Feature: " << feature_msg->points.size() << std::endl;


    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("vins_estimator");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");    
    // Publisher
    registerPub(n);

    rclcpp::QoS qos(rclcpp::KeepLast(5000)); 
    qos.reliable();
    // qos.best_effort();

    // Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = NULL;
    if(USE_IMU)
    {   
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, qos, imu_callback);
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img0_callback);
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1 = NULL;
    if(STEREO)
    {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, rclcpp::QoS(rclcpp::KeepLast(100)), img1_callback);
    }
    
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);
    auto sub_imu_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)), imu_switch_callback);
    auto sub_cam_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)), cam_switch_callback);

    std::thread sync_thread{sync_process};
    rclcpp::spin(n);

    return 0;
}
