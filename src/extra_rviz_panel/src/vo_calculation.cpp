#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp> // For cv::imshow

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cv_bridge/cv_bridge.hpp>

#include <vector>
#include <memory>

class VisualOdometry
{
private:
    cv::Mat K_;        // Camera intrinsics
    cv::Mat cam_tf;        // Camera rotation

    cv::Ptr<cv::Feature2D> fe_method;
    cv::Ptr<cv::FlannBasedMatcher> flann_;

    cv::Mat visual_T;
    cv::Mat gt_T;
    cv::Mat img_matches;
    size_t match_size;
    
public:
    bool K_received_;

    VisualOdometry(double camera_pitch_angle = 60.0)
    {
        // Initialize feature detector and FLANN matcher
        std::string feature_detector = "SIFT";  // Default feature detector
        
        // Initialize the feature detector based on the string
        if (feature_detector == "SIFT") {
            fe_method = cv::SIFT::create();
        } else if (feature_detector == "ORB") {
            fe_method = cv::ORB::create();
        } else {
            fe_method = cv::SIFT::create();  // Fallback to SIFT
        }
        
        // Initialize FLANN matcher
        flann_ = cv::FlannBasedMatcher::create();
              
        K_received_ = false;
        
        // Step 1: Define C_Cros_Ccv (OpenCV Cam to ROS-style Cam)
        // OpenCV (Ccv): X-right, Y-down, Z-forward
        // ROS-style (Cros): X-forward, Y-left, Z-up
        cv::Mat C_Cros_Ccv = (cv::Mat_<double>(3, 3) <<
             0,  0,  1,   // ROS X = CV Z
            -1,  0,  0,   // ROS Y = -CV X
             0, -1,  0);  // ROS Z = -CV Y

        // Step 2: Define C_B_Cros (ROS-style Cam to Drone Body)
        // This is the static camera pitch angle around the Y-axis.
        double angle_rad = camera_pitch_angle * M_PI / 180.0;
        cv::Mat C_B_Cros = (cv::Mat_<double>(3, 3) <<
            cos(angle_rad), 0, sin(angle_rad),
                         0, 1,              0,
           -sin(angle_rad), 0, cos(angle_rad)
        );
        
        // Step 3: Combine them to get C_B_Ccv (OpenCV Cam to Drone Body)
        cam_tf = C_B_Cros * C_Cros_Ccv;
    }

    ~VisualOdometry()
    {
        // Clean up OpenCV windows
        cv::destroyAllWindows();
    }

    void visualize_matches()
    {
        if (K_received_)
        {   
            compare_transformations();
            cv::imshow("matches", img_matches);
            cv::waitKey(1);
        }
        else
        {
            std::cerr << "Warning: Camera intrinsics not set." << std::endl;
        }
    }

    void set_K_from_CameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr& camera_info_msg)
    {
        K_ = cv::Mat(3, 3, CV_64F);
        for (int i = 0; i < 9; i++) {
            K_.at<double>(i / 3, i % 3) = camera_info_msg->k[i];
        }
        K_received_ = true;
    }

    void calculate_T_with_frames(const sensor_msgs::msg::Image::SharedPtr& image_msg1,
                                    const sensor_msgs::msg::Image::SharedPtr& image_msg2)
    {
        if (!K_received_)
        {
            std::cerr << "Warning: Waiting for camera intrinsics..." << std::endl;
            return;
        }

        // Convert ROS Image msg to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr1, cv_ptr2;
        cv::Mat frame1, frame2;
        try
        {
            cv_ptr1 = cv_bridge::toCvCopy(image_msg1, sensor_msgs::image_encodings::BGR8);
            cv::cvtColor(cv_ptr1->image, frame1, cv::COLOR_BGR2GRAY);

            cv_ptr2 = cv_bridge::toCvCopy(image_msg2, sensor_msgs::image_encodings::BGR8);
            cv::cvtColor(cv_ptr2->image, frame2, cv::COLOR_BGR2GRAY);
        }
        catch (cv_bridge::Exception& e)
        {
            std::cerr << "Error: cv_bridge exception: " << e.what() << std::endl;
            return;
        }

        // 1. Feature Detection and Description
        std::vector<cv::KeyPoint> kp1, kp2;
        cv::Mat des1, des2;
        fe_method->detectAndCompute(frame1, cv::Mat(), kp1, des1);
        fe_method->detectAndCompute(frame2, cv::Mat(), kp2, des2);

        if (des1.empty() || des2.empty())
        {
            std::cerr << "Warning: No descriptors found." << std::endl;
            return;
        }
        
        // 2. Feature Matching (FLANN)
        std::vector<std::vector<cv::DMatch>> matches;
        flann_->knnMatch(des1, des2, matches, 2); // k=2 for ratio test

        // 3. Ratio Test
        std::vector<cv::DMatch> good_matches;
        for (const auto& match_pair : matches)
        {
            if (match_pair.size() == 2 && match_pair[0].distance < 0.8 * match_pair[1].distance)
            {
                good_matches.push_back(match_pair[0]);
            }
        }

        if (good_matches.size() < 10)
        {
            std::cerr << "Warning: Not enough good matches: " << good_matches.size() << std::endl;
            return;
        }

        // 4. Get corresponding points
        std::vector<cv::Point2f> q1, q2;
        for(const auto& m : good_matches)
        {
            q1.push_back(kp1[m.queryIdx].pt);
            q2.push_back(kp2[m.trainIdx].pt);
        }

        // 5. Estimate motion
        cv::Mat E, R, t;
        E = cv::findEssentialMat(q1, q2, K_, cv::RANSAC, 0.999, 1.0);
        cv::recoverPose(E, q1, q2, K_, R, t);

        // 6. Transform from OpenCV Camera Frame to Drone Body Frame

        // Transform rotation: R_body = C * R_cam * C^T
        cv::Mat R_ros = cam_tf * R * cam_tf.t(); // C.t() is C-transpose (which is C-inverse for rotation)

        // Transform translation: t_body = C * t_cam
        cv::Mat t_ros = cam_tf * t;

        // 7. Integrate motion
        visual_T = cv::Mat::eye(4, 4, CV_64F);
        R_ros.copyTo(visual_T(cv::Rect(0, 0, 3, 3))); // Copy transformed R to T's rotation part
        t_ros.copyTo(visual_T(cv::Rect(3, 0, 1, 3))); // Copy transformed t to T's translation part

        cv::drawMatches(frame1, kp1, frame2, kp2, good_matches, img_matches);
        match_size = good_matches.size();
    }

    void calculate_gt_T(const geometry_msgs::msg::Pose pose_msg1,
                             const geometry_msgs::msg::Pose pose_msg2)
    {   
        // Convert pose1 to transformation matrix
        cv::Mat T1 = cv::Mat::eye(4, 4, CV_64F);
        
        // Extract rotation from quaternion
        tf2::Quaternion q1(
            pose_msg1.orientation.x,
            pose_msg1.orientation.y,
            pose_msg1.orientation.z,
            pose_msg1.orientation.w
        );
        tf2::Matrix3x3 m1(q1);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T1.at<double>(i, j) = m1[i][j];
            }
        }
        
        // Extract translation
        T1.at<double>(0, 3) = pose_msg1.position.x;
        T1.at<double>(1, 3) = pose_msg1.position.y;
        T1.at<double>(2, 3) = pose_msg1.position.z;

        // Convert pose2 to transformation matrix
        cv::Mat T2 = cv::Mat::eye(4, 4, CV_64F);
        
        // Extract rotation from quaternion
        tf2::Quaternion q2(
            pose_msg2.orientation.x,
            pose_msg2.orientation.y,
            pose_msg2.orientation.z,
            pose_msg2.orientation.w
        );
        tf2::Matrix3x3 m2(q2);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                T2.at<double>(i, j) = m2[i][j];
            }
        }
        
        // Extract translation
        T2.at<double>(0, 3) = pose_msg2.position.x;
        T2.at<double>(1, 3) = pose_msg2.position.y;
        T2.at<double>(2, 3) = pose_msg2.position.z;

        // Compute relative transformation: T_relative = T1^-1 * T2
        gt_T = T1.inv() * T2;
    }

    void compare_transformations()
    {
        if (visual_T.empty() || gt_T.empty())
        {
            std::cerr << "Warning: Cannot compare - transformations not computed yet." << std::endl;
            return;
        }

        // Extract rotation matrices
        cv::Mat R_visual = visual_T(cv::Rect(0, 0, 3, 3));
        cv::Mat R_gt = gt_T(cv::Rect(0, 0, 3, 3));
        
        // Extract translation vectors
        cv::Mat t_visual = visual_T(cv::Rect(3, 0, 1, 3));
        cv::Mat t_gt = gt_T(cv::Rect(3, 0, 1, 3));

        double gt_magnitude = cv::norm(t_gt);
        double visual_magnitude = cv::norm(t_visual);
        t_visual = t_visual * (gt_magnitude / visual_magnitude);

        // 1. Calculate rotation error (angle difference in degrees)
        cv::Mat R_error = R_gt.t() * R_visual; // R_gt^T * R_visual
        double trace = R_error.at<double>(0,0) + R_error.at<double>(1,1) + R_error.at<double>(2,2);
        double rotation_error_rad = std::acos(std::min(1.0, std::max(-1.0, (trace - 1.0) / 2.0)));
        double rotation_error_deg = rotation_error_rad * 180.0 / M_PI;

        // 2. Calculate translation direction error (angular difference)
        cv::Mat t_visual_norm = t_visual / cv::norm(t_visual);
        cv::Mat t_gt_norm = t_gt / cv::norm(t_gt);
        double dot_product = t_visual_norm.dot(t_gt_norm);
        double translation_angle_error_rad = std::acos(std::min(1.0, std::max(-1.0, dot_product)));
        double translation_angle_error_deg = translation_angle_error_rad * 180.0 / M_PI;

        // 3. Calculate translation magnitude error
        double t_visual_magnitude = cv::norm(t_visual);
        double t_gt_magnitude = cv::norm(t_gt);
        double translation_magnitude_error = std::abs(t_visual_magnitude - t_gt_magnitude);
        double translation_magnitude_error_percent = (translation_magnitude_error / t_gt_magnitude) * 100.0;

        // 4. Calculate Euclidean distance between translation vectors
        double translation_euclidean_error = cv::norm(t_visual - t_gt);

        // Print comparison results
        std::cout << "\n========== Transformation Comparison ==========" << std::endl;
        std::cout << "Rotation Error: " << rotation_error_deg << " degrees" << std::endl;
        std::cout << "\nTranslation:" << std::endl;
        std::cout << "  Visual Odometry: [" << t_visual.at<double>(0) << ", " 
                  << t_visual.at<double>(1) << ", " << t_visual.at<double>(2) << "]" << std::endl;
        std::cout << "  Ground Truth:    [" << t_gt.at<double>(0) << ", " 
                  << t_gt.at<double>(1) << ", " << t_gt.at<double>(2) << "]" << std::endl;
        std::cout << "  Visual Magnitude: " << t_visual_magnitude << std::endl;
        std::cout << "  GT Magnitude:     " << t_gt_magnitude << std::endl;
        std::cout << "  Direction Error:  " << translation_angle_error_deg << " degrees" << std::endl;
        std::cout << "  Magnitude Error:  " << translation_magnitude_error 
                  << " (" << translation_magnitude_error_percent << "%)" << std::endl;
        std::cout << "  Euclidean Error:  " << translation_euclidean_error << std::endl;
        std::cout << "  Number of Matches Used: " << match_size << std::endl;
        std::cout << "=============================================\n" << std::endl;
    }

    cv::Mat get_visual_T() const { return visual_T; }
    cv::Mat get_gt_T() const { return gt_T; }
};