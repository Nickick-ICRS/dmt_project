#include "glove_position/pose_finder.hpp"

PoseFinder::PoseFinder() :nh_("~") {
    // Get positions of LEDs on glove
    float temp;
    if(!nh_.getParam("green_x", temp))
        ROS_FATAL("Failed to get param 'green_x'");
    led_pos_.A.x = temp;
    if(!nh_.getParam("green_y", temp))
        ROS_FATAL("Failed to get param 'green_y'");
    led_pos_.A.y = temp;
    if(!nh_.getParam("green_z", temp))
        ROS_FATAL("Failed to get param 'green_z'");
    led_pos_.A.z = temp;

    if(!nh_.getParam("blue_1_x", temp))
        ROS_FATAL("Failed to get param 'blue_1_x'");
    led_pos_.B.x = temp;
    if(!nh_.getParam("blue_1_y", temp))
        ROS_FATAL("Failed to get param 'blue_1_y'");
    led_pos_.B.y = temp;
    if(!nh_.getParam("blue_1_z", temp))
        ROS_FATAL("Failed to get param 'blue_1_z'");
    led_pos_.B.z = temp;

    if(!nh_.getParam("blue_2_x", temp))
        ROS_FATAL("Failed to get param 'blue_2_x'");
    led_pos_.C.x = temp;
    if(!nh_.getParam("blue_2_y", temp))
        ROS_FATAL("Failed to get param 'blue_2_y'");
    led_pos_.C.y = temp;
    if(!nh_.getParam("blue_2_z", temp))
        ROS_FATAL("Failed to get param 'blue_2_z'");
    led_pos_.C.z = temp;

    // Constants to adjust for linear errors in x, y, z, theta
    if(!nh_.param<float>("x_grad", x_grad_, 1))
        ROS_WARN("Failed to get param 'x_grad', defaulting to %d", x_grad_);
    if(!nh_.param<float>("x_intercept", x_intercept_, 1))
        ROS_WARN("Failed to get param 'x_intercept', defaulting to %d",
                 x_intercept_);
    if(!nh_.param<float>("y_grad", y_grad_, 1))
        ROS_WARN("Failed to get param 'y_grad', defaulting to %d", y_grad_);
    if(!nh_.param<float>("y_intercept", y_intercept_, 1))
        ROS_WARN("Failed to get param 'y_intercept', defaulting to %d",
                 y_intercept_);
    if(!nh_.param<float>("z_grad", z_grad_, 1))
        ROS_WARN("Failed to get param 'z_grad', defaulting to %d", z_grad_);
    if(!nh_.param<float>("z_intercept", z_intercept_, 1))
        ROS_WARN("Failed to get param 'z_intercept', defaulting to %d",
                 z_intercept_);
    if(!nh_.param<float>("yaw_grad", yaw_grad_, 1))
        ROS_WARN("Failed to get param 'yaw_grad', defaulting to %d", 
                 yaw_grad_);
    if(!nh_.param<float>("yaw_intercept", yaw_intercept_, 1))
        ROS_WARN("Failed to get param 'yaw_intercept', defaulting to %d",
                 yaw_intercept_);

    gb1_len_ = mag(led_pos_.A - led_pos_.B);
    gb2_len_ = mag(led_pos_.A - led_pos_.C);
    bb_len_ = mag(led_pos_.B - led_pos_.C);
}

PoseFinder::~PoseFinder() {
    // dtor
}

void PoseFinder::find_pose(std::vector<LedPoint>& green_points,
                           std::vector<LedPoint>& blue_points) {
    if(green_points.size() == 0 || blue_points.size() < 2) return;

    // Store the error for each combination
    float error = 99999;
    cv::Point3f best_green;
    cv::Point3f best_blue_1;
    cv::Point3f best_blue_2;

    // Find the best error
    for(int i = 0; i < green_points.size(); i++) {
        for(int j = 0; j < blue_points.size() - 1; j++) {
            for(int k = j+1; k < blue_points.size(); k++) {
                // This seems to be unnecessary at the moment
                // It's invalid if the blue LED pair share a ray
                //if(blue_points[j].shares_a_ray(blue_points[k]))
                //    break;
                float e = find_error(green_points[i].point,
                                     blue_points[j].point,
                                     blue_points[k].point);
                // If e isnegative then b1 and b2 are in the wrong order
                if(e < 0) {
                    e = -e;
                    if(e < error) {
                        error = e;
                        best_green = green_points[i].point;
                        best_blue_1 = blue_points[k].point;
                        best_blue_2 = blue_points[j].point;
                    }
                }
                else if(e < error) {
                    error = e;
                    best_green = green_points[i].point;
                    best_blue_1 = blue_points[j].point;
                    best_blue_2 = blue_points[k].point;
                }
            }
        }
    }


    // Find the pose of the glove from the points if the error changed
    if(error != 99999)
        find_glove_pose(best_green, best_blue_1, best_blue_2);
}

float PoseFinder::find_error(cv::Point3f green,
                             cv::Point3f blue_1,
                             cv::Point3f blue_2) {
    // calculate distances between the points
    float gb1, gb2, bb;
    gb1 = mag(green - blue_1);
    gb2 = mag(green - blue_2);
    bb = mag(blue_1 - blue_2);

    // There are two combinations: g->b1->b2 or g->b2->b1
    float err1, err2;
    err1 = pow(gb1 - gb1_len_, 2) + 
           pow(gb2 - gb2_len_, 2) + 
           pow(bb - bb_len_, 2);
    err2 = pow(gb2 - gb1_len_, 2) + 
           pow(gb1 - gb2_len_, 2) + 
           pow(bb - bb_len_, 2);

    // Negative tells us that the blues are reversed
    if(err1 > err2)
        return -err2;

    return err1;
}

void PoseFinder::find_glove_pose(cv::Point3f green,
                                 cv::Point3f blue_1,
                                 cv::Point3f blue_2) {
    Triangle glove;
    glove.A = green;
    glove.B = blue_1;
    glove.C = blue_2;

    // Find the rotation from the untransformed glove 
    // at 0,0,0 to the real glove
    Quaterniond q = get_quaternion(led_pos_, glove);

    // Now that we have the rotation, calculate the translation
    // Average the translation for each point
    cv::Vec3f trans = (green - q * led_pos_.A +
                       blue_1 - q * led_pos_.B +
                       blue_2 - q * led_pos_.C) / 3;

    // Asjust with empirical linear estimations of error
    trans[0] = trans[0]*x_grad_ + x_intercept_;
    trans[1] = trans[1]*y_grad_ + y_intercept_;
    trans[2] = trans[2]*z_grad_ + z_intercept_;
    
    // Calculate the pose
    geometry_msgs::TransformStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.child_frame_id = "glove";

    msg.transform.translation.x = trans[0];
    msg.transform.translation.y = trans[1];
    msg.transform.translation.z = trans[2];

    msg.transform.rotation.x = q.x;
    msg.transform.rotation.y = q.y;
    msg.transform.rotation.z = q.z;
    msg.transform.rotation.w = q.w;

    tf_broadcaster_.sendTransform(msg);

    /*

    msg.child_frame_id = "green";

    msg.transform.translation.x = green.x;
    msg.transform.translation.y = green.y;
    msg.transform.translation.z = green.z;

    msg.transform.rotation.x = 0;
    msg.transform.rotation.y = 0;
    msg.transform.rotation.z = 0;
    msg.transform.rotation.w = 1;

    tf_broadcaster_.sendTransform(msg);

    msg.child_frame_id = "blue_1";

    msg.transform.translation.x = blue_1.x;
    msg.transform.translation.y = blue_1.y;
    msg.transform.translation.z = blue_1.z;

    msg.transform.rotation.x = 0;
    msg.transform.rotation.y = 0;
    msg.transform.rotation.z = 0;
    msg.transform.rotation.w = 1;

    tf_broadcaster_.sendTransform(msg);

    msg.child_frame_id = "blue_2";

    msg.transform.translation.x = blue_2.x;
    msg.transform.translation.y = blue_2.y;
    msg.transform.translation.z = blue_2.z;

    msg.transform.rotation.x = 0;
    msg.transform.rotation.y = 0;
    msg.transform.rotation.z = 0;
    msg.transform.rotation.w = 1;

    tf_broadcaster_.sendTransform(msg); */
}

