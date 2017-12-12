#include "semantic_SLAM.h"

semantic_SLAM::semantic_SLAM()
{
    init();
    std::cout << "semantic SLAM constructor " << std::endl;
}

semantic_SLAM::~semantic_SLAM()
{
    std::cout << "semantic SLAM destructor " << std::endl;
}

void semantic_SLAM::init()
{
    imu_data_available_ = false;
    prev_time_ = 0;
    particle_filter_obj_.init(state_size_);
    return;

}

void semantic_SLAM::run()
{
    current_time_ = (double) ros::Time::now().sec + ((double) ros::Time::now().nsec / (double) 1E9);

    //std::cout << "current time  " << current_time_ << std::endl;
    float time_diff;

    time_diff = current_time_ - prev_time_;
    //std::cout << "time diff " << time_diff << std::endl;

    if(imu_data_available_)
    {
        particle_filter_obj_.prediction(time_diff, imu_world_acc_mat_, imu_world_ang_vel_);
        imu_data_available_ = false;
    }

    prev_time_ = current_time_;
}

void semantic_SLAM::open(ros::NodeHandle n)
{
    //ros subsriber
    stereo_odometry_sub_ = n.subscribe("/stereo_odometer/pose", 1, &semantic_SLAM::stereoOdometryCallback, this);
    imu_sub_             = n.subscribe("/imu", 1, &semantic_SLAM::imuCallback, this);

}

void semantic_SLAM::stereoOdometryCallback(const geometry_msgs::PoseStamped &msg)
{

    Eigen::Vector4f camera_pose_mat, world_pose_mat;
    Eigen::Matrix4f transformation_mat;

    camera_pose_mat.setOnes(), world_pose_mat.setOnes();
    //assume roll, pitch and yaw zero for now//
    this->transformCameraToWorld(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,0,0,0, transformation_mat);

    camera_pose_mat(0) = msg.pose.position.x;
    camera_pose_mat(1) = msg.pose.position.y;
    camera_pose_mat(2) = msg.pose.position.z;

    world_pose_mat = transformation_mat * camera_pose_mat;

    std::cout << "world pose mat " << std::endl
              << "x: " << world_pose_mat(0) << std::endl
              << "y: " << world_pose_mat(1) << std::endl
              << "z: " << world_pose_mat(2) << std::endl;



}

void semantic_SLAM::imuCallback(const sensor_msgs::Imu &msg)
{
    //for converting the IMU from NED to world frame (ENU)
    transformation_mat_acc_.setZero(), transformation_mat_ang_vel_.setZero();
    imu_local_acc_mat_.setOnes(), imu_world_acc_mat_.setOnes();
    imu_local_ang_vel_.setOnes(), imu_world_ang_vel_.setOnes();

    imu_local_acc_mat_(0) = msg.linear_acceleration.x;
    imu_local_acc_mat_(1) = msg.linear_acceleration.y;
    imu_local_acc_mat_(2) = msg.linear_acceleration.z;

    this->transformIMUtoWorld(imu_local_acc_mat_(0), imu_local_acc_mat_(1), imu_local_acc_mat_(2), transformation_mat_acc_);

    //converting the imu acclerations in world frame
    imu_world_acc_mat_ = transformation_mat_acc_ * imu_local_acc_mat_;

    std::cout << "Imu acc in world " << std::endl
              << "ax: " << imu_world_acc_mat_(0) << std::endl
              << "ay: " << imu_world_acc_mat_(1) << std::endl
              << "az: " << imu_world_acc_mat_(2) << std::endl;


    imu_local_ang_vel_(0) = msg.angular_velocity.x;
    imu_local_ang_vel_(1) = msg.angular_velocity.y;
    imu_local_ang_vel_(2) = msg.angular_velocity.z;

    this->transformIMUtoWorld(imu_local_ang_vel_(0), imu_local_ang_vel_(1), imu_local_ang_vel_(2), transformation_mat_ang_vel_);

    imu_world_ang_vel_ = transformation_mat_acc_ * imu_local_ang_vel_;


    std::cout << "Imu angular vel in world " << std::endl
              << "wx: " << imu_world_ang_vel_(0) << std::endl
              << "wy: " << imu_world_ang_vel_(1) << std::endl
              << "wz: " << imu_world_ang_vel_(2) << std::endl;

    imu_data_available_ = true;

}

void semantic_SLAM::transformCameraToWorld(float x, float y, float z,
                                           float roll, float pitch, float yaw,
                                           Eigen::Matrix4f &transformation_mat)
{

    Eigen::Matrix4f rot_x_cam, rot_x_robot, rot_z_robot, T_robot_world, translation_cam;
    rot_x_cam.setZero(4,4), rot_x_robot.setZero(4,4), rot_z_robot.setZero(4,4), T_robot_world.setZero(4,4), translation_cam.setZero(4,4);

    //    rot_x_cam(0,0) = 1;
    //    rot_x_cam(1,1) =  cos(-camera_pitch_angle_);
    //    rot_x_cam(1,2) = -sin(-camera_pitch_angle_);
    //    rot_x_cam(2,1) =  sin(-camera_pitch_angle_);
    //    rot_x_cam(2,2) =  cos(-camera_pitch_angle_);
    //    rot_x_cam(3,3) = 1;

    //rotation of -90
    rot_x_robot(0,0) = 1;
    rot_x_robot(1,1) =  cos(-1.5708);
    rot_x_robot(1,2) = -sin(-1.5708);
    rot_x_robot(2,1) =  sin(-1.5708);
    rot_x_robot(2,2) =  cos(-1.5708);
    rot_x_robot(3,3) = 1;

    //rotation of -90
    rot_z_robot(0,0) = cos(-1.5708);
    rot_z_robot(0,1) = -sin(-1.5708);
    rot_z_robot(1,0) = sin(-1.5708);
    rot_z_robot(1,1) = cos(-1.5708);
    rot_z_robot(2,2) = 1;
    rot_z_robot(3,3) = 1;

    //    //tranlation of 0cm in
    //    translation_cam(0,0) = 1;
    //    translation_cam(0,3) = 0.0;

    //    translation_cam(1,1) = 1;
    //    translation_cam(2,2) = 1;
    //    translation_cam(3,3) = 1;


    //transformation from robot to world
    T_robot_world(0,0) = cos(yaw)*cos(pitch);
    T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

    T_robot_world(1,0) = sin(yaw)*cos(pitch);
    T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

    T_robot_world(2,0) = -sin(pitch);
    T_robot_world(2,1) = cos(pitch)*sin(roll);
    T_robot_world(2,2) = cos(pitch)*cos(roll);

    //fill the x, y and z variables over here if there is a fixed transform between the camera and the world frame
    //currently its they are all zero as the pose is obtained of the camera with respect to the world.
    //    T_robot_world(0,3) = prev_pose_x_;
    //    T_robot_world(1,3) = prev_pose_y_;
    //    T_robot_world(2,3) = prev_pose_z_;
    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * rot_z_robot * rot_x_robot ;


    //std::cout << "transformation matrix " << transformation_mat << std::endl;

}

void semantic_SLAM::transformIMUtoWorld(float ax, float ay, float az, Eigen::Matrix4f &transformation_mat)
{
    Eigen::Matrix4f rot_x_imu, T_robot_world;
    double roll, pitch, yaw;

    //for now considering roll, pitch and yaw zero
    roll = pitch = yaw =0;

    rot_x_imu.setZero(), T_robot_world.setZero();

    //rotation of -180 to convert to robot frame
    rot_x_imu(0,0) = 1;
    rot_x_imu(1,1) =  cos(-3.14);
    rot_x_imu(1,2) = -sin(-3.14);
    rot_x_imu(2,1) =  sin(-3.14);
    rot_x_imu(2,2) =  cos(-3.14);
    rot_x_imu(3,3) = 1;

    //rotation of the robot with respect to the world
    T_robot_world(0,0) = cos(yaw)*cos(pitch);
    T_robot_world(0,1) = cos(yaw)*sin(pitch)*sin(roll) - sin(yaw)*cos(roll);
    T_robot_world(0,2) = cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(pitch);

    T_robot_world(1,0) = sin(yaw)*cos(pitch);
    T_robot_world(1,1) = sin(yaw)*sin(pitch)*sin(roll) + cos(yaw)*cos(roll);
    T_robot_world(1,2) = sin(yaw)*sin(pitch)*cos(roll) - cos(yaw)*sin(roll);

    T_robot_world(2,0) = -sin(pitch);
    T_robot_world(2,1) = cos(pitch)*sin(roll);
    T_robot_world(2,2) = cos(pitch)*cos(roll);
    T_robot_world(3,3) = 1;

    transformation_mat = T_robot_world * rot_x_imu;


}
