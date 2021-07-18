#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <iostream>
#include <tf/transform_broadcaster.h>

using namespace std;

double pointToLine(double A, double B, double C, double x0, double y0){
    return fabs(A * x0 + B * y0 + C) / sqrt(A * A + B * B);
}

class LaserDock
{
private:
    double last_err{};
    double dis_to_mid{};
    double kp{};
    double kd{};
    double k{};
    double b{};
    double laser2base{};
    double linearV{};
    double angleV{};
    double yDesire{};
    double dis_to_mid_err{};
    double yy{};
    double goal_x{};
    double goal_y{};
    double goal_dir{};
    geometry_msgs::Twist move_cmd{};
    geometry_msgs::Pose2D goal_pose{};
    ros::NodeHandle nh;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber sub;
    void pos_angle_callback(const geometry_msgs::Pose2D::ConstPtr& pose_data);
    double pid_control(double dis_err);
public:
    LaserDock();
    ~LaserDock();
    void move();
};

LaserDock::LaserDock() {
    last_err = 0;
    dis_to_mid = 0;
    k = 0;
    b = 0;
    yy = 0;
    sub = nh.subscribe("/pos_angle", 10, &LaserDock::pos_angle_callback, this);
    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::param::get("~kp",kp);
    ros::param::get("~kd",kd);
    ros::param::get("~laser2base",laser2base);
    ros::param::get("~linearV",linearV);
    ros::param::get("~angleV",angleV);
    ros::param::get("~yDesire",yDesire);
    ros::param::get("~dis_to_mid_err",dis_to_mid_err);
    ros::param::get("~goal_x",goal_x);
    ros::param::get("~goal_y",goal_y);
    ros::param::get("~goal_dir",goal_dir);
}


LaserDock::~LaserDock()
= default;

void LaserDock::pos_angle_callback(const geometry_msgs::Pose2D::ConstPtr& pose_data) {
    double k1 = -tan(-pose_data->theta - M_PI / 2 + M_PI);
    double x1 = pointToLine(k1, -1, 0, goal_x, -goal_y);
    double y1 = sqrt(goal_y*goal_y+goal_x*goal_x-x1*x1);
    if (goal_x<0){
        goal_pose.x = pose_data->y-x1;
    } else{
        goal_pose.x = pose_data->y+x1;
    }
    //goal_pose.x = pose_data->y;
    goal_pose.y = pose_data->x-y1;
    goal_pose.theta = (-pose_data->theta - M_PI / 2)+goal_dir+M_PI_2;
    k = tan(goal_pose.theta + M_PI);
    b = goal_pose.y - k * goal_pose.x;
    dis_to_mid = pointToLine(k, -1, b, 0, -laser2base);
    //cout<<"dis_to_mid"<<dis_to_mid<<endl;
    yy = sqrt(goal_pose.x*goal_pose.x+goal_pose.y*goal_pose.y-dis_to_mid*dis_to_mid);

}

double LaserDock::pid_control(double dis_err){
    double dis_err_diff = dis_err - last_err;
    double ang = kp * dis_err + kd * dis_err_diff;
    last_err = dis_err;
    return ang;
}

void LaserDock::move(){
    cout<<"start!"<<endl;
    int rate = 50;
    ros::Rate loop_rate(rate);
    ros::spinOnce();
    cout<<"dis_to_mid"<<dis_to_mid<<endl;
    while (dis_to_mid>dis_to_mid_err&&yy>yDesire){
        ros::spinOnce();
        double angle_speed = pid_control(dis_to_mid);
        if(angle_speed>0.3){angle_speed=0.3;}
        move_cmd=geometry_msgs::Twist();
        move_cmd.linear.x = linearV;
        double x_label = (k * k + k * (-laser2base - k - b)) / (k * k + 1);
        //cout<<"x_label: "<<x_label<<endl;
        if (x_label > 0){
            move_cmd.angular.z = -angle_speed;
        }
        else{
            move_cmd.angular.z = angle_speed;
        }
        //cout<<"angle speed: "<<angle_speed<<endl;
        cmd_vel_pub_.publish(move_cmd);
        loop_rate.sleep();
        cout<<"dis_to_mid: "<<dis_to_mid<<endl;
    }
    ros::spinOnce();
    double angle = goal_pose.theta;
    cout<<"angle: "<<angle<<endl;
    if (-1 * M_PI < angle && angle < -1 * M_PI / 2){
        double goal_angle = -1 * angle - M_PI / 2;
        double angular_duration = abs(goal_angle) / angleV;
        move_cmd=geometry_msgs::Twist();
        move_cmd.angular.z = -1 * angleV;
        int ticks = int(angular_duration * rate);
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    }
    else{
        double goal_angle = angle + M_PI / 2;
        double angular_duration = abs(goal_angle) / angleV;
        move_cmd=geometry_msgs::Twist();
        move_cmd.angular.z = angleV;
        int ticks = int(angular_duration * rate);
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    }

    move_cmd=geometry_msgs::Twist();
    move_cmd.linear.x = linearV/2;
    ros::spinOnce();

    while (goal_pose.y>yDesire){
        cmd_vel_pub_.publish(move_cmd);
        //loop_rate.sleep();
        //cout<<"goal_pose.y: "<<goal_pose.y<<endl;
        ros::spinOnce();
        //cout<<"goal_pose.y2: "<<goal_pose.y<<endl;
        if(goal_pose.y<yDesire){
            cmd_vel_pub_.publish(geometry_msgs::Twist());
        }
    }

    ros::spinOnce();
    angle = goal_pose.theta;
    cout<<"angle: "<<angle<<endl;
    if (-1 * M_PI < angle && angle < -1 * M_PI / 2){
        double goal_angle = -1 * angle - M_PI / 2;
        double angular_duration = abs(goal_angle) / angleV;
        move_cmd=geometry_msgs::Twist();
        move_cmd.angular.z = -1 * angleV;
        int ticks = int(angular_duration * rate);
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    }
    else{
        double goal_angle = angle + M_PI / 2;
        double angular_duration = abs(goal_angle) / angleV;
        move_cmd=geometry_msgs::Twist();
        move_cmd.angular.z = angleV;
        int ticks = int(angular_duration * rate);
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    }

    /*double y_label = goal_cout<<"dis_to_mid"<<dis_to_mid<<endl;pose.y;
    double goal_distance = y_label - yDesire;
    double linear_duration = goal_distance / linearV;
    int ticks = int(linear_duration * rate);
    move_cmd.linear.x = linearV;
    for (int t=0; t<ticks; t++){
        cmd_vel_pub_.publish(move_cmd);
        loop_rate.sleep();
    }*/
    /*if (goal_distance<0){
        goal_distance = -goal_distance;
        double linear_duration = goal_distance / linearV;
        int ticks = int(linear_duration * rate);
        move_cmd.linear.x = -linearV;
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    } else{
        double linear_duration = goal_distance / linearV;
        int ticks = int(linear_duration * rate);
        move_cmd.linear.x = linearV;
        for (int t=0; t<ticks; t++){
            cmd_vel_pub_.publish(move_cmd);
            loop_rate.sleep();
        }
    }*/

    move_cmd=geometry_msgs::Twist();
    cmd_vel_pub_.publish(move_cmd);
    ros::Duration(1).sleep();
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "laser_dock");
    LaserDock laserDock;
    ros::Duration(1).sleep();
    cout<<"now to move!"<<endl;
    laserDock.move();
    //ros::spin();
    return 0;
}


