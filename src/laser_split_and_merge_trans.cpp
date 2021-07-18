#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"
#include <cmath>
#include <iostream>
#include <vector>
#include <algorithm>
#include <ceres/ceres.h>
#include <tf/transform_broadcaster.h>
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"

using namespace std;

struct CURVE_FITTING_COST
{
    CURVE_FITTING_COST ( double x, double y ) : _x ( x ), _y ( y ) {}
    template <typename T>
    bool operator() (
            const T* const ab,
            T* residual ) const
    {
        residual[0] = T ( _y ) - ab[0]*T ( _x )  - ab[1]; // y-(ax+b)
        return true;
    }
    const double _x, _y;
};

void CeresLineFit(vector<double> data_x, vector<double> data_y, unsigned long data_n, vector<double> &vResult)
{
    double ab[2] = {0,0};
    ceres::Problem problem;
    for ( unsigned long i=0; i<data_n; i++ )
    {
        problem.AddResidualBlock (
                new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 2> (
                        new CURVE_FITTING_COST ( data_x[i], data_y[i] )
                ),
                nullptr,
                ab
        );
    }
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve ( options, &problem, &summary );
    vResult.push_back(ab[0]);
    vResult.push_back(ab[1]);
}

double pointToLine(double A, double B, double C, double x0, double y0){
    return fabs(A * x0 + B * y0 + C) / sqrt(A * A + B * B);
}

double cal_angle_cos(double a1, double b1, double a2, double b2){
    return acos(fabs(a1 * a2 + b1 * b2) / sqrt(a1 * a1 + b1 * b1) / sqrt(a2 * a2 + b2 * b2));
}

typedef struct Point{
    double x;
    double y;
}Point;

typedef struct PointWithIndex{
    int index;
    double x;
    double y;
}PointWithIndex;

typedef struct PointWithDir{
    double angle;
    double x;
    double y;
}PointWithDir;

typedef struct GoalStructure{
    double A1;
    double B1;
    double A2;
    double distance;
}GoalStructure;

class LaserTriangle
{
private:
    double goal_range{};
    double tri_len{};
    double tri_len_err{};
    double angleDif{};
    double angleDif_err{};
    double disDif{};
    double threshold{};
    int minRegionPointNum{};
public:
    LaserTriangle();
    PointWithDir PWD{};
    bool detect_flag;
    bool handle_function1(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res);
    bool handle_function2(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res);
    static double getAngle(int i, double angle_min, double angle_increment)
    { return angle_min + angle_increment * (i - 1);}
    static Point getXY(double rho, double angle)
    {
        Point p;
        p.y = rho * cos(angle);
        p.x = -rho * sin(angle);
        return p;
    }
    static double getDis(PointWithIndex point1, PointWithIndex point2){
        double deltaX = point1.x - point2.x;
        double deltaY = point1.y - point2.y;
        return sqrt(deltaX*deltaX+deltaY*deltaY);
    }

    void registerScan(const sensor_msgs::LaserScan::ConstPtr& scan_data);
};

LaserTriangle::LaserTriangle() {
    ros::param::get("~goal_range",goal_range);
    ros::param::get("~tri_len",tri_len);
    ros::param::get("~tri_len_err",tri_len_err);
    ros::param::get("~angleDif",angleDif);
    ros::param::get("~angleDif_err",angleDif_err);
    ros::param::get("~disDif",disDif);
    ros::param::get("~minRegionPointNum",minRegionPointNum);
    ros::param::get("~threshold",threshold);
    PWD.x = 0;
    PWD.y = 0;
    PWD.angle = 0;
    detect_flag = true;
}

bool LaserTriangle::handle_function1(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    ROS_INFO("Start Detect Mark");
    res.success = true;
    detect_flag = res.success;
    res.message = "Start Detect Mark";
    return true;
}

bool LaserTriangle::handle_function2(std_srvs::Trigger::Request &req,
                                     std_srvs::Trigger::Response &res)
{
    ROS_INFO("End Detect Mark");
    res.success = false;
    detect_flag = res.success;
    res.message = "End Detect Mark";
    return true;
}

void LaserTriangle::registerScan(const sensor_msgs::LaserScan::ConstPtr &scan_data) {
    int points_num = scan_data->ranges.size();
    if (detect_flag) {
        vector<PointWithIndex> usefulPoints;
        for (int i = 0; i < points_num; i++) {
            if (goal_range > (scan_data->ranges)[i] && (scan_data->ranges)[i] > scan_data->range_min) {
                double angle = getAngle(i, scan_data->angle_min, scan_data->angle_increment);
                Point p = getXY((scan_data->ranges)[i], angle);
                PointWithIndex pwi;
                pwi.index = i;
                pwi.x = p.x;
                pwi.y = p.y;
                usefulPoints.push_back(pwi);
            }
        }

        if (usefulPoints.size() < 30) {
            cout << "no point in range 1m" << endl;
        } else {
            vector<vector<int> > regions;
            vector<int> miniRegion;
            miniRegion.push_back(0);
            for (unsigned long i = 1; i < usefulPoints.size() - 1; i++) {
                miniRegion.push_back(i);
                double dis = getDis(usefulPoints[i], usefulPoints[i - 1]);
                if (dis > disDif) {
                    miniRegion.pop_back();
                    if (miniRegion.size() > minRegionPointNum) {
                        regions.push_back(miniRegion);
                    }
                    miniRegion.clear();
                    miniRegion.push_back(i);
                }
            }
            if (miniRegion.size() > minRegionPointNum) {
                regions.push_back(miniRegion);
            }

            if (regions.empty()) {
                cout << "no region in range 1m" << endl;
            } else {
                vector<vector<int> > goalRegion;
                for (auto region : regions) {
                    unsigned long p_num = region.size();
                    double x[p_num], y[p_num];
                    for (unsigned long j = 0; j < p_num; j++) {
                        x[j] = usefulPoints[region[j]].x;
                        y[j] = usefulPoints[region[j]].y;
                    }
                    double xMin = *min_element(x, x + p_num);
                    double xMax = *max_element(x, x + p_num);
                    double yMin = *min_element(y, y + p_num);
                    double yMax = *max_element(y, y + p_num);
                    double length = sqrt((xMax - xMin) * (xMax - xMin) + (yMax - yMin) * (yMax - yMin));
                    cout << "length: " << length << endl;
                    if ((tri_len + tri_len_err) > length && length > (tri_len - tri_len_err)) {
                        goalRegion.push_back(region);
                    }
                }
                if (goalRegion.empty()) {
                    cout << "no goal region found" << endl;
                } else {
                    vector<GoalStructure> line_num_in_region;
                    for (const auto& goal_region : goalRegion) {
                        vector<PointWithIndex> goalpoints;
                        vector<PointWithIndex> vertexes;
                        vector<PointWithIndex> vertexesnew;
                        //cout << "goal_region.size(): " << goal_region.size() << endl;

                        for (unsigned long j = 0; j < goal_region.size(); j++) {
                            PointWithIndex goalpoint;
                            goalpoint.x = usefulPoints[goal_region[j]].x;
                            goalpoint.y = usefulPoints[goal_region[j]].y;
                            goalpoint.index = int(j);
                            goalpoints.push_back(goalpoint);
                        }

                        vertexes.push_back(goalpoints[0]);
                        vertexes.push_back(goalpoints[goalpoints.size() - 1]);
                        vertexesnew = vertexes;
                        int flagF = 0;
                        while (flagF == 0) {
                            int vertexNum = 0;
                            for (unsigned long j = 0; j < vertexes.size() - 1; j++) {
                                double k0 = (vertexes[j + 1].y - vertexes[j].y) / (vertexes[j + 1].x - vertexes[j].x);
                                double b0 = vertexes[j + 1].y - k0 * vertexes[j + 1].x;
                                double max_dis = 0;
                                unsigned long maxindex = 0;
                                for (int k = vertexes[j].index + 1; k < vertexes[j + 1].index; k++) {
                                    double x0 = goalpoints[k].x;
                                    double y0 = goalpoints[k].y;
                                    double dist = pointToLine(k0, -1, b0, x0, y0);
                                    if (dist > max_dis) {
                                        max_dis = dist;
                                        maxindex = k;
                                    }
                                }
                                if (max_dis > threshold) {
                                    vertexesnew.insert(vertexesnew.begin() + j + 1 + vertexNum, goalpoints[maxindex]);
                                    vertexNum = vertexNum + 1;
                                }
                            }
                            vertexes = vertexesnew;
                            if (vertexNum == 0) {
                                flagF = 1;
                            }
                        }
                        vector<double> lens;
                        vector<vector<double> > Result;

                        for (unsigned long j = 0; j < vertexes.size() - 1; j++) {
                            double len = sqrt(
                                    (vertexes[j + 1].y - vertexes[j].y) * (vertexes[j + 1].y - vertexes[j].y) +
                                    (vertexes[j + 1].x - vertexes[j].x) * (vertexes[j + 1].x - vertexes[j].x));
                            lens.push_back(len);
                            vector<double> xL, yL;
                            for (int k = vertexes[j].index + 1; k < vertexes[j + 1].index; k++) {
                                xL.push_back(goalpoints[k].x);
                                yL.push_back(goalpoints[k].y);
                            }
                            vector<double> result;
                            CeresLineFit(xL, yL, xL.size(), result);
                            Result.push_back(result);
                            //result.clear();
                        }
                        //cout << "num line: " << lens.size() << endl;
                        if (lens[0] < 0.05 || (vertexes[1].index - vertexes[0].index) < 5) {
                            lens.erase(lens.begin());
                            Result.erase(Result.begin());
                        }
                        if (lens[lens.size() - 1] < 0.05 ||
                            (vertexes[vertexes.size() - 1].index - vertexes[vertexes.size() - 2].index) < 5) {
                            lens.pop_back();
                            Result.pop_back();
                        }
                        //cout << "final num line: " << lens.size() << endl;
                        for (double len : lens) {
                            cout << "len: " << len << endl;
                        }
                        for (auto &j : Result) {
                            cout << "k: " << j[0] << endl;
                            //cout<<"b: "<<j[1]<<endl;
                        }
                        vector<double> angle_difs;
                        for (unsigned long j = 0; j < Result.size() - 1; j++) {
                            double angle_dif = cal_angle_cos(Result[j][0], -1, Result[j + 1][0], -1);
                            cout << "angle_dif: " << angle_dif << endl;
                            angle_difs.push_back(angle_dif);
                        }
                        int score = 0;
                        for (double len : lens) {
                            if (len > 0.1 && len < 0.2) {
                                score = score + 1;
                            }
                        }

                        for (double angle_dif : angle_difs) {
                            if ((angle_dif > 0.45 && angle_dif < 0.65) ||
                                (angle_dif > 0.95 && angle_dif < 1.15)) {
                                score = score + 1;
                            }
                        }

                        cout << "score: " << score << endl;

                        if (score >= 5) {
                            if (lens.size() == 4) {
                                double x_corner_fit = (-Result[2][1] + Result[1][1]) / (-Result[1][0] + Result[2][0]);
                                double y_corner_fit =
                                        (Result[1][1] * Result[2][0] - Result[2][1] * Result[1][0]) /
                                        (-Result[1][0] + Result[2][0]);
                                cout << "corner_fit(x,y): " << x_corner_fit << ", " << y_corner_fit << endl;
                                double x_left;
                                if (Result[2][0] > 0 || (Result[2][0] < 0 && Result[1][0] < 0) || (x_corner_fit>0&&Result[2][0] < 0 && Result[1][0] > 0)) {
                                    x_left = x_corner_fit - 0.1;
                                } else {
                                    x_left = x_corner_fit + 0.1;
                                }
                                double y_left = Result[2][0] * x_left + Result[2][1];
                                double x_right;
                                if (Result[2][0] < 0 && Result[1][0] > 0 && x_corner_fit>0) {
                                    x_right = x_corner_fit - 0.1;
                                } else {
                                    x_right = x_corner_fit + 0.1;
                                }
                                //x_right = x_corner_fit + 0.1;
                                double y_right = Result[1][0] * x_right + Result[1][1];
                                double vector_left[] = {x_left - x_corner_fit, y_left - y_corner_fit};
                                double vector_right[] = {x_right - x_corner_fit, y_right - y_corner_fit};
                                double angle_left = atan2(vector_left[1], vector_left[0]);
                                double angle_right = atan2(vector_right[1], vector_right[0]);
                                //cout << angle_left <<" "<< angle_right << endl;
                                double angle_dift;
                                if (angle_left > 0 && angle_right < 0) {
                                    angle_dift = 2*M_PI - angle_left + angle_right;
                                } else {
                                    angle_dift = fabs(angle_left - angle_right);
                                }
                                double angle_mid;
                                if (angle_left<0 && angle_right<0){
                                    angle_mid = (angle_left + angle_right) / 2.0;
                                } else if(angle_left>0 && angle_right<0){
                                    angle_mid = angle_right - angle_dift/2;
                                } else{
                                    angle_mid = angle_left + angle_dift/2;
                                }

                                cout << "angle_dif: " << angle_dift << endl;
                                if ((angleDif + angleDif_err) > angle_dift && angle_dift > (angleDif - angleDif_err)) {
                                    GoalStructure gs;
                                    gs.A1 = x_corner_fit;
                                    gs.A2 = y_corner_fit;
                                    gs.B1 = angle_mid;
                                    gs.distance = x_corner_fit*x_corner_fit + y_corner_fit*y_corner_fit;
                                    line_num_in_region.push_back(gs);
                                }
                            }
                        }
                    }
                    if (line_num_in_region.empty()) {
                        cout << "no final goal detected" << endl;
                    }
                    else if (line_num_in_region.size() == 1) {
                        GoalStructure gs = line_num_in_region[0];
                        PWD.x = gs.A1;
                        PWD.y = gs.A2;
                        PWD.angle = gs.B1;
                        double angle_degree = gs.B1 * 180.0 / M_PI;
                        cout << "direction /degree: " << angle_degree << endl;
                    }
                    else {
                        double min_dis = line_num_in_region[0].distance;
                        unsigned long ind = 0;
                        for(unsigned long i=1;i<line_num_in_region.size();i++){
                            if (min_dis>line_num_in_region[i].distance){
                                min_dis = line_num_in_region[i].distance;
                                ind = i;
                            }
                        }
                        GoalStructure gs = line_num_in_region[ind];
                        PWD.x = gs.A1;
                        PWD.y = gs.A2;
                        PWD.angle = gs.B1;
                        double angle_degree = gs.B1 * 180.0 / M_PI;
                        cout << "direction /degree: " << angle_degree << endl;
                    }
                }
            }
        }
    }
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(msg->x+0.40834389165276, -msg->y, 0.0887514391572773) );
    tf::Quaternion q;
    q.setRPY(M_PI, 0, -msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_footprint", "triangle_ref"));
}

int main(int argc,char **argv)
{
    ros::init(argc, argv, "laser_test_trans");
    ros::NodeHandle n;
    LaserTriangle laserTriangle;
    ros::Subscriber sub = n.subscribe("/scan_1", 10, &LaserTriangle::registerScan, &laserTriangle);
    ros::Publisher goalPosAnglePub = n.advertise<geometry_msgs::Pose2D>("/pos_angle", 10);
    ros::Publisher statePub = n.advertise<std_msgs::Bool>("/detect_state", 10);
    ros::Subscriber sub1 = n.subscribe("/pos_angle", 10, &poseCallback);
    ros::ServiceServer ss1 = n.advertiseService("start_detect", &LaserTriangle::handle_function1, &laserTriangle);
    ros::ServiceServer ss2 = n.advertiseService("end_detect", &LaserTriangle::handle_function2, &laserTriangle);
    ros::Rate loop_rate(20);
    while (ros::ok())
    {
        geometry_msgs::Pose2D posAngle;
        posAngle.y = laserTriangle.PWD.x;
        posAngle.x = laserTriangle.PWD.y;
        posAngle.theta = -(laserTriangle.PWD.angle+M_PI_2);
        goalPosAnglePub.publish(posAngle);

        std_msgs::Bool state;
        state.data = laserTriangle.detect_flag;
        statePub.publish(state);

        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::spin();

    return 0;
}


