#include<robot_arm/robot_arm.h>
#include<ros/ros.h>
#include<tuple>
#include<chrono>
#include<Eigen/Dense>

using namespace Eigen;
using std::chrono;
using arm_tuple = std::tuple<float, float, float>;

int main(int argc, char **argv){
    //腕の長さ
    constexpr float L1 = 100;
    constexpr float L2 = 100;
    //スタート位置
    constexpr Vector3d pos_start(0, 0, 0);
    //移動後の目標座標
    constexpr Vector3d pos_end(100, 100, 100); 
    ros::init(argc, argv, "robot_arm");
    ros::NodeHandle n;
    ros::Rate loop_rate(400);

    double timer_start = chrono::system_clock::now();

    Arm3Dof::RobotArm3Dof robotModel<method1, profile1>(pos_start, pos_end, L1, L2);

    while(ros::ok()){
        double timer_now = chrono::system_clock::now();
        //経過時間をミリ秒で取得
        double time_elapsed = std::chrono::duration_cast<chrono::milliseconds>(timer_now - timer_start).count();

        //間接角を取得
        Vector3d pos_current = robotModel(std::move(time_elapsed));
        ros::spinOnce();
        loop_rate.sleep();
    }
}