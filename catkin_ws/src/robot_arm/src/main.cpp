#include<robot_arm/robot_arm.h>
#include<ros/ros.h>
#include<tuple>

using arm_tuple = std::tuple<float, float, float>;

int main(int argc, char **argv){
    //腕の長さ
    constexpr float L1 = 100;
    constexpr float L2 = 100;
    //スタート位置
    constexpr arm_tuple pos_start(0, 0, 0);
    //移動後の目標座標
    constexpr arm_tuple pos_end(100, 100, 100); 
    ros::init(argc, argv, "robot_arm");
    ros::NodeHandle n;
    ros::Rate loop_rate(400);

    Arm3Dof::RouteRobotArm robotRoute();
    Arm3Dof::RobotArm3Dof robotModel(L1, L2);

    while(ros::ok()){
        //目標手先位置を取得
        arm_tuple pos_goal = robotRoute<method1, profile1>();
        //間接角を取得
        arm_tuple pos_current = robotModel(std::move(pos_goal));
        ros::spinOnce();
        loop_rate.sleep();
    }
}