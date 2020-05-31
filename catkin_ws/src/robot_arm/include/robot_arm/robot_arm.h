#ifndef ARM_H_
#define ARM_H_
#include<cmath>
#include<tuple>
#include<array>
#include<Eigen/Dense>

using namespace Eigen;
using route_tuple_ini = std::tuple<std::tuple<float, float, float>, std::tuple<float, float, float>>;
using arm_tuple = std::tuple<float, float, float>

namespace Arm3Dof{
    enum class RouteMethod{
        //作業空間
        method1,
        //間接空間
        method2
    };
    enum class RouteProfile{
        //速度台形
        profile1,
        //時間多項式
        profile2
    };
    template<RouteMethod user_method, RouteProfile user_profile>
    class RouteRobotArm{
        public:
            //コンストラクタ
            RouteRobotArm(arm_tuple _pos_start, arm_tuple _pos_end):pos_start(_pos_start), pos_end(_pos_end)const{
                generateRoute();
            }
            void generateRoute(){
                switch(user_method){
                    case method::method1:
                        break;
                    case method::method2:
                        break;
                    default:
                        break;
                }
                switch(user_profile){
                    case profile::profile1:
                        break;
                    case profile::profile2:
                        break;
                    default:
                        break;
                }
            }
            arm_tuple operator()(){
                //実際の軌道の計算
                //逐一データを渡してあげる
                //タイマーで管理する
            }
        private:
            constexpr arm_tuple pos_start;
            constexpr arm_tuple pos_end;
    };
    class RobotArm{
        public:
            //コンストラクタ
            RobotArm(constexpr float _L1, constexpr float _L2) : L1(_L1), L2(_L2){
            }
            arm_tuple operator()(arm_tuple pos_fingers){
                //逆運動学の計算
                arm_tuple calc;
                std::array<float> theta[3]{};
                sqrt_x2y2 = std::sqrt(std::get<0>(pos_fingers) * std::get<0>(pos_fingers) + std::get<1>(pos_fingers) * std::get<1>(pos_fingers))
                theta[2] = acos((std::pow(std::get<0>(pos_fingers), 2) + std::pow(std::get<1>(pos_fingers), 2) + std::pow(std::get<2>(pos_fingers), 2)) - L1 * L1 - L2 * L2) / (2 * L1 * _L2)
                theta[0] = atan2(std::get<1>(pos_fingers), std::get<0>(pos_fingers));
                theta[1] = atan2(-1 * (L2 * sin(theta[2]) * sqrt_x2y2 + (L1 + L2 * cos(theta[2]) * std::get<2>(pos_fingers) + (L1 + L2 * cos(theta[2]) * sqrt_x2y2 + L2 * sin(theta[2]) * std::get<2>(pos_fingers)));
                calc = (theta[0], theta[1], theta[2]);
                return std::move(calc);
            }
        private:
            constexpr float L1;
            constexpr float L2;
    };
}
#endif
