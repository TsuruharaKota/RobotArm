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
        //関節空間
        method2
    };
    enum class RouteProfile{
        //LCPB
        profile1,
        //時間3次多項式
        profile2
        //時間5次多項式
        profile3
    };
    template<RouteMethod user_method, RouteProfile user_profile>
    class RobotArm{
        public:
            //コンストラクタ
            RobotArm(Vector3d _pos_start, Vector3d _pos_end, double time_total, constexpr float _L1, constexpr float _L2) : pos_start(_pos_start), pos_end(_pos_end), time_total(_time_total), L1(_L1), L2(_L2){
            }
            Vector3d operator()(double time_now){
                //実際の軌道の計算
                //逐一データを渡してあげる
                //タイマーで管理する
                double param_s{};
                Vector3d pos_fingers;
                 switch(user_profile){
                    //LCPB
                    case profile::profile1:
                    param_s = ;
                        break;
                    //時間3次多項式
                    case profile::profile2:
                    param_s = ;
                        break;
                    //時間5次多項式
                    case profile::profile3:
                    param_s = ;
                        break;
                    default:
                    param_s = time_now / time_total;
                        break;
                }
                switch(user_method){
                    //関節空間
                    case method::method1:
                        pos_fingers=  pos_start * (1 - param_s) + pos_end * s;
                        break;
                    //作業空間
                    case method::method2:
                        pos_fingers = pos_start * (1 - param_s) + pos_end * s;
                        break;
                    default:
                        pos_fingers = pos_start * (1 - param_s) + pos_end * s;
                        break;
                }
                //逆運動学の計算
                Vector3d calc;
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
            constexpr Vector3d pos_start;
            constexpr Vector3d pos_end;
            constexpr time_total;
    };
}
#endif
