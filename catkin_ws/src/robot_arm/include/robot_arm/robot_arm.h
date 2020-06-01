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
        working_space,
        //関節空間
        joint_space
    };
    enum class RouteProfile{
        //LCPB
        lcpb,
        //時間3次多項式
        cubic_polynomial,
        //時間5次多項式
        quintic_polynomial
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
                double param_tt = time_now / time_total;
                 switch(user_profile){
                    //LCPB
                    case profile::lcpb:
                        double param_vel_max = 10;
                        double tb = time_total - 1 / param_vel_max;
                        double param_accel = (param_vel_max * param_vel_max) / (param_vel_max * time_total - 1);
                        if(time_now < tb){
                            param_s = param_accel * time_now * time_now / 2; 
                        }else if(time_now < time_total - tb){
                            param_s = param_vel_max * (time_now - tb / 2);
                        }else{
                            param_s = 1 - a * (time_total - time_now) * (time_total - time_now) / 2;
                        }
                        break;
                    //時間3次多項式
                    case profile::cubic_polynomial:
                        param_s = -2 * param_tt * param_tt * param_tt + 3 * param_tt * param_tt;
                        break;
                    //時間5次多項式
                    case profile::quintic_polynomial:
                        param_s = 6 * std::pow(param_tt, 5) - 15 * std::pow(param_tt, 4) + 10 * std::pow(param_tt, 3);
                        break;
                    default:
                        param_s = param_tt;
                        break;
                }
                switch(user_method){
                    //関節空間
                    case method::joint_space:
                        Vector3d theta_start = invKinema(pos_start);
                        Vector3d thtea_end = invKinema(pos_end);
                        pos_fingers =  theta_start * (1 - param_s) + thtea_end * s;
                        break;
                    //作業空間
                    case method::working_space:
                        pos_fingers = pos_start * (1 - param_s) + pos_end * s;
                        break;
                    default:
                        pos_fingers = pos_start * (1 - param_s) + pos_end * s;
                        break;
                }
                return ;
            }
            Vector3d directKinema(Vector3d pos){
                double x = cos(pos(0)) * (-L1 * sin(pos(1) - L2 * sin(pos(1) + pos(2))));
                double y = sin(pos(0)) * (-L1 * sin(pos(1) - L2 * sin(pos(1) + pos(2))));
                double z = L1 * cos(pos(1)) + L2 * cos(pos(1) + pos(2));
                return Vector3d(x, y, z);
            }
            Vector3d invKinema(Vector3d pos){
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
