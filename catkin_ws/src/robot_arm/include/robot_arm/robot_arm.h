#ifndef ARM_H_
#define ARM_H_
#include<cmath>
#include<tuple>
#include<array>
#include<utility>
#include</usr/local/include/eigen3/Dense>
#include<iostream>

using namespace Eigen;
using route_tuple_ini = std::tuple<std::tuple<double, double, double>, std::tuple<double, double, double>>;
using arm_tuple = std::tuple<double, double, double>;

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
            RobotArm(Vector3d _pos_start, Vector3d _pos_end, double _time_total, double _L1, double _L2) : pos_start(_pos_start), pos_end(_pos_end), time_total(_time_total), L1(_L1), L2(_L2){
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
                    case RouteProfile::lcpb:{
                        double param_vel_max = 1.0 / 8.0;
                        double tb = time_total - (1.0 / param_vel_max);
                        double param_accel = (param_vel_max * param_vel_max) / (param_vel_max * time_total - 1);
                        std::cout << "tb " << tb << std::endl;
                        std::cout << "time_now " << time_now << std::endl;
                        if(time_now < tb){
                            param_s = param_accel * time_now * time_now / 2; 
                            std::cout << "fase1 " << param_s << std::endl;
                        }else if(time_now < time_total - tb){
                            param_s = param_vel_max * (time_now - tb / 2);
                            std::cout << "fase2 " << param_s << std::endl;
                        }else if(time_now < time_total){
                            param_s = 1 - param_accel * (time_total - time_now) * (time_total - time_now) / 2;
                            std::cout << "fase3 " << param_s << std::endl;
                        }else{
                            param_s = 1.0;
                            std::cout << "fase4" << std::endl;
                        }
                        std::cout << "lcpb" << std::endl;
                        break;
                    }
                    //時間3次多項式
                    case RouteProfile::cubic_polynomial:{
                        param_s = -2 * param_tt * param_tt * param_tt + 3 * param_tt * param_tt;
                        std::cout << "cubic_polynominal" << std::endl;
                        break;
                    }
                    //時間5次多項式
                    case RouteProfile::quintic_polynomial:{
                        param_s = 6 * std::pow(param_tt, 5) - 15 * std::pow(param_tt, 4) + 10 * std::pow(param_tt, 3);
                        std::cout << "quintic_polynomial" << std::endl;
                        break;
                    }
                    default:{
                        param_s = param_tt;
                        break;
                    }
                }
                switch(user_method){
                    //関節空間
                    case RouteMethod::joint_space:{
                        Vector3d a(0, -45, -90);
                        std::cout << "directKinema" << std::endl;
                        std::cout << directKinema(a * M_PI / 180) << std::endl;
                        std::cout << "---------------" << std::endl;
                        Vector3d theta_start = invKinema(pos_start);
                        Vector3d thtea_end = invKinema(pos_end);
                        std::cout << "start" << std::endl;
                        std::cout << theta_start * 180 / M_PI << std::endl;
                        pos_fingers =  theta_start * (1 - param_s) + thtea_end * param_s;
                        std::cout << "joint_space" << std::endl;
                        break;
                    }
                    //作業空間
                    case RouteMethod::working_space:{
                        pos_fingers = pos_start * (1 - param_s) + pos_end * param_s;
                        std::cout << "working_space" << std::endl;
                        break;
                    }
                    default:{
                        pos_fingers = pos_start * (1 - param_s) + pos_end * param_s;
                        break;
                    }
                }
                return pos_fingers;
            }
            Vector3d directKinema(Vector3d pos){
                double x = cos(pos(0)) * (-L1 * sin(pos(1)) - L2 * sin(pos(1) + pos(2)));
                double y = sin(pos(0)) * (-L1 * sin(pos(1)) - L2 * sin(pos(1) + pos(2)));
                double z = (L1 * cos(pos(1))) + (L2 * cos(pos(1) + pos(2)));
                return Vector3d(x, y, z);
            }
            Vector3d invKinema(Vector3d pos){
                //逆運動学の計算
                std::array<double, 3> theta;
                double sqrt_x2y2 = std::sqrt(pos(0) * pos(0) + pos(1) * pos(1));
                double param_theta2 = ((pos(0) * pos(0)) + (pos(1) * pos(1)) + (pos(2) * pos(2)) - (L1 * L1) - (L2 * L2)) / (2 * L1 * L2);
                theta[2] = atan2(-1 * sqrt(1 - (param_theta2 * param_theta2)), param_theta2);
                theta[0] = atan2(pos(1), pos(0));
                theta[1] = atan2((-1 * (L1 + L2 * cos(theta[2])) * sqrt_x2y2) - (L2 * sin(theta[2]) * pos(2)), -1 * (L2 * sin(theta[2])
                                    * sqrt_x2y2) + ((L1 + (L2 * cos(theta[2]))) * pos(2)));
                Vector3d calc(theta[0], theta[1], theta[2]);
                return calc;
            }
        private:
            double L1;
            double L2;
            double time_total;
            Vector3d pos_start;
            Vector3d pos_end;
    };
#endif
