//
// Created by user on 24-7-10.
//

#include "tool.h"


Eigen::Matrix3d SuperTool::RotateMatrix3d(const int degree, const Eigen::Vector3d& axis)
{
    Eigen::Matrix3d R;
    const double angle = M_PI / (180.0/degree);
    R = Eigen::AngleAxisd(angle, axis);
    return R;
}

void SuperTool::STPrint(const Eigen::Matrix3d& R)
{
    std::cout << "Rotation: " << std::endl;
    std::cout << R << std::endl;
    std::cout << "Quaternion: " << std::endl;
    Eigen::Quaterniond q(R);
    std::cout << " w: " << q.coeffs().w()
              << " x: " << q.coeffs().x()
              << " y: " << q.coeffs().y()
              << " z: " << q.coeffs().z() << std::endl;
    std::cout << ">>>>>>>>>><<<<<<<<<<" << std::endl;

}

void SuperTool::STPrint(const Eigen::Vector3d& t)
{
    std::cout << "translation：" << std::endl;
    std::cout << t.transpose() << std::endl;
    std::cout << ">>>>>>>>>><<<<<<<<<<" << std::endl;
}

void SuperTool::external_process()
{
    LOG(INFO) << "start process external!!!!";
    /*Eigen::Matrix3d R1 = RotateMatrix3d(-90,Eigen::Vector3d(0,0,1));
    Eigen::Matrix3d R2 = RotateMatrix3d(180, Eigen::Vector3d(1,0,0));
    Eigen::Matrix3d Rbi = R1 * R2;
    Eigen::Vector3d tbi = Eigen::Vector3d(0.29631,0.01045,0.3444);
    Eigen::Vector3d tbs = Eigen::Vector3d(0.24925,0,0);

    Eigen::Matrix3d R1_;
    R1_ << 1.0,0.0,0.0,
            0.0,-1.0,0.0,
            0.0, 0.0, -1.0;

    // Front
    R1 = RotateMatrix3d(90,Eigen::Vector3d(0,0,1));
    R2 = RotateMatrix3d(-65, Eigen::Vector3d(0,1,0));
    Eigen::Matrix3d front_Ric = R1* R2;
    Eigen::Vector3d front_tic = Eigen::Vector3d(-0.01065,-0.06309,-0.11805);
    front_Ric = R1_ * front_Ric;
    front_tic = R1_ * front_tic;
    Eigen::Matrix3d front_Rbc = Rbi * front_Ric;
    Eigen::Vector3d front_tsc = Rbi * front_tic +  tbi - tbs;
    // STPrint(front_Rbc);
    // STPrint(front_tsc);
    STPrint(front_Ric);
    STPrint(front_tic);

    // Right
    R1 = RotateMatrix3d(90,Eigen::Vector3d(0,0,1));
    R2 = RotateMatrix3d(65,Eigen::Vector3d(1,0,0));
    Eigen::Matrix3d right_Ric = R1 * R2;
    Eigen::Vector3d right_tic = Eigen::Vector3d(0.24072, 0.13982, -0.19439);
    right_Ric = R1_ * right_Ric;
    right_tic = R1_ * right_tic;
    Eigen::Matrix3d right_Rbc = Rbi * right_Ric;
    Eigen::Vector3d right_tsc = Rbi * right_tic + tbi - tbs;

    STPrint(right_Ric);
    STPrint(right_tic);
    // STPrint(right_Rbc);
    // STPrint(right_tsc);

    Eigen::Matrix3d Rsi;
    Eigen::Vector3d tsi;
    Eigen::Matrix3d R_ci;
    Eigen::Vector3d t_ci;
    R_ci = right_Ric.transpose();
    t_ci = -R_ci * right_tic;
    Rsi = Eigen::Matrix3d::Identity() * right_Rbc * R_ci;
    tsi = Eigen::Matrix3d::Identity() * right_Rbc * t_ci + right_tsc;
    // STPrint(Rsi);
    // STPrint(tsi);

    // Left
    R1 = RotateMatrix3d(-90, Eigen::Vector3d(0, 0, 1));
    R2 = RotateMatrix3d(65, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d left_Ric = R1 * R2;
    Eigen::Vector3d left_tic = Eigen::Vector3d(-0.24072, 0.13983, -0.19439);
    left_Ric = R1_ * left_Ric;
    left_tic = R1_ * left_tic;
    Eigen::Matrix3d left_Rbc = Rbi * left_Ric;
    Eigen::Vector3d left_tsc = Rbi * left_tic + tbi - tbs;
    /*STPrint(left_Rbc);
    STPrint(left_tsc);#1#
    STPrint(left_Ric);
    STPrint(left_tic);

    // Rear
    R1 = RotateMatrix3d(180, Eigen::Vector3d(0, 0, 1));
    R2 = RotateMatrix3d(65, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rear_Ric = R1 * R2;
    Eigen::Vector3d rear_tic = Eigen::Vector3d(-0.01045, 0.49434, -0.17166);
    rear_Ric = R1_ * rear_Ric;
    rear_tic = R1_ * rear_tic;
    Eigen::Matrix3d rear_Rbc = Rbi * rear_Ric;
    Eigen::Vector3d rear_tsc = Rbi * rear_tic + tbi - tbs;
    /*STPrint(rear_Rbc);
    STPrint(rear_tsc);#1#
    STPrint(rear_Ric);
    STPrint(rear_tic);*/

    Eigen::Matrix3d R_1,R_2;
    Eigen::Vector3d t_1,t_2;
    R_1 << 0.008446082421512235, -0.9979061557182723, 0.06412462920987821,
            0.8283978778596589, -0.028934691477022656, -0.5593922948939674,
            0.5600764408983285, 0.05784538018463403, 0.8264189569110527;
    R_2 << 0.031248975759694708, -0.9930630662803406, -0.11335452308513289,
        0.7901657671268274, 0.09399622688162107, -0.60564244385024,
        0.6120960398294064, -0.07064315764394125, 0.7876217253880513;
    t_1 << 0.049294275102045794, -0.07488867919443268, -0.09032965476043249;
    t_2 << 0.010124664008993848, -0.04495730905499684, -0.10534417945246377;
    Eigen::Matrix3d Rbc1,Rbc2;
    Eigen::Vector3d tbc1,tbc2;
    Rbc1 = R_1.transpose();
    Rbc2 = R_2.transpose();
    tbc1 = -Rbc1 * t_1;
    tbc2 = -Rbc2 * t_2;
    STPrint(Rbc1);
    STPrint(tbc1);
    STPrint(Rbc2);
    STPrint(tbc2);
    /*Eigen::Matrix3d Rc2c1;
    Eigen::Vector3d tc2c1;
    Rc2c1 = Rbc2.transpose() * Rbc1;
    tc2c1 = Rbc2.transpose() * tbc1 - Rbc2.transpose() * tbc2;
    STPrint(tc2c1);*/
    
    /*Eigen::Matrix3d R_after, R_befor;
    R_after << -0.075515241534, -0.996607252244, 0.032732752255,
        0.303567485248, 0.00829106845, 0.952773866185,
        -0.949812734278, 0.081885547919, 0.301911455307;
    R_befor << 0.075515241534,-0.996607252244 , -0.032732752255,
           -0.303567485248, 0.00829106845, -0.952773866185 ,
           0.949812734278, 0.081885547919, -0.301911455307;
    STPrint(R_after);
    STPrint(R_befor);*/
}

void SuperTool::generate_mask()
{
    const std::string IMAGE_PATH = "/home/user/workspace/test/SuperTool/fisheye_mask_tum.jpg";
    const cv::Mat src_img = cv::imread(IMAGE_PATH);

    cv::Mat dst_img;
    cv::threshold(src_img, dst_img, 20, 255, cv::THRESH_BINARY);

    cv::imshow("Original Image", src_img);
    cv::imshow("Binary Image", dst_img);
    cv::imwrite("/home/user/fisheye_mask_tum.jpg",dst_img);
    cv::waitKey(0);
}
