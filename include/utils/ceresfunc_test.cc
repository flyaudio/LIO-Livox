#include "test/entrance.h"

TEST(ceresfunc, Cost_Initial_G) {
    // 生成真实四元数（假设传感器坐标系绕x轴旋转30度）
    const double angle = M_PI/6; // 30度
    Eigen::Quaterniond q_truth(Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()));
    q_truth.normalize();

    // 生成模拟加速度计数据（无噪声）
    Eigen::Vector3d g_I(0, 0, -9.805);
    Eigen::Vector3d acc_meas = q_truth * g_I;

    // 初始猜测（设为错误值：绕y轴旋转15度）
    Eigen::Quaterniond q_init(Eigen::AngleAxisd(M_PI/12, Eigen::Vector3d::UnitY()));
    q_init.normalize();
    double q[4] = {q_init.w(), q_init.x(), q_init.y(), q_init.z()};

    // 构建优化问题
    ceres::Problem problem;
    problem.AddResidualBlock(
        Cost_Initial_G::Create(acc_meas),
        nullptr,
        q
    );

    // 设置四元数参数化（保持单位长度）
    problem.SetParameterization(q, 
        new ceres::EigenQuaternionParameterization());

    // 配置求解器
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // 运行优化
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    // 结果输出
    std::cout << summary.BriefReport() << "\n";
    Eigen::Quaterniond q_result(q[0], q[1], q[2], q[3]);
    std::cout << "True quaternion:\n" << q_truth.coeffs() << "\n";
    std::cout << "Initial quaternion:\n" << q_init.coeffs() << "\n";
    std::cout << "Optimized quaternion:\n" << q_result.coeffs() << "\n";
}