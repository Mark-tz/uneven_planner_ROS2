#include "mpc/mpc.h"
 
using namespace std;
 
void MPC::init(std::shared_ptr<rclcpp::Node> node)
{
    node_ = node;

    // Declare parameters (keep original keys)
    node_->declare_parameter<double>("mpc/du_threshold", du_th);
    node_->declare_parameter<double>("mpc/dt", dt);
    node_->declare_parameter<int>("mpc/max_iter", max_iter);
    node_->declare_parameter<int>("mpc/predict_steps", T);
    node_->declare_parameter<double>("mpc/max_omega", max_omega);
    node_->declare_parameter<double>("mpc/max_domega", max_domega);
    node_->declare_parameter<double>("mpc/max_steer", max_steer);
    node_->declare_parameter<double>("mpc/max_dsteer", max_dsteer);
    node_->declare_parameter<double>("mpc/max_speed", max_speed);
    node_->declare_parameter<double>("mpc/min_speed", min_speed);
    node_->declare_parameter<double>("mpc/max_accel", max_accel);
    node_->declare_parameter<double>("mpc/wheel_base", wheel_base);
    node_->declare_parameter<int>("mpc/delay_num", delay_num);
    node_->declare_parameter<bool>("mpc/test_mpc", test_mpc);
    node_->declare_parameter<int>("mpc/model_type", model_type);
    node_->declare_parameter<bool>("mpc/bk_mode", bk_mode);
    node_->declare_parameter<std::string>("mpc/traj_file", traj_file);
    node_->declare_parameter<std::vector<double>>("mpc/matrix_q", Q);
    node_->declare_parameter<std::vector<double>>("mpc/matrix_r", R);
    node_->declare_parameter<std::vector<double>>("mpc/matrix_rd", Rd);

    // Get parameters
    node_->get_parameter("mpc/du_threshold", du_th);
    node_->get_parameter("mpc/dt", dt);
    node_->get_parameter("mpc/max_iter", max_iter);
    node_->get_parameter("mpc/predict_steps", T);
    node_->get_parameter("mpc/max_omega", max_omega);
    node_->get_parameter("mpc/max_domega", max_domega);
    node_->get_parameter("mpc/max_steer", max_steer);
    node_->get_parameter("mpc/max_dsteer", max_dsteer);
    node_->get_parameter("mpc/max_speed", max_speed);
    node_->get_parameter("mpc/min_speed", min_speed);
    node_->get_parameter("mpc/max_accel", max_accel);
    node_->get_parameter("mpc/wheel_base", wheel_base);
    node_->get_parameter("mpc/delay_num", delay_num);
    node_->get_parameter("mpc/test_mpc", test_mpc);
    node_->get_parameter("mpc/model_type", model_type);
    node_->get_parameter("mpc/bk_mode", bk_mode);
    node_->get_parameter("mpc/traj_file", traj_file);
    node_->get_parameter("mpc/matrix_q", Q);
    node_->get_parameter("mpc/matrix_r", R);
    node_->get_parameter("mpc/matrix_rd", Rd);

    traj_analyzer.setClock(node_->get_clock());

    has_odom = false;
    receive_traj = false;
    max_comega = max_domega * dt;
    max_csteer = max_dsteer * dt;
    max_cv = max_accel * dt;
    xref = Eigen::Matrix<double, 3, 500>::Zero(3, 500);
    last_output = output = dref = Eigen::Matrix<double, 2, 500>::Zero(2, 500);
    for (int i=0; i<delay_num; i++)
        output_buff.push_back(Eigen::Vector2d::Zero());
    errs.clear();
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;

    // Publishers
    pos_cmd_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd", 10);
    vis_pub      = node_->create_publisher<visualization_msgs::msg::Marker>("/mpc/vis", 10);
    predict_pub  = node_->create_publisher<visualization_msgs::msg::Marker>("/mpc/predict", 10);
    ref_pub      = node_->create_publisher<visualization_msgs::msg::Marker>("/mpc/ref", 10);
    err_pub      = node_->create_publisher<std_msgs::msg::Float64>("/mpc/err", 10);

    // Subscriptions
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "odom", rclcpp::SensorDataQoS(),
        std::bind(&MPC::rcvOdomCallBack, this, std::placeholders::_1)
    );
    traj_sub_ = node_->create_subscription<mpc_controller::msg::SE2Traj>(
        "traj", 10,
        std::bind(&MPC::rcvTrajCallBack, this, std::placeholders::_1)
    );

    if (test_mpc || bk_mode)
    {
        trigger_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/move_base_simple/goal", 10,
            std::bind(&MPC::rcvTriggerCallBack, this, std::placeholders::_1)
        );
    }

    if (bk_mode)
    {
        gazebo_pub = node_->create_publisher<geometry_msgs::msg::Point>("/set_model_location", 10);
        string o = traj_file;
        outfile.open(o.insert(o.find("trajs"), "err_"), std::ofstream::out);
        outfile.clear();
        traj_analyzer.setTraj(traj_file);
    }

    // Timer
    using namespace std::chrono_literals;
    cmd_timer_ = node_->create_wall_timer(100ms, std::bind(&MPC::cmdCallback, this));
}

void MPC::rcvTriggerCallBack(const geometry_msgs::msg::PoseStamped msg)
{
    if (test_mpc)
    {
        receive_traj = true;
        traj_analyzer.setTestTraj(max_speed*0.5);
        eight_path = traj_analyzer.getTajWps(0.1);
    }
    else if (bk_mode)
    {
        Eigen::Vector3d initp = traj_analyzer.getNextInitPose();
        geometry_msgs::msg::Point init_point;

        init_point.x = initp.x();
        init_point.y = initp.y();
        init_point.z = initp.z();

        gazebo_pub->publish(init_point);
        rclcpp::sleep_for(std::chrono::seconds(1));

        traj_analyzer.beginNextTraj();
        eight_path = traj_analyzer.getTajWps(0.1);
        receive_traj = true;
    }
}

void MPC::rcvTrajCallBack(mpc_controller::msg::SE2Traj::SharedPtr msg)
{
    receive_traj = true;
    traj_start_time_ = rclcpp::Time(msg->start_time, node_->get_clock()->get_clock_type());
    traj_start_ns_ = static_cast<int64_t>(msg->start_time.sec) * 1000000000LL + static_cast<int64_t>(msg->start_time.nanosec);

    traj_analyzer.setTraj(msg);
}

void MPC::rcvOdomCallBack(nav_msgs::msg::Odometry::SharedPtr msg)
{
    has_odom = true;
    now_state.x = msg->pose.pose.position.x;
    now_state.y = msg->pose.pose.position.y;
    Eigen::Quaterniond q(msg->pose.pose.orientation.w,
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z);
    Eigen::Matrix3d R(q);
    Eigen::Vector2d lvel(msg->twist.twist.linear.x,msg->twist.twist.linear.y);
    now_state.theta = atan2(R.col(0)[1],R.col(0)[0]);
    now_input.vx = lvel.norm();

    double direction = atan2(lvel(1), lvel(0));
    if ((direction-now_state.theta)>M_PI/2)
    {
        now_input.vx = -now_input.vx;
    }
}

void MPC::cmdCallback()
{
    drawFollowPath();

    if (!has_odom || !receive_traj)
        return;

    auto now = node_->get_clock()->now();
    int64_t now_ns = now.nanoseconds();

    if (receive_traj && traj_start_ns_ > 0) {
        if (now.get_clock_type() != traj_start_time_.get_clock_type()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "[MPC] Clock mismatch: now_type=%d traj_type=%d; using numeric ns diff for dt to avoid crash",
                         static_cast<int>(now.get_clock_type()),
                         static_cast<int>(traj_start_time_.get_clock_type()));
        }
        double dt = (now_ns - traj_start_ns_) * 1e-9;
        if (dt < 0.0) {
            RCLCPP_WARN(node_->get_logger(),
                        "[MPC] dt negative (dt=%.6f). now_ns=%ld, start_ns=%ld; skip one cycle.",
                        dt, now_ns, traj_start_ns_);
            // 防御：跳过本周期，避免后续使用负时间
            return;
        }
    }


    vector<TrajPoint> P = traj_analyzer.getRefPoints(T, dt);
    if (!P.empty())
    {
        Eigen::Vector2d err_vec(P[0].x - now_state.x, P[0].y - now_state.y);
        std_msgs::msg::Float64 err_msg;
        errs.push_back(err_vec.norm());
        err_msg.data = errs.back();
        err_pub->publish(err_msg);
    }

    if (traj_analyzer.at_goal)
    {
        // cmd.longitude_speed = 0.0;
        // cmd.angular_vel = 0.0;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pos_cmd_pub_->publish(cmd);

        if (bk_mode)
        {
            Eigen::Vector3d initp = traj_analyzer.getNextInitPose();
            double mean_err = std::accumulate(errs.begin(), errs.end(), 0.0) / (1.0 * errs.size());
            mean_err_all += mean_err;
            outfile << mean_err << std::endl;
            if (initp == Eigen::Vector3d::Zero())
            {
                outfile << "all_mean_track_err: " << mean_err_all <<std::endl;
                outfile.close();
                RCLCPP_WARN(node_->get_logger(), "all_mean_track_err: %lf", mean_err_all);
                RCLCPP_WARN(node_->get_logger(), "Benchmark Done.");
                receive_traj = false;
                return;
            }
            geometry_msgs::msg::Point init_point;

            init_point.x = initp.x();
            init_point.y = initp.y();
            init_point.z = initp.z();

            gazebo_pub->publish(init_point);
            receive_traj = false;
            errs.clear();
            rclcpp::sleep_for(std::chrono::seconds(1));

            traj_analyzer.beginNextTraj();
            eight_path = traj_analyzer.getTajWps(0.1);
            receive_traj = true;
        }
    }
    else
    {
        for (int i=0; i<T; i++)
        {
            xref(0, i) = P[i].x;
            xref(1, i) = P[i].y;
            xref(2, i) = P[i].theta;
            dref(0, i) = 0.0;
            dref(1, i) = 0.0;
        }
        smooth_yaw();
        getCmd();
    }

    pos_cmd_pub_->publish(cmd);
}

void MPC::getLinearModel(const MPCNode& node)
{
    if (model_type == DIFF)
    {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(node.first.theta) * dt;
        B(1, 0) = sin(node.first.theta) * dt;
        B(2, 1) = dt;

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * node.second.vx;
        A(1, 2) = B(0, 0) * node.second.vx;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * node.first.theta; 
        C(1) = -A(1, 2) * node.first.theta;
    }
    else if (model_type == ACKER)
    {
        B = Eigen::Matrix<double, 3, 2>::Zero();
        B(0, 0) = cos(node.first.theta) * dt;
        B(1, 0) = sin(node.first.theta) * dt;
        B(2, 0) = dt * tan(node.second.delta) / wheel_base;
        B(2, 1) = dt * node.second.vx / (wheel_base * pow(cos(node.second.delta), 2));

        A = Eigen::Matrix3d::Identity();
        A(0, 2) = -B(1, 0) * node.second.vx;
        A(1, 2) = B(0, 0) * node.second.vx;

        C = Eigen::Vector3d::Zero();
        C(0) = -A(0, 2) * node.first.theta; 
        C(1) = -A(1, 2) * node.first.theta; 
        C(2) = -B(2, 1) * node.second.delta; 
    }
     
}

void MPC::stateTrans(MPCNode& node)
{
    node.second.vx = max(min(node.second.vx, max_speed), min_speed);
    node.second.w = max(min(node.second.w, max_omega), -max_omega);
    node.second.delta = max(min(node.second.delta, max_steer), -max_steer);

    node.first.x = node.first.x + node.second.vx * cos(node.first.theta) * dt;
    node.first.y = node.first.y + node.second.vx * sin(node.first.theta) * dt;
    if (model_type == DIFF)
    {
        node.first.theta = node.first.theta + node.second.w * dt;
    }else if (model_type == ACKER)
    {
        node.first.theta = node.first.theta + node.second.vx / wheel_base * tan(node.second.delta) * dt;
    }
}

void MPC::predictMotion(void)
{
    xbar[0].first = now_state;

    for (int i=1; i<T+1; i++)
    {
        xbar[i-1].second.vx = output(0, i-1);
        xbar[i-1].second.w = output(1, i-1);
        xbar[i-1].second.delta = output(1, i-1);
        xbar[i] = xbar[i-1];
        stateTrans(xbar[i]);
    }
}

void MPC::predictMotion(MPCState* b)
{
    b[0] = xbar[0].first;

    Eigen::MatrixXd Ax;
    Eigen::MatrixXd Bx;
    Eigen::MatrixXd Cx;
    Eigen::MatrixXd xnext;
    MPCState temp = xbar[0].first;
         
    for (int i=1; i<T+1; i++)
    {  
        MPCState temp_i = xbar[i-1].first;
        
        if (model_type == DIFF)
        {
            Bx = Eigen::Matrix<double, 3, 2>::Zero();
            Bx(0, 0) = cos(temp_i.theta) * dt;
            Bx(1, 0) = sin(temp_i.theta) * dt;
            Bx(2, 1) = dt;
            
            Ax = Eigen::Matrix3d::Identity();
            Ax(0, 2) = -Bx(1, 0) * xbar[i-1].second.vx;
            Ax(1, 2) = Bx(0, 0) * xbar[i-1].second.vx;

            Cx = Eigen::Vector3d::Zero();
            Cx(0) = -Ax(0, 2) * temp_i.theta;
            Cx(1) = -Ax(1, 2) * temp_i.theta;

            if (Ax.rows() != 3 || Ax.cols() != 3) {
                RCLCPP_ERROR(node_->get_logger(),
                             "[MPC] Invalid Ax dims for Ax*Vector3d at DIFF i=%d: Ax(%ld,%ld)",
                             i, Ax.rows(), Ax.cols());
                continue;
            }

            xnext = Ax * Eigen::Vector3d(temp.x, temp.y, temp.theta)
                  + Bx * Eigen::Vector2d(output(0, i-1), output(1, i-1))
                  + Cx;
            temp.x = xnext(0);
            temp.y = xnext(1);
            temp.theta = xnext(2);
            b[i] = temp;
        }
        else if (model_type == ACKER)
        {  
            Bx = Eigen::Matrix<double, 3, 2>::Zero();
            Bx(0, 0) = cos(temp_i.theta) * dt;
            Bx(1, 0) = sin(temp_i.theta) * dt;
            Bx(2, 0) = dt * tan(xbar[i-1].second.delta) / wheel_base;
            Bx(2, 1) = dt * xbar[i-1].second.vx / (wheel_base * pow(cos(xbar[i-1].second.delta), 2));

            Ax = Eigen::Matrix3d::Identity();
            Ax(0, 2) = -Bx(1, 0) * xbar[i-1].second.vx;
            Ax(1, 2) = Bx(0, 0) * xbar[i-1].second.vx;

            Cx = Eigen::Vector3d::Zero();
            Cx(0) = -Ax(0, 2) * temp_i.theta;
            Cx(1) = -Ax(1, 2) * temp_i.theta;
            Cx(2) = -Bx(2, 1) * xbar[i-1].second.delta;

            if (Ax.rows() != 3 || Ax.cols() != 3) {
                RCLCPP_ERROR(node_->get_logger(),
                             "[MPC] Invalid Ax dims for Ax*Vector3d at ACKER i=%d: Ax(%ld,%ld)",
                             i, Ax.rows(), Ax.cols());
                continue;
            }

            xnext = Ax * Eigen::Vector3d(temp.x, temp.y, temp.theta)
                  + Bx * Eigen::Vector2d(output(0, i-1), output(1, i-1))
                  + Cx;
            temp.x = xnext(0);
            temp.y = xnext(1);
            temp.theta = xnext(2);
            b[i] = temp;
        }
    }

}

void MPC::solveMPCDiff(void)
{
    const int dimx = 3 * (T - delay_num);
    const int dimu = 2 * (T - delay_num);
    const int nx = dimx + dimu;
    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=delay_num, k=0; i<dimx; i+=3, j++, k+=2)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
        gradient[dimx+k] = -2 * Q[2] * dref(0, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0] + Q[2]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0] + Q[2]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCNode temp = xbar[delay_num];
    getLinearModel(temp);

    // 调试输出与尺寸保护
    if (A.rows() != 3 || A.cols() != 3) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MPC] Invalid A dims for A*Vector3d at solveMPCDiff: A(%ld,%ld)",
                     A.rows(), A.cols());
        return;
    }
    if (C.size() != 3) {
        RCLCPP_ERROR(node_->get_logger(),
                     "[MPC] Invalid C size (expected 3) at solveMPCDiff: %ld", C.size());
        return;
    }

    int my = dimx;
    double b[my];
    const int nnzA = 11 * (T-delay_num) - 5;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector3d temp_vec(temp.first.x, temp.first.y, temp.first.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    irowA[dimx] = 0;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(0, 0);
    irowA[dimx+1] = 1;
    jcolA[dimx+1] = dimx;
    dA[dimx+1] = -B(1, 0);
    irowA[dimx+2] = 2;
    jcolA[dimx+2] = dimx+1;
    dA[dimx+2] = -B(2, 1);
    int ABidx = 8*(T-delay_num) - 8;
    int ABbegin = dimx+3;
    for (int i=0, j=1; i<ABidx; i+=8, j++)
    {
        getLinearModel(xbar[j+delay_num]);
        for (int k=0; k<3; k++)
        {
            b[3*j+k] = C[k];
            irowA[ABbegin + i + k] = 3*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 3] = 3*j;
        jcolA[ABbegin + i + 3] = 3*j - 1;
        dA[ABbegin + i + 3] = -A(0, 2);

        irowA[ABbegin + i + 4] = 3*j + 1;
        jcolA[ABbegin + i + 4] = 3*j - 1;
        dA[ABbegin + i + 4] = -A(1, 2);
        
        irowA[ABbegin + i + 5] = 3*j;
        jcolA[ABbegin + i + 5] = dimx + 2*j;
        dA[ABbegin + i + 5] = -B(0, 0);
        
        irowA[ABbegin + i + 6] = 3*j + 1;
        jcolA[ABbegin + i + 6] = dimx + 2*j;
        dA[ABbegin + i + 6] = -B(1, 0);
        
        irowA[ABbegin + i + 7] = 3*j + 2;
        jcolA[ABbegin + i + 7] = dimx + 2*j + 1;
        dA[ABbegin + i + 7] = -B(2, 1);
    }

    // iequality constraints
    const int mz  = 2 * (T-delay_num) - 2;
    const int nnzC = 2 * dimu - 4;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, k=0; i<mz; i+=2, k+=4)
    {
        irowC[k] = i;
        jcolC[k] = dimx  + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] +2;
        dC[k+1] = 1.0;

        irowC[k+2] = i + 1;
        jcolC[k+2] = dimx + 1 + i;
        dC[k+2] = -1.0;

        irowC[k+3] = i + 1;
        jcolC[k+3] = jcolC[k+2] +2;
        dC[k+3] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<mx; i+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_omega;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_omega;
        linearMatrix.insert(i, dimx+i) = 1;
        linearMatrix.insert(i+1, dimx+i+1) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<mz; i+=2)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
        lowerBound[mx+my+i+1] = -max_comega;
        upperBound[mx+my+i+1] = max_comega;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) {
        RCLCPP_ERROR(node_->get_logger(), "[MPC] OSQP initSolver failed");
        return;
    }

    // solve the QP problem
    auto status = solver.solveProblem();
    if(status != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_ERROR(node_->get_logger(), "[MPC] OSQP solveProblem error, abort this cycle");
        return;
    }

    // get the controller input
    Eigen::VectorXd QPSolution = solver.getSolution();
    for (int i=0; i<delay_num; i++)
    {
        output(0, i) = output_buff[i][0];
        output(1, i) = output_buff[i][1];
    }
    for (int i=0, j=0; i<dimu; i+=2, j++)
    {
        output(0, j+delay_num) = QPSolution[dimx+i];
        output(1, j+delay_num) = QPSolution[dimx+i+1];
    }
}

void MPC::solveMPCAcker(void)
{
    const int dimx = 3 * (T - delay_num);
    const int dimu = 2 * (T - delay_num);
    const int nx = dimx + dimu;
    RCLCPP_INFO(node_->get_logger(), "[MPC] solveMPCAcker dims: dimx=%d dimu=%d nx=%d", dimx, dimu, nx);

    Eigen::SparseMatrix<double> hessian;
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(nx);
    Eigen::SparseMatrix<double> linearMatrix;
    Eigen::VectorXd lowerBound;
    Eigen::VectorXd upperBound;

    // first-order
    for (int i=0, j=delay_num, k=0; i<dimx; i+=3, j++, k+=2)
    {
        gradient[i] = -2 * Q[0] * xref(0, j);
        gradient[i+1] = -2 * Q[1] * xref(1, j);
        gradient[i+2] = -2 * Q[2] * xref(2, j);
        gradient[dimx+k] = -2 * Q[2] * dref(0, j);
    }

    // second-order
    const int nnzQ = nx + dimu - 2;
    int irowQ[nnzQ];
    int jcolQ[nnzQ];
    double dQ[nnzQ];
    for (int i=0; i<nx; i++)
    {
        irowQ[i] = jcolQ[i] = i;
    }
    for (int i=nx; i<nnzQ; i++)
    {
        irowQ[i] = i - dimu + 2;
        jcolQ[i] = i - dimu;
    }
    for (int i=0; i<dimx; i+=3)
    {
        dQ[i] = Q[0] * 2.0;
        dQ[i+1] = Q[1] * 2.0;
        dQ[i+2] = Q[2] * 2.0;
    }
    dQ[dimx] = dQ[nx-2] = (R[0] + Rd[0] + Q[2]) * 2.0;
    dQ[dimx + 1] = dQ[nx-1] = (R[1] + Rd[1]) * 2.0;
    for (int i=dimx+2; i<nx-2; i+=2)
    {
        dQ[i] = 2 * (R[0] + 2 * Rd[0] + Q[2]);
        dQ[i+1] = 2 * (R[1] + 2 * Rd[1]);
    }
    for (int i=nx; i<nnzQ; i+=2)
    {
        dQ[i] = -Rd[0] * 2.0;
        dQ[i+1] = -Rd[1] * 2.0;
    }
    hessian.resize(nx, nx);
    Eigen::MatrixXd QQ(nx, nx);
    for (int i=0; i<nx; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
    }
    for (int i=nx; i<nnzQ; i++)
    {
        hessian.insert(irowQ[i], jcolQ[i]) = dQ[i];
        hessian.insert(jcolQ[i], irowQ[i]) = dQ[i];
    }

    // equality constraints
    MPCNode temp = xbar[delay_num];
    getLinearModel(temp);
    int my = dimx;
    double b[my];
    const int nnzA = 12 * (T-delay_num) - 5;
    int irowA[nnzA];
    int jcolA[nnzA];
    double dA[nnzA];
    Eigen::Vector3d temp_vec(temp.first.x, temp.first.y, temp.first.theta);
    Eigen::Vector3d temp_b = A*temp_vec + C;
    
    for (int i=0; i<dimx; i++)
    {
        irowA[i] = jcolA[i] = i;
        dA[i] = 1;
    }
    b[0] = temp_b[0];
    b[1] = temp_b[1];
    b[2] = temp_b[2];
    irowA[dimx] = 0;
    jcolA[dimx] = dimx;
    dA[dimx] = -B(0, 0);
    irowA[dimx+1] = 1;
    jcolA[dimx+1] = dimx;
    dA[dimx+1] = -B(1, 0);
    irowA[dimx+2] = 2;
    jcolA[dimx+2] = dimx;
    dA[dimx+2] = -B(2, 0);
    irowA[dimx+3] = 2;
    jcolA[dimx+3] = dimx+1;
    dA[dimx+3] = -B(2, 1);
    int ABidx = 9*(T-delay_num) - 9;
    int ABbegin = dimx+4;
    for (int i=0, j=1; i<ABidx; i+=9, j++)
    {
        getLinearModel(xbar[j+delay_num]);
        for (int k=0; k<3; k++)
        {
            b[3*j+k] = C[k];
            irowA[ABbegin + i + k] = 3*j + k;
            jcolA[ABbegin + i + k] = irowA[ABbegin + i + k] - 3;
            dA[ABbegin + i + k] = -A(k, k);
        }
        irowA[ABbegin + i + 3] = 3*j;
        jcolA[ABbegin + i + 3] = 3*j - 1;
        dA[ABbegin + i + 3] = -A(0, 2);

        irowA[ABbegin + i + 4] = 3*j + 1;
        jcolA[ABbegin + i + 4] = 3*j - 1;
        dA[ABbegin + i + 4] = -A(1, 2);
        
        irowA[ABbegin + i + 5] = 3*j;
        jcolA[ABbegin + i + 5] = dimx + 2*j;
        dA[ABbegin + i + 5] = -B(0, 0);
        
        irowA[ABbegin + i + 6] = 3*j + 1;
        jcolA[ABbegin + i + 6] = dimx + 2*j;
        dA[ABbegin + i + 6] = -B(1, 0);
        
        irowA[ABbegin + i + 7] = 3*j + 2;
        jcolA[ABbegin + i + 7] = dimx + 2*j;
        dA[ABbegin + i + 7] = -B(2, 0);

        irowA[ABbegin + i + 8] = 3*j + 2;
        jcolA[ABbegin + i + 8] = dimx + 2*j + 1;
        dA[ABbegin + i + 8] = -B(2, 1);
    }

    // iequality constraints
    const int mz  = 2 * (T-delay_num) - 2;
    const int nnzC = 2 * dimu - 4;
    int   irowC[nnzC];
    int   jcolC[nnzC];
    double   dC[nnzC];
    for (int i=0, k=0; i<mz; i+=2, k+=4)
    {
        irowC[k] = i;
        jcolC[k] = dimx  + i;
        dC[k] = -1.0;

        irowC[k+1] = i;
        jcolC[k+1] = jcolC[k] +2;
        dC[k+1] = 1.0;

        irowC[k+2] = i + 1;
        jcolC[k+2] = dimx + 1 + i;
        dC[k+2] = -1.0;

        irowC[k+3] = i + 1;
        jcolC[k+3] = jcolC[k+2] +2;
        dC[k+3] = 1.0;
    }

    // xlimits and all
    int mx = dimu;
    int nc = mx+my+mz;
    lowerBound.resize(nc);
    upperBound.resize(nc);
    linearMatrix.resize(nc, nx);
    for (int i=0; i<mx; i+=2)
    {
        lowerBound[i] = min_speed;
        lowerBound[i+1] = -max_steer;
        upperBound[i] = max_speed;
        upperBound[i+1] = max_steer;
        linearMatrix.insert(i, dimx+i) = 1;
        linearMatrix.insert(i+1, dimx+i+1) = 1;
    }

    for (int i=0; i<nnzA; i++)
    {
        linearMatrix.insert(irowA[i]+mx, jcolA[i]) = dA[i];
    }

    for (int i=0; i<my; i++)
    {
        lowerBound[mx+i] = upperBound[mx+i] = b[i];
    }

    for (int i=0; i<nnzC; i++)
    {
        linearMatrix.insert(irowC[i]+mx+my, jcolC[i]) = dC[i];
    }

    for (int i=0; i<mz; i+=2)
    {
        lowerBound[mx+my+i] = -max_cv;
        upperBound[mx+my+i] = max_cv;
        lowerBound[mx+my+i+1] = -max_csteer;
        upperBound[mx+my+i+1] = max_csteer;
    }

    // instantiate the solver
    OsqpEigen::Solver solver;

    // settings
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-6);
    solver.settings()->setMaxIteration(30000);
    solver.settings()->setRelativeTolerance(1e-6);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(nx);
    solver.data()->setNumberOfConstraints(nc);
    if(!solver.data()->setHessianMatrix(hessian)) return;
    if(!solver.data()->setGradient(gradient)) return;
    if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return;
    if(!solver.data()->setLowerBound(lowerBound)) return;
    if(!solver.data()->setUpperBound(upperBound)) return;

    // instantiate the solver
    if(!solver.initSolver()) {
        RCLCPP_ERROR(node_->get_logger(), "[MPC] OSQP initSolver failed");
        return;
    }

    // solve the QP problem
    auto status = solver.solveProblem();
    if(status != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_ERROR(node_->get_logger(), "[MPC] OSQP solveProblem error, abort this cycle");
        return;
    }

    // get the controller input
    Eigen::VectorXd QPSolution = solver.getSolution();
    for (int i=0; i<delay_num; i++)
    {
        output(0, i) = output_buff[i][0];
        output(1, i) = output_buff[i][1];
    }
    for (int i=0, j=0; i<dimu; i+=2, j++)
    {
        output(0, j+delay_num) = QPSolution[dimx+i];
        output(1, j+delay_num) = QPSolution[dimx+i+1];
    }
}

void MPC::getCmd(void)
{
    int iter;
    rclcpp::Time begin = node_->get_clock()->now();

    for (iter=0; iter<max_iter; iter++)
    {
        predictMotion();
        last_output = output;
        if (model_type == DIFF)
            solveMPCDiff();
        else
            solveMPCAcker();
        double du = 0;
        for (int i=0; i<output.cols(); i++)
        {
            du += std::fabs(output(0, i) - last_output(0, i))
                + std::fabs(output(1, i) - last_output(1, i));
        }

        if (du <= du_th || (node_->get_clock()->now()-begin).seconds()>0.01)
        {
            break;
        }
    }
    if (iter == max_iter)
    {
        RCLCPP_WARN(node_->get_logger(), "[MPC] Iterative reached max_iter=%d", max_iter);
    }

    predictMotion(xopt);
    drawRefPath();
    drawPredictPath(xopt);
    
    cmd.linear.x = output(0, delay_num);
    cmd.angular.z = output(1, delay_num);

    if (delay_num>0)
    {
        output_buff.erase(output_buff.begin());
        output_buff.push_back(Eigen::Vector2d(output(0, delay_num),output(1, delay_num)));
    }
}
