// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <boost/scoped_ptr.hpp>
#include <boost/lexical_cast.hpp>
#include <math.h>
#include <Eigen/LU>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>

#include <cmath>

#define PI 3.141592
#define D2R PI / 180.0
#define R2D 180.0 / PI
#define SaveDataMax 49
#define num_taskspace 6
#define A 0.1
#define b 2.5
#define f 1
#define t_set 1

int PRINT_INTERVAL = 100;

namespace arm_controllers
{
class DemoController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
  public:
    bool init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &n)
    {
        // ********* 1. Get joint name / gain from the parameter server *********
        // 1.1 Joint Name
        if (!n.getParam("joints", joint_names_))
        {
            ROS_ERROR("Could not find joint name");
            return false;
        }
        n_joints_ = joint_names_.size();

        if (n_joints_ == 0)
        {
            ROS_ERROR("List of joint names is empty.");
            return false;
        }
        else
        {
            ROS_INFO("Found %d joint names", n_joints_);
            for (int i = 0; i < n_joints_; i++)
            {
                ROS_INFO("%s", joint_names_[i].c_str());
            }
        }

        // 1.2 Gain
        // 1.2.1 Joint Controller
        Kp_.resize(n_joints_);
        Kd_.resize(n_joints_);
        Ki_.resize(n_joints_);
        
        lower_limits_.resize(n_joints_);
        upper_limits_.resize(n_joints_);
        

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/DemoController/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/DemoController/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/DemoController/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/DemoController/gains/elfin_joint" + si + "/pid/d", Kd[i]))
            {
                Kd_(i) = Kd[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/d gain");
                return false;
            }
        }

        // 2. ********* urdf *********
        urdf::Model urdf;
        if (!urdf.initParam("robot_description"))
        {
            ROS_ERROR("Failed to parse urdf file");
            return false;
        }
        else
        {
            ROS_INFO("Found robot_description");
        }

        // 3. ********* Get the joint object to use in the realtime loop [Joint Handle, URDF] *********
        for (int i = 0; i < n_joints_; i++)
        {
            try
            {
                joints_.push_back(hw->getHandle(joint_names_[i]));
            }
            catch (const hardware_interface::HardwareInterfaceException &e)
            {
                ROS_ERROR_STREAM("Exception thrown: " << e.what());
                return false;
            }

            urdf::JointConstSharedPtr joint_urdf = urdf.getJoint(joint_names_[i]);
            if (!joint_urdf)
            {
                ROS_ERROR("Could not find joint '%s' in urdf", joint_names_[i].c_str());
                return false;
            }
            joint_urdfs_.push_back(joint_urdf);
        }

        // 4. ********* KDL *********
        // 4.1 kdl parser
        if (!kdl_parser::treeFromUrdfModel(urdf, kdl_tree_))
        {
            ROS_ERROR("Failed to construct kdl tree");
            return false;
        }
        else
        {
            ROS_INFO("Constructed kdl tree");
        }

        // 4.2 kdl chain
        std::string root_name, tip_name;
        if (!n.getParam("root_link", root_name))
        {
            ROS_ERROR("Could not find root link name");
            return false;
        }
        if (!n.getParam("tip_link", tip_name))
        {
            ROS_ERROR("Could not find tip link name");
            return false;
        }
        if (!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
        {
            ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
            ROS_ERROR_STREAM("  " << root_name << " --> " << tip_name);
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfJoints() << " joints");
            ROS_ERROR_STREAM("  Tree has " << kdl_tree_.getNrOfSegments() << " segments");
            ROS_ERROR_STREAM("  The segments are:");

            KDL::SegmentMap segment_map = kdl_tree_.getSegments();
            KDL::SegmentMap::iterator it;

            for (it = segment_map.begin(); it != segment_map.end(); it++)
                ROS_ERROR_STREAM("    " << (*it).first);

            return false;
        }
        else
        {
            ROS_INFO("Got kdl chain");
        }

        // 4.3 inverse dynamics solver 초기화
        gravity_ = KDL::Vector::Zero(); // ?
        gravity_(2) = -9.81;            // 0: x-axis 1: y-axis 2: z-axis

        id_solver_.reset(new KDL::ChainDynParam(kdl_chain_, gravity_));
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        
        q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        q_dotdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        
        lower_limits_.data = Eigen::VectorXd::Zero(n_joints_);
        upper_limits_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);

        q1_.data=Eigen::VectorXd::Zero(n_joints_);

        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        J_.resize(kdl_chain_.getNrOfJoints());
        Jd_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        lower_limits_(0)=-3.14;
        upper_limits_(0)=3.14;
        lower_limits_(1)=-2.35;
        upper_limits_(1)=2.35;
        lower_limits_(2)=-2.61;
        upper_limits_(2)=2.61;
        lower_limits_(3)=-3.14;
        upper_limits_(3)=3.14;
        lower_limits_(4)=-2.56;
        upper_limits_(4)=2.56;
        lower_limits_(5)=-3.14;
        upper_limits_(5)=3.14;
        
        target_time = 5;
        update_round = 0;
        distance_to_target_limit = 0.01;
        start_time = ros::Time::now();

        p_init_ = get_current_frame();
        init_p_reached = true;
        double init_roll, init_pitch, init_yaw; // -1.571048
        p_init_.M.GetRPY(init_roll, init_pitch, init_yaw);
        double roll_down, pitch_down, yaw_down;
        roll_down = 1.571048;
        pitch_down = 0;
        yaw_down = 1.571048;

        p_demo_start_.p(0) = p_init_.p(0);
        p_demo_start_.p(1) = p_init_.p(1);
        p_demo_start_.p(2) = p_init_.p(2) - 0.2;
        p_demo_start_.M = KDL::Rotation(KDL::Rotation::RPY(init_roll, init_pitch, init_yaw));

        // Touch the ball
        p_touch_ball_1.p(0) = p_init_.p(0);
        p_touch_ball_1.p(1) = -0.3;
        p_touch_ball_1.p(2) = 0.5;
        p_touch_ball_1.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));

        weld_p_1_reached = false;
        weld_p_2_reached = false;

        weld_point_1.p(0) = 0.2;
        weld_point_1.p(1) = p_init_.p(1);
        weld_point_1.p(2) = 0.5;
        weld_point_1.M = KDL::Rotation(KDL::Rotation::RPY(1.571048, 0, 1.571048));

        return true;
    }

    void commandCB(const std_msgs::Float64MultiArrayConstPtr &msg)
    {
        if (msg->data.size() != n_joints_)
        {
            ROS_ERROR_STREAM("Dimension of command (" << msg->data.size() << ") does not match number of joints (" << n_joints_ << ")! Not executing!");
            return;
        }
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        ROS_INFO("Starting Demo Controller");
    }


    KDL::JntArray getRepVelocity()
	{
		KDL::JntArray rep_velocities;
		rep_velocities.data = Eigen::VectorXd::Zero(n_joints_);
		int k = 1;
		double q_star = 0.5;
		double F;
		double q_limit_dist;
		
		if (update_round % PRINT_INTERVAL == 0 && repulsive_F_prints)
			{
				printf("*** Repulsive force prints  ***\n");
			}
		
		for (int i = 0; i < n_joints_; i++)
		{
			
			q_limit_dist = upper_limits_(i) - std::abs(q_(i));
			
			if (update_round % PRINT_INTERVAL == 0 && repulsive_F_prints)
			{
				ROS_INFO("q_(%d) here:  %f", i, q_(i));
				ROS_INFO("i==%d, q_limit_dist %f", i, q_limit_dist);
			}
			
			if (q_limit_dist <= q_star)
			{
				//1/2*k*(1/q_limit_dist-1/q_star)
				if (q_(i) < 0)
				{
					F = -k*(1/q_limit_dist-1/q_star)*1/(pow(q_limit_dist, 2));
				}
				else
				{
					F = k*(1/q_limit_dist-1/q_star)*1/(pow(q_limit_dist, 2));
				}
			}
			else
			{
				F = 0;
			}
			
			if (update_round % PRINT_INTERVAL == 0 && repulsive_F_prints)
			{
				ROS_INFO("F: %f", F);
				ROS_INFO(" ");
			}
			
			rep_velocities(i) = F;
		}
		return rep_velocities;
		
	}

	KDL::Frame get_current_frame()
	{
        KDL::JntArray q_curr, qdot_curr;
        q_curr.data=Eigen::VectorXd::Zero(n_joints_);
        qdot_curr.data = Eigen::VectorXd::Zero(n_joints_);

	    KDL::Frame frame_to_return;
        for (int i = 0; i < n_joints_; i++)
        {
            q_curr(i) = joints_[i].getPosition();
        }
        fk_pos_solver_->JntToCart(q_curr, frame_to_return);
        return frame_to_return;
	}

    void printFrame(KDL::Frame frame, const std::string& frame_name = "empty")
    {
        printf("\n------------------Frame------------------\n");
        if (frame_name.compare("empty") != 0)
        {
            std::cout << frame_name << std::endl;
        }
        printf("Position:\n");
        printf("[");
        printf("%f, ", frame.p(0));
        printf("%f, ", frame.p(1));
        printf("%f]\n", frame.p(2));
        printf("Orientation:\n");
        for(int i = 0; i < 9; i++)
        {
            if (i == 0 || i == 3 || i == 6)
            {
                printf("[%f\t", frame.M.data[i]);
            }
            else if (i == 1 || i == 4 || i == 7)
            {
                printf("%f\t", frame.M.data[i]);
            }

            if (i == 2 || i == 5 || i == 8)
            {
                printf("%f]\n", frame.M.data[i]);
            }
        }
        printf("\n\n");
    }

    std::vector<KDL::Frame> get_start_goal_frames()
    {
        KDL::Frame start_f;
        KDL::Frame goal_f;
        if (update_round < 2000)
        {
            start_f = get_current_frame();
            goal_f = p_init_;
            if (update_round == 0)
            {
                printf("update round: %d", update_round);
                printFrame(goal_f, "f1");
            }
        }
        else
        {
            if (p_start_reached == false && init_p_reached == true)
            {
                start_f = get_current_frame();
                goal_f = p_demo_start_;
                //goal_f = weld_point_1;
            }
            else if (p_start_reached == true && init_p_reached == false)
            {
                start_f = get_current_frame();
                goal_f = p_init_;
            }
            //printFrame(f1_, "f1");
            //printf("update round: %d", update_round);
            distance_to_target = sqrt(pow(goal_f.p(0)-x_.p(0),2)+pow(goal_f.p(1)-x_.p(1),2)+pow(goal_f.p(2)-x_.p(2),2));

            if (distance_to_target <= distance_to_target_limit && p_start_reached == false && init_p_reached == true)
            {
                printf("Starting point reached!\n");
                init_p_reached = false;
                p_start_reached = true;
                goal_f = p_init_;
            }
            else if (distance_to_target <= distance_to_target_limit && p_start_reached == true && init_p_reached == false)
            {
                printf("Init point reached!\n");
                init_p_reached = true;
                p_start_reached = false;
                goal_f = p_demo_start_;
            }
        }

        std::vector<KDL::Frame> frames_to_return;
        frames_to_return.push_back(start_f);
        frames_to_return.push_back(goal_f);

        return frames_to_return;

    }

    void update(const ros::Time &time, const ros::Duration &period)
    {	
    	
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        //double dt = period.toSec();
        t = t + 0.001;
        
        double v_tau = 0.01;
        double tau_0 = 0;
        double dt;
        curr_time = ros::Time::now();

        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        }

        fk_pos_solver_->JntToCart(q_, x_);

        std::vector<KDL::Frame> temp_frames;
        temp_frames = get_start_goal_frames();
        f0_ = temp_frames[0];
        f1_ = temp_frames[1];

        V0_ = diff(f0_, f1_)/target_time;

        if (update_round == 0)
        {
            dt = (curr_time - start_time).toSec();
            tau_old = tau_0;
            xd_ = get_current_frame();
        }
        else
        {
            dt = (curr_time - prev_time).toSec();
            tau_old = tau_k;
        }

        tau_k = tau_old + dt * v_tau;
        if (tau_k >= 1)
        {
            tau_k = 1;
        }

        // For printing
        e_.data = qd_.data - q_.data;

        xerr_temp_ = diff(x_, xd_, 1);
        for (int i = 0; i < n_joints_; i++)
        {
            xerr_(i) = xerr_temp_(i);
        }

        xd_ = KDL::addDelta(xd_, V0_, dt);
        double roll, pitch, yaw;
        xd_.M.GetRPY(roll, pitch, yaw);
        xd_temp_(0) = xd_.p(0);
        xd_temp_(1) = xd_.p(1);
        xd_temp_(2) = xd_.p(2);
        xd_temp_(3) = roll;
        xd_temp_(4) = pitch;
        xd_temp_(5) = yaw;

        jnt_to_jac_solver_->JntToJac(q_, J_);
        V_cmd_ = xd_temp_ + Kp_.data.cwiseProduct(xerr_);

        q_dot_cmd_.data = J_.data.inverse() * V_cmd_;
        //e_dot_cmd_.data = q_dot_cmd_.data - qdot_.data;

        if (repulsive_F_control)
        {
		    KDL::JntArray q_rep = getRepVelocity();
		    e_dot_cmd_.data = q_dot_cmd_.data - (qdot_.data + q_rep.data);
		}
		else
		{
			e_dot_cmd_.data = q_dot_cmd_.data - qdot_.data;
		}

        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_);

        aux_d_.data = M_.data * (Kd_.data.cwiseProduct(e_dot_cmd_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
        }

        // ********* 3. data 저장 *********
        save_data();

        // ********* 4. state 출력 *********
        //print_state();

        if (update_round % 1000 == 0)
        {
            //printf("update round: %d", update_round);
            printFrame(f1_, "f1");
            //KDL::Frame curr_f = get_current_frame();
            //printFrame(curr_f, "Current frame");
            printf("\n%f\n", distance_to_target);
        }

        update_round += 1;
        prev_time = curr_time;
    }

    void stopping(const ros::Time &time)
    {
    }

    void save_data()
    {
        // 1
        // Simulation time (unit: sec)
        SaveData_[0] = t;

        // Desired position in joint space (unit: rad)
        SaveData_[1] = qd_(0);
        SaveData_[2] = qd_(1);
        SaveData_[3] = qd_(2);
        SaveData_[4] = qd_(3);
        SaveData_[5] = qd_(4);
        SaveData_[6] = qd_(5);

        // Desired velocity in joint space (unit: rad/s)
        SaveData_[7] = qd_dot_(0);
        SaveData_[8] = qd_dot_(1);
        SaveData_[9] = qd_dot_(2);
        SaveData_[10] = qd_dot_(3);
        SaveData_[11] = qd_dot_(4);
        SaveData_[12] = qd_dot_(5);

        // Desired acceleration in joint space (unit: rad/s^2)
        SaveData_[13] = qd_ddot_(0);
        SaveData_[14] = qd_ddot_(1);
        SaveData_[15] = qd_ddot_(2);
        SaveData_[16] = qd_ddot_(3);
        SaveData_[17] = qd_ddot_(4);
        SaveData_[18] = qd_ddot_(5);

        // Actual position in joint space (unit: rad)
        SaveData_[19] = q_(0);
        SaveData_[20] = q_(1);
        SaveData_[21] = q_(2);
        SaveData_[22] = q_(3);
        SaveData_[23] = q_(4);
        SaveData_[24] = q_(5);

        // Actual velocity in joint space (unit: rad/s)
        SaveData_[25] = qdot_(0);
        SaveData_[26] = qdot_(1);
        SaveData_[27] = qdot_(2);
        SaveData_[28] = qdot_(3);
        SaveData_[29] = qdot_(4);
        SaveData_[30] = qdot_(5);

        // Error position in joint space (unit: rad)
        SaveData_[31] = e_(0);
        SaveData_[32] = e_(1);
        SaveData_[33] = e_(2);
        SaveData_[34] = e_(3);
        SaveData_[35] = e_(4);
        SaveData_[36] = e_(5);

        // Error velocity in joint space (unit: rad/s)
        SaveData_[37] = e_dot_(0);
        SaveData_[38] = e_dot_(1);
        SaveData_[39] = e_dot_(2);
        SaveData_[40] = e_dot_(3);
        SaveData_[41] = e_dot_(4);
        SaveData_[42] = e_dot_(5);

        // Error intergal value in joint space (unit: rad*sec)
        SaveData_[43] = e_int_(0);
        SaveData_[44] = e_int_(1);
        SaveData_[45] = e_int_(2);
        SaveData_[46] = e_int_(3);
        SaveData_[47] = e_int_(4);
        SaveData_[48] = e_int_(5);

        // 2
        msg_qd_.data.clear();
        msg_q_.data.clear();
        msg_e_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
        }

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > PRINT_INTERVAL-1)
        {
            printf("*********************************************************\n\n");
            printf("*** Simulation Time (unit: sec)  ***\n");
            printf("t = %f\n", t);
            printf("\n");

            printf("*** Desired State in Joint Space (unit: deg) ***\n");
            printf("qd_(0): %f, ", qd_(0)*R2D);
            printf("qd_(1): %f, ", qd_(1)*R2D);
            printf("qd_(2): %f, ", qd_(2)*R2D);
            printf("qd_(3): %f, ", qd_(3)*R2D);
            printf("qd_(4): %f, ", qd_(4)*R2D);
            printf("qd_(5): %f\n", qd_(5)*R2D);
            printf("\n");

            printf("*** Actual State in Joint Space (unit: deg) ***\n");
            printf("q_(0): %f, ", q_(0) * R2D);
            printf("q_(1): %f, ", q_(1) * R2D);
            printf("q_(2): %f, ", q_(2) * R2D);
            printf("q_(3): %f, ", q_(3) * R2D);
            printf("q_(4): %f, ", q_(4) * R2D);
            printf("q_(5): %f\n", q_(5) * R2D);
            printf("\n");


            printf("*** Joint Space Error (unit: deg)  ***\n");
            printf("%f, ", R2D * e_(0));
            printf("%f, ", R2D * e_(1));
            printf("%f, ", R2D * e_(2));
            printf("%f, ", R2D * e_(3));
            printf("%f, ", R2D * e_(4));
            printf("%f\n", R2D * e_(5));
            printf("\n");
            printf("*********************************************************\n\n");


            count = 0;
        }
        count++;
    }

  private:
    // others
    double t;
    bool stay_still = false;
    int update_round;
    bool repulsive_F_prints = false;
    bool repulsive_F_control = false;
    int target_time;
    double tau_k;
    double tau_old;
    ros::Time start_time;
    ros::Time curr_time;
    ros::Time prev_time;
    double distance_to_target;
    double distance_to_target_limit;
    bool p_start_reached = false;
    bool init_p_reached = false;
    KDL::Frame weld_point_1;
    KDL::Frame weld_point_2;
    bool weld_p_1_reached;
    bool weld_p_2_reached;

    // Ball touch points
    KDL::Frame p_touch_ball_1;
    KDL::Frame p_touch_ball_2;
    KDL::Frame p_touch_ball_3;
    KDL::Frame p_touch_ball_4;
    bool touch_ball_1_reached;
    bool touch_ball_2_reached;
    bool touch_ball_3_reached;
    bool touch_ball_4_reached;

    KDL::JntArray q1_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
    KDL::Frame xd_;
    KDL::Frame x_;
    KDL::Twist xerr_temp_;
    KDL::Jacobian J_;
    KDL::Jacobian Jd_;
    Eigen::Matrix<double, num_taskspace, 1> Vd_;
    Eigen::Matrix<double, num_taskspace, 1> V_cmd_;
    Eigen::Matrix<double, num_taskspace, 1> xerr_;
    Eigen::Matrix<double, num_taskspace, 1> xd_temp_;
    KDL::Frame f0_;
    KDL::Frame f1_;
    KDL::Frame p_demo_start_;
    KDL::Frame p_init_;
    KDL::Twist V0_;

    //Joint handles
    unsigned int n_joints_;                               // joint 숫자
    std::vector<std::string> joint_names_;                // joint name ??
    std::vector<hardware_interface::JointHandle> joints_; // ??
    std::vector<urdf::JointConstSharedPtr> joint_urdfs_;  // ??

    // kdl
    KDL::Tree kdl_tree_;   // tree?
    KDL::Chain kdl_chain_; // chain?

    // kdl M,C,G
    KDL::JntSpaceInertiaMatrix M_; // intertia matrix
    KDL::JntArray C_;              // coriolis
    KDL::JntArray G_;              // gravity torque vector
    KDL::Vector gravity_;

    // kdl solver
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_;                  // Solver To compute the inverse dynamics

    // Joint Space State
    KDL::JntArray qd_, qd_dot_, qd_ddot_;
    KDL::JntArray qd_old_;
    KDL::JntArray q_, qdot_;
    KDL::JntArray e_, e_dot_, e_int_;
    
    KDL::JntArray q_dot_cmd_, q_dotdot_cmd_;
    KDL::JntArray e_dot_cmd_;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;
    
    KDL::JntArray lower_limits_, upper_limits_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_;
    ros::Publisher pub_SaveData_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_;
    std_msgs::Float64MultiArray msg_SaveData_;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::DemoController, controller_interface::ControllerBase)
