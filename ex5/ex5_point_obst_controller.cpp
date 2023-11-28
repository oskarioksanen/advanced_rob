// Ramin ja Oskarin koodi yhdistetty
// from ros-control meta packages
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <pluginlib/class_list_macros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <Eigen/LU>

#include <urdf/model.h>

// from kdl packages
#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chaindynparam.hpp>              // inverse dynamics
#include <kdl/chainjnttojacsolver.hpp>        // jacobian
#include <kdl/chainfksolverpos_recursive.hpp> // forward kinematics

#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/framevel.hpp>

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


namespace arm_controllers
{
class Point_obst_controller : public controller_interface::Controller<hardware_interface::EffortJointInterface>
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

        std::vector<double> Kp(n_joints_), Ki(n_joints_), Kd(n_joints_);
        for (size_t i = 0; i < n_joints_; i++)
        {
            std::string si = boost::lexical_cast<std::string>(i + 1);
            if (n.getParam("/elfin/Point_obst_controller/gains/elfin_joint" + si + "/pid/p", Kp[i]))
            {
                Kp_(i) = Kp[i];
            }
            else
            {
                std::cout << "/elfin/Point_obst_controller/gains/elfin_joint" + si + "/pid/p" << std::endl;
                ROS_ERROR("Cannot find pid/p gain");
                return false;
            }

            if (n.getParam("/elfin/Point_obst_controller/gains/elfin_joint" + si + "/pid/i", Ki[i]))
            {
                Ki_(i) = Ki[i];
            }
            else
            {
                ROS_ERROR("Cannot find pid/i gain");
                return false;
            }

            if (n.getParam("/elfin/Point_obst_controller/gains/elfin_joint" + si + "/pid/d", Kd[i]))
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
        //Own code:
        fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
        jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
        
        fk_vel_solver_.reset(new KDL::ChainFkSolverVel_recursive(kdl_chain_));

        // ********* 5. 각종 변수 초기화 *********

        // 5.1 Vector 초기화 (사이즈 정의 및 값 0)
        tau_d_.data = Eigen::VectorXd::Zero(n_joints_);

        qd_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_ddot_.data = Eigen::VectorXd::Zero(n_joints_);
        qd_old_.data = Eigen::VectorXd::Zero(n_joints_);
        
        //Own code:
        q_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        q_dotdot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_cmd_.data = Eigen::VectorXd::Zero(n_joints_);

        q_.data = Eigen::VectorXd::Zero(n_joints_);
        qdot_.data = Eigen::VectorXd::Zero(n_joints_);

        e_.data = Eigen::VectorXd::Zero(n_joints_);
        e_dot_.data = Eigen::VectorXd::Zero(n_joints_);
        e_int_.data = Eigen::VectorXd::Zero(n_joints_);
        
        q0_.data=Eigen::VectorXd::Zero(n_joints_);
        qdot0_.data = Eigen::VectorXd::Zero(n_joints_);
        q_1_.data=Eigen::VectorXd::Zero(n_joints_);
        
        f0_ = get_current_frame();

        //trajectory_received = false;
        point1_reached = false;
        point2_reached = false;
        distance_limit = 0.01;
        subs_count=0;
        
        // 5.2 Matrix 초기화 (사이즈 정의 및 값 0)
        M_.resize(kdl_chain_.getNrOfJoints());
        C_.resize(kdl_chain_.getNrOfJoints());
        G_.resize(kdl_chain_.getNrOfJoints());
        
        //Own code:
        J_.resize(kdl_chain_.getNrOfJoints());
        Jd_.resize(kdl_chain_.getNrOfJoints());

        // ********* 6. ROS 명령어 *********
        // 6.1 publisher
        pub_qd_ = n.advertise<std_msgs::Float64MultiArray>("qd", 1000);
        pub_q_ = n.advertise<std_msgs::Float64MultiArray>("q", 1000);
        pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);
        
        pub_qdot_ = n.advertise<std_msgs::Float64MultiArray>("qdot", 1000);
        pub_dist_ = n.advertise<std_msgs::Float64MultiArray>("dist", 1000);
        //pub_e_ = n.advertise<std_msgs::Float64MultiArray>("e", 1000);
        
        pub_x_ = n.advertise<std_msgs::Float64MultiArray>("x", 1000);
        pub_y_ = n.advertise<std_msgs::Float64MultiArray>("y", 1000);
        pub_z_ = n.advertise<std_msgs::Float64MultiArray>("z", 1000);
        
        pub_obst_x_ = n.advertise<std_msgs::Float64MultiArray>("obst_x", 1000);
        pub_obst_y_ = n.advertise<std_msgs::Float64MultiArray>("obst_y", 1000);
        pub_obst_z_ = n.advertise<std_msgs::Float64MultiArray>("obst_z", 1000);
        pub_obst_dist_ = n.advertise<std_msgs::Float64MultiArray>("obst_dist", 1000);

        pub_SaveData_ = n.advertise<std_msgs::Float64MultiArray>("SaveData", 1000); // 뒤에 숫자는?

        // 6.2 subsriber
        //sub_x_cmd_ = n.subscribe<std_msgs::Float64MultiArray>("/trajectory_topic", 1, &Reactive_Controller_V2::commandCB, this);
        sub_keyboard_input_ = n.subscribe<std_msgs::String>("/ex5_keyboard_input_topic", 1, &Point_obst_controller::keyboardCommandCB, this);
        
        obstacle_.p(0) = -0.2;
        obstacle_.p(1) = -0.1;
        obstacle_.p(2) = 0.5;
        obstacle_.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));
        
        point1_.p(0) = obstacle_.p(0)+0.13;
        point1_.p(1) = obstacle_.p(1);
        point1_.p(2) = obstacle_.p(2);
        point1_.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));
        
        point2_.p(0) = obstacle_.p(0)-0.15;
        point2_.p(1) = obstacle_.p(1);
        point2_.p(2) = obstacle_.p(2)+0.01;
        point2_.M = KDL::Rotation(KDL::Rotation::RPY(0, 0, 0));
        
        trajectory_received = true;
        f1_=point1_;
        current_f1_name_ = "point1_";
        robot_stopped = false;

        return true;
    }
    
    KDL::Frame get_current_frame()
    {
        KDL::Frame frame_to_return;
        for (int i = 0; i < n_joints_; i++)
        {
            q0_(i) = joints_[i].getPosition();
            //qdot0_(i) = joints_[i].getVelocity();
            qdot0_(i) = 0;
        }
        
        fk_pos_solver_->JntToCart(q0_, frame_to_return);
        return frame_to_return;
    }
    
    void keyboardCommandCB(const std_msgs::String::ConstPtr &keyboard_msg)
    {
    	
    	if (keyboard_msg->data == "stop")
     	{
     		ROS_INFO("Received stop-command!");
     		f1_when_stopped_ = f1_;
     		f1_name_when_stopped_ = current_f1_name_;
     		f0_ = get_current_frame();
     		//f1_ = f0_;
     		robot_stopped = true;
     	}
     	else if (keyboard_msg->data == "continue")
     	{
     		ROS_INFO("Received continue-command!");
     		f1_= f1_when_stopped_;
     		current_f1_name_ = f1_name_when_stopped_;
     		f0_ = get_current_frame();
     		robot_stopped = false;
     	}
     
    }

    void starting(const ros::Time &time)
    {
        t = 0.0;
        start_time=ros::Time::now();
        round = 1;
        ROS_INFO("Starting Computed Torque Controller");
        
    }
    
    KDL::JntArray getRepVelocity()
    {
	int k = 2;
	double q_star = 0.1;
	KDL::JntArray F;
	F.data = Eigen::VectorXd::Zero(n_joints_);
        dist_to_obst_tot_ = sqrt(pow(obstacle_.p(0)-x_.p(0),2)+pow(obstacle_.p(1)-x_.p(1),2)+pow(obstacle_.p(2)-x_.p(2),2));
        
        if (dist_to_obst_tot_ <= q_star)
        {
        	KDL::JntArray difference_vec;
        	difference_vec.data = Eigen::VectorXd::Zero(3);
        	difference_vec(0)=x_.p(0)-obstacle_.p(0);
        	difference_vec(1)=x_.p(1)-obstacle_.p(1);
        	difference_vec(2)=x_.p(2)-obstacle_.p(2);
        
        	KDL::JntArray dist_vec;
        	dist_vec.data =  difference_vec.data.transpose()*difference_vec.data;
        	Eigen::Matrix<double, 3, 6> diff_p_diff_q;
        	jnt_to_jac_solver_->JntToJac(q_, J_);
        	
        	diff_p_diff_q(0,0)=J_(0,0);
        	diff_p_diff_q(0,1)=J_(0,1);
        	diff_p_diff_q(0,2)=J_(0,2);
        	diff_p_diff_q(0,3)=J_(0,3);
        	diff_p_diff_q(0,4)=J_(0,4);
        	diff_p_diff_q(0,5)=J_(0,5);
        
        	diff_p_diff_q(1,0)=J_(1,0);
        	diff_p_diff_q(1,1)=J_(1,1);
        	diff_p_diff_q(1,2)=J_(1,2);
        	diff_p_diff_q(1,3)=J_(1,3);
        	diff_p_diff_q(1,4)=J_(1,4);
        	diff_p_diff_q(1,5)=J_(1,5);
        
        	diff_p_diff_q(2,0)=J_(2,0);
        	diff_p_diff_q(2,1)=J_(2,1);
        	diff_p_diff_q(2,2)=J_(2,2);
        	diff_p_diff_q(2,3)=J_(2,3);
        	diff_p_diff_q(2,4)=J_(2,4);
        	diff_p_diff_q(2,5)=J_(2,5);
        
        	KDL::JntArray doo_D_doo_q;
        	doo_D_doo_q.data = 2*difference_vec.data.transpose()*diff_p_diff_q;
        
        	double K = k*((1/dist_to_obst_tot_)-(1/q_star))*(1/(pow(dist_to_obst_tot_, 2)));
        	
        	ROS_INFO("Dist_to_obst_tot_ <= q_star, avoid obstacle! Distance: %f", (double)dist_to_obst_tot_);
        	F.data = K*doo_D_doo_q.data;
        	
        	if (F.data.dot(qdot_.data) < 0)
        	{
        		F.data = K*doo_D_doo_q.data;
        	}
        	else
        	{
        		F.data = Eigen::VectorXd::Zero(n_joints_);
        	}
        }
        else
        {
        	F.data = Eigen::VectorXd::Zero(n_joints_);
        }
        return F;
    }

    void update(const ros::Time &time, const ros::Duration &period)
    {
        // ********* 0. Get states from gazebo *********
        // 0.1 sampling time
        double dt = period.toSec();
        t = t + 0.001;

        // 0.2 joint state
        for (int i = 0; i < n_joints_; i++)
        {
            q_(i) = joints_[i].getPosition();
            qdot_(i) = joints_[i].getVelocity();
        } 

        // ********* 1. Desired Trajecoty in Joint Space *********
        //Straight line trajectory in joint space:
        double v_tau=0.1;
        double tau_0=0;
        double own_dt;
        target_time_ = 5;
        now=ros::Time::now();
        
        fk_pos_solver_->JntToCart(q_, x_);
        distance = sqrt(pow(f1_.p(0)-x_.p(0),2)+pow(f1_.p(1)-x_.p(1),2)+pow(f1_.p(2)-x_.p(2),2));
        //ROS_INFO("Distance here: %f", distance);
        
        if ((!trajectory_received)||(robot_stopped))
        {
        	f1_=f0_;
        }
        else
        {
        	//Tässä riittää varmaan vaan pointista 1 pointtiin 2 ja siihen väliin obstacle
        	if (distance <= distance_limit && point1_reached == false)
        	{
        		f0_=get_current_frame();
        		f1_=point2_;
        		current_f1_name_ = "point2_";
        		point1_reached = true;
        		ROS_INFO("Point1 reached!");
        	}
        	else if (distance <= distance_limit && point1_reached == true && point2_reached == false)
        	{
        		f0_=get_current_frame();
        		//f1_=point3_;
        		f1_=f0_;	
        		point2_reached = true;
        		current_f1_name_ = "point3_";
        		ROS_INFO("Point2 reached!");
        	}       	
        }
        		
        V0_ = diff(f0_, f1_)/target_time_;
        
        
        if (round==1)
        {
        	own_dt=(now-start_time).toSec();
        	tau_k=tau_0+own_dt*v_tau;
        	tau_old=tau_k;
        	xd_=KDL::addDelta(f0_,V0_,own_dt);
        }
        else
        {
        	own_dt=(now-old_time).toSec();
        	tau_k=tau_old+own_dt*v_tau;
        	tau_old=tau_k;
        	xd_=KDL::addDelta(xd_,V0_,own_dt);
        }
        if (tau_k >= 1)
        {
        	tau_k = 1;
        }

        // ********* 2. Motion Controller in Joint Space*********
        // *** 2.1 Error Definition in Joint Space ***
        e_.data = qd_.data - q_.data;
        e_dot_.data = qd_dot_.data - qdot_.data;
        e_int_.data = qd_.data - q_.data; // (To do: e_int 업데이트 필요요)
        
        // 1. Error as X_err=diff(T_e,T_d):
        xerr_temp_ = diff(x_, xd_,1);
        
        //2. Error as 𝑋𝑒𝑟𝑟 = [diff(𝑅𝑒, 𝑅𝑑), diff(𝑝𝑒, 𝑝𝑑)]:
        //xerr_temp_.rot = diff(x_.M, xd_.M);
        //xerr_temp_.vel = diff(x_.p, xd_.p);
        
        xerr_(0) = xerr_temp_(0);
        xerr_(1) = xerr_temp_(1);
        xerr_(2) = xerr_temp_(2);
        xerr_(3) = xerr_temp_(3);
        xerr_(4) = xerr_temp_(4);
        xerr_(5) = xerr_temp_(5);
        
        jnt_to_jac_solver_->JntToJac(q_, J_);
        double roll, pitch, yaw;
        xd_.M.GetRPY(roll, pitch, yaw);
        
        xd_temp_(0)=xd_.p(0);
        xd_temp_(1)=xd_.p(1);
        xd_temp_(2)=xd_.p(2);
        xd_temp_(3)=roll;
        xd_temp_(4)=pitch;
        xd_temp_(5)=yaw;
        
        V_cmd_ = xd_temp_ + Kp_.data.cwiseProduct(xerr_);
        float det = J_.data.determinant();
        if ((det >= 0 && det <= 0.01) || (det <= 0 && det >= -0.01))
        {
        	
        	pseudo_inverse(J_.data,J_inv_,false);
        }
        else
        {
        	J_inv_ = J_.data.inverse();
        }
        
        //q_dot_cmd_.data = J_inv_*V_cmd_; // which is sent to velocity controller
        KDL::JntArray q_rep=getRepVelocity();
        
        q_dot_cmd_.data = J_inv_*V_cmd_+q_rep.data;
        
        e_dot_cmd_.data=q_dot_cmd_.data - qdot_.data;

        // *** 2.2 Compute model(M,C,G) ***
        id_solver_->JntToMass(q_, M_);
        id_solver_->JntToCoriolis(q_, qdot_, C_);
        id_solver_->JntToGravity(q_, G_); 

        // *** 2.3 Apply Torque Command to Actuator ***
        
        //Own code for velocity controller (For velocity controller, K_p=0):

        aux_d_.data = M_.data * (Kd_.data.cwiseProduct(e_dot_cmd_.data));
        comp_d_.data = C_.data + G_.data;
        tau_d_.data = aux_d_.data + comp_d_.data;

        for (int i = 0; i < n_joints_; i++)
        {
            joints_[i].setCommand(tau_d_(i));
            // joints_[i].setCommand(0.0);
        }

        // ********* 3. data 저장 *********
        save_data();
        //print_frame(f1_);

        // ********* 4. state 출력 *********
        //print_state();
        
        round+=1;
        old_time=now;
        
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
        
        msg_x_.data.clear();
        msg_y_.data.clear();
        msg_z_.data.clear();
        
        msg_obst_x_.data.clear();
        msg_obst_y_.data.clear();
        msg_obst_z_.data.clear();
        msg_obst_dist_.data.clear();
        
        msg_dist_.data.clear();
        msg_qdot_.data.clear();

        msg_SaveData_.data.clear();

        // 3
        for (int i = 0; i < n_joints_; i++)
        {
            msg_qd_.data.push_back(qd_(i));
            msg_q_.data.push_back(q_(i));
            msg_e_.data.push_back(e_(i));
            
            msg_qdot_.data.push_back(qdot_(i));
        }
        msg_x_.data.push_back(x_.p(0));
        msg_y_.data.push_back(x_.p(1));
        msg_z_.data.push_back(x_.p(2));
        
        msg_obst_x_.data.push_back(obstacle_.p(0));
        msg_obst_y_.data.push_back(obstacle_.p(1));
        msg_obst_z_.data.push_back(obstacle_.p(2));
        msg_obst_dist_.data.push_back(dist_to_obst_tot_);
        
        msg_dist_.data.push_back(distance);

        for (int i = 0; i < SaveDataMax; i++)
        {
            msg_SaveData_.data.push_back(SaveData_[i]);
        }

        // 4
        pub_qd_.publish(msg_qd_);
        pub_q_.publish(msg_q_);
        pub_e_.publish(msg_e_);
        
        pub_x_.publish(msg_x_);
        pub_y_.publish(msg_y_);
        pub_z_.publish(msg_z_);
        
        pub_obst_x_.publish(msg_obst_x_);
        pub_obst_y_.publish(msg_obst_y_);
        pub_obst_z_.publish(msg_obst_z_);
        pub_obst_dist_.publish(msg_obst_dist_);
        
        pub_qdot_.publish(msg_qdot_);
        pub_dist_.publish(msg_dist_);

        pub_SaveData_.publish(msg_SaveData_);
    }

    void print_state()
    {
        static int count = 0;
        if (count > 99)
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


            count = 0;
        }
        count++;
    }
    
    void print_frame(KDL::Frame f_to_print)
    {
        printf("\n");
        printf("*** Print Frame  ***\n");
        printf("Position:\n");
        printf("[");
        printf("%f, ", f0_.p(0));
        printf("%f, ", f0_.p(1));
        printf("%f]\n", f0_.p(2));
        printf("\n");
        printf("Orientation:\n");
        printf("[[%f, %f, %f]\n", f0_.M.data[0], f0_.M.data[1], f0_.M.data[2]);
        printf(" [%f, %f, %f]\n", f0_.M.data[3], f0_.M.data[4], f0_.M.data[5]);
        printf(" [%f, %f, %f]]\n", f0_.M.data[6], f0_.M.data[7], f0_.M.data[8]);
        printf("*** End of Print Frame  ***\n\n");
    }


  private:
    // others
    double t;

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
    
    //Own code:
    KDL::JntArray q_dot_cmd_, q_dotdot_cmd_;
    KDL::JntArray e_dot_cmd_;
    
    //boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_; //Solver to compute the forward kinematics (position)
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;
    boost::scoped_ptr<KDL::ChainFkSolverVel_recursive> fk_vel_solver_;
    KDL::Frame xd_; // x.p: frame position(3x1), x.m: frame orientation (3x3)
    KDL::Frame x_;
    KDL::Twist xerr_temp_;
    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_; //Solver to compute the jacobian
    KDL::Jacobian J_;
    KDL::Jacobian Jd_;
    //KDL::Twist Vd_;
    Eigen::Matrix<double, num_taskspace, 1> Vd_;
    Eigen::Matrix<double, num_taskspace, 1> V_cmd_;
    Eigen::Matrix<double, num_taskspace, 1> xerr_;
    int round;
    double tau_k;
    double tau_old;
    KDL::JntArray q_1_;
    KDL::Frame f0_;
    KDL::Frame f1_;
    KDL::Frame f1_when_stopped_;
    KDL::Twist V0_;
    Eigen::Matrix<double, num_taskspace, 1> xd_temp_;
    KDL::JntArray q0_, qdot0_;
    
    KDL::Frame point1_;
    KDL::Frame point2_;
    KDL::Frame point3_;
    KDL::Frame point4_;
    
    KDL::Frame obstacle_;
    
    bool trajectory_received;
    double distance_limit;
    double distance;
    double dist_to_obst_tot_;
    bool point1_reached;
    bool point2_reached;
    bool point3_reached;
    bool point4_reached;
    bool opposite_direction;
    int subs_count;
    KDL::JntArray e_v_;
    Eigen::Matrix<double, num_taskspace, 1> V_feedback_;
    Eigen::MatrixXd J_inv_;
    bool robot_stopped;
    
    //Tämä lähinnä siksi että framien vertailu == tai equal voi olla haastavaa
    std::string current_f1_name_;
    std::string f1_name_when_stopped_;
    
    std::vector<KDL::Frame> points_;
    
    ros::Time start_time;
    ros::Time now;
    ros::Time old_time;

    // Input
    KDL::JntArray aux_d_;
    KDL::JntArray comp_d_;
    KDL::JntArray tau_d_;

    // gains
    KDL::JntArray Kp_, Ki_, Kd_;

    // save the data
    double SaveData_[SaveDataMax];

    // ros publisher
    ros::Publisher pub_qd_, pub_q_, pub_e_, pub_qdot_;
    
    ros::Publisher pub_x_, pub_y_, pub_z_, pub_dist_;
    ros::Publisher pub_obst_x_, pub_obst_y_, pub_obst_z_, pub_obst_dist_;
    
    ros::Publisher pub_SaveData_;
    
    ros::Subscriber sub_x_cmd_;
    ros::Subscriber sub_keyboard_input_;

    // ros message
    std_msgs::Float64MultiArray msg_qd_, msg_q_, msg_e_, msg_qdot_;
    std_msgs::Float64MultiArray msg_SaveData_;
    
    std_msgs::Float64MultiArray msg_x_, msg_y_, msg_z_, msg_dist_;
    std_msgs::Float64MultiArray msg_obst_x_, msg_obst_y_, msg_obst_z_, msg_obst_dist_;
    
    int target_time_ = 1;
};
}; // namespace arm_controllers
PLUGINLIB_EXPORT_CLASS(arm_controllers::Point_obst_controller, controller_interface::ControllerBase)
