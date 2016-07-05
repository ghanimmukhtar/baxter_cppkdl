#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sensor_msgs/JointState.h>
#include <boost/timer.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <baxter_core_msgs/JointCommand.h>

Eigen::VectorXd ee_twist(6), ee_twist_ext(7), ee_twist_sup(6), left_arm_joints_velocity(7), left_arm_joints_position(7), qdot(7),joint_values(7);
std::vector<double> joints_values(7),feedback_joints_values(7),q(7);
Eigen::MatrixXd my_jac(6,7);
Eigen::MatrixXd my_final_jac(6,8);
std::ofstream vel_command_file,vel_feedback_file,position_feedback_file,calculated_twist,feedback_twist;

void jocommCallback(sensor_msgs::JointState jo_state)
{
    left_arm_joints_velocity(0) = jo_state.velocity[5]; left_arm_joints_velocity(1) = jo_state.velocity[6]; left_arm_joints_velocity(2) = jo_state.velocity[3];
    left_arm_joints_velocity(3) = jo_state.velocity[4]; left_arm_joints_velocity(4) = jo_state.velocity[7]; left_arm_joints_velocity(5) = jo_state.velocity[8];
    left_arm_joints_velocity(6) = jo_state.velocity[9];
    left_arm_joints_position(0) = jo_state.position[5]; left_arm_joints_position(1) = jo_state.position[6]; left_arm_joints_position(2) = jo_state.position[3];
    left_arm_joints_position(3) = jo_state.position[4]; left_arm_joints_position(4) = jo_state.position[7]; left_arm_joints_position(5) = jo_state.position[8];
    left_arm_joints_position(6) = jo_state.position[9];

    for (int i = 0; i < joints_values.size(); i++)
        joints_values[i] = left_arm_joints_position(i);
    vel_command_file << qdot(0) << "," << qdot(1) << "," << qdot(2) << "," << qdot(3) << "," << qdot(4) << "," << qdot(5) << "," << qdot(6) << "\n";
    vel_feedback_file << left_arm_joints_velocity(0) << "," << left_arm_joints_velocity(1) << "," << left_arm_joints_velocity(2) << "," << left_arm_joints_velocity(3) << "," <<
                left_arm_joints_velocity(4) << "," << left_arm_joints_velocity(5) << "," << left_arm_joints_velocity(6) << "\n";
    position_feedback_file << left_arm_joints_position(0) << "," << left_arm_joints_position(1) << "," << left_arm_joints_position(2) << "," << left_arm_joints_position(3) << "," <<
            left_arm_joints_position(4) << "," << left_arm_joints_position(5) << "," << left_arm_joints_position(6) << "\n";

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_with_pykdl");
    ros::NodeHandle node;
    ros::CallbackQueue my_queue;
    node.setCallbackQueue(&my_queue);

    qdot(0) = 0.0; qdot(1) = 0.0; qdot(2) = 0.0; qdot(3) = 0.0; qdot(4) = 0.0; qdot(5) = 0.0; qdot(6) = 0.0;
    joint_values(0) = 0.0; joint_values(1) = 0.0; joint_values(2) = 0.0; joint_values(3) = 0.0; joint_values(4) = 0.0; joint_values(5) = 0.0; joint_values(6) = 0.0;
    vel_command_file.open("command.csv");
    vel_feedback_file.open("feedback.csv");
    position_feedback_file.open("position.csv");
    calculated_twist.open("calculated_twist.csv");
    feedback_twist.open("feedback_twist.csv");
    ee_twist << 0.0,0.0,0.0,0.0,0.0,0.0;
    ee_twist_sup << 0.0,0.0,0.0,0.0,0.0,0.0;
    ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg;
    sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    pub_msg=node.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
    my_queue.callAvailable();
    double rate_hz = 1000;
    ros::Rate rate(rate_hz);
    usleep(1e6);
    Eigen::VectorXd current_position(3);
    //defining joints limits, mean values, ranges and other variables to avoid joint limits later
    Eigen::VectorXd qmin(7),qmax(7),qmoy(7),deltaq(7),Z(7);
    Eigen::MatrixXd Id = Eigen::VectorXd::Ones(7).asDiagonal();
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    double alpha = -2;
    qmin << -1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059;
    qmax << 1.7016,1.047,3.0541,2.618,3.059,2.094,3.059;
    qmoy = 0.5*(qmin + qmax);
    //double safe_q = 0.5;
    //qmoy << safe_q, safe_q, safe_q, safe_q, safe_q, safe_q, safe_q;
    deltaq = qmax - qmin;

    Eigen::VectorXd target_pose(3),distance(3),my_values;
    target_pose << atof(argv[1]),atof(argv[2]),atof(argv[3]);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    const moveit::core::LinkModel *right_arm = robot_model->getLinkModel("left_gripper");

    robot_state::RobotState my_robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = my_robot_state.getJointModelGroup("left_arm");

    std::vector <std::string> variable_names(7);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";

    my_queue.callAvailable();
    my_robot_state.setVariablePositions(variable_names,joints_values);
    my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
    Eigen::VectorXd initial_joint_values(7);
    initial_joint_values = my_values;
    joint_values = initial_joint_values;
    current_position = my_robot_state.getGlobalLinkTransform("left_gripper").translation();
    std::cout << "X is: " << current_position(0) << std::endl
                 << "Y is: " << current_position(1) << std::endl
                    << "Z is: " << current_position(2) << std::endl
                       << "*************************************************" << std::endl;
    distance << target_pose(0) - current_position(0), target_pose(1) - current_position(1), target_pose(2) - current_position(2);
    std::cout << distance << std::endl;
    std::cout << "********************************************" << std::endl;
    double duration = 7.5*distance.norm()/0.8;

    std::cout << my_values << std::endl;
    std::cout << "********************************************" << std::endl;
    baxter_core_msgs::JointCommand command_msg;

    command_msg.mode = command_msg.VELOCITY_MODE;
    command_msg.names.push_back("left_s0"); command_msg.names.push_back("left_s1"); command_msg.names.push_back("left_e0");
    command_msg.names.push_back("left_e1"); command_msg.names.push_back("left_w0"); command_msg.names.push_back("left_w1");
    command_msg.names.push_back("left_w2");

    double time_elapsed = 0.0,last_time = 0.0, dt = 0.0;
    boost::timer my_timer;
    ros::Time start_my_timer = ros::Time::now();
    while(ros::ok() && time_elapsed < duration){
        double rtdot =(30*pow(time_elapsed,2))/pow(duration,3) - (60*pow(time_elapsed,3))/pow(duration,4) + (30*pow(time_elapsed,4))/pow(duration,5);
        ee_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2), 0.0, 0.0, 0.0;
        my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
        for (int i = 0; i < q.size(); i++)
            q[i] = my_values(i);
        my_robot_state.setVariablePositions(variable_names,q);
        my_robot_state.getJacobian(joint_model_group,right_arm,reference_point,my_jac,false);

        Eigen::JacobiSVD<Eigen::MatrixXd> svdOfJ(my_jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const Eigen::MatrixXd U = svdOfJ.matrixU();
        const Eigen::MatrixXd V = svdOfJ.matrixV();
        const Eigen::VectorXd S = svdOfJ.singularValues();
        Eigen::VectorXd Sinv = S;

        static const double pinvtoler = std::numeric_limits<float>::epsilon();
        double maxsv = 0.0 ;
        for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
            if (fabs(S(i)) > maxsv) maxsv = fabs(S(i));
        for (std::size_t i = 0; i < static_cast<std::size_t>(S.rows()); ++i)
          {
            //Those singular values smaller than a percentage of the maximum singular value are removed
            if (fabs(S(i)) > maxsv * pinvtoler)
              Sinv(i) = 1.0 / S(i);
            else Sinv(i) = 0.0;
          }
        Eigen::MatrixXd Jinv = (V * Sinv.asDiagonal() * U.transpose());
        //compute Z
        for (int i=0; i < Z.size();i++)
            Z(i) = 2*alpha*(my_values(i) - qmoy(i))/(deltaq(i)*deltaq(i));
        qdot = Jinv * ee_twist + (Id - Jinv * my_jac)*Z;
        my_robot_state.integrateVariableVelocity(joint_model_group,qdot,1.0/rate_hz);
        rate.sleep();
        time_elapsed = ros::Time::now().toSec() - start_my_timer.toSec();
    }
    std::cout << " ************************ I finished ******************************* " << std::endl;
    vel_command_file.close();
    vel_feedback_file.close();
    position_feedback_file.close();
    my_robot_state.update(true);
    command_msg.mode = command_msg.POSITION_MODE;
    command_msg.command.clear();
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_s0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_s1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_e0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_e1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_w1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w2"));
    pub_msg.publish(command_msg);
    std::cout << my_robot_state.getGlobalLinkTransform("left_gripper").translation() << std::endl;
    std::cout << "s0 is: " << *my_robot_state.getJointPositions("left_s0") << std::endl
                     << "s1 is: " << *my_robot_state.getJointPositions("left_s1") << std::endl
                        << "e0 is: " << *my_robot_state.getJointPositions("left_e0") << std::endl
                        << "e1 is: " << *my_robot_state.getJointPositions("left_e1") << std::endl
                        << "w0 is: " << *my_robot_state.getJointPositions("left_w0") << std::endl
                        << "w1 is: " << *my_robot_state.getJointPositions("left_w1") << std::endl
                        << "w2 is: " << *my_robot_state.getJointPositions("left_w2") << std::endl;
    return 0;
}
