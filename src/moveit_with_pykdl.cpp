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


Eigen::VectorXd ee_twist(6), ee_twist_ext(7), ee_twist_sup(6), right_arm_joints_velocity(7), right_arm_joints_position(7), qdot(7),joint_values(7);
std::vector<double> joints_values(7),feedback_joints_values(7),q(7);
Eigen::MatrixXd my_jac(6,7);
//std::cout << " ********************* hello there ******************** " << std::endl;
Eigen::MatrixXd my_final_jac(6,8);
std::ofstream vel_command_file,vel_feedback_file,position_feedback_file,calculated_twist,feedback_twist;


void jocommCallback(sensor_msgs::JointState jo_state)
{
    //std::cout << jo_state.name[14] << " " << jo_state.name[15] << " " << jo_state.name[12] << " " << jo_state.name[13] << " " << jo_state.name[16] << " "
      //                             << jo_state.name[17] << " " << jo_state.name[18] << std::endl;
    /*right_arm_joints_velocity(0) = jo_state.velocity[11]; right_arm_joints_velocity(1) = jo_state.velocity[12]; right_arm_joints_velocity(2) = jo_state.velocity[9];
    right_arm_joints_velocity(3) = jo_state.velocity[10]; right_arm_joints_velocity(4) = jo_state.velocity[13]; right_arm_joints_velocity(5) = jo_state.velocity[14];
    right_arm_joints_velocity(6) = jo_state.velocity[15];
    right_arm_joints_position(0) = jo_state.position[11]; right_arm_joints_position(1) = jo_state.position[12]; right_arm_joints_position(2) = jo_state.position[9];
    right_arm_joints_position(3) = jo_state.position[10]; right_arm_joints_position(4) = jo_state.position[13]; right_arm_joints_position(5) = jo_state.position[14];
    right_arm_joints_position(6) = jo_state.position[15];*/
    right_arm_joints_position(0) = jo_state.position[4]; right_arm_joints_position(1) = jo_state.position[5]; right_arm_joints_position(2) = jo_state.position[2];
    right_arm_joints_position(3) = jo_state.position[3]; right_arm_joints_position(4) = jo_state.position[6]; right_arm_joints_position(5) = jo_state.position[7];
    right_arm_joints_position(6) = jo_state.position[8];

    for (int i = 0; i < joints_values.size(); i++)
        joints_values[i] = right_arm_joints_position(i);
    /*

    //ee_twist_sup = my_jac*qdot;
    //std::cout << "i am here" << std::endl;

    //std::cout << "velocity command is: " << std::endl << qdot << std::endl << "velocity feedback is: " << std::endl << right_arm_joints_velocity << std::endl
      //           << "*********************************************" << std::endl;
    vel_command_file << qdot(0) << "," << qdot(1) << "," << qdot(2) << "," << qdot(3) << "," << qdot(4) << "," << qdot(5) << "," << qdot(6) << "\n";
    vel_feedback_file << right_arm_joints_velocity(0) << "," << right_arm_joints_velocity(1) << "," << right_arm_joints_velocity(2) << "," << right_arm_joints_velocity(3) << "," <<
                right_arm_joints_velocity(4) << "," << right_arm_joints_velocity(5) << "," << right_arm_joints_velocity(6) << "\n";
    //vel_command_file << ee_twist(0) << "," << ee_twist(1) << "," << ee_twist(2) << "," << ee_twist(3) << "," << ee_twist(4) << "," << ee_twist(5) << "\n";
    //vel_feedback_file << ee_twist_sup(0) << "," << ee_twist_sup(1) << "," << ee_twist_sup(2) << "," << ee_twist_sup(3) << "," << ee_twist_sup(4) << "," << ee_twist_sup(5) << "\n";
    //vel_command_file << joint_values(0) << "," << joint_values(1) << "," << joint_values(2) << "," << joint_values(3) << "," << joint_values(4) << "," << joint_values(5) << "," << joint_values(6) << "\n";
    position_feedback_file << right_arm_joints_position(0) << "," << right_arm_joints_position(1) << "," << right_arm_joints_position(2) << "," << right_arm_joints_position(3) << "," <<
            right_arm_joints_position(4) << "," << right_arm_joints_position(5) << "," << right_arm_joints_position(6) << "\n";*/

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_with_pykdl");
    ros::NodeHandle node;
    //moveit::planning_interface::MoveGroup group("right_arm");
    //ros::AsyncSpinner spinner(10);
    //spinner.start();
    ros::CallbackQueue my_queue;
    node.setCallbackQueue(&my_queue);


    qdot(0) = 0.0; qdot(1) = 0.0; qdot(2) = 0.0; qdot(3) = 0.0; qdot(4) = 0.0; qdot(5) = 0.0; qdot(6) = 0.0;
    joint_values(0) = 0.0; joint_values(1) = 0.0; joint_values(2) = 0.0; joint_values(3) = 0.0; joint_values(4) = 0.0; joint_values(5) = 0.0; joint_values(6) = 0.0;
    /*vel_command_file.open("command.csv");
    vel_feedback_file.open("feedback.csv");
    position_feedback_file.open("position.csv");
    calculated_twist.open("calculated_twist.csv");
    feedback_twist.open("feedback_twist.csv");*/
    //ee_twist.setZero; ee_twist_sup.setZero; my_jac.setZero;
    ee_twist << 0.0,0.0,0.0,0.0,0.0,0.0;
    ee_twist_sup << 0.0,0.0,0.0,0.0,0.0,0.0;
    ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg;
    sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    pub_msg=node.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
    //ros::WallDuration my_dur(0.2);
    my_queue.callAvailable();
    //ros::spinOnce();
    double rate_hz = 1000;
    ros::Rate rate(rate_hz);
    //geometry_msgs::PoseStamped current_pose;
    //current_pose = group.getCurrentPose("right_gripper");
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
    //std::cout << "target X is: " << target_pose(0) << std::endl
      //           << "target Y is: " << target_pose(1) << std::endl
        //            << "target Z is: " << target_pose(2) << std::endl
          //             << "*************************************************" << std::endl;




    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    const moveit::core::LinkModel *right_arm = robot_model->getLinkModel("left_gripper");

    /*const moveit::core::JointModel *right_s0 = robot_model->getJointModel("right_s0"); const moveit::core::JointModel *right_s1 = robot_model->getJointModel("right_s1");
    const moveit::core::JointModel *right_e0 = robot_model->getJointModel("right_e0"); const moveit::core::JointModel *right_e1 = robot_model->getJointModel("right_e1");
    const moveit::core::JointModel *right_w0 = robot_model->getJointModel("right_w0"); const moveit::core::JointModel *right_w1 = robot_model->getJointModel("right_w1");
    const moveit::core::JointModel *right_w2 = robot_model->getJointModel("right_w2");
    //initial joint values
    robot_state::RobotState my_robot_initial_state(*group.getCurrentState());
    Eigen::VectorXd initial_joint_values(7);
    initial_joint_values << *my_robot_initial_state.getJointPositions("right_s0"), *my_robot_initial_state.getJointPositions("right_s1"),
                           *my_robot_initial_state.getJointPositions("right_e0"), *my_robot_initial_state.getJointPositions("right_e1"),
                           *my_robot_initial_state.getJointPositions("right_w0"), *my_robot_initial_state.getJointPositions("right_w1"),
                           *my_robot_initial_state.getJointPositions("right_w2");
    joint_values = initial_joint_values;
    ros::Rate rate(50);

    robot_state::JointStateGroup* joint_state_group;*/

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
    //ros::spinOnce();
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
        //my_queue.callAvailable();
        double rtdot =(30*pow(time_elapsed,2))/pow(duration,3) - (60*pow(time_elapsed,3))/pow(duration,4) + (30*pow(time_elapsed,4))/pow(duration,5);
        //double rtdot =(30*pow(my_timer.elapsed(),2))/pow(duration,3) - (60*pow(my_timer.elapsed(),3))/pow(duration,4) + (30*pow(my_timer.elapsed(),4))/pow(duration,5);
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
        for (int i=0; i < Z.size();i++)
            //Z(i) = alpha*(pow(my_values(i) - qmoy(i),5))/u;
            Z(i) = 2*alpha*(my_values(i) - qmoy(i))/(deltaq(i)*deltaq(i));
        //qdot = Jinv * ee_twist;
        //vel_command_file << qdot(0) << "," << qdot(1) << "," << qdot(2) << "," << qdot(3) << "," << qdot(4) << "," << qdot(5) << "," << qdot(6) << "\n";
        qdot = Jinv * ee_twist + (Id - Jinv * my_jac)*Z;
        //qdot = right_arm_joints_velocity + qdot;
        //vel_feedback_file << qdot(0) << "," << qdot(1) << "," << qdot(2) << "," << qdot(3) << "," << qdot(4) << "," << qdot(5) << "," << qdot(6) << "\n";
        //qdot(0) = 0.0; qdot(1) = 0.0; qdot(3) = 0.0; qdot(4) = 0.0; qdot(5) = 0.0; qdot(6) = 0.0;
        //qdot(2) = -qdot(2); qdot(4) = -qdot(4); qdot(5) = -qdot(5); qdot(6) = -qdot(6);
        my_robot_state.integrateVariableVelocity(joint_model_group,qdot,1.0/rate_hz);
        rate.sleep();

        time_elapsed = ros::Time::now().toSec() - start_my_timer.toSec();

        //ee_twist_ext << ee_twist(0), ee_twist(1), ee_twist(2), ee_twist(3), ee_twist(4), ee_twist(5), 0.0;
        //calculated_twist << ee_twist(0) << "," <<  ee_twist(1) << "," <<  ee_twist(2) << "\n";
        /*Eigen::Affine3d eMb = my_robot_state.getGlobalLinkTransform(right_arm).inverse();
        Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
        eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
        eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
        ee_twist = eWb * ee_twist;*/
        //my_robot_state.setVariablePositions(variable_names,joints_values);

        //my_robot_state.computeVariableVelocity(joint_model_group,qdot,ee_twist, right_arm);

        //my_final_jac.block(0,0, 6,7) = my_jac;
        //my_final_jac.col(7) << ee_twist(0), ee_twist(1), ee_twist(2), ee_twist(3), ee_twist(4), ee_twist(5);
        //ee_twist_sup = my_jac*right_arm_joints_velocity;
        //feedback_twist << ee_twist_sup(0) << "," <<  ee_twist_sup(1) << "," <<  ee_twist_sup(2) << "\n";
        //Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp(my_jac);
        //Eigen::FullPivLU<Eigen::MatrixXd> lu_decomp_f(my_final_jac);
        //std::cout << "original jacobian rank is: " << lu_decomp.rank() << std::endl;
        //std::cout << "augemented jacobian rank is: " << lu_decomp_f.rank() << std::endl;

        //compute Z
        /*double u = pow(my_values(0) - qmoy(0),6) + pow(my_values(1) - qmoy(1),6) + pow(my_values(2) - qmoy(2),6)
                 + pow(my_values(3) - qmoy(3),6) + pow(my_values(4) - qmoy(4),6) + pow(my_values(5) - qmoy(5),6)
                 + pow(my_values(6) - qmoy(6),6);
        u = pow(u,5/6);*/


        /*command_msg.command.clear();

        command_msg.command.push_back(qdot(0)); command_msg.command.push_back(qdot(1));
        command_msg.command.push_back(qdot(2)); command_msg.command.push_back(qdot(3));
        command_msg.command.push_back(qdot(4)); command_msg.command.push_back(qdot(5));
        command_msg.command.push_back(qdot(6));
        pub_msg.publish(command_msg);*/

        //ros::spinOnce();
        //std::cout << my_timer.elapsed() << std::endl;
        /*my_robot_state.setJointPositions(right_s0,&right_arm_joints_position(0)); my_robot_state.setJointPositions(right_s1,&right_arm_joints_position(1));
        my_robot_state.setJointPositions(right_e0,&right_arm_joints_position(2)); my_robot_state.setJointPositions(right_e1,&right_arm_joints_position(3));
        my_robot_state.setJointPositions(right_w0,&right_arm_joints_position(4)); my_robot_state.setJointPositions(right_w1,&right_arm_joints_position(5));
        my_robot_state.setJointPositions(right_w2,&right_arm_joints_position(6));*/
        //robot_state::RobotStatePtr kinematic_state = robot_state::RobotStatePtr ( new robot_state::RobotState (robot_model_loader));

        //double rtdot =(30*pow(my_timer.elapsed(),2))/pow(duration,3) - (60*pow(my_timer.elapsed(),3))/pow(duration,4) + (30*pow(my_timer.elapsed(),4))/pow(duration,5);

        //Rotate the twist to the end-effector frame
        /*Eigen::Affine3d eMb = my_robot_state.getGlobalLinkTransform(right_arm).inverse();
        Eigen::MatrixXd eWb = Eigen::ArrayXXd::Zero(6, 6);
        eWb.block(0, 0, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
        eWb.block(3, 3, 3, 3) = eMb.matrix().block(0, 0, 3, 3);
        ee_twist = eWb * ee_twist;*/
        //my_robot_state.computeVariableVelocity(joint_model_group,qdot,ee_twist, right_arm);
        /*kinematic_state->setJointPositions(right_arm_joints_position);
        kinematic_state->updateLinkTransforms();
        const robot_state::JointModelGroup *joint_state_group = kinematic_state->getJointModelGroup( "right_arm" );
        joint_state_group->computeJointVelocity( qdot, ee_twist, joint_state_group->getJointModelGroup ()->getLinkModelNames ().back () );*/

        //my_jac = eWb * my_jac;
        //Do the Jacobian moore-penrose pseudo-inverse

        //qdot = 1.3*qdot;
        //double gain = 3;
        //qdot(2) = gain*qdot(2); qdot(4) = gain*qdot(4); qdot(5) = gain*qdot(5); qdot(6) = gain*qdot(6);

        //dt =  time_elapsed - last_time;

        //joint_values = joint_values + (1.0/rate_hz) * qdot;


        /*command_msg.mode = command_msg.POSITION_MODE;

        command_msg.command.push_back(joint_values(0)); command_msg.command.push_back(joint_values(1));
        command_msg.command.push_back(joint_values(2)); command_msg.command.push_back(joint_values(3));
        command_msg.command.push_back(joint_values(4)); command_msg.command.push_back(joint_values(5));
        command_msg.command.push_back(joint_values(6));
        pub_msg.publish(command_msg);*/
        //std::cout << "the ros time now is: " << time_elapsed << std::endl;
        //std::cout << "the boost time now is: " << my_timer.elapsed() << std::endl;*/
        //std::cout << "dt is: " << time_elapsed - last_time << std::endl;
        //last_time = time_elapsed;
    }
    /*command_msg.mode = command_msg.POSITION_MODE;
    time_elapsed = 0.0;
    start_my_timer = ros::Time::now();
    while(ros::ok() && time_elapsed <= duration){
        command_msg.command.clear();
        command_msg.command.push_back(joint_values(0)); command_msg.command.push_back(joint_values(1));
        command_msg.command.push_back(joint_values(2)); command_msg.command.push_back(joint_values(3));
        command_msg.command.push_back(joint_values(4)); command_msg.command.push_back(joint_values(5));
        command_msg.command.push_back(joint_values(6));
        pub_msg.publish(command_msg);
        rate.sleep();
        time_elapsed = ros::Time::now().toSec() - start_my_timer.toSec();
    }*/
    std::cout << " ************************ I finished ******************************* " << std::endl;
    //vel_command_file.close();
    //vel_feedback_file.close();
    //position_feedback_file.close();
    //robot_state::RobotState my_robot_state(robot_model);
    //my_robot_state = *group.getCurrentState();
    my_robot_state.update(true);

    //baxter_core_msgs::JointCommand command_msg;
    command_msg.mode = command_msg.POSITION_MODE;

    command_msg.command.clear();

    command_msg.command.push_back(*my_robot_state.getJointPositions("left_s0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_s1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_e0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_e1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_w1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w2"));
    double pub_time_elapsed = 0.0;
    ros::Time start_my_publish_timer = ros::Time::now();
    while(ros::ok() && pub_time_elapsed < duration){
        pub_msg.publish(command_msg);
        pub_time_elapsed = ros::Time::now().toSec() - start_my_publish_timer.toSec();
    }


    //current_pose = group.getCurrentPose("right_gripper");
    //std::cout << "X is: " << current_pose.pose.position.x << std::endl
      //           << "Y is: " << current_pose.pose.position.y << std::endl
        //            << "Z is: " << current_pose.pose.position.z << std::endl;
    //std::cout << "duration was: " << duration << std::endl;
    //my_queue.callAvailable();
    //my_robot_state.setVariablePositions(variable_names,q);
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

