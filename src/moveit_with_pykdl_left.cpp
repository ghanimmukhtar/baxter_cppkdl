#include <ros/ros.h>
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

Eigen::VectorXd ee_twist(6), qdot(7);
std::vector<double> joints_values(7),q(7);
Eigen::MatrixXd my_jac(6,7);
Eigen::MatrixXd my_final_jac(6,8);

void jocommCallback(sensor_msgs::JointState jo_state)
{
    joints_values[0] = jo_state.position[5]; joints_values[1] = jo_state.position[6]; joints_values[2] = jo_state.position[3];
    joints_values[3] = jo_state.position[4]; joints_values[4] = jo_state.position[7]; joints_values[5] = jo_state.position[8];
    joints_values[6] = jo_state.position[9];
}

//This is a function that takes as inputs: a character that name an axis (x, y or z), and a float that represents an angle of rotation about that axis. It returns the corresponding rotation matrix (3x3)
Eigen::Matrix3d Rot(char axis, float angle) {
    Eigen::Matrix3d RotX,RotY,RotZ,empty_Rot;
    if (axis == 'x'){
        RotX << 1,          0,           0,
                0, cos(angle), -sin(angle),
                0, sin(angle),  cos(angle);
        return RotX;
    }
    else if (axis == 'y'){
        RotY << cos(angle),  0, sin(angle),
                         0,  1,          0,
               -sin(angle),  0, cos(angle);
        return RotY;
    }
    else if (axis == 'z'){
        RotZ << cos(angle), -sin(angle), 0,
                sin(angle),  cos(angle), 0,
                         0,           0, 1;
        return RotZ;
    }
    else{
        std::cout << "enter valid coordinate: X, Y or Z -------------------------------------------------" << std::endl;
        return empty_Rot;
    }

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_with_pykdl_left");
    ros::NodeHandle node;
    ros::Publisher pub_msg;
    ros::Subscriber sub_jointmsg;
    sub_jointmsg = node.subscribe<sensor_msgs::JointState>("/robot/joint_states",1,jocommCallback);
    pub_msg=node.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command",1);
    ros::AsyncSpinner my_spinner(4);
    my_spinner.start();
    usleep(1e6);
    double rate_hz = 1000;
    Eigen::VectorXd current_position(3), current_angles(3);
    //defining joints limits, mean values, ranges and other variables to avoid joint limits later
    Eigen::VectorXd qmin(7),qmax(7),qmoy(7),deltaq(7),Z(7);
    Eigen::MatrixXd Id = Eigen::VectorXd::Ones(7).asDiagonal();
    Eigen::Vector3d reference_point(0.0, 0.0, 0.0);
    double alpha = -2;
    qmin << -1.7016,-2.147,-3.0541,-0.05,-3.059,-1.5707,-3.059;
    qmax << 1.7016,1.047,3.0541,2.618,3.059,2.094,3.059;
    qmoy = 0.5*(qmin + qmax);
    deltaq = qmax - qmin;

    Eigen::VectorXd target_pose(6),distance(6),my_values;
    target_pose << atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]);

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();
    const moveit::core::LinkModel *right_arm = robot_model->getLinkModel("left_gripper");

    robot_state::RobotState my_robot_state(robot_model);
    const robot_state::JointModelGroup *joint_model_group = my_robot_state.getJointModelGroup("left_arm");

    std::vector <std::string> variable_names(7);
    variable_names[0] = "left_s0"; variable_names[1] = "left_s1"; variable_names[2] = "left_e0";
    variable_names[3] = "left_e1"; variable_names[4] = "left_w0"; variable_names[5] = "left_w1";
    variable_names[6] = "left_w2";

    my_robot_state.setVariablePositions(variable_names,joints_values);
    my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
    current_position = my_robot_state.getGlobalLinkTransform("left_gripper").translation();
    Eigen::Affine3d f_trans_mat;
    f_trans_mat = my_robot_state.getGlobalLinkTransform("left_gripper");
    Eigen::Matrix4d transform_l_ee_w = f_trans_mat.matrix();
    double Roll, Pitch, Yaw;

    Roll = atan2(transform_l_ee_w(1, 0), transform_l_ee_w(0, 0));
    Pitch = atan2(-transform_l_ee_w(2, 0), cos(Roll) * transform_l_ee_w(0, 0) + sin(Roll) * transform_l_ee_w(1, 0));
    Yaw = atan2(sin(Roll) * transform_l_ee_w(0, 2) - cos(Roll) * transform_l_ee_w(1, 2), -sin(Roll) * transform_l_ee_w(0, 1) + cos(Roll) * transform_l_ee_w(1, 1));

    current_angles << Roll,Pitch,Yaw;
    //std::cout << "translation part is: " << std::endl << current_position << std::endl;
    //std::cout << "orientation part is: " << std::endl << current_angles << std::endl;

    Eigen::Vector3d u;
    Eigen::Matrix3d Rfinal;
    Eigen::Matrix3d Rinitial = f_trans_mat.rotation();
    //if orientation is not defined make the end effector point towards the point
    if(target_pose(3) == 0.0 && target_pose(4) == 0.0 && target_pose(5) == 0.0) {
        double angle = atan2(target_pose[1], target_pose[0]);
        double ang_x = 0, ang_y = 2., ang_z = angle;
        Rfinal = Rot('z',ang_z)*Rot('y',ang_y)*Rot('x',ang_x);
        //std::cout << "angle z is: " << ang_z << std::endl;
        //std::cout << "angle y is: " << ang_y << std::endl;
        //std::cout << "angle x is: " << ang_x << std::endl;
    }
    else
        Rfinal = Rot('z',target_pose[3])*Rot('y',target_pose[4])*Rot('x',target_pose[5]);
    //std::cout << "initial orientation is: " << std::endl << Rinitial << std::endl;
    //std::cout << "final orientation should be: " << std::endl << Rfinal << std::endl;

    //then deduce the difference between the intial and final points of a section in oreintation:
    Eigen::Matrix3d RotuAlpha = Rfinal*Rinitial.transpose();

    //then we extract the angle alpha:
    double Calpha=0.5*(RotuAlpha(0,0)+RotuAlpha(1,1)+RotuAlpha(2,2)-1);
    double Salpha=0.5*sqrt(pow(RotuAlpha(1,2) - RotuAlpha(2,1),2) + pow(RotuAlpha(2,0) - RotuAlpha(0,2),2) + pow(RotuAlpha(0,1) - RotuAlpha(1,0),2));
    double my_alpha=atan2(Salpha,Calpha);

    //now that we have alpha let's deduce the vector u and skew symmetric which will be used to evaluate the evolution of the orientation with times
    if (my_alpha == 0){
        u = Eigen::VectorXd::Zero(3);
    }
    else{
        u << (RotuAlpha(2,1)-RotuAlpha(1,2))/(2*Salpha),
            (RotuAlpha(0,2)-RotuAlpha(2,0))/(2*Salpha),
            (RotuAlpha(1,0)-RotuAlpha(0,1))/(2*Salpha);
        Eigen::Matrix3d uskew;
        uskew <<     0,  -u(2),  u(1),
                  u(2),      0, -u(0),
                 -u(1),   u(0),     0;
    }
    std::cout << "vector u is: " << std::endl << u << std::endl;
    //std::cout << "the whole matrix is: " << std::endl << f_trans_mat.matrix() << std::endl;
    //std::cout << "initial orientation is: " << std::endl << Rinitial << std::endl;
    distance << target_pose(0) - current_position(0), target_pose(1) - current_position(1), target_pose(2) - current_position(2),
            target_pose(3) - current_angles(0), target_pose(4) - current_angles(1),target_pose(5) - current_angles(2);
    Eigen::Vector3d reduced_distance;
    reduced_distance << distance(0), distance(1), distance(2);
    double duration = std::max(7.5*u.norm(),7.5*reduced_distance.norm());

    std::cout << "duration is: " << duration << std::endl;
    std::cout << "distance is: " << std::endl << reduced_distance << std::endl;

    double time_elapsed = 0.0;
    while(ros::ok() && time_elapsed < duration){
        double rtdot =(30*pow(time_elapsed,2))/pow(duration,3) - (60*pow(time_elapsed,3))/pow(duration,4) + (30*pow(time_elapsed,4))/pow(duration,5);
        ee_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2), rtdot * my_alpha * u(0), rtdot * my_alpha * u(1), rtdot * my_alpha * u(2);
        //ee_twist << rtdot * distance(0), rtdot * distance(1), rtdot * distance(2), 0, 0, 0;
        my_robot_state.copyJointGroupPositions(my_robot_state.getRobotModel()->getJointModelGroup("left_arm"),my_values);
        for (int i = 0; i < q.size(); i++)
            q[i] = my_values(i);
        my_robot_state.setVariablePositions(variable_names,q);
        my_robot_state.getJacobian(joint_model_group,right_arm,reference_point,my_jac,false);
        if(fabs((my_jac * my_jac.transpose()).determinant()) < 0.000008){
            std::cout << "jacobian determinant is less than 0.01: " << std::endl;
            std::cout << (my_jac * my_jac.transpose()).determinant() << std::endl;
            return 0;
        }
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
        time_elapsed = time_elapsed + 1.0/rate_hz;
    }
    std::cout << " ************************ I finished ******************************* " << std::endl;

    my_robot_state.update(true);
    baxter_core_msgs::JointCommand command_msg;
    command_msg.mode = command_msg.POSITION_MODE;
    command_msg.names.push_back("left_s0"); command_msg.names.push_back("left_s1"); command_msg.names.push_back("left_e0");
    command_msg.names.push_back("left_e1"); command_msg.names.push_back("left_w0"); command_msg.names.push_back("left_w1");
    command_msg.names.push_back("left_w2");

    command_msg.command.push_back(*my_robot_state.getJointPositions("left_s0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_s1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_e0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_e1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w0")); command_msg.command.push_back(*my_robot_state.getJointPositions("left_w1"));
    command_msg.command.push_back(*my_robot_state.getJointPositions("left_w2"));
    pub_msg.publish(command_msg);

    Eigen::VectorXd final_ee_pose = my_robot_state.getGlobalLinkTransform("left_gripper").translation();
    Eigen::Vector4d goal_position;
    goal_position << target_pose(0), target_pose(1), target_pose(2), 1.0;
    Eigen::Vector3d reduced_goal;
    reduced_goal << target_pose(0), target_pose(1), target_pose(2);
    Eigen::Affine3d trans_mat;
    trans_mat = my_robot_state.getGlobalLinkTransform("left_gripper");
    transform_l_ee_w = trans_mat.matrix();
    Eigen::VectorXd result = final_ee_pose - reduced_goal;
    Eigen::Vector4d error, transform;
    error << result(0),result(1),result(2),1.0;
    transform = transform_l_ee_w * error;
    Eigen::Vector3d last_angles;
    Pitch = asin(transform_l_ee_w(0,2));
    if (Pitch == M_PI/2)
    {
        Yaw = 0;
        Roll = atan2(transform_l_ee_w(1,0),-transform_l_ee_w(2,0));
    }
    else if (Pitch == -M_PI/2)
    {
        Yaw = 0;
        Roll = -atan2(transform_l_ee_w(1,0),transform_l_ee_w(2,0));
    }
    else
    {
        Yaw = atan2(-transform_l_ee_w(0,1)/cos(Pitch),transform_l_ee_w(0,0)/cos(Pitch));
        Roll = atan2(-transform_l_ee_w(1,2)/cos(Pitch),transform_l_ee_w(2,2)/cos(Pitch));
    }
    last_angles << Roll,Pitch,Yaw;
    std::cout << "translation is: " << std::endl << trans_mat.translation() << std::endl;
    std::cout << "last angles are: " << std::endl << last_angles << std::endl;
    //std::cout << "error is: " << std::endl << result << std::endl;
    //std::cout << "goal in ee frame: " << std::endl << transform << std::endl;
    //std::cout << "ee final pose is: " << std::endl << final_ee_pose << std::endl;
    std::cout << "real final orientation matrix is: " << std::endl << trans_mat.rotation() << std::endl;
    return 0;
}
