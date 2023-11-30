#include "controller_components/controller.hpp"
#include "controller_components/pid.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/impl/utils.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <math.h>

#define RAD_TO_DEG 		(double)(180.0/M_PI)
#define DEG_TO_RAD 		(double)(M_PI/180.0)

#define MAX_ROLL_DEG	20.0
#define MAX_PITCH_DEG	20.0
#define MAX_YAW_DEG		160.0
#define MAX_THROTTLE	65000
#define TRIM_THROTTLE	40000

void Controller::set_target(double _x, double _y, double _z) {
	target_position[0] = _x;
	target_position[1] = _y;
	target_position[2] = _z;
}

Controller::Controller() {
	set_target(0.0,0.0,0.0);

	for(int i=0; i<3; i++) {
		current_position[i] = 0.0;
		current_orientation[i] = 0.0;
	}
}
Controller::~Controller() {

}

void Controller::rotate(const double _yaw, const double* _from, double* _to) {
	_to[0] =  _from[0] * cos(_yaw) + _from[1] * sin(_yaw);
	_to[1] = -_from[0] * sin(_yaw) + _from[1] * cos(_yaw);
	_to[2] =  _from[2];
}
void Controller::reset()
{
    pid_x.reset();
    pid_y.reset();
    pid_z.reset();
}
void Controller::initialize( PID_Config_t _config_x, PID_Config_t _config_y, PID_Config_t _config_z) {
	pid_x.initialize(_config_x);
	pid_y.initialize(_config_y);
	pid_z.initialize(_config_z);

    switch_to_standby();
}
void Controller::compute_outputs(geometry_msgs::msg::Twist* _cmd) {

    // position error in world frame
    double p_err_world[3], p_err_body[3];
    for(int i=0;i<3;i++) {
        p_err_world[i] = target_position[i] - current_position[i];
    }
    rotate(current_orientation[2], p_err_world, p_err_body);

    // generate commands
    double roll_cmd  =  pid_x.get_output(-p_err_body[0]) * RAD_TO_DEG;
    double pitch_cmd = -pid_y.get_output(-p_err_body[1]) * RAD_TO_DEG;
    double yaw_cmd	 =  target_yaw* RAD_TO_DEG;
    double throttle  =  pid_z.get_output( p_err_body[2]);

    _cmd->linear.y  = limit(roll_cmd, -MAX_ROLL_DEG, MAX_ROLL_DEG);
    _cmd->linear.x  = limit(pitch_cmd, -MAX_PITCH_DEG, MAX_PITCH_DEG);
    _cmd->angular.z = yaw_cmd;
    switch(flight_state) {
        case STANDBY:
            _cmd->linear.z  = limit(0, 0, MAX_THROTTLE);
            break;
        case EMERGENCY:
            _cmd->linear.z  = limit(0, 0, MAX_THROTTLE);
            break;
        default:
            _cmd->linear.z  = limit(throttle + TRIM_THROTTLE, 0.0, MAX_THROTTLE);
    }
}
void Controller::update_target(const geometry_msgs::msg::Pose::SharedPtr& _target_position) {
    set_target(_target_position->position.x, _target_position->position.y, _target_position->position.z);

    double roll, pitch;
    tf2::Quaternion q;
    tf2::fromMsg(_target_position->orientation, q);
    // tf2::quaternionMsgToTF(_target_position->orientation, q);
    tf2::Matrix3x3(q).getRPY(roll, pitch, target_yaw);
}
void Controller::update_position(const geometry_msgs::msg::PoseStamped::SharedPtr& _current_position) {
	current_position[0] = _current_position->pose.position.x;
	current_position[1] = _current_position->pose.position.y;
	current_position[2] = _current_position->pose.position.z;

    tf2::Quaternion q;
    tf2::fromMsg(_current_position->pose.orientation, q);
    tf2::Matrix3x3(q).getRPY(current_orientation[1], current_orientation[0], current_orientation[2]);
}

void Controller::switch_to_standby(void) {
    reset();
    if (flight_state != STANDBY)
	   flight_state = STANDBY;
}
void Controller::switch_to_tracking(void) {
    if (flight_state != TRACKING)
    {
        reset();
        flight_state = TRACKING;
    }
}
void Controller::switch_to_emergency(void) {
    reset();
    if (flight_state != EMERGENCY)
	   flight_state = EMERGENCY;
}
