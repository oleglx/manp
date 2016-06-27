#include<cstdio>
#include<ctime>

#include<ros/ros.h>
#include<std_msgs/Float64.h>
//#include<std_msgs/String.h>

std::vector<float> joint_positions(3), joint_efforts(3);
const std::vector<float> link_lengths(2), base_point(3);

class Manipulator {
	private:
		std::vector<float> base_point, angles, link_lengths;
		std::vector<float> point_zero, point_one, point_two;

		void updateManipulator();
	public:
		Manipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			setManipulator( manp_base_point, manp_angles, manp_link_lengths );		
		}
		
		void setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths );
		void getManipulator( std::vector<float>& manp_base_point, std::vector<float>& manp_angles, std::vector<float>& manp_link_lengths );
		void getPoint ( float link_length, std::vector<float> stationary, std::vector<float> moving, std::vector<float>& out );		
		void FABRIK( int iter, std::vector<float> dest );
};

/* 

// !!!! TODO: check thread safety, improve
robot_state st = {0,0,0,0,0,0,true};

// data for lines callback
ros::Time lines_last_time; 
bool line_data_new;
uint32_t p1x;
uint32_t p1y;
uint32_t p2x;
uint32_t p2y;

// data for control signals callback
ros::Time control_last_time; 
bool control_data_new;
char direction;

void lines_callback(const poly_detector::VectorOfLines::ConstPtr& inp_msg){
  double signal_lw = 0.0, signal_rw = 0.0; // values to send
  if(inp_msg->len>0){
    // read 1st line in the message
    assert(inp_msg->x.size()>=2); // sanity checks
    assert(inp_msg->y.size()==inp_msg->x.size());
    p1x = inp_msg->x[0];
    p2x = inp_msg->x[1];
    p1y = inp_msg->y[0];
    p2y = inp_msg->y[1];
    lines_last_time = ros::Time::now();
    line_data_new = true;
  }
}

void manual_direction_callback(const std_msgs::String::ConstPtr& inp_msg){
  	if(inp_msg->data.length()>0){
    		direction = tolower(inp_msg->data[0]);
    		if(direction!='w' && direction!='a' && direction!='s' && direction!='d'){
      			direction = 'n';
    		}
    		control_last_time = ros::Time::now();
    		control_data_new = true;
  	}
}

void timer_callback(const ros::TimerEvent &te){
  	if(!st.reinitialize_flag && (te.current_real > lines_last_time + ros::Duration(2))){
    		st.reinitialize_flag = true;
  	}
}

*/

void joints_callback( const sensor_msgs::JointState::ConstPtr& inp_msg ) {
	vector<sensor_msgs::JointState>::iterator it1;

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"world_to_base");
	if (it1==inp_msg->name.end()) {
		fprintf("stderr", "world_to_base not found \n");
	}
	else {
		joint_positions[2] = it1->position;
		joint_efforts[2] = it1->effort;
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link1_to_link2");
	if (it1==inp_msg->name.end()) {
		fprintf("stderr", "link1_to_link2 not found \n");
	}
	else {
		joint_positions[0] = it1->position;
		joint_efforts[0] = it1->effort;
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link2_to_link3");
	if (it1==inp_msg->name.end()) {
		fprintf("stderr", "link2_to_link3 not found \n");
	}
	else {
		joint_positions[1] = it1->position;
		joint_efforts[1] = it1->effort;
	}

}


int main( int argc, char **argv ) {
	ros::Publisher wtb_pub, l1tl2_pub, l2tl3_pub;
/*
	camera_geometry g_ldtester;
	// initialize global variables
	init_global_geometric_constants_ldtester(g_ldtester);
	line_data_new = control_data_new = false;
*/
	// initialize node
	ros::init(argc, argv, "manp_controller");
	// initialize publishing to the topic /poly_detector/lines
	ros::NodeHandle n;
	wtb_pub = n.advertise<std_msgs::Float64>("/manp/world_to_base_state_controller/command", 50);
	l1tl2_pub = n.advertise<std_msgs::Float64>("/manp/link1_to_link2_state_controller/command", 50);
	l2tl3_pub = n.advertise<std_msgs::Float64>("/manp/link2_to_link3_state_controller/command", 50);

	ros::Subscriber sub_coordinates = n.subscribe("/manp/joint_state", 50, joints_callback);

	Manipulator model(base_point, joint_positions, link_lengths);	

/*
	// timer to process command times
	ros::Timer timer = n.createTimer(ros::Duration(0.1), timer_callback);
	// subscribe to the topic with raw images from the camera
	ros::Subscriber sub_lines = n.subscribe("/poly_detector/lines", 50, lines_callback);
	// subscribe to the topic with commands from console
	ros::Subscriber sub_md = n.subscribe("/ldtester_linecontroller/manual_direction", 50, manual_direction_callback);
	// process input data from topics
*/
	ros::Rate r(50); // rate is 50 Hz
	//ros::spinOnce();
	r.sleep();
	while(ros::ok()){
/*
		double signal_lw = 0, signal_rw = 0;
		// line detected; process and send control signals to topic
		if(control_data_new && control_last_time>lines_last_time){
			// control signal detected and last time when a line was detected
			// earlier then 2 seconds ago
			move_by_manual_commands(direction, signal_lw, signal_rw);
			control_data_new = false;
		}
		else if(line_data_new && control_last_time + ros::Duration(1)<ros::Time::now()){
      			// 1. calculate coordinates of endpoints
      			p3d p1proj, p2proj;
      			calculate_point_projection_on_floor(g_ldtester, p1x, p1y, p1proj);
      			calculate_point_projection_on_floor(g_ldtester, p2x, p2y, p2proj);
      			// 2. calculate signals to the model motors
      			update_state_by_line(p1proj,p2proj,st);
      			get_control_signals(st, signal_lw, signal_rw);
      			line_data_new = false;
      			printf("%f %f\n", st.dist, st.orient*180.0/M_PI);
    		}
*/
		// send signals to the model motors
		std_msgs::Float64 out_msg;
		out_msg.data = 0.0;
		wtb_pub.publish(out_msg);
		out_msg.data = 0.0;
		l1tl2_pub.publish(out_msg);
		out_msg.data = 0.0;
		l2tl3_pub.publish(out_msg);
    		// wait next message
    		//ros::spinOnce(); 
    		r.sleep();
  	}
  	return 0;
}

//----------------------------------------------------------
//-----------------Defining functions-----------------------
//----------------------------------------------------------


void Manipulator::updateManipulator() {
	base_point[0] = point_zero[0];
	base_point[1] = point_zero[1];

	angles[0] = atan( (point_one[1] - point_zero[1])/(point_one[0] - point_zero[0]) );
	angles[1] = atan( (point_two[1] - point_one[1])/(point_two[0] - point_one[0]) );

	link_lengths[0] = pow(pow((point_one[0] - point_zero[0]),2) + pow((point_one[1] - point_zero[1]),2), 0.5);
	link_lengths[1] = pow(pow((point_two[0] - point_one[0]),2) + pow((point_two[1] - point_one[1]),2), 0.5);
}


void Manipulator::setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
	base_point = manp_base_point;
	angles = manp_angles;
	link_lengths = manp_link_lengths;

	point_zero[0] = base_point[0];
	point_zero[1] = base_point[1];

	point_one[0] = base_point[0] + link_lengths[0]*cos(angles[0]);
	point_one[1] = base_point[1] + link_lengths[0]*sin(angles[0]);

	point_two[0] = point_one[0] + link_lengths[1]*cos(angles[1]);
	point_two[1] = point_one[1] + link_lengths[1]*sin(angles[1]);
}

void Manipulator::getManipulator( std::vector<float>& manp_base_point, std::vector<float>& manp_angles, std::vector<float>& manp_link_lengths ) {
	manp_base_point = base_point;
	manp_angles = angles;
	manp_link_lengths = link_lengths;
}

void Manipulator::getPoint ( float link_length, std::vector<float> stationary, std::vector<float> moving, std::vector<float>& out ) {
	std::vector<float> vect;

	vect[0] = moving[0] - stationary[0];
	vect[1] = moving[1] - stationary[1];

	float k = pow(pow(vect[0],2) + pow(vect[1],2), 0.5);
	
	vect[0] /= k;
	vect[1] /= k;
	
	out[0] = stationary[0] + link_length*vect[0];
	out[1] = stationary[1] + link_length*vect[1];

}

void Manipulator::FABRIK( int iter, std::vector<float> dest ) {
	std::vector<float> zero_pv, one_pv, two_pv;
								
	for ( int i = 0; i < iter/2; i++ ) {
	//Staring reverse pass				
		zero_pv = point_zero;
		one_pv = point_one;			
		two_pv = point_two;

		//Moved point TWO to destination
		point_two = dest;
			
		//Moved point ONE for the reverse pass
		getPoint( link_lengths[1], point_two, one_pv, point_one );
			
		//Moved point ZERO for the reverse pass
		getPoint( link_lengths[0], point_one, zero_pv, point_zero);
						
	//Starting direct pass
		one_pv = point_one;			
		two_pv = point_two;
				
		//Moved point point_zero to previous position
		point_zero = zero_pv;
				
		//Moved point point_one for the direct pass
		getPoint( link_lengths[0], point_zero, one_pv, point_one );
				
		//Moved point point_two for the direct pass
		getPoint( link_lengths[1], point_one, two_pv, point_two );
	}
}
