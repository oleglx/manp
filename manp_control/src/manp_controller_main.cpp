#include<cstdio>
#include<ctime>

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>

std::vector<float> joint_positions(3), joint_efforts(3);
const std::vector<float> link_lengths(2), base_point(3), destination(3);

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
		bool checkDestination ( std::vector<float> joint_positions(3) );
};


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

	ros::init(argc, argv, "manp_controller");

	ros::NodeHandle n;
	wtb_pub = n.advertise<std_msgs::Float64>("/manp/world_to_base_state_controller/command", 50);
	l1tl2_pub = n.advertise<std_msgs::Float64>("/manp/link1_to_link2_state_controller/command", 50);
	l2tl3_pub = n.advertise<std_msgs::Float64>("/manp/link2_to_link3_state_controller/command", 50);

	ros::Subscriber sub_coordinates = n.subscribe("/manp/joint_state", 50, joints_callback);

	ros::Rate r(50);
	ros::spinOnce();

	Manipulator model(base_point, joint_positions, link_lengths);
	model.FABRIK(10, destination);
	model.updateManipulator();	

	r.sleep();
	while(ros::ok() & model.checkDestination(joint_positions) == false ) {
		
		ros::spinOnce();

		std_msgs::Float64 out_msg;
		out_msg.data = 0.0;
		wtb_pub.publish(out_msg);
		out_msg.data = 0.0;
		l1tl2_pub.publish(out_msg);
		out_msg.data = 0.0;
		l2tl3_pub.publish(out_msg);
    		
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

bool Manipulator::checkDestination ( std::vector<float> joint_positions ) {
	if (joint_positions == angles)
		return true;
	else return false;
}
