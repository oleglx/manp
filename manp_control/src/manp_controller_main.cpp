#include<cstdio>
#include<ctime>

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>

std::vector<float> joint_positions(3), joint_efforts(3);
std::vector<float> link_lengths(3), base_point(3), destination(3);

class Manipulator {
	//private
	public:
		std::vector<float> base_point, angles, link_lengths;
		std::vector<float> point_zero, point_one, point_two;

		void updateManipulator();
	public:
		Manipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) : base_point(3), angles(3), link_lengths(3), point_zero(3), point_one(3), point_two(3) { 

			setManipulator( manp_base_point, manp_angles, manp_link_lengths );		
		}
		
		void setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths );
		void getManipulator( std::vector<float>& manp_base_point, std::vector<float>& manp_angles, std::vector<float>& manp_link_lengths );
		void getPoint ( float link_length, std::vector<float> stationary, std::vector<float> moving, std::vector<float>& out );
		void transform(std::vector<float>& vect, float angle);
		void atransform(std::vector<float>& vect, float angle);		
		void FABRIK( int iter, std::vector<float> dest );
		bool checkDestination ( std::vector<float> joint_positions );
		void controlSynth (std::vector<float> pv_joint_positions, std::vector<float> joint_positions, std::vector<float>& control );
};

int initialize_globals(){  
	base_point[0] = 0;
	base_point[1] = 0;
	base_point[2] = 0;

	link_lengths[0] = 0;
	link_lengths[1] = 4;
	link_lengths[2] = 4;

	destination[0] = 4;
	destination[1] = 4;
	destination[2] = 4;
}

void joints_callback( const sensor_msgs::JointState::ConstPtr& inp_msg ) {
	std::vector<std::string>::const_iterator it1;

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"world_to_base");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "world_to_base not found \n");
	}
	else {
		joint_positions[0] = inp_msg->position[it1 - inp_msg->name.begin()];
                
		joint_efforts[0] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link1_to_link2");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "link1_to_link2 not found \n");
	}
	else {
		joint_positions[1] = inp_msg->position[it1 - inp_msg->name.begin()];
                joint_efforts[1] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}

	it1 = find(inp_msg->name.begin(),inp_msg->name.end(),"link2_to_link3");
	if (it1==inp_msg->name.end()) {
		fprintf(stderr, "link2_to_link3 not found \n");
	}
	else {
		joint_positions[2] = inp_msg->position[it1 - inp_msg->name.begin()];
                joint_efforts[2] = inp_msg->effort[it1 - inp_msg->name.begin()];
	}
	fprintf(stderr, "joint_callback called \n");

}


int main( int argc, char **argv ) {
	std::vector<float> control(3);
	std::vector<float> pv_joint_positions(3);

	initialize_globals();	
	
	ros::Publisher wtb_pub, l1tl2_pub, l2tl3_pub;

	ros::init(argc, argv, "manp_controller");

	ros::NodeHandle n;
	wtb_pub = n.advertise<std_msgs::Float64>("/manp/world_to_base_state_controller/command", 50);
	l1tl2_pub = n.advertise<std_msgs::Float64>("/manp/link1_to_link2_state_controller/command", 50);
	l2tl3_pub = n.advertise<std_msgs::Float64>("/manp/link2_to_link3_state_controller/command", 50);

	ros::Subscriber sub_coordinates = n.subscribe("/manp/joint_states", 50, joints_callback);
	ros::Rate r(50);
	ros::spinOnce();

	Manipulator model(base_point, joint_positions, link_lengths);
	pv_joint_positions = joint_positions;

	fprintf(stderr, "------------Constructor-------------- \n");
	fprintf(stderr, "point_two[0] %lf\n", model.point_two[0]);
	fprintf(stderr, "point_two[1] %lf\n", model.point_two[1]);
	fprintf(stderr, "point_two[2] %lf\n", model.point_two[2]);
	fprintf(stderr, "point_one[0] %lf\n", model.point_one[0]);
	fprintf(stderr, "point_one[1] %lf\n", model.point_one[1]);
	fprintf(stderr, "point_one[2] %lf\n", model.point_one[2]);
	fprintf(stderr, "point_zero[0] %lf\n", model.point_zero[0]);
	fprintf(stderr, "point_zero[1] %lf\n", model.point_zero[1]);
	fprintf(stderr, "point_zero[2] %lf\n", model.point_zero[2]);
	fprintf(stderr, "-------------------------------- \n");

	model.FABRIK(10, destination);

	fprintf(stderr, "------------FABRIK-------------- \n");
	fprintf(stderr, "point_two[0] %lf\n", model.point_two[0]);
	fprintf(stderr, "point_two[1] %lf\n", model.point_two[1]);
	fprintf(stderr, "point_two[2] %lf\n", model.point_two[2]);
	fprintf(stderr, "point_one[0] %lf\n", model.point_one[0]);
	fprintf(stderr, "point_one[1] %lf\n", model.point_one[1]);
	fprintf(stderr, "point_one[2] %lf\n", model.point_one[2]);
	fprintf(stderr, "point_zero[0] %lf\n", model.point_zero[0]);
	fprintf(stderr, "point_zero[1] %lf\n", model.point_zero[1]);
	fprintf(stderr, "point_zero[2] %lf\n", model.point_zero[2]);
	fprintf(stderr, "angles[0] %lf\n", model.angles[0]);
	fprintf(stderr, "angles[1] %lf\n", model.angles[1]);
	fprintf(stderr, "angles[2] %lf\n", model.angles[2]);
	fprintf(stderr, "-------------------------------- \n");

	r.sleep();
	while(ros::ok() & model.checkDestination(joint_positions) == false ) {
		
		ros::spinOnce();

		model.controlSynth(pv_joint_positions, joint_positions, control);		

		std_msgs::Float64 out_msg;
		out_msg.data = control[0];
		wtb_pub.publish(out_msg);

		out_msg.data = control[1];
		l1tl2_pub.publish(out_msg);

		out_msg.data = control[2];
		l2tl3_pub.publish(out_msg);

		pv_joint_positions = joint_positions;
    		
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
	base_point[2] = point_zero[2];

	//Setting world_to_base angle
	if (point_two[1] == 0 & point_two[0] == 0) {
		angles[0] = 0;
	}
	else {
		angles[0] = atan( (point_two[1] - point_zero[1])/(point_two[0] - point_zero[0]) );
	}
	
		
	//Setting link1_to_link2 angle
	angles[1] = acos( (point_one[2] - point_zero[2])/link_lengths[1]);

	//Setting link2_to_link3 angle
	angles[2] = acos( (point_two[2] - point_one[2])/link_lengths[2]);
}


void Manipulator::setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {

	base_point = manp_base_point;
	angles = manp_angles;
	link_lengths = manp_link_lengths;

	point_zero[0] = base_point[0];
	point_zero[1] = base_point[1];
	point_zero[2] = base_point[2];
	
	point_one[0] = base_point[0] + link_lengths[1]*cos(angles[0])*sin(angles[1]);
	point_one[1] = base_point[1] + link_lengths[1]*sin(angles[0])*sin(angles[1]);
	point_one[2] = base_point[2] + link_lengths[1]*cos(angles[1]);

	point_two[0] = point_one[0] + link_lengths[2]*cos(angles[0])*sin(angles[2]);	
	point_two[1] = point_one[1] + link_lengths[2]*sin(angles[0])*sin(angles[2]);
	point_two[2] = point_one[2] + link_lengths[2]*cos(angles[2]);
}

void Manipulator::getManipulator( std::vector<float>& manp_base_point, std::vector<float>& manp_angles, std::vector<float>& manp_link_lengths ) {
	manp_base_point = base_point;
	manp_angles = angles;
	manp_link_lengths = link_lengths;
}

void Manipulator::getPoint ( float link_length, std::vector<float> stationary, std::vector<float> moving, std::vector<float>& out ) {
	std::vector<float> vect(2);

	vect[0] = moving[0] - stationary[0];
	vect[1] = moving[2] - stationary[2];

	float k = pow(pow(vect[0],2) + pow(vect[1],2), 0.5);
	
	vect[0] /= k;
	vect[1] /= k;
	
	out[0] = stationary[0] + link_length*vect[0];
	out[2] = stationary[2] + link_length*vect[1];
}

void Manipulator::transform(std::vector<float>& vect, float angle) {
	double tmp_v0 = vect[0]*cos(angle) - vect[1]*sin(angle);
	vect[1] = vect[0]*sin(angle) + vect[1]*cos(angle);
        vect[0] = tmp_v0;
	vect[2] = vect[2];
}


void Manipulator::atransform(std::vector<float>& vect, float angle) {
	double tmp_v0 = vect[0]*cos(angle) + vect[1]*sin(angle);
	vect[1] = vect[1]*cos(angle) - vect[0]*sin(angle);
        vect[0] = tmp_v0;
	vect[2] = vect[2];
}

void Manipulator::FABRIK( int iter, std::vector<float> dest ) {
	std::vector<float> zero_pv(3), one_pv(3), two_pv(3);
	float rot_angle;
	float l1,l2;
	
	//Check for the starting position
	if (point_one[0] == 0 & point_two[1] == 0) {
		rot_angle = acos( dest[1]/pow( pow( dest[0], 2) + pow(dest[1],2), 0.5 )  );	
	}
	else {
		rot_angle = acos( (point_one[0]*dest[0] + point_one[1]*dest[1])/( link_lengths[1]*sin(angles[1])*pow( pow(dest[0],2) + pow(dest[1],2), 0.5 )) );
	}
	      
	//Rotating vectors to destination
	transform(point_one, rot_angle);
	transform(point_two, rot_angle);

	//Transforming to another coordinate system
	transform(point_one, rot_angle);
	transform(point_two, rot_angle);
	transform(dest, rot_angle);

	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE LOCAL SYSTEM OF COORDINATES
				
	for ( int i = 0; i < iter/2; i++ ) {
	//Staring reverse pass				
		zero_pv = point_zero;
		one_pv = point_one;			
		two_pv = point_two;

		//Moved point TWO to destination
		point_two[0] = dest[0];
		point_two[2] = dest[2];
			
		//Moved point ONE for the reverse pass
		getPoint( link_lengths[2], point_two, one_pv, point_one );
			
		//Moved point ZERO for the reverse pass
		getPoint( link_lengths[1], point_one, zero_pv, point_zero);
			
	//Starting direct pass
		one_pv = point_one;			
		two_pv = point_two;
				
		//Moved point point_zero to previous position
		point_zero = zero_pv;
				
		//Moved point point_one for the direct pass
		getPoint( link_lengths[1], point_zero, one_pv, point_one );
				
		//Moved point point_two for the direct pass
		getPoint( link_lengths[2], point_one, two_pv, point_two );
	}

	fprintf(stderr, "------------Local Solved-------- \n");
	fprintf(stderr, "point_two[0] %lf\n", point_two[0]);
	fprintf(stderr, "point_two[1] %lf\n", point_two[1]);
	fprintf(stderr, "point_two[2] %lf\n", point_two[2]);
	fprintf(stderr, "point_one[0] %lf\n", point_one[0]);
	fprintf(stderr, "point_one[1] %lf\n", point_one[1]);
	fprintf(stderr, "point_one[2] %lf\n", point_one[2]);
	fprintf(stderr, "point_zero[0] %lf\n", point_zero[0]);
	fprintf(stderr, "point_zero[1] %lf\n", point_zero[1]);
	fprintf(stderr, "point_zero[2] %lf\n", point_zero[2]);
	fprintf(stderr, "-------------------------------- \n");
	
	//Link check
	l1 = pow( pow( point_one[0] - point_zero[0] ,2) + pow( point_one[1] - point_zero[1] ,2) + pow( point_one[2] - point_zero[2] ,2) , 0.5 );
	l2 = pow( pow( point_two[0] - point_one[0] ,2) + pow( point_two[1] - point_one[1] ,2) + pow( point_two[2] - point_one[2] ,2) , 0.5 );

	if (fabs(l1 - link_lengths[1])>1e-3 || fabs(l2- link_lengths[2])>1e-3)
		fprintf(stderr, "links length error (local) \n");      

	//Transforming to basic coordinate system
	transform(point_one, rot_angle);
	transform(point_two, rot_angle);

	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE GLOBAL SYSTEM OF COORDINATES

	fprintf(stderr, "------------Global Solved-------- \n");
	fprintf(stderr, "point_two[0] %lf\n", point_two[0]);
	fprintf(stderr, "point_two[1] %lf\n", point_two[1]);
	fprintf(stderr, "point_two[2] %lf\n", point_two[2]);
	fprintf(stderr, "point_one[0] %lf\n", point_one[0]);
	fprintf(stderr, "point_one[1] %lf\n", point_one[1]);
	fprintf(stderr, "point_one[2] %lf\n", point_one[2]);
	fprintf(stderr, "point_zero[0] %lf\n", point_zero[0]);
	fprintf(stderr, "point_zero[1] %lf\n", point_zero[1]);
	fprintf(stderr, "point_zero[2] %lf\n", point_zero[2]);
	fprintf(stderr, "-------------------------------- \n");
	
	//Link check
	l1 = pow( pow( point_one[0] - point_zero[0] ,2) + pow( point_one[1] - point_zero[1] ,2) + pow( point_one[2] - point_zero[2] ,2) , 0.5 );
	l2 = pow( pow( point_two[0] - point_one[0] ,2) + pow( point_two[1] - point_one[1] ,2) + pow( point_two[2] - point_one[2] ,2) , 0.5 );

	if (fabs(l1 - link_lengths[1])>1e-3 || fabs(l2- link_lengths[2])>1e-3 )
		fprintf(stderr, "links length error (global) %lf %lf \n", l1, l2);

	updateManipulator();
}

bool Manipulator::checkDestination ( std::vector<float> joint_positions  ) {
	return (joint_positions == angles);
}

void Manipulator::controlSynth (std::vector<float> pv_joint_positions, std::vector<float> joint_positions, std::vector<float>& control ) {
	float p = 2;
	std::vector<float> stat_moments(2);
	control[0] = p*(angles[0] - joint_positions[0]);
	for (int i = 1; i < 3; i++) {
		stat_moments[i-1] = 3200*sin(joint_positions[i]);
		control[i] = p*(angles[i] - joint_positions[i]) + stat_moments[i-1];
	}
	fprintf(stderr, "-------------------------------- \n");
	//fprintf(stderr, "control 0 %lf\n", control[0]);
	//fprintf(stderr, "control 1 %lf\n", control[1]);
	fprintf(stderr, "angles 2: %lf\n", angles[2]);
	fprintf(stderr, "joint 2: %e\n", joint_positions[2]);
	fprintf(stderr, "stat_moment 2: %e\n", stat_moments[1]);
	fprintf(stderr, "control 2: %lf\n", control[2]);
	fprintf(stderr, "-------------------------------- \n");
}
