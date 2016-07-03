#include<cstdio>
#include<ctime>
#include<cmath> 

#include<ros/ros.h>
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>
#include<sensor_msgs/JointState.h>

#define PI 3.1415926535

//FILE* f;

/*Configuration arrays consist of:
	[0]-base point coordinates;
	[1]-link lengths
	[2]-joint angles
*/

std::vector<std::vector<float> > start_configuration(3, std::vector<float> (3));
std::vector<float> destination(3);
std::vector<float> joint_positions(3), joint_efforts(3);

void transformVector(std::vector<float>& vect, float angle);

//Unit tests
int unitTestsBody();
int getManipulatorConfiguration_test(std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration);
int updateManipulator_test (std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration);
int manipulatorLengths_test (std::vector<std::vector<float> > out_points, std::vector<std::vector<float> > out_configuration);
//int getPoint_test();


class Manipulator {
	private:
		std::vector<std::vector<float> > configuration;
		std::vector<std::vector<float> > points;		
	public:		
		Manipulator( std::vector<std::vector<float> > manp_configuration ) : configuration(3, std::vector<float> (3)), points(3, std::vector<float> (3)) {
			setManipulator( manp_configuration );		
		}
		//Working as planned
		void setManipulator( std::vector<std::vector<float> > manp_configuration );
		//Working as planned
		void getManipulatorPoints( std::vector<std::vector<float> >& out_points );
		//Working as planned
		void getManipulatorConfiguration( std::vector<std::vector<float> >& out_configuration );
		//Working as planned		
		void updateManipulator();
		//Working as planned
		void getRotAngle(std::vector<float> dest, float& rot_angle );
		//Working as planned
		void transformManipulator(float angle);
		//
		int getPoint ( float link_length, std::vector<float> stationary, std::vector<float> reference, std::vector<float>& moving );
		//void FABRIK( int iter, std::vector<float> dest );
		//bool checkDestination ( std::vector<float> joint_positions );
		//void controlSynth (std::vector<float> pv_joint_positions, std::vector<float> joint_positions, std::vector<float>& control );
};

int initialize_globals(){  
	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;

	start_configuration[1][0] = 0;
	start_configuration[1][1] = 4;
	start_configuration[1][2] = 4;

	start_configuration[2][0] = 0;
	start_configuration[2][1] = 0;
	start_configuration[2][2] = 1;

	destination[0] = 2;
	destination[1] = 1;
	destination[2] = 6;
}

//Working as planned
/*void joints_callback( const sensor_msgs::JointState::ConstPtr& inp_msg ) {
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
}*/

//----------------------------------------------------------
//-----------------MAIN FUNCTION----------------------------
//----------------------------------------------------------

int main( int argc, char **argv ) {

	unitTestsBody();
	//getPoint_test();

	/*f = fopen("/home/pal/catkin_ws/logfile", "wt");
	if (f == 0) {
		fprintf(stderr, "File not found \n");
	}*/
	
	//initialize_globals();	
	
	/*ros::Publisher wtb_pub, l1tl2_pub, l2tl3_pub;

	ros::init(argc, argv, "manp_controller");

	ros::NodeHandle n;
	wtb_pub = n.advertise<std_msgs::Float64>("/manp/world_to_base_state_controller/command", 100);
	l1tl2_pub = n.advertise<std_msgs::Float64>("/manp/link1_to_link2_state_controller/command", 100);
	l2tl3_pub = n.advertise<std_msgs::Float64>("/manp/link2_to_link3_state_controller/command", 100);

	ros::Subscriber sub_coordinates = n.subscribe("/manp/joint_states", 100, joints_callback);
	ros::Rate r(100);
	ros::spinOnce();*/
	
	//Manipulator model(base_point, start_angles, link_lengths);
	//pv_joint_positions = joint_positions;

	/*fprintf(stderr, "------------Constructor-------------- \n");
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

	model.getAngles( test_angles );

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
	fprintf(stderr, "angles[0] %lf\n", test_angles[0]);
	fprintf(stderr, "angles[1] %lf\n", test_angles[1]);
	fprintf(stderr, "angles[2] %lf\n", test_angles[2]);
	fprintf(stderr, "-------------------------------- \n");

	r.sleep();
	while(ros::ok() & model.checkDestination(joint_positions) == false ) {
		
		ros::spinOnce();

		model.controlSynth(pv_joint_positions, joint_positions, control);		

		std_msgs::Float64 out_msg;
		//out_msg.data = control[0];
		//wtb_pub.publish(out_msg);

		out_msg.data = control[1];
		l1tl2_pub.publish(out_msg);

		out_msg.data = control[2];
		l2tl3_pub.publish(out_msg);

		pv_joint_positions = joint_positions;
    		
    		r.sleep();
  	}
	fclose(f);*/
  	return 0;
}

//----------------------------------------------------------
//-----------------Defining functions-----------------------
//----------------------------------------------------------

void Manipulator::setManipulator( std::vector<std::vector<float> > manp_configuration ) {

	configuration = manp_configuration;

	points[0] = configuration[0]; 
	
	points[1][0] = points[0][0] + configuration[1][1]*cos(configuration[2][0])*sin(configuration[2][1]);
	points[1][1] = points[0][1] + configuration[1][1]*sin(configuration[2][0])*sin(configuration[2][1]);
	points[1][2] = points[0][2] + configuration[1][1]*cos(configuration[2][1]);

	points[2][0] = points[1][0] + configuration[1][2]*cos(configuration[2][0])*sin(configuration[2][2]+configuration[2][1]);
	points[2][1] = points[1][1] + configuration[1][2]*sin(configuration[2][0])*sin(configuration[2][2]+configuration[2][1]);
	points[2][2] = points[1][2] + configuration[1][2]*cos(configuration[2][2]+configuration[2][1]);
}

void Manipulator::getManipulatorPoints( std::vector<std::vector<float> >& out_points ) {
	out_points = points;
}

void Manipulator::getManipulatorConfiguration( std::vector<std::vector<float> >& out_configuration ) {
	out_configuration = configuration;
}

void Manipulator::updateManipulator() {
	int k;

	configuration[0] = points[0];

	//Setting world_to_base angle
	if (points[2][1] == 0 & points[2][0] == 0) {
		configuration[2][0] = 0;
	}
	else {
		configuration[2][0] = atan2((points[2][1] - points[0][1]),(points[2][0] - points[0][0]));
	}
	
	//Setting link1_to_link2 angle

	if ( configuration[2][0] > 0 ) {

		if ( (points[1][0] > 0 & points[1][1] < 0) || (points[1][0] < 0 & points[1][1] < 0) || (points[1][0] == 0 & points[1][1] < 0) )
			k = -1;
		else k = 1;
	}
	else
	{
		if ( (points[1][0] > 0 & points[1][1] > 0) || (points[1][0] < 0 & points[1][1] > 0) || (points[1][0] == 0 & points[1][1] > 0) )
			k = -1;
		else k = 1;
	}	
	configuration[2][1] = atan2( k*pow(pow(points[1][0]-points[0][0],2) + pow(points[1][1]-points[0][1],2), 0.5), (points[1][2] - points[0][2]));

	//Setting link2_to_link3 angle

	if ( configuration[2][0] > 0 ) {

		if ( (points[2][0] > 0 & points[2][1] < 0) || (points[2][0] < 0 & points[2][1] < 0) || (points[2][0] == 0 & points[2][1] < 0) )
			k = -1;
		else k = 1;
	}
	else
	{
		if ( (points[2][0] > 0 & points[2][1] > 0) || (points[2][0] < 0 & points[2][1] > 0) || (points[2][0] == 0 & points[2][1] > 0) )
			k = -1;
		else k = 1;
	}
	configuration[2][2] = atan2( k*pow(pow(points[2][0]-points[1][0],2) + pow(points[2][1]-points[1][1],2), 0.5) ,(points[2][2] - points[1][2])) - configuration[2][1];
}

void Manipulator::getRotAngle(std::vector<float> dest, float& rot_angle) {
	int k;

	if (points[2][0] == 0 & points[2][1] == 0) {
		if ( (dest[1] - dest[0]) >= 0 )
			k = 1;
		else k = -1;
		rot_angle = k*acos( dest[0]/ (pow( pow( dest[0], 2) + pow(dest[1],2), 0.5 ) ) );
	}
	
	else
	{
		if ( (points[2][0]*dest[1] - points[2][1]*dest[0]) >= 0 )
			k = 1;
		else k = -1;

		rot_angle = k*acos( (points[2][0]*dest[0] + points[2][1]*dest[1])/ ( (configuration[1][1]*sin(configuration[2][1]) + configuration[1][2]*sin(configuration[2][2]+configuration[2][1]))*pow( pow(dest[0],2) + pow(dest[1],2), 0.5 )) );
	}

}

void Manipulator::transformManipulator(float angle) {
	
	double tmp = points[0][0]*cos(angle) - points[0][1]*sin(angle);
	points[0][1] = points[0][0]*sin(angle) + points[0][1]*cos(angle);
        points[0][0] = tmp;
	points[0][2] = points[0][2];

	tmp = points[1][0]*cos(angle) - points[1][1]*sin(angle);
	points[1][1] = points[1][0]*sin(angle) + points[1][1]*cos(angle);
        points[1][0] = tmp;
	points[1][2] = points[1][2];

	tmp = points[2][0]*cos(angle) - points[2][1]*sin(angle);
	points[2][1] = points[2][0]*sin(angle) + points[2][1]*cos(angle);
        points[2][0] = tmp;
	points[2][2] = points[2][2];
}

void transformVector(std::vector<float>& vect, float angle) {

	double tmp = vect[0]*cos(angle) - vect[1]*sin(angle);
	vect[1] = vect[0]*sin(angle) + vect[1]*cos(angle);
        vect[0] = tmp;
	vect[2] = vect[2];
}


int Manipulator::getPoint ( float link_length, std::vector<float> stationary, std::vector<float> reference, std::vector<float>& moving ) {
	std::vector<float> vect(2);
	int i = -1; //Invoking segmentation fault

	if ((points[0][0] == 0 & points[1][0] == 0 & points[2][0] == 0))
		i = 1;
	if ((points[0][1] == 0 & points[1][1] == 0 & points[2][1] == 0))
		i = 0;
	if (i == -1)
		return -1;


	vect[0] = reference[i] - stationary[i];
	vect[1] = reference[2] - stationary[2];

	float k = pow(pow(vect[0],2) + pow(vect[1],2), 0.5);
	if ( k >= 1e-6 ) {
  		vect[0] /= k;
		vect[1] /= k;
	
		moving[i] = stationary[i] + link_length*vect[0];
		moving[2] = stationary[2] + link_length*vect[1];
        }
	else
	{
        	moving[i] = stationary[i];
        	moving[2] = stationary[2];
        }
	return 0;
}
/*
void Manipulator::FABRIK( int iter, std::vector<float> dest ) {
	std::vector<float> zero_pv(3), one_pv(3), two_pv(3);
	float rot_angle;
	float l1,l2;
	
	//Check for the starting position
	void getRotAngle( dest, rot_angle );	
	fprintf(stderr, "rot_angle %lf\n", rot_angle);
	      
	//Rotating vectors to destination
	transform(point_one, rot_angle);
	transform(point_two, rot_angle);

	//Transforming to another coordinate system
	transform(point_one, rot_angle);
	transform(point_two, rot_angle);
	transform(dest, rot_angle);
	
	//----------------------------------------------------LOCAL CHECK--------------------------------------------------------------
	fprintf(stderr, "------------Local-------------- \n");
	fprintf(stderr, "point_two[0] %lf\n", point_two[0]);
	fprintf(stderr, "point_two[1] %lf\n", point_two[1]);
	fprintf(stderr, "point_two[2] %lf\n", point_two[2]);
	fprintf(stderr, "point_one[0] %lf\n", point_one[0]);
	fprintf(stderr, "point_one[1] %lf\n", point_one[1]);
	fprintf(stderr, "point_one[2] %lf\n", point_one[2]);
	fprintf(stderr, "point_zero[0] %lf\n", point_zero[0]);
	fprintf(stderr, "point_zero[1] %lf\n", point_zero[1]);
	fprintf(stderr, "point_zero[2] %lf\n", point_zero[2]);
	fprintf(stderr, "destination[0] %lf\n", dest[0]);
	fprintf(stderr, "destination[1] %lf\n", dest[1]);
	fprintf(stderr, "destination[2] %lf\n", dest[2]);
	fprintf(stderr, "-------------------------------- \n"); 
	
	//----------------------------------------------------LOCAL LINK CHECK------------------------------------------------------------------
	l1 = pow( pow( point_one[0] - point_zero[0] ,2) + pow( point_one[1] - point_zero[1] ,2) + pow( point_one[2] - point_zero[2] ,2) , 0.5 );
	l2 = pow( pow( point_two[0] - point_one[0] ,2) + pow( point_two[1] - point_one[1] ,2) + pow( point_two[2] - point_one[2] ,2) , 0.5 );

	if (fabs(l1 - link_lengths[1])>1e-3 || fabs(l2- link_lengths[2])>1e-3)
		fprintf(stderr, "links length error (local before)  %lf %lf \n", l1, l2); 
     
	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE LOCAL SYSTEM OF COORDINATES
				
	for ( int i = 0; i < iter/2; i++ ) {
	//Staring reverse pass				
		zero_pv = point_zero;
		one_pv = point_one;			
		two_pv = point_two;

		//Moved point TWO to destination
		point_two = dest;
			
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

	//----------------------------------------------------LOCAL SOLUTION CHECK--------------------------------------------------------------
	fprintf(stderr, "------------Local Solution------ \n");
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
	
	//----------------------------------------------------LOCAL LINK CHECK------------------------------------------------------------------
	l1 = pow( pow( point_one[0] - point_zero[0] ,2) + pow( point_one[1] - point_zero[1] ,2) + pow( point_one[2] - point_zero[2] ,2) , 0.5 );
	l2 = pow( pow( point_two[0] - point_one[0] ,2) + pow( point_two[1] - point_one[1] ,2) + pow( point_two[2] - point_one[2] ,2) , 0.5 );

	if (fabs(l1 - link_lengths[1])>1e-3 || fabs(l2- link_lengths[2])>1e-3)
		fprintf(stderr, "links length error (local after)  %lf %lf \n", l1, l2);      

	//Transforming to basic coordinate system
	atransform(point_one, rot_angle);
	atransform(point_two, rot_angle);

	//----------------ATTENTION-------------------------				
	//SOLUTION NOW IS IN THE GLOBAL SYSTEM OF COORDINATES
	
	//---------------------------------------------------GLOBAL LINK CHECK------------------------------------------------------------------
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
	float p = 1000;
	std::vector<float> stat_moments(2);
	//control[0] = p*(angles[0] - joint_positions[0]);
	stat_moments[0] = -3400*sin(joint_positions[1]);
	stat_moments[1] = -3400*sin(joint_positions[1]+joint_positions[2]);
	control[1] = p*(angles[1] - joint_positions[1]) + stat_moments[0];
	control[2] = p*(angles[2] - joint_positions[2]-joint_positions[1]) + stat_moments[1];
	
	//fprintf(stderr, "-------------------------------- \n");
	//fprintf(stderr, "control 0 %lf\n", control[0]);
	//fprintf(stderr, "control 1 %lf\n", control[1]);
	fprintf(f, "%lf;", joint_positions[2]);
	fprintf(f, "%lf;", joint_positions[1]);
	fprintf(f, "%lf;", joint_positions[0]);
	fprintf(f, "%lf;", angles[2]);
	fprintf(f, "%lf;", angles[1]);
	fprintf(f, "%lf;", angles[0]);
	fprintf(f, "%lf;\n", control[2]);
	//fprintf(stderr, "-------------------------------- \n");
}
*/
//----------------------------------------------------------
//---------------------Unit tests---------------------------
//----------------------------------------------------------


int unitTestsBody() {
	std::vector<std::vector<float> > start_configuration(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_points(3, std::vector<float> (3));
	std::vector<std::vector<float> > out_configuration(3, std::vector<float> (3));

	std::vector<float> destination(3);
	float rot_angle;
	
	//------------------
	//Constructing test1
	//------------------

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 0;
	start_configuration[1][1] = 1;
	start_configuration[1][2] = 1;	
	
	start_configuration[2][0] = PI/2;
	start_configuration[2][1] = PI/4;
	start_configuration[2][2] = PI/3;
	
	Manipulator test1( start_configuration );

	test1.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 0.7071067812) > 1e-6 || fabs(out_points[1][2] - 0.7071067812) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0)  > 1e-6 || fabs(out_points[2][1] - 1.6730326075) > 1e-6 || fabs(out_points[2][2] - 0.4482877361) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}

	//Testing getManipulatorConfiguration
	test1.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing updateManipulator
	
	test1.updateManipulator();
	test1.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 3;
	destination[1] = 0;
	destination[2] = 13;

	test1.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle + PI/2)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test1.transformManipulator(rot_angle);
	test1.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test1.transformManipulator(rot_angle);
	test1.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}
		
	fprintf(stderr, "Test1 passed the test \n");
	
	//------------------
	//Constructing test2
	//------------------

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 2;
	start_configuration[1][1] = 3;
	start_configuration[1][2] = 4;	
	
	start_configuration[2][0] = PI/2;
	start_configuration[2][1] = PI/4;
	start_configuration[2][2] = PI/3;
	
	Manipulator test2( start_configuration );

	test2.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 2.1213203436) > 1e-6 || fabs(out_points[1][2] - 2.1213203436) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0)  > 1e-6 || fabs(out_points[2][1] - 5.9850236487) > 1e-6 || fabs(out_points[2][2] - 1.0860441631) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}


	//Testing getManipulatorConfiguration
	test2.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing updateManipulator
	test2.updateManipulator();
	test2.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 4;
	destination[1] = 4;
	destination[2] = 0;

	test2.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle + PI/4)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test2.transformManipulator(rot_angle);
	test2.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test2.transformManipulator(rot_angle);
	test2.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}
	
	fprintf(stderr, "Test2 passed the test \n");
	
	//------------------
	//Constructing test3
	//------------------	

	start_configuration[0][0] = 0;
	start_configuration[0][1] = 0;
	start_configuration[0][2] = 0;	
	
	start_configuration[1][0] = 0;
	start_configuration[1][1] = 1;
	start_configuration[1][2] = 1;	
	
	start_configuration[2][0] = 0;
	start_configuration[2][1] = 0;
	start_configuration[2][2] = 5*PI/6;
	
	Manipulator test3( start_configuration );

	test3.getManipulatorPoints( out_points );

	//Testing setManipulator&getManipulatorPoints
	if (fabs(out_points[0][0] - 0)  > 1e-6 || fabs(out_points[0][1] - 0) > 1e-6 || fabs(out_points[0][2] - 0) > 1e-6 ) {
		fprintf(stderr, "Wrong points[0]: %e %e %e \n", out_points[0][0], out_points[0][1], out_points[0][2]);
		return -1;
	}
	
	if (fabs(out_points[1][0] - 0) > 1e-6 || fabs(out_points[1][1] - 0) > 1e-6 || fabs(out_points[1][2] - 1) > 1e-6 ) {
		fprintf(stderr, "Wrong points[1]: %e %e %e \n", out_points[1][0], out_points[1][1], out_points[1][2]);
		return -1;
	}
	
	if (fabs(out_points[2][0] - 0.5)  > 1e-6 || fabs(out_points[2][1] - 0) > 1e-6 || fabs(out_points[2][2] - 0.1339745962) > 1e-6 ) {
		fprintf(stderr, "Wrong points[2]: %e %e %e \n", out_points[2][0], out_points[2][1], out_points[2][2]);
		return -1;
	}

	//Testing getManipulatorConfiguration
	test3.getManipulatorConfiguration( out_configuration );	
	if( getManipulatorConfiguration_test(out_configuration, start_configuration) == -1)
		return -1;
	
	//Testing updateManipulator
	test3.updateManipulator();
	test3.getManipulatorConfiguration( out_configuration );
	if( updateManipulator_test(out_configuration, start_configuration) == -1)
		return -1;

	//Testing getRotAngle
	
	destination[0] = 1;
	destination[1] = 1;
	destination[2] = 6;

	test3.getRotAngle( destination, rot_angle);
	
	if (fabs(rot_angle - PI/4)  > 1e-6 ) {
		fprintf(stderr, "Wrong rot_angle: %e \n", rot_angle );
		return -1;
	}

	//Testing transform
	test3.transformManipulator(rot_angle);
	test3.getManipulatorPoints(out_points);
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;
	transformVector(destination, rot_angle);
	test3.transformManipulator(rot_angle);
	test3.getManipulatorPoints(out_points);	
	if (manipulatorLengths_test( out_points, out_configuration ) == -1) //Testing lengths
		return -1;	

	if ( (out_points[0][0] != 0 & out_points[1][0] != 0 & out_points[2][0] != 0) & (out_points[0][1] != 0 & out_points[1][1] != 0 & out_points[2][1] != 0) ) {
		fprintf(stderr, "Wrong rotation. Points x's %e %e %e \n", out_points[0][0], out_points[1][0], out_points[2][0]);
		fprintf(stderr, "Points y's %e %e %e \n", out_points[0][1], out_points[1][1], out_points[2][1]);
		return -1;
	}

	fprintf(stderr, "Test3 passed the test \n");

	return 0;
}

int getManipulatorConfiguration_test(std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration) {

	if (fabs(out_configuration[0][0] - start_configuration[0][0]) > 1e-6 || fabs(out_configuration[0][1] - start_configuration[0][1]) > 1e-6 || fabs(out_configuration[0][2] - start_configuration[0][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[0]: %e %e %e \n", out_configuration[0][0], out_configuration[0][1], out_configuration[0][2]);
		return -1;
	}
	
	if (fabs(out_configuration[1][0] - start_configuration[1][0]) > 1e-6 || fabs(out_configuration[1][1] - start_configuration[1][1]) > 1e-6 || fabs(out_configuration[1][2] - start_configuration[1][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[1]: %e %e %e \n", out_configuration[1][0], out_configuration[1][1], out_configuration[1][2]);
		return -1;
	}	

	if (fabs(out_configuration[2][0] - start_configuration[2][0]) > 1e-6 || fabs(out_configuration[2][1] - start_configuration[2][1])  > 1e-6 || fabs(out_configuration[2][2] - start_configuration[2][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2]: %e %e %e \n", out_configuration[2][0], out_configuration[2][1], out_configuration[2][2]);	
		return -1;
	}

}

int updateManipulator_test (std::vector<std::vector<float> > out_configuration, std::vector<std::vector<float> > start_configuration) { 

	if (fabs(out_configuration[2][0] - start_configuration[2][0]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][0]: %e \n", out_configuration[2][0]);
		return -1;
	}
	
	if (fabs(out_configuration[2][1] - start_configuration[2][1]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][1]: %e \n", out_configuration[2][1]);
		return -1;
	}	

	if (fabs(out_configuration[2][2] - start_configuration[2][2]) > 1e-6 ) {
		fprintf(stderr, "Wrong out_configuration[2][2]: %e \n", out_configuration[2][2]);
		return -1;
	}
}

int manipulatorLengths_test (std::vector<std::vector<float> > out_points, std::vector<std::vector<float> > out_configuration) {
	float l1, l2;
	
	l1 = pow( pow( out_points[1][0] - out_points[0][0] ,2) + pow( out_points[1][1] - out_points[0][1] ,2) + pow( out_points[1][2] - out_points[0][2] ,2) , 0.5 );
	l2 = pow( pow( out_points[2][0] - out_points[1][0] ,2) + pow( out_points[2][1] - out_points[1][1] ,2) + pow( out_points[2][2] - out_points[1][2] ,2) , 0.5 );

	if (fabs(l1 - out_configuration[1][1])>1e-3 || fabs(l2- out_configuration[1][2])>1e-3 )
		fprintf(stderr, "links length error %e %e \n",l1, l2);

}
