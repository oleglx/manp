#include <vector>
#include <algorithm>
#include <cmath>
#include <cv.h>
#include <highgui.h>

#define w 800
using namespace cv;
using std::vector;


void getLine (float link_length, Point2f stationary, Point2f moving, float& k, float& b);
void choosePoint (Point2f inter_one, Point2f inter_two, Point2f moving, Point2f& out );
int solvePol (float A, float B, float C, std::vector<float>& solution);
int getInterPoint ( float link_length, Point2f one, Point2f two, Point2f& out );
void printImg(char* graph_window, Mat graph_image, Point2f zero, Point2f one, Point2f two );

int getPoints_test();
int choosePoint_test();
int getLine_test();
int solvePol_test();

class Manipulator {
	private:
		std::vector<float> base_point, angles, link_lengths;
	public:
		Manipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			setManipulator( manp_base_point, manp_angles, manp_link_lengths );		
		}
		
		void setManipulator( std::vector<float> manp_base_point, std::vector<float> manp_angles, std::vector<float> manp_link_lengths ) {
			base_point = manp_base_point;
			angles = manp_angles;
			link_lengths = manp_link_lengths;
		}

		void getPoints( Point2f& point_zero, Point2f& point_one, Point2f& point_two ) {	
			point_zero.x = base_point[0];
			point_zero.y = base_point[1];

			point_one.x = base_point[0] + link_lengths[0]*cos(angles[0]);
			point_one.y = base_point[1] + link_lengths[0]*sin(angles[0]);
			
			point_two.x = point_one.x + link_lengths[1]*cos(angles[1]);
			point_two.y = point_one.y + link_lengths[1]*sin(angles[1]);
		}
		
		void FABRIK(char* graph_window, int iter, Point2f zero, Point2f one, Point2f two, Point2f dest, Mat graph_image ) {
			Point2f zero_pv, one_pv, two_pv;
								
			for ( int i = 0; i < iter/2; i++ ) {
			//Staring reverse pass				
				zero_pv = zero;
				one_pv = one;			
				two_pv = two;

				//Moved point TWO to destination
				two = dest;

				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
			
				//Moved point ONE for the reverse pass
				getInterPoint( link_lengths[1], two, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
				
				//Moved point ZERO for the reverse pass
				getInterPoint( link_lengths[0], one, zero_pv, zero);
				
				//Printing this step
				printImg(graph_window, graph_image, zero, one, two );
						
			//Starting direct pass
				one_pv = one;			
				two_pv = two;
				
				//Moved point zero to previous position
				zero = zero_pv;

				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
				
				//Moved point one for the direct pass
				getInterPoint( link_lengths[0], zero, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
				
				//Moved point two for the direct pass
				getInterPoint( link_lengths[1], one, two_pv, two );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
			}

		}


};


int main( void ) {

	//--------Testing-----------

        if(getPoints_test()!=0)
        	return -1;

	if(getLine_test()!=0)
		return -1;

	if(choosePoint_test()!=0)
		return -1;	
	
	if(solvePol_test()!=0)
		return -1;

	std::vector<float> manp_base_point(2);
	manp_base_point[0] = w/2;
	manp_base_point[1] = w/2;

	std::vector<float> manp_angles(2);
	manp_angles[0] = M_PI/2;
	manp_angles[1] = M_PI/2;

	std::vector<float> manp_link_lengths(2);
	manp_link_lengths[0] = w/5;
	manp_link_lengths[1] = w/5;

	char graph_window[] = "Graph FABRIK";
	Mat graph_image = Mat(Size(w,w),CV_8UC3,Scalar(255,255,255));
	
	Manipulator manp_1( manp_base_point, manp_angles, manp_link_lengths);

	Point2f zero;
	Point2f one;
	Point2f two;
	
	manp_1.getPoints(zero, one, two);

	Point2f dest;
	dest.x = 3*w/4;
	dest.y = 3*w/4;

	manp_1.FABRIK( graph_window, 2, zero, one, two, dest, graph_image );

	imshow(graph_window,graph_image);
	moveWindow(graph_window, 0, 200 );
	waitKey(0);
	return(0);
	
}

//--------------------------------------------------------------------------------------------------

void getLine (Point2f stationary, Point2f moving, float& k, float& b) {	
	k = (stationary.y - moving.y)/(stationary.x - moving.x);
	b = (moving.y - k * moving.x);
	// Check extreme positions
}

void choosePoint (Point2f inter_one, Point2f inter_two, Point2f moving, Point2f& out ) {
	float eps_one, eps_two;

	eps_one = pow( pow(inter_one.x - moving.x, 2 ) + pow(inter_one.y - moving.y, 2 ), 0.5 );
	eps_two = pow( pow(inter_two.x - moving.x, 2 ) + pow(inter_two.y - moving.y, 2 ), 0.5 );
	
	if ( eps_one > eps_two )
		out = inter_two;
	else out = inter_one;

}

int solvePol (float A, float B, float C, std::vector<float>& solution) {
	float desc;

	desc = pow( B, 2 ) - 4 * A * C; 
	if (desc < 0) {
		fprintf( stderr, "Desc < 0 %lf\n", desc );
		return -1;
	}
	solution[0] = (-B + pow( desc, 0.5 ) )/(2*A);
	solution[1] = (-B - pow( desc, 0.5 ) )/(2*A);
	return 0;
}

int getInterPoint ( float link_length, Point2f stationary, Point2f moving, Point2f& out ) {
	float k, b, A, B, C;
	std::vector<float> solution(2);
	Point2f inter_one, inter_two;

	getLine( stationary, moving, k, b );

	A = pow( k, 2 ) + 1;
	B = 2*b*k - 2*k*stationary.y - 2*stationary.x;
	C = pow( b, 2 ) + 2*b*stationary.y + pow( stationary.x, 2 ) + pow( stationary.y, 2) - pow( link_length, 2 );
	
	solvePol( A, B, C, solution);

	inter_one.x = solution[0];
	inter_two.x = solution[1];

	inter_one.y = k*inter_one.x + b;
	inter_two.y = k*inter_two.x + b;

	choosePoint (inter_one, inter_two, moving, out);

	return 0;

}

void printImg(char* graph_window, Mat graph_image, Point2f zero, Point2f one, Point2f two ) {
	
	graph_image.setTo(Scalar(255,255,255));

	line( graph_image, zero, one, Scalar( 0, 0, 0 ), 2, 8 );
	line( graph_image, one, two, Scalar( 0, 0, 0 ), 2, 8 );

	circle( graph_image, zero, w/200, Scalar( 255, 0, 0 ), -1, 8 );
	circle( graph_image, one, w/200, Scalar( 0, 0, 255 ), -1, 8 );
	circle( graph_image, two, w/200, Scalar( 0, 0, 255 ), -1, 8 );

	imshow(graph_window,graph_image);
	moveWindow(graph_window, 0, 200 );
	waitKey(0);
}

//------------------------------------Unit Tests--------------------------------------

int getPoints_test() { 
	vector<float> base_point(2), angles(2), link_lengths(2);

	Point2f zero, one, two;

        base_point[0] = 3;
        base_point[1] = 3;
        angles[0] = M_PI/4;
        angles[1] = M_PI/6;
        link_lengths[0] = 1;
        link_lengths[1] = 3;

	Manipulator test( base_point, angles, link_lengths );
        test.getPoints(zero, one, two);

        float eps = 1e-6;

        if ( fabs ( zero.x-3 ) > eps ) {
        	fprintf( stderr, "Wrong zero.x %lf\n", zero.x );
          	return -1;
        }

        if ( fabs ( zero.y-3 ) > eps ) {
        	fprintf( stderr, "Wrong zero.y %lf\n", zero.y );
        	return -1;
        }
	
	if ( fabs ( one.x-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong one.x %lf\n", one.x );
        	return -1;
        }
	
	if ( fabs ( one.y-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong one.y %lf\n", one.y );
        	return -1;
        }

	if ( fabs ( two.x-6.305182992539864 ) > eps ) {
        	fprintf( stderr, "Wrong two.x %lf\n", two.x );
        	return -1;
        }

	if ( fabs ( two.y-5.207106781186548 ) > eps ) {
        	fprintf( stderr, "Wrong two.y %lf\n", two.y );
        	return -1;
        }

        return 0;
}	

int getLine_test() {
	float k,b;
	Point2f stationary, moving;

	stationary.x = 47;
	stationary.y = -65;
	moving.x = 13;
	moving.y = 80;
	
	getLine( stationary, moving, k, b);

	float eps = 1e-6;

	if ( fabs ( k + 4.2647058824 ) > eps ){
		fprintf( stderr, "Wrong k %lf\n", k);
		return -1;	
	}

	if ( fabs ( b - 135.4411764706 ) > eps ){
		fprintf( stderr, "Wrong b %lf\n", b);
		return -1;	
	}
	return 0;
}

int choosePoint_test() {
	Point2f one, two, moving, out;

	one.x = 76;
	one.y = -8;

	two.x = 0;
	two.y = -52;

	moving.x = 12;
	moving.y = 1;

	choosePoint(one, two, moving, out);
	
	if (out == one) {
		fprintf( stderr, "Wrong point chosen: One.\n");
		return -1;
	}
	return 0;
	
}

int solvePol_test() {
	float A = 3, B = -8, C = 2;
	std::vector<float> solution(2);
	
	solvePol( A, B, C, solution );
	
	float eps = 1e-6;

	if ( fabs ( solution[0] - 2.3874258867 ) > eps ){
		fprintf( stderr, "Wrong x1 %lf\n", solution[0]);
		return -1;	
	}
	
	if ( fabs ( solution[1] - 0.27924078 ) > eps ){
		fprintf( stderr, "Wrong x2 %lf\n", solution[1]);
		return -1;	
	}

	return 0;
}
