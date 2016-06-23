#include <vector>
#include <algorithm>
#include <cmath>
#include <cv.h>
#include <highgui.h>

#define w 800
using namespace cv;
using std::vector;

int getPoint ( float link_length, Point one, Point two, Point& out ) {
	float k, b, desc, A, B, C, eps_one, eps_two;
	Point inter_one, inter_two;

	k = (one.y - two.y)/(one.x - two.x);
	b = (one.y - k * one.x);

	A = pow( k, 2 ) + 1;
	B = 2*b*k - 2*k*one.y - 2*one.x;
	C = pow( b, 2 ) + 2*b*one.y + pow( one.x, 2 ) + pow( one.y, 2) - pow( link_length, 2 );
 
	desc = pow( B, 2 ) - 4 * A * C; 
	if (desc < 0) {
		fprintf( stderr, "Desc < 0\n" );
		return -1;
	}
	inter_one.x = (-B + pow( desc, 0.5 ) )/(2*A);
	inter_two.x = (-B - pow( desc, 0.5 ) )/(2*A);

	inter_one.y = k*inter_one.x + b;
	inter_two.y = k*inter_two.x + b;

	eps_one = pow( pow(inter_one.x - two.x, 2 ) + pow(inter_one.y - two.y, 2 ), 0.5 );
	eps_two = pow( pow(inter_one.x - two.x, 2 ) + pow(inter_one.y - two.y, 2 ), 0.5 );
	
	if ( eps_one > eps_two )
		out = inter_two;
	else out = inter_one;

	return 0;

}

void printImg(char* graph_window, Mat graph_image, Point zero, Point one, Point two ) {
	
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

		void getPoints( Point& point_zero, Point& point_one, Point& point_two ) {	
			point_zero.x = base_point[0];
			point_zero.y = base_point[1];

			point_one.x = base_point[0] + link_lengths[0]*cos(angles[0]);
			point_one.y = base_point[1] + link_lengths[0]*sin(angles[0]);
			
			point_two.x = point_one.x + link_lengths[1]*cos(angles[1]);
			point_two.y = point_one.y + link_lengths[1]*sin(angles[1]);
		}
		
		void FABRIK(char* graph_window, int iter, Point zero, Point one, Point two, Point dest, Mat graph_image ) {
			Point zero_pv, one_pv, two_pv;
								
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
				getPoint( link_lengths[1], two, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
				
				//Moved point ZERO for the reverse pass
				getPoint( link_lengths[0], one, zero_pv, zero);
				
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
				getPoint( link_lengths[0], zero, one_pv, one );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
				
				//Moved point two for the direct pass
				getPoint( link_lengths[1], one, two_pv, two );
				
				//Prinring this step
				printImg(graph_window, graph_image, zero, one, two );
			}

		}


};
/*int Manipulator_test() { 
	vector<float> bp(2),an(2),ll(2);

        bp[0] = 3;
        bp[1] = 3;
        an[0] = M_PI/4;
        an[1] = M_PI/6;
        ll[0] = 1;
        ll[1] = 3;

	Manipulator m1( bp, an, ll );
        m1.convertCoord();

        float eps = 1e-6;

        if ( fabs ( m1.point_zero[0]-3 ) > eps ) {
        	fprintf( stderr, "Wrong point_zero[0]\n" );
          	return -1;
        }

        if ( fabs ( m1.point_zero[1]-3 ) > eps ) {
        	fprintf( stderr, "Wrong point_zero[1]\n" );
        	return -1;
        }
	
	if ( fabs ( m1.point_one[0]-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong point_one[0]\n" );
        	return -1;
        }
	
	if ( fabs ( m1.point_one[1]-3.7071067811865475 ) > eps ) {
        	fprintf( stderr, "Wrong point_one[1]\n" );
        	return -1;
        }

	if ( fabs ( m1.point_two[0]-6.305182992539864 ) > eps ) {
        	fprintf( stderr, "Wrong point_two[0]\n" );
        	return -1;
        }

	if ( fabs ( m1.point_two[1]-5.207106781186548 ) > eps ) {
        	fprintf( stderr, "Wrong point_two[1]\n" );
        	return -1;
        }

        return 0;
}*/

/*
void MyEllipse( Mat img, double angle );
void MyFilledCircle( Mat img, Point center );
void MyPolygon( Mat img );
void MyLine( Mat img, Point start, Point end );
*/ 
int main( void ) {

        //if(Manipulator_test()!=0)
        //	return -1;

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

	Point zero;
	Point one;
	Point two;
	
	manp_1.getPoints(zero, one, two);

	Point dest;
	dest.x = 3*w/4;
	dest.y = 3*w/4;

	manp_1.FABRIK( graph_window, 2, zero, one, two, dest, graph_image );
	
	/*
	/// Windows names
	char atom_window[] = "Drawing 1: Atom";
	char rook_window[] = "Drawing 2: Rook";
	/// Create black empty images
	Mat atom_image = Mat::zeros( w, w, CV_8UC3 );
	Mat rook_image = Mat::zeros( w, w, CV_8UC3 );
	/// 1. Draw a simple atom:
	/// -----------------------
	/// 1.a. Creating ellipses
	MyEllipse( atom_image, 90 );
	MyEllipse( atom_image, 0 );
	MyEllipse( atom_image, 45 );
	MyEllipse( atom_image, -45 );
	/// 1.b. Creating circles
	MyFilledCircle( atom_image, Point( w/2, w/2) );
	/// 2. Draw a rook
	/// ------------------
	/// 2.a. Create a convex polygon
	MyPolygon( rook_image );
	/// 2.b. Creating rectangles
	rectangle( rook_image,
	Point( 0, 7*w/8 ),
	Point( w, w),
	Scalar( 0, 255, 255 ),-1,8 );
	/// 2.c. Create a few lines
	MyLine( rook_image, Point( 0, 15*w/16 ), Point( w, 15*w/16 ) );
	MyLine( rook_image, Point( w/4, 7*w/8 ), Point( w/4, w ) );
	MyLine( rook_image, Point( w/2, 7*w/8 ), Point( w/2, w ) );
	MyLine( rook_image, Point( 3*w/4, 7*w/8 ), Point( 3*w/4, w ) );
	/// 3. Display your stuff!
	imshow( atom_window, atom_image );
	moveWindow( atom_window, 0, 200 );
	imshow( rook_window, rook_image );
	moveWindow( rook_window, w, 200 );
	waitKey( 0 );
	return(0);
	*/
	
	imshow(graph_window,graph_image);
	moveWindow(graph_window, 0, 200 );
	waitKey(0);
	return(0);
	
}
/*
void MyEllipse( Mat img, double angle )
{
	int thickness = 2;
	int lineType = 8;
	ellipse( img,
	Point( w/2, w/2 ),
	Size( w/4, w/16 ),
	angle,
	0,
	360,
	Scalar( 255, 0, 0 ),
	thickness,
	lineType );
}
void MyFilledCircle( Mat img, Point center )
{
	int thickness = -1;
	int lineType = 8;
	circle( img,
	center,
	w/32,
	Scalar( 0, 0, 255 ),
	thickness,
	lineType );
}

void MyPolygon( Mat img )
{
	int lineType = 8;
	Point rook_points[1][20];
	rook_points[0][0] = Point( w/4, 7*w/8 );
	rook_points[0][1] = Point( 3*w/4, 7*w/8 );
	rook_points[0][2] = Point( 3*w/4, 13*w/16 );
	rook_points[0][3] = Point( 11*w/16, 13*w/16 );
	rook_points[0][4] = Point( 19*w/32, 3*w/8 );
	rook_points[0][5] = Point( 3*w/4, 3*w/8 );
	rook_points[0][6] = Point( 3*w/4, w/8 );
	rook_points[0][7] = Point( 26*w/40, w/8 );
	rook_points[0][8] = Point( 26*w/40, w/4 );
	rook_points[0][9] = Point( 22*w/40, w/4 );
	rook_points[0][10] = Point( 22*w/40, w/8 );
	rook_points[0][11] = Point( 18*w/40, w/8 );
	rook_points[0][12] = Point( 18*w/40, w/4 );
	rook_points[0][13] = Point( 14*w/40, w/4 );
	rook_points[0][14] = Point( 14*w/40, w/8 );
	rook_points[0][15] = Point( w/4, w/8 );
	rook_points[0][16] = Point( w/4, 3*w/8 );
	rook_points[0][17] = Point( 13*w/32, 3*w/8 );
	rook_points[0][18] = Point( 5*w/16, 13*w/16 );
	rook_points[0][19] = Point( w/4, 13*w/16 );
	const Point* ppt[1] = { rook_points[0] };
	int npt[] = { 20 };
	fillPoly( img,
	ppt,
	npt,
	1,
	Scalar( 255, 255, 255 ),
	lineType );
}

void MyLine( Mat img, Point start, Point end )
{
	int thickness = 2;
	int lineType = 8;
	line( img,
	start,
	end,
	Scalar( 0, 0, 0 ),
	thickness,
	lineType );
}
*/
