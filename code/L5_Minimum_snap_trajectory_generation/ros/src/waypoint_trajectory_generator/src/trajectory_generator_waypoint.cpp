#include "trajectory_generator_waypoint.h"
#include <stdio.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include <string>
#include "OsqpEigen/OsqpEigen.h"
#include <Eigen/Dense>

using namespace std;    
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(){}
TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint(){}

//define factorial function, input i, output i!
int TrajectoryGeneratorWaypoint::Factorial(int x)
{
    int fac = 1;
    for(int i = x; i > 0; i--)
        fac = fac * i;
    return fac;
}
/*

    STEP 2: Learn the "Closed-form solution to minimum snap" in L5, then finish this PolyQPGeneration function

    variable declaration: input       const int d_order,                    // the order of derivative
                                      const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
                                      const Eigen::MatrixXd &Vel,           // boundary velocity
                                      const Eigen::MatrixXd &Acc,           // boundary acceleration
                                      const Eigen::VectorXd &Time)          // time allocation in each segment
                          output      MatrixXd PolyCoeff(m, 3 * p_num1d);   // position(x,y,z), so we need (3 * p_num1d) coefficients

*/

Eigen::MatrixXd TrajectoryGeneratorWaypoint::PolyQPGeneration(
            const int d_order,                    // the order of derivative
            const Eigen::MatrixXd &Path,          // waypoints coordinates (3d)
            const Eigen::MatrixXd &Vel,           // boundary velocity
            const Eigen::MatrixXd &Acc,           // boundary acceleration
            const Eigen::VectorXd &Time)          // time allocation in each segment
{
    // enforce initial and final velocity and accleration, for higher order derivatives, just assume them be 0;
    int p_order   = 2 * d_order - 1;              // the order of polynomial
    int p_num1d   = p_order + 1;                  // the number of variables in each segment

    int m = Time.size();                          // the number of segments
    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d);           // position(x,y,z), so we need (3 * p_num1d) coefficients
    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

	// Initialize all the matrix/vector
	SparseMatrix<double> Q();
	VectorXd f();
	SparseMatrix<double> Aeq();
	VectorXd Beq();

	
	Xd Q_values; Q_indices; Q_num;
	int i_seg, l_seg;
	double qil;
	 
	for (int seg=0;seg<m;seg++){
		Q_seg = MatrixXd:Zero(p_num1d, p_num1d);
		
		for (int i=p_order-1;i>2;i--){
		
			for (int l=p_order-1;l>2;l--){
		
				i_seg = i + 1;
				l_seg = l + 1; 
				qil = power(Time[seg],i_seg+l_seg-7)*i_seg*(i_seg-1)*(i_seg-2)*(i_seg-3)*l_seg*(l_seg-1)*(l_seg-2)*(l_seg-3)/(i_seg+l_seg-4);
				if (i==l){Q(p_num1d-i_seg, p_num1d-l_seg) = qil;}
				else {Q(p_num1d-i_seg, p_num1d-l_seg) = qil/2;}
				
			} // end for
		} // end for
		
		Q = 
		
	} //end for

	OsqpEigen::Solver solver;
	solver.settings()-->setVerbosity(false);
	solver.settings()-->setWarmStart(true);
	
	solver.data()-->setNumberOfVariables();
	solver.date()-->setNumberOfConstraints();
	solver.data()->setHessianMatrix(hessian);
	solver.data()->setGradient(gradient);
	solver.data()->setLinearConstraintsMatrix(linearMatrix);
	solver.data()->setLowerBound(lowerBound);
	solver.data()->setUpperBound(upperBound);
	solver.initSolver();
	
	Eigen::VectorXd QPSolution;
	QPSolution = solver.getSolution();



















    return PolyCoeff;
}
