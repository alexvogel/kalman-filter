/*
 * testmain.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: avoge
 */
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

using namespace std;

void checkConversion(VectorXd state4d, VectorXd radar);

int main()
{


	VectorXd radar_1 =  VectorXd(3);
	radar_1 << 83.2166, 0.57134, 27.81897;

	VectorXd state4d_1 =  VectorXd(4);
	state4d_1 << 70., 45., 17., 25.;


	cout << "============" << endl;
	cout << "check POS-POS" << endl;
	checkConversion(state4d_1, radar_1);
	cout << "============" << endl;


	//
	VectorXd radar_2 =  VectorXd(3);
	radar_2 << 55.9017, -0.46365, -1.7889;

	VectorXd state4d_2 =  VectorXd(4);
	state4d_2 << 50., -25., 4., 12.;


	cout << "============" << endl;
	cout << "check POS-NEG" << endl;
	checkConversion(state4d_2, radar_2);
	cout << "============" << endl;

	//
	VectorXd radar_3 =  VectorXd(3);
	radar_3 << 13.6015, -0.6288, 0.73521;

	VectorXd state4d_3 =  VectorXd(4);
	state4d_3 << -11., 8., 2., 4.;


	cout << "============" << endl;
	cout << "check NEG-POS" << endl;
	checkConversion(state4d_3, radar_3);
	cout << "============" << endl;

	//
	VectorXd radar_4 =  VectorXd(3);
	radar_4 << 40.2492, 0.46365, -31305;

	VectorXd state4d_4 =  VectorXd(4);
	state4d_4 << -36., -18., 3., 1.;


	cout << "============" << endl;
	cout << "check NEG-NEG" << endl;
	checkConversion(state4d_4, radar_4);
	cout << "============" << endl;


}


void checkConversion(VectorXd state4d, VectorXd radar)
{
	Tools tools;


	cout << "++++++++++++" << endl;
	cout << "check radar2state" << endl;
	cout << "------------" << endl;
	cout << "radar data: " << radar << endl;
	cout << "------------" << endl;

	cout << "state after radar2state: " << tools.Radar2state(radar) << endl;
	cout << "------------" << endl;

	cout << "state should be: " << state4d << endl;


	cout << "++++++++++++" << endl;
	cout << "check state2radar" << endl;
	cout << "------------" << endl;
	cout << "state data: " << state4d << endl;
	cout << "------------" << endl;

	cout << "radar after state2radar: " << tools.State2radar(state4d) << endl;
	cout << "------------" << endl;

	cout << "radar should be: " << radar << endl;

}

