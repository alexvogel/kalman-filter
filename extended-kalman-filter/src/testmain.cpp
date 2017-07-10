/*
 * testmain.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: avoge
 */
#include <iostream>
#include "tools.h"
#include "Eigen/Dense"


using namespace std;

int main()
{
	Tools tools;


	VectorXd radar =  VectorXd(3);
	radar << 83.2166, 0.57134, 27.81897;


	VectorXd state4d =  VectorXd(4);
	state4d << 70., 45., 17., 25.;

	cout << "============" << endl;
	cout << "check radar2state" << endl;
	cout << "------------" << endl;
	cout << "radar data: " << radar << endl;
	cout << "------------" << endl;

	cout << "state after radar2state: " << tools.Radar2state(radar) << endl;
	cout << "------------" << endl;

	cout << "state should be: " << state4d << endl;


	cout << "============" << endl;
	cout << "check state2radar" << endl;
	cout << "------------" << endl;
	cout << "state data: " << state4d << endl;
	cout << "------------" << endl;

	cout << "radar after state2radar: " << tools.State2radar(state4d) << endl;
	cout << "------------" << endl;

	cout << "radar should be: " << radar << endl;

}

