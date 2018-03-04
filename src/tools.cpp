#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

/**
  * A helper method to calculate RMSE.
  * @param estimations Estimations
  * @param ground_truth Ground truth values
  */
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() == 0 || estimations.size() != ground_truth.size()){
	    cout << "Error: estimations size zero or estimations and ground_truth different" << endl;
	}
    else{
    	// accumulate squared residuals
	    for(int i=0; i < estimations.size(); ++i){

            VectorXd res = estimations[i]-ground_truth[i];

            res = res.array()*res.array();

            rmse += res;

	    }

	    // calculate the mean
	    rmse = rmse / estimations.size();

	    // calculate the squared root
	    rmse = rmse.array().sqrt();
    }
	// return the result
	return rmse;
}

/**
 * A helper method to adjust an angle between -PI and PI.
 * @param vecElement Vector element where the angle is stored
 * @param Vector Vector where the angle to be adjusted is located
 */
void Tools::AdjustAngle(int vecElement, VectorXd &vector){

	// reduce or increase angle until it is between (-PI..PI)
    while (vector(vecElement)> M_PI) vector(vecElement)-=2.*M_PI;
    while (vector(vecElement)<-M_PI) vector(vecElement)+=2.*M_PI;
}
