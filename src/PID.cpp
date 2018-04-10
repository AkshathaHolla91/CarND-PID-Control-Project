#include "PID.h"

using namespace std;


PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
    //Initializing the tau values for all 3 parameters
    this->Kp=Kp;
    this->Ki=Ki;
    this->Kd=Kd;
}

void PID::UpdateError(double cte) {
    //Updating the error values
    d_error=cte-p_error;
    p_error=cte;
    i_error+=cte;
}

double PID::TotalError() {
    //Calculating the steer value while maintaining it in the range[-1,1]
    double steer= -Kp*p_error-Kd*d_error-Ki*i_error;
    if(steer<-1){
        steer=-1;
    }
    else if(steer>1){
        steer=1;
    }
    return steer;
}

