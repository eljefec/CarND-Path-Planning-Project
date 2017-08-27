#include <vector>

#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

vector<double> JMT(vector<double> start, vector<double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    // AX = B
    // X = A^(-1)*B
    Eigen::Matrix3d A;
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    A << T3, T4, T5,
        3*T2, 4*T3, 5*T4,
        6*T, 12*T2, 20*T3;

    Eigen::Vector3d B;
    B << end[0] - (start[0] + start[1] * T + start[2] / 2 * T2),
        end[1] - (start[1] + start[2] * T),
        end[2] - start[2];

    Eigen::Vector3d X = A.inverse() * B;

    return {start[0], start[1], start[2] /2, X[0], X[1], X[2]};

}
