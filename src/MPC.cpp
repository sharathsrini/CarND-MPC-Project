#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// TODO: done
// Set the timestep length and duration
size_t N = 5;
double dt = 0.5;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// Define indices for variable vars
// State
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
// Actuators
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N-1; // since there are N-1 actuators for N steps

const size_t state_dim = 6;
const size_t act_dim = 2;

const AD<double> v_ref =  30;

const int w_cte =         1000; // cross-track error weight
const int w_epsi =        1000; // orientation error weight
const int w_delta =       1; // steering actuator action weight
const int w_a =           1; // acceleration actuator action weight
const int w_vel_diff =    1; // v - v_ref weight
const int w_delta_diff =  10; // change in steering actuator action weight
const int w_a_diff =      10; // change in acceleration actuator action weight
//const int w_epsi_diff = 100;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: done
    // implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    /*
     * Construct cost function.
     * The first element of vector fg contains cost function terms
     */
    fg[0] = 0.0;
    for (size_t i=0; i < N; ++i) {
      fg[0] += w_cte * CppAD::pow(vars[cte_start + i], 2);  // minimise cross-track error
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + i], 2); // minimise orientation error
      fg[0] += w_vel_diff * CppAD::pow(vars[v_start + i] - v_ref, 2);
    }
    for (size_t i=0; i < N-1; ++i) {
      fg[0] += w_delta * CppAD::pow(vars[delta_start + i], 2);  // minimise steering actuator action
      fg[0] += w_a * CppAD::pow(vars[a_start + i], 2); // minimise throttle/brake action
      //fg[0] += w_epsi_diff * CppAD::pow(vars[epsi_start + i + 1] - vars[epsi_start + i], 2);
    }
    for (size_t t=0; t < N-2; ++t) {
      fg[0] += w_delta_diff * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);  // smooth steering angle action
      fg[0] += w_a_diff * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2); // smooth throttle/brake action
    }

    /*
     * Construct the rest of fg.
     */
    /*
    double delay = 0.5;
    AD<double> v0_ = vars[v_start];
    AD<double> psi0_ = vars[psi_start];
    AD<double> delta0_ = vars[delta_start]; 
    AD<double> a0_ = vars[a_start]; 
    
    fg[x_start + 1] = vars[x_start] + v0_ * CppAD::cos(psi0_) * delay;
    fg[y_start + 1] = vars[y_start] + v0_ * CppAD::sin(psi0_) * delay;;
    fg[psi_start + 1] = vars[psi_start] - v0_/Lf * delta0_ * delay;;
    fg[v_start + 1] = vars[v_start] + a0_ * delay ;
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];
    */

    fg[x_start + 1] = vars[x_start];
    fg[y_start + 1] = vars[y_start];
    fg[psi_start + 1] = vars[psi_start];
    fg[v_start + 1] = vars[v_start];
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];


    for (size_t t=1; t < N; ++t) {
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2);
      AD<double> f_prime0 = CppAD::atan(coeffs[1] + 2*coeffs[2]*x0);

      fg[x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[psi_start + t + 1] = psi1 - (psi0 - v0/Lf * delta0 * dt);
      fg[v_start + t + 1] = v1 - (v0 + a0 * dt);
      fg[cte_start + t + 1] = cte1 - ((y0 - f0) + v0 * CppAD::sin(epsi0) * dt);
      fg[epsi_start + t + 1] = epsi1 - ((psi0 - f_prime0) - v0/Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<vector<double>> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x0 = state[0];
  double y0 = state[1];
  double psi0 = state[2];
  double v0 = state[3];
  double cte0 = state[4];
  double epsi0 = state[5];

  // TODO: done
  // Set the number of model variables (includes both states and inputs)
  size_t n_vars = N * state_dim + (N-1) * act_dim;
  // TODO: done
  // Set the number of constraints
  size_t n_constraints = N * state_dim;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  vars[x_start] = x0;
  vars[y_start] = y0;
  vars[psi_start] = psi0;
  vars[v_start] = v0;
  vars[cte_start] = cte0;
  vars[epsi_start] = epsi0;

  /*
  for (int i=0; i< n_vars; i++) {
    cout << "x_" << i << ": " << vars[i] << endl;
  }
  */

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: done
  // Set lower and upper limits for variables.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0.0;
    constraints_upperbound[i] = 0.0;
  }
  constraints_lowerbound[x_start] = x0;
  constraints_upperbound[x_start] = x0;
  constraints_lowerbound[y_start] = y0;
  constraints_upperbound[y_start] = y0;
  constraints_lowerbound[psi_start] = psi0;
  constraints_upperbound[psi_start] = psi0;
  constraints_lowerbound[v_start] = v0;
  constraints_upperbound[v_start] = v0;
  constraints_lowerbound[cte_start] = cte0;
  constraints_upperbound[cte_start] = cte0;
  constraints_lowerbound[epsi_start] = epsi0;
  constraints_upperbound[epsi_start] = epsi0;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          100\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (!ok) {
    cout << "Couldn't solve... :(" << endl;
    return {{-1.0}, {-1.0}};

  }
  else {
    // Cost
    auto cost = solution.obj_value;
    std::cout << "Solver results." <<std::endl;
    std::cout << "Cost " << cost << std::endl;
    std::vector<double> x_pred;
    std::vector<double> y_pred;
    std::vector<double> actuation{solution.x[delta_start], solution.x[a_start]};

    for (size_t i=0; i < N; ++i) {
      x_pred.push_back(solution.x[x_start + i]);
      y_pred.push_back(solution.x[y_start + i]);
    }
    // TODO: Return the first actuator values. The variables can be accessed with
    // `solution.x[i]`.
    //
    // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
    // creates a 2 element double vector.
    return {x_pred, y_pred, actuation};
  }

}
