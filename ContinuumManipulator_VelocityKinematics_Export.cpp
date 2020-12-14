#include <acado_toolkit.hpp>
#include <acado_gnuplot.hpp>

USING_NAMESPACE_ACADO

int main()
{

  DifferentialState x, y, z, th1, th2, ph1, ph2, x_int, y_int, z_int; // [x y z]_int are the intermediate states.
  Control dth1, dth2, dph1, dph2;
  // OnlineData obst;

  const double t_start = 0.00;
  const double samplingTime = 0.05;
  const double PI = 3.14159;
  // double x_ref = 0.2, y_ref = 0.1, z_ref = 0.25;


  double s = 0.182;

  DifferentialEquation f;

  int T = 10;

  const double t_end = t_start+T*samplingTime;

// Differential Eqn

  f << dot(x) == -dth2*((s*cos(ph2)*(pow(sin(ph1),2.0)+pow(cos(ph1),2.0)*cos(th1))*((th2*th2)*3.0-1.2E+1))/2.4E+1-(s*sin(ph2)*(cos(ph1)*sin(ph1)-cos(ph1)*cos(th1)*sin(ph1))*((th2*th2)*3.0-1.2E+1))/2.4E+1+(s*cos(ph1)*sin(th1)*(th2*4.0E+1-(th2*th2*th2)*4.0))/1.2E+2)-dth1*((s*cos(ph1)*((th1*th1)*3.0-1.2E+1))/2.4E+1-(s*cos(ph1)*cos(th1)*((th2*th2)*-2.0E+1+th2*th2*th2*th2+1.2E+2))/1.2E+2+(s*pow(cos(ph1),2.0)*cos(ph2)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph1)*sin(ph1)*sin(ph2)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1)-dph2*((s*sin(ph2)*(pow(sin(ph1),2.0)+pow(cos(ph1),2.0)*cos(th1))*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph2)*(cos(ph1)*sin(ph1)-cos(ph1)*cos(th1)*sin(ph1))*(th2*1.2E+1-th2*th2*th2))/2.4E+1)-dph1*((s*sin(ph1)*(th1*1.2E+1-th1*th1*th1))/2.4E+1+(s*sin(ph1)*sin(th1)*((th2*th2)*-2.0E+1+th2*th2*th2*th2+1.2E+2))/1.2E+2+(s*sin(ph2)*(th2*1.2E+1-th2*th2*th2)*(pow(cos(ph1),2.0)-pow(sin(ph1),2.0)-pow(cos(ph1),2.0)*cos(th1)+cos(th1)*pow(sin(ph1),2.0)))/2.4E+1-(s*cos(ph2)*(cos(ph1)*sin(ph1)*2.0-cos(ph1)*cos(th1)*sin(ph1)*2.0)*(th2*1.2E+1-th2*th2*th2))/2.4E+1);
  f << dot(y) == -dth2*((s*sin(ph1)*sin(th1)*(th2*4.0E+1-(th2*th2*th2)*4.0))/1.2E+2+(s*sin(ph2)*(pow(cos(ph1),2.0)+cos(th1)*pow(sin(ph1),2.0))*((th2*th2)*3.0-1.2E+1))/2.4E+1-(s*cos(ph2)*(cos(ph1)*sin(ph1)-cos(ph1)*cos(th1)*sin(ph1))*((th2*th2)*3.0-1.2E+1))/2.4E+1)-dth1*((s*sin(ph1)*((th1*th1)*3.0-1.2E+1))/2.4E+1-(s*cos(th1)*sin(ph1)*((th2*th2)*-2.0E+1+th2*th2*th2*th2+1.2E+2))/1.2E+2+(s*pow(sin(ph1),2.0)*sin(ph2)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph1)*cos(ph2)*sin(ph1)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1)+dph2*((s*cos(ph2)*(pow(cos(ph1),2.0)+cos(th1)*pow(sin(ph1),2.0))*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*sin(ph2)*(cos(ph1)*sin(ph1)-cos(ph1)*cos(th1)*sin(ph1))*(th2*1.2E+1-th2*th2*th2))/2.4E+1)+dph1*((s*cos(ph1)*(th1*1.2E+1-th1*th1*th1))/2.4E+1-(s*cos(ph2)*(th2*1.2E+1-th2*th2*th2)*(pow(cos(ph1),2.0)-pow(sin(ph1),2.0)-pow(cos(ph1),2.0)*cos(th1)+cos(th1)*pow(sin(ph1),2.0)))/2.4E+1-(s*sin(ph2)*(cos(ph1)*sin(ph1)*2.0-cos(ph1)*cos(th1)*sin(ph1)*2.0)*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph1)*sin(th1)*((th2*th2)*-2.0E+1+th2*th2*th2*th2+1.2E+2))/1.2E+2);
  f << dot(z) == -dth1*((s*(th1*4.0E+1-(th1*th1*th1)*4.0))/1.2E+2+(s*sin(th1)*((th2*th2)*-2.0E+1+th2*th2*th2*th2+1.2E+2))/1.2E+2+(s*cos(ph1)*cos(ph2)*cos(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1-(s*cos(th1)*sin(ph1)*sin(ph2)*(th2*1.2E+1-th2*th2*th2))/2.4E+1)-dth2*((s*cos(th1)*(th2*4.0E+1-(th2*th2*th2)*4.0))/1.2E+2+(s*sin(ph1)*sin(ph2)*sin(th1)*((th2*th2)*3.0-1.2E+1))/2.4E+1-(s*cos(ph1)*cos(ph2)*sin(th1)*((th2*th2)*3.0-1.2E+1))/2.4E+1)+dph1*((s*cos(ph1)*sin(ph2)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph2)*sin(ph1)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1)+dph2*((s*cos(ph1)*sin(ph2)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1+(s*cos(ph2)*sin(ph1)*sin(th1)*(th2*1.2E+1-th2*th2*th2))/2.4E+1);
  f << dot(th1) == dth1;
  f << dot(th2) == dth2;
  f << dot(ph1) == dph1;
  f << dot(ph2) == dph2;


// LSQ

  Function h;
  Function hN;
  h << x;
  h << y;
  h << z;
  h << dth1;
  h << dth2;
  h << dph1;
  h << dph2;
  hN << x;
  hN << y;
  hN << z;

  BMatrix W(7,7);
  // DMatrix W(3,3);
  W.setIdentity();
  W(0,0) = 150;
  W(1,1) = 150;
  W(2,2) = 150;
  BMatrix WN(3,3);
  WN.setIdentity();
  WN(0,0) = 250;
  WN(1,1) = 250;
  WN(2,2) = 250;

  // BMatrix W = eye<bool>(h.getDim());
  // BMatrix WN = eye<bool>(hN.getDim());


  DVector r(7);
  // r.setZero();
  // r(0) = x_ref;
  // r(1) = y_ref;
  // r(2) = z_ref;

  // Optimal Control PROBLEM

  OCP ocp(0, 0.5, 10);

  ocp.subjectTo(f);

  ocp.subjectTo(-0.3 <=   x  <= 0.3);
  ocp.subjectTo(-0.3 <=   y  <= 0.3);
  ocp.subjectTo(  0  <=   z  <= 0.48);
  ocp.subjectTo(-5*PI/3 <= th1 <= 5*PI/3);
  ocp.subjectTo(-5*PI/3 <= th2 <= 5*PI/3);
  ocp.subjectTo(-2*PI   <= ph1 <= 2*PI);
  ocp.subjectTo(-2*PI   <= ph2 <= 2*PI);
  ocp.subjectTo(-0.8 <= dph1 <= 0.8);
  ocp.subjectTo(-0.8 <= dth1 <= 0.8);
  ocp.subjectTo(-0.8 <= dph2 <= 0.8);
  ocp.subjectTo(-0.8 <= dth2 <= 0.8);

  ocp.minimizeLSQ(W,h);
  ocp.minimizeLSQEndTerm(WN,hN);

  OCPexport mpc(ocp);

  mpc.set(HESSIAN_APPROXIMATION, GAUSS_NEWTON);
  //mpc.set(DISCRETIZATION_TYPE, SINGLE_SHOOTING);
  mpc.set(DISCRETIZATION_TYPE, MULTIPLE_SHOOTING);
  mpc.set(INTEGRATOR_TYPE, INT_RK4);
  // mpc.set(INTEGRATOR_TYPE, INT_IRK_RIIA3);
  // mpc.set(NUM_INTEGRATOR_STEPS, N * Ni);
  mpc.set(SPARSE_QP_SOLUTION, FULL_CONDENSING);
  //	mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(QP_SOLVER, QP_QPOASES);
  //	mpc.set(QP_SOLVER, QP_FORCES);
  //	mpc.set(MAX_NUM_QP_ITERATIONS, 20);
  mpc.set(HOTSTART_QP, YES);
  //	mpc.set(SPARSE_QP_SOLUTION, SPARSE_SOLVER);
  //	mpc.set(LEVENBERG_MARQUARDT, 1.0e-10);
  mpc.set(GENERATE_TEST_FILE, YES);
  mpc.set(GENERATE_MAKE_FILE, YES);
  // mpc.set(GENERATE_MATLAB_INTERFACE, YES);
  //	mpc.set(USE_SINGLE_PRECISION, YES);
  mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX, YES);
  //mpc.set( CG_HARDCODE_CONSTRAINT_VALUES, NO);
  //	mpc.set(CG_USE_OPENMP, YES);
  // NOTE: This is crucial for export of MHE!
  //mpc.set(SPARSE_QP_SOLUTION, CONDENSING);
  mpc.set(FIX_INITIAL_STATE, YES);

  if (mpc.exportCode( "ContinuumManipulator" ) != SUCCESSFUL_RETURN)
          exit( EXIT_FAILURE );


//   OutputFcn identity;
//   DynamicSystem dynamicSystem(f, identity);
//
//   Process process(dynamicSystem, INT_RK45);
//
//   RealTimeAlgorithm algorithm(ocp, samplingTime);
//
//   StaticReferenceTrajectory zeroReference;
//
//   Controller controller(algorithm, zeroReference);
//
//   double simStartTime = 0.0;
//   double simEndTime = 20;
//
//   SimulationEnvironment sim(simStartTime, simEndTime, process, controller);
//
//   DVector x0(7);
//
//   x0.setZero();
//   x0(2) = 0.48;
//
//   if (sim.init( x0 ) != SUCCESSFUL_RETURN)
//     exit( EXIT_FAILURE );
//   if (sim.run( ) != SUCCESSFUL_RETURN)
//     exit( EXIT_FAILURE );
//
//   VariablesGrid diffStates;
//   sim.getProcessDifferentialStates(diffStates);
//   VariablesGrid feedbackControl;
//   sim.getFeedbackControl(feedbackControl);
//
//   GnuplotWindow window;
// window.addSubplot( diffStates(0), "x");
// window.addSubplot( diffStates(1), "y");
// window.addSubplot( diffStates(2), "z");
// window.addSubplot( diffStates(3), "th1");
// window.addSubplot( diffStates(4), "th2");
// window.addSubplot( diffStates(5), "ph1");
// window.addSubplot( diffStates(6), "ph2");
// window.addSubplot( feedbackControl(0), "dth1");
// window.addSubplot( feedbackControl(1), "dth2");
// window.addSubplot( feedbackControl(2), "dph1");
// window.addSubplot( feedbackControl(3), "dph2");
//   window.plot();


  return EXIT_SUCCESS;



}
