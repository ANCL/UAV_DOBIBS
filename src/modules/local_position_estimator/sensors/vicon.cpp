#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_VCN_INIT_COUNT = 100;
static const uint32_t 		VCN_TIMEOUT =     100000;	// 0.1 s

void BlockLocalPositionEstimator::viconInit()
{
	// measure
	Vector<float, n_y_vicon> y;

	if (viconMeasure(y) != OK) {
		_viconStats.reset();
		return;
	}

	// if finished
	if (_viconStats.getCount() > REQ_VCN_INIT_COUNT) {
		mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vicon position init: "
					     "(%5.2f, %5.2f, %5.2f)m std: (%5.2f, %5.2f, %5.2f)m, (%5.2f, %5.2f, %5.2f)m/s std: (%5.2f, %5.2f, %5.2f)m/s",
					     double(_viconStats.getMean()(0)),
					     double(_viconStats.getMean()(1)),
					     double(_viconStats.getMean()(2)),
					     double(_viconStats.getStdDev()(0)),
					     double(_viconStats.getStdDev()(1)),
					     double(_viconStats.getStdDev()(2)),
					     double(_viconStats.getMean()(3)),
					     double(_viconStats.getMean()(4)),
					     double(_viconStats.getMean()(5)),
					     double(_viconStats.getStdDev()(3)),
					     double(_viconStats.getStdDev()(4)),
					     double(_viconStats.getStdDev()(5)));
		_viconInitialized = true;
		_viconFault = FAULT_NONE;

		if (!_altOriginInitialized) {
			_altOriginInitialized = true;
			_altOrigin = 0;
		}
	}
}

int BlockLocalPositionEstimator::viconMeasure(Vector<float, n_y_vicon> &y)
{
	// vicon measurment
	y.setZero();
	y(0) = _sub_vicon.get().p[0];
	y(1) = _sub_vicon.get().p[1];
	y(2) = _sub_vicon.get().p[2];
	y(3) = _sub_vicon.get().v[0];
	y(4) = _sub_vicon.get().v[1];
	y(5) = _sub_vicon.get().v[2];

	// increment sums for mean
	_viconStats.update(y);
	_time_last_vicon = _timeStamp;
	return OK;
}

void BlockLocalPositionEstimator::viconCorrect()
{
	//measure
	Vector<float, n_y_vicon> y;

	if (viconMeasure(y) != OK) { return; }

	// vicon measurment matrix, measures position and velocity
	Matrix<float, n_y_vicon, n_x> C;
	C.setZero();
	C(Y_vicon_x, X_x) = 1;
	C(Y_vicon_y, X_y) = 1;
	C(Y_vicon_z, X_z) = 1;
	C(Y_vicon_vx, X_vx) = 1;
	C(Y_vicon_vy, X_vy) = 1;
	C(Y_vicon_vz, X_vz) = 1;

	// vicon covariance matrix
	SquareMatrix<float, n_y_vicon> R;
	R.setZero();
	float vicon_p_var = _vicon_p_stddev.get() * _vicon_p_stddev.get();
	float vicon_v_var = _vicon_v_stddev.get() * _vicon_v_stddev.get();

	R(Y_vicon_x,Y_vicon_x) = vicon_p_var;
	R(Y_vicon_y,Y_vicon_y) = vicon_p_var;
	R(Y_vicon_z,Y_vicon_z) = vicon_p_var;
	R(Y_vicon_vx,Y_vicon_vx) = vicon_v_var;
	R(Y_vicon_vy,Y_vicon_vy) = vicon_v_var;
	R(Y_vicon_vz,Y_vicon_vz) = vicon_v_var;

	// residual
	Matrix<float, n_y_vicon, n_y_vicon> S_I = inv<float, n_y_vicon>((C * _P * C.transpose())+R);
	Matrix<float, n_y_vicon, 1> r = y - C * _x;

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0,0);

	if (beta > BETA_TABLE[n_y_vicon]) {
		if (_viconFault < FAULT_MINOR) {
			_viconFault = FAULT_MINOR;
		}
	} else if (_viconFault) {
		_viconFault = FAULT_NONE;
	}

	//kalman filter correction if no fault
	if (_viconFault <fault_lvl_disable) {
		Matrix<float, n_x, n_y_vicon> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
	}
}


void BlockLocalPositionEstimator::viconCheckTimeout()
{
	if (_timeStamp - _time_last_vicon > VCN_TIMEOUT) {
		if (_viconInitialized) {
			_viconInitialized = false;
			_viconStats.reset();
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] vicon timeout ");
		}
	}
}

	

	
