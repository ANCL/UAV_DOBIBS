#pragma once

#include <px4_posix.h>
#include <controllib/uorb/blocks.hpp>
#include <math.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
using namespace std;


using namespace control;

class BlockTASKController : public control::BlockTASKLoop
{
public:
    BlockTASKController() :
        BlockTASKLoop(NULL,"TASK"),
        _iop(this,"I_PARAM"),
        _u_int(this,"I_PARAM"),
        _iw0(this,"I_PARAM"),
        _iw1(this,"I_PARAM"),
        _iw2(this,"I_PARAM"),
        i_df_0(this,"I_PARAM"),
        i_df_1(this,"I_PARAM"),
        i_df_2(this,"I_PARAM"),
        _int_er_0(this,"I_PARAM"),
        _int_er_1(this,"I_PARAM"),
        _int_er_2(this,"I_PARAM"),
        _fds(),
        _t(0),
        _switch_traj(0),
	    prev_rates(),
	    prev_rates2(),
        rate_int()
    {
        _fds[0].fd = _pos.getHandle();
        _fds[0].events = POLLIN;
        switch_traj = param_find("MC_DOBIBS_TRAJ");
        param_get(switch_traj, &_switch_traj);
    }
    void update();
private:
    BlockIntegral _iop;
    BlockIntegral _u_int;
    BlockIntegral _iw0;
    BlockIntegral _iw1;
    BlockIntegral _iw2;
    BlockIntegral i_df_0;
    BlockIntegral i_df_1;
    BlockIntegral i_df_2;
    BlockIntegral _int_er_0;
    BlockIntegral _int_er_1;
    BlockIntegral _int_er_2;
   // BlockIntegral integ_test;
    px4_pollfd_struct_t _fds[1];
    uint64_t _t;

    // params
    param_t switch_traj;

    float obs=0;
    float thrust_input=10;
    //float u_dot;
    float d_zdf_0=0;
    float z_df_0=0;
    float d_zdf_1=0;
    float z_df_1=0;
    float d_zdf_2=0;
    float z_df_2=0;
    float traj_t=0;
    float T=20;
    float iw0;
    float iw1;
    float iw2;
    float intu=0;
    float intw0=0;
    float intw1=0;
    float intw2=0;
    int traj=0;
    float switch_time=0;

    int _switch_traj;
    matrix::Vector<float, 3>  prev_rates;
    matrix::Vector<float, 3>  prev_rates2;
    matrix::Vector<float, 3>  rate_int;    
};