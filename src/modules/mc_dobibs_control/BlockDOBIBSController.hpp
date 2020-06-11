#pragma once

#include <px4_posix.h>
#include <px4_defines.h>
#include <controllib/uorb/blocks.hpp>
#include <math.h>
#include <vector>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
using namespace std;


using namespace control;

class BlockDOBIBSController : public control::BlockDOBIBSLoop
{
public:
    BlockDOBIBSController() :
        BlockDOBIBSLoop(NULL,"DOBIBS"),
        _i_u(this,"I_U_PAR"),
        _i_df_0(this,"I_DF_1"),
        _i_df_1(this,"I_DF_2"),
        _i_df_2(this,"I_DF_3"),
        _int_er_0(this,"I_PARAM"),
        _int_er_1(this,"I_PARAM"),
        _int_er_2(this,"I_PARAM"),
        _fds(),
        _t(0),
        pd(),
        dpd(),
        ddpd(),
        dddpd(),
        ddddpd()
    {
        _fds[0].fd = _pos.getHandle();
        _fds[0].events = POLLIN;
        switch_traj = param_find("MC_DOBIBS_TRAJ");
        param_get(switch_traj, &_switch_traj);
        param_get(param_find("DOBIBS_IBS_K1"), &k1);
        param_get(param_find("DOBIBS_IBS_K2"), &k2);
        param_get(param_find("DOBIBS_IBS_K3"), &k3);
        param_get(param_find("DOBIBS_IBS_K4"), &k4);
        param_get(param_find("DOBIBS_IBS_K5"), &k5);
        param_get(param_find("DOBIBS_BS_K1"), &k1_bs);
        param_get(param_find("DOBIBS_BS_K2"), &k2_bs);
        param_get(param_find("DOBIBS_BS_K3"), &k3_bs);
        param_get(param_find("DOBIBS_BS_K4"), &k4_bs);
        param_get(param_find("DOBIBS_DOG_K"), &k_df);
    }
    void update();
    int parameters_update();

private:
    BlockIntegral _i_u;
    BlockIntegral _i_df_0;
    BlockIntegral _i_df_1;
    BlockIntegral _i_df_2;
    BlockIntegral _int_er_0;
    BlockIntegral _int_er_1;
    BlockIntegral _int_er_2;
    px4_pollfd_struct_t _fds[1];
    uint64_t _t;

    // params
    param_t switch_traj;


    // Integral Backstepping  Gains
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float k1_bs;
    float k2_bs;
    float k3_bs;
    float k4_bs;
    //Observer gain
    float k_df;

    float obs=0;
    float u=15.69;
    float u_dot=0;
    float d_zdf_0=0;
    float z_df_0=0;
    float d_zdf_1=0;
    float z_df_1=0;
    float d_zdf_2=0;
    float z_df_2=0;
    float traj_t=0;
    float T=20;
    int traj=0;
    float switch_time=0;

    int _switch_traj;
    matrix::Vector<float, 3>  pd;
    matrix::Vector<float, 3>  dpd;
    matrix::Vector<float, 3>  ddpd;
    matrix::Vector<float, 3>  dddpd;
    matrix::Vector<float, 3>  ddddpd;   
};
