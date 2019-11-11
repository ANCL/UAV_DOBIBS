#include "BlockTASKController.hpp"
#include <uORB/topics/battery_status.h>
void BlockTASKController::update()
{
        // wait for an image feature, timeout every 500 ms
        int poll_ret = px4_poll(_fds,(sizeof(_fds) / sizeof(_fds[0])), 500);// to check if

        if (poll_ret <= 0)
        {
                if (poll_ret == 0) warn("time out");
                else warn("poll error %d, %d", poll_ret, errno);
                _att_sp.get().valid=false;
        }
        else {
                uint64_t t1 = hrt_absolute_time();

                static uint64_t last_run = 0;
                float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
                last_run = hrt_absolute_time();

                _t=t1;
                //Set message timestamp
                _att_sp.get().timestamp = t1;
                _control_sp.get().timestamp = t1;
                // check for sane values of dt
                if (dt>1.0f || dt<0) {
                        warn("dt=%3.3f",(double)dt);
                        _att_sp.get().valid=false;
                } else {
                        // set dt for all child blocks
                        setDt(dt);

                        // check for new updates   //_param_update is a topic structure
                        if (_param_update.updated())
                        {
                                updateParams();
                        }
                        // get new information from subscriptions
                        updateSubscriptions();
                        
                        //initialization
                        float m = 1.6;
                        float g = 9.81;
                        float J1 = 0.005;//0.005;
                        float J2 = 0.0045;
                        float J3 = 0.08;


                        //variable state definition
                        matrix::Vector<float, 3>  p;
                        p(0)=_pos.get().x;
                        p(1)=_pos.get().y;
                        p(2)=_pos.get().z;

                        matrix::Vector<float, 3>  v;

                        v(0) = _pos.get().vx;
                        v(1) = _pos.get().vy;
                        v(2) = _pos.get().vz;

                        matrix::Eulerf euler = matrix::Quatf(_att.get().q);// converting quaternion to Euler angles
                        float phi =euler.phi();
                        float theta = euler.theta();
                        float psi = euler.psi();

                        matrix::Vector<float, 3>  w;
                        w(0) = _att.get().rollspeed;
                        w(1) = _att.get().pitchspeed;
                        w(2) = _att.get().yawspeed;

                        // matrix::Dcm<float> R_att(_att.get().q);// not working!

                        matrix::Matrix<float, 3, 3>  R;

                        R(0,0)=cos(psi)*cos(theta);
                        R(0,1)=-sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi);
                        R(0,2)=sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi);

                        R(1,0)=sin(psi)*cos(theta);
                        R(1,1)=cos(psi)*cos(phi)+sin(psi)*sin(theta)*sin(phi);
                        R(1,2)=-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi);

                        R(2,0)=-sin(theta);
                        R(2,1)=cos(theta)*sin(phi);
                        R(2,2)=cos(theta)*cos(phi);

                        float quad_input[4];

                        //vector and matrix definition
                        matrix::Matrix<float, 3, 3>  Ie3;
                        Ie3(0,0)=1;
                        Ie3(0,1)=0;
                        Ie3(0,2)=0;

                        Ie3(1,0)=0;
                        Ie3(1,1)=1;
                        Ie3(1,2)=0;

                        Ie3(2,0)=0;
                        Ie3(2,1)=0;
                        Ie3(2,2)=0;

                        matrix::Matrix<float, 3, 3>  Sw;
                        Sw(0,0)=0;
                        Sw(0,1)=-w(2);
                        Sw(0,2)=w(1);

                        Sw(1,0)=w(2);
                        Sw(1,1)=0;
                        Sw(1,2)=-w(0);

                        Sw(2,0)=-w(1);
                        Sw(2,1)=w(0);
                        Sw(2,2)=0;

                        matrix::Vector<float, 3>  n1;
                        n1(0)=1;
                        n1(1)=0;
                        n1(2)=0;

                        matrix::Vector<float, 3>  n2;
                        n2(0)=0;
                        n2(1)=1;
                        n2(2)=0;

                        matrix::Vector<float, 3>  n3;
                        n3(0)=0;
                        n3(1)=0;
                        n3(2)=1;

                        // desired trajectory

                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2 && traj==0){
                            traj=1;
                            traj_t=t1;
                            switch_time=t1;

                        }

                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1 && traj==1){
                            switch_time=t1;
                        }


                        float T_d=20.0;
                        if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1){
                            traj=0;
                            T=T_d;
                        }

                        float t=(t1-traj_t)/1000000.0f;
                        float A=1.50*traj;
                        float B=3.00*traj;
                        float pi=3.14;

                        if (traj==1 && T>12.0f){
                            T=T_d-t;
                        }
                        
                        matrix::Vector<float, 3>  pd;
                        matrix::Vector<float, 3>  dpd;
                        matrix::Vector<float, 3>  ddpd;
                        matrix::Vector<float, 3>  dddpd;
                        matrix::Vector<float, 3>  ddddpd;
                        if (_switch_traj == 0){
                            //  Fig 8
                            
                            pd(0)=A*sinf(2*pi*t/T);
                            pd(1)=B*sinf(4*pi*t/T)/4;
                            pd(2)=-0.85f;

                            dpd(0)=(2*A*pi*cosf((2*pi*t)/T))/T;
                            dpd(1)=(B*pi*cosf((4*pi*t)/T))/T;
                            dpd(2)=0;

                            
                            ddpd(0)=-(4*A*(pi*pi)*sinf((2*pi*t)/T))/(T*T);
                            ddpd(1)=-(4*B*(pi*pi)*sinf((4*pi*t)/T))/(T*T);
                            ddpd(2)=0;

                            
                            dddpd(0)=-(8*A*(pi*pi*pi)*cosf((2*pi*t)/T))/(T*T*T);
                            dddpd(1)=-(16*B*(pi*pi*pi)*cosf((4*pi*t)/T))/(T*T*T);
                            dddpd(2)=0;

                            
                            ddddpd(0)=(16*A*(pi*pi*pi*pi)*sinf((2*pi*t)/T))/(T*T*T*T);
                            ddddpd(1)=(64*B*(pi*pi*pi*pi)*sinf((4*pi*t)/T))/(T*T*T*T);
                            ddddpd(2)=0;
                        }else if (_switch_traj == 1){
                           // Circle traj
                           //matrix::Vector<float, 3>  pd;
                           pd(0)=A*sinf(2*pi*t/T);
                           pd(1)=B*cosf(2*pi*t/T);
                           pd(2)=-0.9f;

                           //matrix::Vector<float, 3>  dpd;
                           dpd(0)=2*pi*A*cosf(2*pi*t/T)/T;
                           dpd(1)=-2*pi*B*sinf(2*pi*t/T)/T;
                           dpd(2)=0;

                           //matrix::Vector<float, 3>  ddpd;
                           ddpd(0)=-4*(pi*pi)*A*sinf(2*pi*t/T)/(T*T);
                           ddpd(1)=-4*(pi*pi)*B*cosf(2*pi*t/T)/(T*T);
                           ddpd(2)=0;

                           //matrix::Vector<float, 3>  dddpd;
                           dddpd(0)=-8*A*(pi*pi*pi)*cosf(2*pi*t/T)/(T*T*T);
                           dddpd(1)=8*A*(pi*pi*pi)*sinf(2*pi*t/T)/(T*T*T);
                           dddpd(2)=0;

                           //matrix::Vector<float, 3>  ddddpd;
                           ddddpd(0)=16*A*(pi*pi*pi*pi)*sinf(2*pi*t/T)/(T*T*T*T);
                           ddddpd(1)=16*A*(pi*pi*pi*pi)*cosf(2*pi*t/T)/(T*T*T*T);
                           ddddpd(2)=0;
                        }else if (_switch_traj == 2){
                           //Setpoint 
                           //matrix::Vector<float, 3>  pd;
                           pd(0)=0.0f;
                           pd(1)=0.0f;
                           pd(2)=-0.85f;
                        

                           if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2)
                           {
                           pd(0)=1.0f;
                           pd(1)=0.0f;
                           pd(2)=-0.85f;}

                           //matrix::Vector<float, 3>  dpd;
                           dpd(0)=0;
                           dpd(1)=0;
                           dpd(2)=0;

                           //matrix::Vector<float, 3>  ddpd;
                           ddpd(0)=0;
                           ddpd(1)=0;
                           ddpd(2)=0;

                           //matrix::Vector<float, 3>  dddpd;
                           dddpd(0)=0;
                           dddpd(1)=0;
                           dddpd(2)=0;

                           //matrix::Vector<float, 3>  ddddpd;
                           ddddpd(0)=0;
                           ddddpd(1)=0;
                           ddddpd(2)=0;
                        }

                           // Integral Backstepping  Gains
                           float k1 = 4;
                           float k2 = 4;
                           float k3 = 4;
                           float k4 = 4;
                           float k5 = 4;

                           // Backstepping Gains
//                          float k1 =5;
//                          float k2 = 5;
//                          float k3 = 5;
//                          float k4 = 5;

                           //Observer gain
                           float k_df=0.5;

// ----------------------------- Disturbance Observer--------------------------------------------------
                             if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2)
                             {
                                obs=1;
                            }else{
                                    obs=0; // turn on the dis observer when switched to ANCL2
                                 }

                          /* matrix::Vector<float, 3>  dis=-(m*g*n3-m*g*R*n3);
                           if(get_counter()==23){
                                PX4_INFO("counter =0");

                                z_df_0=dis(0);
                                z_df_1=dis(1);
                                z_df_2=dis(2);
                             }*/ // initial value for the dis estimates


                           //
                        matrix::Vector<float, 3>  z_df;
                        z_df(0)=  z_df_0;
                        z_df(1)=  z_df_1;
                        z_df(2)=  z_df_2;
                        matrix::Vector<float, 3>  df_hat1=z_df+obs*k_df*m*v; // DOB equation

                        //Saturation of df_hat
                        if (df_hat1(0) > 2.0f){
                        df_hat1(0) = 2.0f;
                        } else if (df_hat1(0) < -2.0f) {
                                df_hat1(0) =-2.0f;
                        }

                        if (df_hat1(1) > 2.0f){
                           df_hat1(1) = 2.0f;
                        } else if (df_hat1(1) < -2.0f) {
                                df_hat1(1) =-2.0f;
                        }

                        if (df_hat1(2) > 7.0f){
                           df_hat1(2) = 7.0f;
                        } else if (df_hat1(2) < -7.0f) {
                                df_hat1(2) =-7.0f;
                        }
//----------------------------------------- Controller -----------------------------------

                          float u;
                          u = thrust_input;
                            matrix::Vector<float, 3> er= p - pd;


                            matrix::Vector<float, 3>  df_hat=df_hat1;//z_df+k_df*m*v;

//-------------------------------Intergal backstepping----------------------------------------

                            matrix::Vector<float, 3> delta1;

                            delta1(0)=_int_er_0.update(er(0));
                            delta1(1)=_int_er_1.update(er(1));
                            delta1(2)=_int_er_2.update(er(2));

                            matrix::Vector<float, 3> delta1_dot=p-pd;

                            matrix::Vector<float, 3> alpha1=-k1*delta1+pd;

                            matrix::Vector<float, 3> delta2=p-alpha1;

                            matrix::Vector<float, 3> delta2_dot=v+k1*(p-pd)-dpd;

                            matrix::Vector<float, 3> alpha2=-delta1-k1*(p-pd)+dpd-k2*delta2;

                            matrix::Vector<float, 3> delta3=m*v-m*alpha2;

                            matrix::Vector<float, 3> alpha3=delta2/m+k3*delta3+m*g*n3+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot;

                            matrix::Vector<float, 3>  delta4=alpha3-u*R*n3;

                            matrix::Vector<float, 3> beta=delta2_dot/m+k3*(m*g*n3-u*R*n3+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot)+
                             +m*(v-dpd)+k1*(m*g*n3-u*R*n3+df_hat-m*ddpd)-m*dddpd+k2*(m*g*n3-u*R*n3+df_hat+m*k1*(v-dpd)-m*ddpd);

                            matrix::Matrix<float, 1 , 1>  udot=n3.T()*R.T()*(beta+delta3+k4*delta4);

                            float  u_dot=udot(0,0);
                             thrust_input = _iop.update(u_dot);

                             intu=_u_int.update(p(2)-pd(2));


                           // matrix::Vector<float, 3> alpha4=R*(eye(3)-n3*n3.T())*R.T()*(beta+delta3+k4*delta4);

                            matrix::Vector<float, 3> alpha4=R* Ie3*R.T()*(beta+delta3+k4*delta4);

                            matrix::Vector<float, 3> delta5=alpha4-u*R*Sw*n3;

                            matrix::Vector<float, 3> beta_dot_hat1=(m*g*n3-u*R*n3+df_hat+m*k1*(v-dpd)-m*ddpd)/(m*m)+
                               +k3*(-u_dot*R*n3-u*R*Sw*n3+m*(v-dpd)+k1*(m*g*n3-u*R*n3+df_hat-m*ddpd)
                                    -m*dddpd+k2*(m*g*n3-u*R*n3+df_hat+m*k1*(v-dpd)-m*ddpd));

                            matrix::Vector<float, 3> beta_dot_hat2=(m*g*n3-u*R*n3+df_hat-m*ddpd)+k1*(-u_dot*R*n3-u*R*Sw*n3-m*dddpd)-m*ddddpd+
                                        k2*(-u_dot*R*n3-u*R*Sw*n3+k1*(m*g*n3-u*R*n3+df_hat-m*ddpd)-m*dddpd);

                            matrix::Vector<float, 3> beta_dot_hat=beta_dot_hat1+beta_dot_hat2;

                            matrix::Vector<float, 3>  delta3_dot_hat=m*g*n3-u*R*n3+df_hat+m*delta1_dot+m*k1*(v-dpd)-m*ddpd+m*k2*delta2_dot;

                            matrix::Vector<float, 3>  delta4_dot_hat=beta-u_dot*R*n3-u*R*Sw*n3;

                            matrix::Vector<float, 3>  alpha4_dot_hat=R*Sw*Ie3*R.T()*(beta+delta3+k4*delta4)+
                                R*Ie3*Sw.T()*R.T()*(beta+delta3+k4*delta4)+
                                R*Ie3*R.T()*(beta_dot_hat+delta3_dot_hat+k4*delta4_dot_hat);

                           matrix::Vector<float, 3> wddot=alpha4_dot_hat - u_dot*R*Sw*n3 + delta4 +k5*delta5-u*R*Sw*Sw*n3;

                           matrix::Matrix<float, 1 , 1>  wddot0;
                           wddot0= -(n2.T()*R.T())*(wddot)/u;

                           matrix::Matrix<float, 1 , 1>  wddot1;
                           wddot1= (n1.T()*R.T())*(wddot)/u;


                        // Backstepping controller

//                          matrix::Vector<float, 3> delta_1 = p - pd;

//                          matrix::Vector<float, 3> alpha_1= dpd-k1 * delta_1;

//                          matrix::Vector<float, 3> delta_2 = m * v - m* alpha_1;

//                          matrix::Vector<float, 3> alpha_2= m*g*n3 +df_hat-m*ddpd+m*k1*v-m*k1*dpd + delta_1/m + k2 * delta_2;

//                          matrix::Vector<float, 3> delta_3 = alpha_2 - u*R*n3;

//                          matrix::Vector<float, 3> beta1=-m*dddpd+k1*m*g*n3-k1*u*R*n3+k1*df_hat-m*k1*ddpd+v/m-dpd/m;

//                         matrix::Vector<float, 3> beta2=k2*m*g*n3-k2*u*R*n3+k2*df_hat-k2*m*ddpd+k2*m*k1*v-k2*m*k1*dpd;

//                         matrix::Vector<float, 3> beta=beta1+beta2;

//                         matrix::Vector<float, 3> s=beta+delta_2+k3*delta_3;

//                         matrix::Matrix<float, 3, 3> RIRT = R*Ie3*R.T();// hard coding this termS

//                         matrix::Vector<float, 3> uRSwn3 =  u*R*Sw*n3;// hard code ....

//                         matrix::Vector<float, 3> delta_4 = RIRT*s - uRSwn3;

//                         matrix::Matrix<float, 1 , 1>  udot= n3.T()*R.T()*s;

//                         float u_dot;
//                           u_dot=udot(0,0);
//                          thrust_input = _iop.update(u_dot);

//                         intu=_u_int.update(p(2)-pd(2));

//                         matrix::Vector<float, 3> beta_dot1=-m*ddddpd-k1*u_dot*R*n3-k1*uRSwn3-m*k1*dddpd+(m*g*n3-u*R*n3+df_hat)/(m*m)-ddpd/m-k2*u_dot*R*n3;

//                         matrix::Vector<float, 3> beta_dot2=-k2*uRSwn3-k2*m*dddpd+k2*k1*(m*g*n3-u*R*n3+df_hat)-k1*k2*m*ddpd;

//                         matrix::Vector<float, 3> delta_2_3_dot=m*g*n3-u*R*n3+df_hat-m*ddpd+m*k1*v-m*k1*dpd+k3*(beta-u_dot*R*n3-uRSwn3);//delta2_dot+k3*delta_3_dot

//                         matrix::Vector<float, 3> alpha_3_dot=R*Sw*Ie3*R.T()*s+R*Ie3*Sw.T()*(R.T())*s+RIRT*(beta_dot1+beta_dot2+delta_2_3_dot);

//                         matrix::Vector<float, 3> wddot=alpha_3_dot-u_dot*R*Sw*n3-u*R*Sw*Sw*n3+delta_3+k4*(delta_4);

//                         matrix::Matrix<float, 1 , 1>  wddot0;
//                         wddot0= -(n2.T()*R.T())*(wddot)/u;

//                         matrix::Matrix<float, 1 , 1>  wddot1;
//                         wddot1= (n1.T()*R.T())*(wddot)/u;

                          // DOB integral action

                          matrix::Vector<float, 3>  h=obs*(-k_df*(df_hat1)-k_df*(m*g*n3-u*R*n3));//-------------------------------------------------

                          if (  z_df_0 > 1.5f && h(0) >0){
                                  d_zdf_0 =0.0f;
                          } else if (  z_df_0 < -1.5f &&  h(0) < 0){
                                  d_zdf_0 =0.0f;
                          }else{
                                  d_zdf_0 = h(0);
                          }
                           z_df_0 = i_df_0.update( d_zdf_0);

                          if (  z_df_1 > 1.5f && h(1) >0){
                                  d_zdf_1 =0.0f;
                          } else if (  z_df_1 < -1.5f &&  h(1) < 0){
                                  d_zdf_1 =0.0f;
                          }else{
                                  d_zdf_1 = h(1);
                          }
                           z_df_1 = i_df_1.update( d_zdf_1);

                          if (  z_df_2 > 6.0f && h(2) >0){
                                  d_zdf_2 =0.0f;
                          } else if (  z_df_2 < -6.0f &&  h(2) < 0){
                                  d_zdf_2 =0.0f;
                          }else{
                                  d_zdf_2 = h(2);
                          }
                           z_df_2 = i_df_2.update( d_zdf_2);

                          // Yaw control
                          float k_psi_1=1.5;
                          float k_psi_2=2.3;
                          float psi_d=0;
                          float psi_d_dot=0;
                          float psi_d_ddot=0;
                          float ep_1=psi-psi_d;
                          float alpha_psi_1=psi_d_dot-k_psi_1*ep_1;

                           float ep_2=w(2)-alpha_psi_1;
                           float cos_theta= cos(theta);
                           float sin_theta= sin(theta);
                           float sin_phi=sin(phi);
                           float cos_phi=cos(phi);
                           float phi_dot=w(0);
                           float theta_dot=w(1);

                           float n3Wdot=(cos_phi*phi_dot/cos_theta+sin_phi/(cos_theta*cos_theta)*sin_theta*theta_dot)*w(1)+(-sin_phi*phi_dot/cos_theta+cos_phi/(cos_theta*cos_theta)*sin_theta*theta_dot)*w(2);
                           float alpha_psi_dot=psi_d_ddot-k_psi_1*(w(2)-psi_d_dot);
                           float wddot3;

                         wddot3= cos_theta*(alpha_psi_dot-ep_1-k_psi_2*ep_2-n3Wdot-sin_phi* wddot1(0,0)/cos_theta)/cos_phi;


                         // Adding robusness terms (pid) to torques
                        // matrix::Vector<float, 3> er=(p-pd);

                         matrix::Vector<float, 3> eb=3.0f*R.T()*(p-pd);

                         matrix::Vector<float, 3> ev=0.5f*R.T()*(v-dpd)+0.2f*eb;

                         // reseting the integral terms
                          if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2){
                          _iw0.setzero();
                          _iw1.setzero();
                          _iw2.setzero();
                          _u_int.setzero();
                          _int_er_0.setzero();
                          _int_er_1.setzero();
                          _int_er_2.setzero();
                          _u_int.setzero();
                          _iop.setzero();
                           }

                         if (intw0 > 0.5f && eb(1) >0){
                                 iw0 =0.0f;
                         } else if (  intw0 < -0.5f &&  eb(1) < 0){
                                 iw0 =0.0f;
                         }else{
                               iw0=eb(1);
                         }
                           intw0= _iw0.update( iw0)+ev(1);



                         if ( intw1 > 0.5f && eb(0) >0){
                                 iw1 =0.0f;
                         } else if (  intw1 < -0.5f &&  eb(0) < 0){
                                 iw1=0.0f;
                         }else{
                               iw1=eb(0);
                         }
                            intw1= _iw1.update( iw1)+ev(0);


                         float epsi_p_v=ep_1+(w(2)-psi_d_dot);
                         if (  intw2 > 0.5f && ep_1 >0){
                                 iw2 =0.0f;
                         } else if (  intw2 < -0.5f && ep_1 < 0){
                                 iw2 =0.0f;
                         }else{
                               iw2=ep_1;
                         }
                           intw2= _iw2.update( iw2)+epsi_p_v;

                         // Saturating the intergal terms
                         float intw0_sat;
                         if (intw0 > 0.70f){
                         intw0_sat = 0.70f;
                         } else if (intw0 < -0.70f) {
                                 intw0_sat =-0.70f;
                         }
                         else{
                             intw0_sat=intw0;
                         }

                         float intw1_sat;
                         if (intw1 > 0.7f){
                         intw1_sat = 0.7f;
                         } else if (intw1 < -0.7f) {
                                 intw1_sat =-0.7f;
                         }
                         else{
                             intw1_sat=intw1;
                         }

                         float intw2_sat;
                         if (intw2 > 0.7f){
                         intw2_sat = 0.7f;
                         } else if (intw2 < -0.7f) {
                                 intw2_sat =-0.7f;
                         }
                         else{
                             intw2_sat=intw2;
                         }


// reseting the saturation variables
                         if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1){
                           //  res=0;
                             intw0_sat=0.0f;
                             intw1_sat=0.0f;
                             intw2_sat=0.0f;
                         }

                         // angular velocity derivative calculation for adding to torque as robustness terms
                         float dw0=0.008f*(prev_rates(0) - w(0)) /(1.0f* dt);

                         if (dw0 > 0.48f){
                         dw0 = 0.48f;
                         } else if (dw0 < -0.48f) {
                                 dw0 =-0.48f;
                         }

                         float dw1=0.006f*(prev_rates(1) - w(1)) /(1.0f* dt);
                         if (dw1 > 0.65f){
                         dw1 = 0.65f;
                         } else if (dw1 < -0.65f) {
                                 dw1 =-0.65f;
                         }

                         // gains for adding pid to the thrust
                         float ki_u=4.5;
                         float kv_u=4.5;
                         float kp_u=8.1;

                         quad_input[0] =J1*wddot0(0,0)+dw0-0.007f*(intw0_sat);//-0.007f
                         quad_input[1] =J2*wddot1(0,0)+dw1+0.025f*(intw1_sat);//+0.007f
                         quad_input[2] =J3*wddot3+0.02f*intw2_sat;//(w(0)-prev_rates2(0))/(dt*2.0f)+0.0f*J3*wddot3;
                         quad_input[3] = thrust_input+4.0f*(ki_u*intu+kv_u*(v(2)-dpd(2))+kp_u*(p(2)-pd(2)));

                           prev_rates2(0) =  prev_rates(0) ;
                           prev_rates2(1) =  prev_rates(1) ;
                           prev_rates2(2) =  prev_rates(2) ;

                           prev_rates(0) = w(0);
                           prev_rates(1) = w(1);
                           prev_rates(2) = w(2);

                         // normalization and saturation of thrust and torques
                        float nor_thrust=(quad_input[3]-15.68f)/110.0f+0.63f;
                        float sat_thrust;

                        if (nor_thrust > 0.7f) {
                        sat_thrust = 0.7f;

                        } else if (nor_thrust < 0.25f) {
                                sat_thrust = 0.25f;
                        }
                        else{
                            sat_thrust=nor_thrust;
                        }

                        if (quad_input[0] > 0.21f) {
                        quad_input[0] = 0.21f;

                        } else if (quad_input[0] < -0.21f) {
                                quad_input[0] = -0.21f;
                        }

                        if (quad_input[1] > 0.14f) {
                        quad_input[1] = 0.14f;

                        } else if (quad_input[1] < -0.14f) {
                                quad_input[1] = -0.14f;
                        }

                        if (quad_input[2] > 0.15f) {
                        quad_input[2] = 0.15f;

                        } else if (quad_input[2] < -0.15f) {
                                quad_input[2] = -0.15f;
                        }

                        // publishing the controls to the secondary control setpoint
                   _control_sp.get().control[0] = quad_input[0];
                   _control_sp.get().control[1] = quad_input[1];
                   _control_sp.get().control[2] = quad_input[2];
                   _control_sp.get().control[3] = sat_thrust;




//                   if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL1){
//                    test=integ_test.update(1.0f);
//                   }
//                   if (_status.get().nav_state == vehicle_status_s::NAVIGATION_STATE_ANCL2){
//                   test=integ_test.setzero();
//                   }


                    // publishing data for logging
                         _att_sp.get().roll   = phi;
                         _att_sp.get().pitch  = theta;
                         _att_sp.get().yaw    = psi;
                         _att_sp.get().thrust = thrust_input;
                         _att_sp.get().valid  = true;

                         _att_sp.get().er[0] =er(0);
                         _att_sp.get().er[1] =er(1);
                         _att_sp.get().er[2] =er(2);
                         _att_sp.get().er[3] =ep_1;

                         _att_sp.get().pd[0] =pd(0);
                         _att_sp.get().pd[1] =pd(1);
                         _att_sp.get().pd[2] =pd(2);

                         _att_sp.get().p[0] =p(0);
                         _att_sp.get().p[1] =p(1);
                         _att_sp.get().p[2] =p(2);

                          _att_sp.get().df[0] =df_hat(0);
                          _att_sp.get().df[1] =df_hat(1);
                          _att_sp.get().df[2] =df_hat(2);

                          _att_sp.get().st =switch_time;

        }
}
        //update all publications
        updatePublications();

}
