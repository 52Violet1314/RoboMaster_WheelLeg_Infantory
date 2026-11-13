/* Pure-C WheelLeg simulator: MuJoCo + VMC + generated explicit MPC. */
#include <mujoco/mujoco.h>
#include "explicit_mpc.h"
#include "ground_detection.h"
#include "linkage_table.h"
#include <windows.h>
#include <GL/gl.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define G 9.81
#define WHEEL_R .060
#define LEG_NOMINAL .1339
#define LEG_MIN .090
#define LEG_MAX .450
#define HIP_VERTICAL .052942
#define WHEEL_LIMIT 7.0
#define HIP_LIMIT 10.0
#define COMPLIANT_HIP_LIMIT 2.0
#define FORCE_LIMIT 200.0
#define SCOPE_POINTS 500
#define SCOPE_DECIMATE 10
#define DERIVATIVE_FILTER_A .15
#define SUPPORT_LINES 6

typedef struct { double kp,ki,kd,out,ilim,i,last; int valid; } PID;
typedef struct { int hip[2],wheel[2],slide[2],awheel[2],ahip[2],aleg[2],bhip[2],bwheel[2],gwheel[2],bbase; } Ids;
typedef struct {
  double yaw,pitch,roll,om[3],len[2],theta[2],wheel_vel[2];
} Meas;
typedef struct {
  int count;
  float time[SCOPE_POINTS], x[10][SCOPE_POINTS], u[4][SCOPE_POINTS];
  float support[SUPPORT_LINES][SCOPE_POINTS];
} Scope;
enum { LINK_FRONT1,LINK_FRONT2,LINK_FRONT3,LINK_FRONT4,LINK_REAR1,
       LINK_REAR2,LINK_REAR2_GUIDE,LINK_SPRING1,LINK_SPRING2,LINK_COUNT };
typedef struct {
  int enabled, geom[2][LINK_COUNT];
  double offset_pos[2][LINK_COUNT][3];
  double offset_rot[2][LINK_COUNT][3][3];
  double angles[2][8];
} LinkageVisual;
typedef struct {
  mjModel* m; mjData* d; Ids id; WheelLegMPC mpc; PID length_pid[2], anti;
  double dt, target_len, speed, target_speed, turn_rate, target_yaw, f0, pos, pos_i, last_theta[2];
  double last_len[2], d_len[2], dd_len[2], d_theta[2], dd_theta[2], last_z_velocity;
  double body_accel_down, leg_force_cmd[2], spring_force[2], tp_cmd[2], contact_fn[2];
  int theta_valid, derivative_valid, rear_pose_active, ticks; float x[10],u[4]; Scope scope; LinkageVisual visual;
  GroundDetector ground_detector;
  GroundDetectionOutput ground[2];
} Ctrl;

static double clip(double x,double lo,double hi) { return x<lo?lo:(x>hi?hi:x); }
static double wrap(double a) { return atan2(sin(a),cos(a)); }
static void pid_reset(PID* p) { p->i=0; p->last=0; p->valid=0; }
static double pid(PID* p,double ref,double val,double dt) {
  double e=ref-val, d=p->valid?(val-p->last)/dt:0;
  p->i=clip(p->i+e*dt,-p->ilim,p->ilim); p->last=val; p->valid=1;
  return clip(p->kp*e+p->ki*p->i-p->kd*d,-p->out,p->out);
}
static int getid(const mjModel* m,mjtObj type,const char* n) {
  int id=mj_name2id(m,type,n); if(id<0) fprintf(stderr,"Missing model object: %s\n",n); return id;
}
static int ids(const mjModel* m,Ids* v) {
  const char* s[2]={"left","right"}; char n[48];
  v->bbase=getid(m,mjOBJ_BODY,"base_link"); if(v->bbase<0) return 0;
  for(int k=0;k<2;k++) {
    snprintf(n,sizeof(n),"%s_hip_joint",s[k]); v->hip[k]=getid(m,mjOBJ_JOINT,n);
    snprintf(n,sizeof(n),"%s_wheel_joint",s[k]); v->wheel[k]=getid(m,mjOBJ_JOINT,n);
    snprintf(n,sizeof(n),"%s_legslide_joint",s[k]); v->slide[k]=getid(m,mjOBJ_JOINT,n);
    snprintf(n,sizeof(n),"%s_wheel_motor",s[k]); v->awheel[k]=getid(m,mjOBJ_ACTUATOR,n);
    snprintf(n,sizeof(n),"%s_hip_motor",s[k]); v->ahip[k]=getid(m,mjOBJ_ACTUATOR,n);
    snprintf(n,sizeof(n),"%s_leg_motor",s[k]); v->aleg[k]=getid(m,mjOBJ_ACTUATOR,n);
    snprintf(n,sizeof(n),"%s_leg",s[k]); v->bhip[k]=getid(m,mjOBJ_BODY,n);
    snprintf(n,sizeof(n),"%s_wheel",s[k]); v->bwheel[k]=getid(m,mjOBJ_BODY,n);
    snprintf(n,sizeof(n),"%s_wheel_geom",s[k]); v->gwheel[k]=getid(m,mjOBJ_GEOM,n);
    if(v->hip[k]<0||v->wheel[k]<0||v->slide[k]<0||v->awheel[k]<0||v->ahip[k]<0||
       v->aleg[k]<0||v->bhip[k]<0||v->bwheel[k]<0||v->gwheel[k]<0) return 0;
  } return 1;
}
static void quat_matrix(const mjtNum*q,double r[3][3]) {
  double w=q[0],x=q[1],y=q[2],z=q[3];
  r[0][0]=1-2*(y*y+z*z); r[0][1]=2*(x*y-w*z); r[0][2]=2*(x*z+w*y);
  r[1][0]=2*(x*y+w*z); r[1][1]=1-2*(x*x+z*z); r[1][2]=2*(y*z-w*x);
  r[2][0]=2*(x*z-w*y); r[2][1]=2*(y*z+w*x); r[2][2]=1-2*(x*x+y*y);
}
static void matmul3(const double a[3][3],const double b[3][3],double out[3][3]) {
  for(int i=0;i<3;i++)for(int j=0;j<3;j++){out[i][j]=0;for(int k=0;k<3;k++)out[i][j]+=a[i][k]*b[k][j];}
}
static void rotation_y(double a,double r[3][3]) {
  double c=cos(a),s=sin(a);double t[3][3]={{c,0,s},{0,1,0},{-s,0,c}};memcpy(r,t,sizeof(t));
}
static void matrix_quat(const double r[3][3],mjtNum*q) {
  double s,tr=r[0][0]+r[1][1]+r[2][2];
  if(tr>0){s=sqrt(tr+1)*2;q[0]=.25*s;q[1]=(r[2][1]-r[1][2])/s;q[2]=(r[0][2]-r[2][0])/s;q[3]=(r[1][0]-r[0][1])/s;}
  else if(r[0][0]>r[1][1]&&r[0][0]>r[2][2]){s=sqrt(1+r[0][0]-r[1][1]-r[2][2])*2;q[0]=(r[2][1]-r[1][2])/s;q[1]=.25*s;q[2]=(r[0][1]+r[1][0])/s;q[3]=(r[0][2]+r[2][0])/s;}
  else if(r[1][1]>r[2][2]){s=sqrt(1+r[1][1]-r[0][0]-r[2][2])*2;q[0]=(r[0][2]-r[2][0])/s;q[1]=(r[0][1]+r[1][0])/s;q[2]=.25*s;q[3]=(r[1][2]+r[2][1])/s;}
  else{s=sqrt(1+r[2][2]-r[0][0]-r[1][1])*2;q[0]=(r[1][0]-r[0][1])/s;q[1]=(r[0][2]+r[2][0])/s;q[2]=(r[1][2]+r[2][1])/s;q[3]=.25*s;}
  double n=sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);for(int i=0;i<4;i++)q[i]/=n;if(q[0]<0)for(int i=0;i<4;i++)q[i]=-q[i];
}
static void rotate2(const double v[2],double a,double out[2]) {double c=cos(a),s=sin(a);out[0]=c*v[0]+s*v[1];out[1]=-s*v[0]+c*v[1];}
typedef struct {double pos[LINK_COUNT][2],angle[LINK_COUNT];} LinkPoses;
static void linkage_forward(const double q[8],LinkPoses*p) {
  static const double f12[2]={.112650,.013046},f23[2]={-.108790,-.079941},f34[2]={-.165050,.113860};
  static const double r12[2]={-.210000,0},r1s[2]={.0023551,-.0449380},s12[2]={-.0329500,.0044022},guide[2]={-.037468,-.014006};
  double v[2],a;
  p->angle[LINK_FRONT1]=q[0];
  rotate2(f12,p->angle[LINK_FRONT1],v);p->pos[LINK_FRONT2][0]=v[0];p->pos[LINK_FRONT2][1]=v[1];p->angle[LINK_FRONT2]=q[0]+q[1];
  rotate2(f23,p->angle[LINK_FRONT2],v);p->pos[LINK_FRONT3][0]=p->pos[LINK_FRONT2][0]+v[0];p->pos[LINK_FRONT3][1]=p->pos[LINK_FRONT2][1]+v[1];p->angle[LINK_FRONT3]=p->angle[LINK_FRONT2]+q[2];
  rotate2(f34,p->angle[LINK_FRONT3],v);p->pos[LINK_FRONT4][0]=p->pos[LINK_FRONT3][0]+v[0];p->pos[LINK_FRONT4][1]=p->pos[LINK_FRONT3][1]+v[1];p->angle[LINK_FRONT4]=p->angle[LINK_FRONT3]+q[3];
  p->angle[LINK_REAR1]=q[4];rotate2(r12,q[4],v);p->pos[LINK_REAR2][0]=v[0];p->pos[LINK_REAR2][1]=v[1];p->angle[LINK_REAR2]=q[4]+q[5];
  rotate2(guide,p->angle[LINK_REAR2],v);p->pos[LINK_REAR2_GUIDE][0]=p->pos[LINK_REAR2][0]+v[0];p->pos[LINK_REAR2_GUIDE][1]=p->pos[LINK_REAR2][1]+v[1];p->angle[LINK_REAR2_GUIDE]=p->angle[LINK_REAR2];
  rotate2(r1s,q[4],v);p->pos[LINK_SPRING1][0]=v[0];p->pos[LINK_SPRING1][1]=v[1];p->angle[LINK_SPRING1]=q[4]+q[6];
  rotate2(s12,p->angle[LINK_SPRING1],v);p->pos[LINK_SPRING2][0]=p->pos[LINK_SPRING1][0]+v[0];p->pos[LINK_SPRING2][1]=p->pos[LINK_SPRING1][1]+v[1];a=p->angle[LINK_SPRING1]+.132820;
  {double slide[2]={q[7],0};rotate2(slide,a,v);p->pos[LINK_SPRING2][0]-=v[0];p->pos[LINK_SPRING2][1]-=v[1];}p->angle[LINK_SPRING2]=a;
}
static int linkage_visual_init(Ctrl*c) {
  static const char*links[]={"front1","front2","front3","front4","rear1","rear2","rear2_guide","spring1","spring2"};
  LinkageVisual*v=&c->visual;memset(v,0,sizeof(*v));for(int s=0;s<2;s++)for(int l=0;l<LINK_COUNT;l++)v->geom[s][l]=-1;
  for(int g=0;g<c->m->ngeom;g++){int mesh=c->m->geom_dataid[g];const char*n=mesh>=0?mj_id2name(c->m,mjOBJ_MESH,mesh):NULL;if(!n)continue;for(int s=0;s<2;s++)for(int l=0;l<LINK_COUNT;l++){char wanted[64];snprintf(wanted,sizeof(wanted),"%s_%s_link_mesh",s?"right":"left",links[l]);if(!strcmp(n,wanted))v->geom[s][l]=g;}}
  for(int s=0;s<2;s++)for(int l=0;l<LINK_COUNT;l++)if(v->geom[s][l]<0)return 0;
  double zero[8]={0};LinkPoses rest={0};linkage_forward(zero,&rest);
  for(int side=0;side<2;side++)for(int link=0;link<LINK_COUNT;link++){int g=v->geom[side][link];double ry[3][3],gr[3][3];rotation_y(rest.angle[link],ry);quat_matrix(c->m->geom_quat+4*g,gr);double diff[3]={c->m->geom_pos[3*g]-rest.pos[link][0],c->m->geom_pos[3*g+1],c->m->geom_pos[3*g+2]-rest.pos[link][1]};for(int i=0;i<3;i++){v->offset_pos[side][link][i]=0;for(int j=0;j<3;j++)v->offset_pos[side][link][i]+=ry[j][i]*diff[j];for(int j=0;j<3;j++){v->offset_rot[side][link][i][j]=0;for(int k=0;k<3;k++)v->offset_rot[side][link][i][j]+=ry[k][i]*gr[k][j];}}}
  v->enabled=1;return 1;
}
static void interpolate_linkage_angles(double slide,double q[8]) {
  slide=clip(slide,linkage_slides[0],linkage_slides[LINKAGE_TABLE_COUNT-1]);int hi=1;while(hi<LINKAGE_TABLE_COUNT-1&&linkage_slides[hi]<slide)hi++;int lo=hi-1;double f=(slide-linkage_slides[lo])/(linkage_slides[hi]-linkage_slides[lo]);for(int i=0;i<8;i++)q[i]=(1-f)*linkage_angles[lo][i]+f*linkage_angles[hi][i];
}
static void update_linkage_visual(Ctrl*c) {
  if(!c->visual.enabled)return;
  for(int side=0;side<2;side++){double q[8];interpolate_linkage_angles(c->d->qpos[c->m->jnt_qposadr[c->id.slide[side]]],q);memcpy(c->visual.angles[side],q,sizeof(q));LinkPoses poses={0};linkage_forward(q,&poses);for(int link=0;link<LINK_COUNT;link++){int g=c->visual.geom[side][link];double ry[3][3],rot[3][3];rotation_y(poses.angle[link],ry);for(int i=0;i<3;i++){c->m->geom_pos[3*g+i]=(i==0?poses.pos[link][0]:(i==2?poses.pos[link][1]:0));for(int j=0;j<3;j++)c->m->geom_pos[3*g+i]+=ry[i][j]*c->visual.offset_pos[side][link][j];}matmul3(ry,c->visual.offset_rot[side][link],rot);matrix_quat(rot,c->m->geom_quat+4*g);}}
}
static double trim(double l) {
  static const double a[]={.09,.1339,.15,.20,.25,.30,.35,.45};
  static const double b[]={.040,.040,.037,.027,.0205,.017,.0145,.012};
  if(l<=a[0]) return b[0]; for(int i=1;i<8;i++) if(l<=a[i]) return b[i-1]+(l-a[i-1])*(b[i]-b[i-1])/(a[i]-a[i-1]); return b[7];
}
/* Exact continuous fit used by app_leg_force_compensation.cpp.  The returned
   value is the motor compensation (negative = retract); the passive spring is
   the equal and opposite positive extension force. */
static double leg_spring_compensation(double length) {
  if(!isfinite(length)||length<.130)return 0;
  if(length>.380)length=.380;
  if(length<.140) {
    double t=(length-.130)/.010,h01=-2*t*t*t+3*t*t,h11=t*t*t-t*t;
    return -28*h01+(.010*-222.33253)*h11;
  }
  double z=(length-.260)/.120;
  double base=((((.83489717*z-6.16044808)*z+2.88790652)*z+.91684180)*z-36.96640998);
  if(length<=.300)return base;
  double t=(length-.300)/.080,s=6*t*t*t*t*t-15*t*t*t*t+10*t*t*t;
  return base-2.5*s;
}
static void measure(const Ctrl*c,Meas*o) {
  double r[3][3],w[3],cy,sy; quat_matrix(c->d->qpos+3,r);
  o->yaw=atan2(r[1][0],r[0][0]); o->pitch=asin(clip(-r[2][0],-1,1)); o->roll=atan2(r[2][1],r[2][2]);
  for(int i=0;i<3;i++) w[i]=r[i][0]*c->d->qvel[3]+r[i][1]*c->d->qvel[4]+r[i][2]*c->d->qvel[5];
  cy=cos(o->yaw); sy=sin(o->yaw); o->om[0]=cy*w[0]+sy*w[1]; o->om[1]=-sy*w[0]+cy*w[1]; o->om[2]=w[2];
  for(int k=0;k<2;k++) {
    const mjtNum*h=c->d->xpos+3*c->id.bhip[k],*q=c->d->xpos+3*c->id.bwheel[k];
    double dx=q[0]-h[0],dy=q[1]-h[1],dz=q[2]-h[2];
    o->len[k]=sqrt(dx*dx+dy*dy+dz*dz); o->theta[k]=atan2(cy*dx+sy*dy,-dz);
    o->wheel_vel[k]=c->d->qvel[c->m->jnt_dofadr[c->id.wheel[k]]];
  }
}
static void update_derivatives(Ctrl*c,const Meas*q) {
  const double z_velocity=c->d->qvel[2];
  if(!c->derivative_valid) {
    for(int k=0;k<2;k++) {
      c->last_len[k]=q->len[k];c->last_theta[k]=q->theta[k];
      c->d_len[k]=c->dd_len[k]=c->d_theta[k]=c->dd_theta[k]=0;
    }
    c->last_z_velocity=z_velocity;c->body_accel_down=0;
    c->derivative_valid=1;c->theta_valid=1;return;
  }
  for(int k=0;k<2;k++) {
    double d_len=(q->len[k]-c->last_len[k])/c->dt;
    double d_theta=wrap(q->theta[k]-c->last_theta[k])/c->dt;
    double dd_len_raw=(d_len-c->d_len[k])/c->dt;
    double dd_theta_raw=(d_theta-c->d_theta[k])/c->dt;
    c->dd_len[k]+=DERIVATIVE_FILTER_A*(dd_len_raw-c->dd_len[k]);
    c->dd_theta[k]+=DERIVATIVE_FILTER_A*(dd_theta_raw-c->dd_theta[k]);
    c->d_len[k]=d_len;c->d_theta[k]=d_theta;
    c->last_len[k]=q->len[k];c->last_theta[k]=q->theta[k];
  }
  /* MuJoCo free-joint translational velocity is world +Z upward. */
  {
    double accel_down_raw=-(z_velocity-c->last_z_velocity)/c->dt;
    c->body_accel_down+=DERIVATIVE_FILTER_A*(accel_down_raw-c->body_accel_down);
    c->last_z_velocity=z_velocity;
  }
}
static void state(Ctrl*c,const Meas*q) {
  double vx=WHEEL_R*.5*(q->wheel_vel[0]+q->wheel_vel[1]);
  c->pos+=(vx-c->target_speed)*c->dt;
  c->x[0]=c->pos; c->x[1]=vx-c->target_speed; c->x[2]=wrap(q->yaw-c->target_yaw); c->x[3]=q->om[2];
  c->x[4]=-(q->theta[0]-trim(c->target_len)); c->x[5]=-c->d_theta[0];
  c->x[6]=-(q->theta[1]-trim(c->target_len)); c->x[7]=-c->d_theta[1]; c->x[8]=q->pitch; c->x[9]=q->om[1];
}
static void measure_contact_support(Ctrl*c) {
  c->contact_fn[0]=c->contact_fn[1]=0;
  for(int i=0;i<c->d->ncon;i++) {
    const mjContact* contact=&c->d->contact[i];
    for(int side=0;side<2;side++) {
      int other=-1;
      if(contact->geom1==c->id.gwheel[side]) other=contact->geom2;
      else if(contact->geom2==c->id.gwheel[side]) other=contact->geom1;
      if(other<0 || c->m->geom_bodyid[other]!=0) continue;
      mjtNum force[6]={0};mj_contactForce(c->m,c->d,i,force);
      if(force[0]>0)c->contact_fn[side]+=force[0];
    }
  }
}
static void update_ground_detection(Ctrl*c,const Meas*q) {
  measure_contact_support(c);
  for(int side=0;side<2;side++) {
    GroundDetectionInput in;
    /* Same reconstruction as the STM32 detector: motor command + passive
       gas-spring force removes the internal compensation pair. */
    in.f0_without_spring_n=c->leg_force_cmd[side]+c->spring_force[side];
    in.tp_vmc_nm=c->tp_cmd[side];
    in.leg_length_m=q->len[side];
    in.leg_velocity_mps=c->d_len[side];
    in.leg_accel_mps2=c->dd_len[side];
    in.body_accel_down_mps2=c->body_accel_down;
    in.leg_angle_rad=q->theta[side];
    in.leg_rate_radps=c->d_theta[side];
    in.leg_accel_radps2=c->dd_theta[side];
    c->ground[side]=ground_detection_update(&c->ground_detector,side,&in);
  }
}
static void scope_sample(Ctrl*c) {
  if ((c->ticks++ % SCOPE_DECIMATE) != 0) return;
  Scope* s=&c->scope;
  if (s->count == SCOPE_POINTS) {
    memmove(s->time, s->time+1, (SCOPE_POINTS-1)*sizeof(s->time[0]));
    for (int i=0;i<10;i++) memmove(s->x[i],s->x[i]+1,(SCOPE_POINTS-1)*sizeof(s->x[i][0]));
    for (int i=0;i<4;i++) memmove(s->u[i],s->u[i]+1,(SCOPE_POINTS-1)*sizeof(s->u[i][0]));
    for (int i=0;i<SUPPORT_LINES;i++) memmove(s->support[i],s->support[i]+1,(SCOPE_POINTS-1)*sizeof(s->support[i][0]));
    s->count--;
  }
  int at=s->count++;
  s->time[at]=(float)c->d->time;
  for (int i=0;i<10;i++) s->x[i][at]=c->x[i];
  for (int i=0;i<4;i++) s->u[i][at]=c->u[i];
  s->support[0][at]=(float)c->ground[0].fn_filtered_n;
  s->support[1][at]=(float)c->ground[1].fn_filtered_n;
  s->support[2][at]=(float)c->contact_fn[0];
  s->support[3][at]=(float)c->contact_fn[1];
  s->support[4][at]=(float)c->ground_detector.config.off_threshold_n;
  s->support[5][at]=(float)(c->ground_detector.config.off_threshold_n*c->ground_detector.config.on_ratio);
}
static void control(Ctrl*c,const Meas*q) {
  double hip_limit=c->rear_pose_active?COMPLIANT_HIP_LIMIT:HIP_LIMIT;
  float lim[4]={WHEEL_LIMIT,WHEEL_LIMIT,(float)hip_limit,(float)hip_limit}, off[4]; mjtNum* v=c->d->ctrl;
  c->pos_i=clip(c->pos_i+c->x[0]*c->dt,-1,1); off[0]=off[1]=.6f*(float)c->pos_i; off[2]=off[3]=0;
  if(!wheelleg_mpc_evaluate(&c->mpc,c->x,lim,off,c->u)) memset(c->u,0,sizeof(c->u));
  for(int k=0;k<2;k++) { v[c->id.awheel[k]]=c->u[k]; v[c->id.ahip[k]]=c->u[k+2]+c->u[k]; }
  double ac=pid(&c->anti,0,q->theta[0]-q->theta[1],c->dt); v[c->id.ahip[0]]-=ac; v[c->id.ahip[1]]+=ac;
  for(int k=0;k<2;k++) {
    double desired_total=c->f0+pid(&c->length_pid[k],c->target_len,q->len[k],c->dt);
    double compensation=leg_spring_compensation(q->len[k]);
    c->spring_force[k]=-compensation;
    c->leg_force_cmd[k]=clip(desired_total+compensation,-FORCE_LIMIT,FORCE_LIMIT);
    v[c->id.aleg[k]]=c->leg_force_cmd[k];
    c->d->qfrc_applied[c->m->jnt_dofadr[c->id.slide[k]]]=c->spring_force[k];
    v[c->id.awheel[k]]=clip(v[c->id.awheel[k]],-WHEEL_LIMIT,WHEEL_LIMIT);
    v[c->id.ahip[k]]=clip(v[c->id.ahip[k]],-hip_limit,hip_limit);
    /* The wheel motor reacts on the leg through the hip.  Therefore the
       virtual pitch torque actually delivered after the final actuator clamp
       is hip_motor - wheel_motor. */
    c->tp_cmd[k]=v[c->id.ahip[k]]-v[c->id.awheel[k]];
  }
}
static void reset(Ctrl*c) {
  mjModel*m=c->m; mjData*d=c->d; mj_resetData(m,d); d->qpos[0]=1.25; d->qpos[1]=6.75; d->qpos[2]=c->target_len+WHEEL_R; d->qpos[3]=1;
  for(int k=0;k<2;k++) { int h=m->jnt_qposadr[c->id.hip[k]],s=m->jnt_qposadr[c->id.slide[k]]; d->qpos[h]=HIP_VERTICAL;
    d->qpos[s]=clip(c->target_len-LEG_NOMINAL,m->jnt_range[2*c->id.slide[k]],m->jnt_range[2*c->id.slide[k]+1]); pid_reset(&c->length_pid[k]); }
  pid_reset(&c->anti); c->pos=c->pos_i=0;c->theta_valid=0;c->derivative_valid=0;c->rear_pose_active=0;c->ticks=0;
  c->body_accel_down=0;c->last_z_velocity=0;
  memset(c->d_len,0,sizeof(c->d_len));memset(c->dd_len,0,sizeof(c->dd_len));
  memset(c->d_theta,0,sizeof(c->d_theta));memset(c->dd_theta,0,sizeof(c->dd_theta));
  memset(c->leg_force_cmd,0,sizeof(c->leg_force_cmd));memset(c->spring_force,0,sizeof(c->spring_force));memset(c->tp_cmd,0,sizeof(c->tp_cmd));
  ground_detection_init(&c->ground_detector,NULL);memset(c->ground,0,sizeof(c->ground));
  memset(&c->scope,0,sizeof(c->scope));
  update_linkage_visual(c); mj_forward(m,d);
}
static int init(Ctrl*c,mjModel*m,mjData*d,double leg,double speed) {
  memset(c,0,sizeof(*c));c->m=m;c->d=d;c->dt=m->opt.timestep;c->target_len=clip(leg,LEG_MIN,LEG_MAX);c->speed=speed;
  c->length_pid[0]=(PID){3000,10,80,FORCE_LIMIT,20,0,0,0};c->length_pid[1]=c->length_pid[0];c->anti=(PID){3,.3,.1,6,2,0,0,0};
  if(!ids(m,&c->id)||!linkage_visual_init(c))return 0;wheelleg_mpc_init(&c->mpc);if(!wheelleg_mpc_select_length(&c->mpc,(float)c->target_len))return 0;
  for(int i=0;i<m->nbody;i++)c->f0+=m->body_mass[i];c->f0=(.5*c->f0-m->body_mass[c->id.bwheel[0]])*G;reset(c);Meas z;measure(c,&z);c->target_yaw=z.yaw;return 1;
}
static void set_leg_target(Ctrl*c,double length) {
  c->target_len=clip(length,LEG_MIN,LEG_MAX);
  wheelleg_mpc_select_length(&c->mpc,(float)c->target_len);
  for(int side=0;side<2;side++) pid_reset(&c->length_pid[side]);
}
static void tick(Ctrl*c) {
  Meas q; measure(c,&q);
  c->target_speed += clip(c->speed-c->target_speed,-5*c->dt,5*c->dt);
  if(fabs(c->turn_rate)>1e-6) {
    double requested=wrap(c->target_yaw+c->turn_rate*c->dt);
    double lead=wrap(requested-q.yaw);
    c->target_yaw=fabs(lead)>.16 ? wrap(q.yaw+(lead>0?.16:-.16)) : requested;
  }
  update_derivatives(c,&q);state(c,&q);control(c,&q);update_ground_detection(c,&q);
  scope_sample(c);update_linkage_visual(c);mj_step(c->m,c->d);
}

typedef struct {
  HWND hwnd; HDC dc; HGLRC gl; int running, paused, follow, show_contacts;
  int forward, backward, turn_left, turn_right, drag_left, drag_right, last_x, last_y;
  double camera_azimuth, camera_elevation, camera_distance;
  Ctrl* ctrl;
} Gui;
static Gui* g_gui;
static LRESULT CALLBACK window_proc(HWND hwnd,UINT msg,WPARAM wp,LPARAM lp) {
  if(g_gui && (msg==WM_KEYDOWN || msg==WM_KEYUP)) {
    int down=msg==WM_KEYDOWN;
    if(wp=='W'||wp==VK_NUMPAD8) g_gui->forward=down;
    if(wp=='S'||wp==VK_NUMPAD5) g_gui->backward=down;
    if(wp=='A'||wp==VK_NUMPAD4) g_gui->turn_left=down;
    if(wp=='D'||wp==VK_NUMPAD6) g_gui->turn_right=down;
    if(wp=='0'||wp==VK_NUMPAD0) g_gui->ctrl->rear_pose_active=down;
    if(down && wp==VK_ESCAPE) g_gui->running=0;
    if(down && wp==VK_SPACE && !(lp&0x40000000)) g_gui->paused=!g_gui->paused;
    if(down && wp=='R' && !(lp&0x40000000)) reset(g_gui->ctrl);
    if(down && wp=='F' && !(lp&0x40000000)) g_gui->follow=!g_gui->follow;
    if(down && wp=='V' && !(lp&0x40000000)) g_gui->show_contacts=!g_gui->show_contacts;
    if(down && wp=='C' && !(lp&0x40000000)) {g_gui->camera_azimuth=135;g_gui->camera_elevation=-18;g_gui->camera_distance=2.1;}
    if(down && wp=='J' && !(lp&0x40000000)) g_gui->ctrl->d->qvel[2]+=1.8;
    if(down && (wp=='1'||wp==VK_NUMPAD1)) set_leg_target(g_gui->ctrl,.150);
    if(down && (wp=='2'||wp==VK_NUMPAD2)) set_leg_target(g_gui->ctrl,.270);
    if(down && (wp=='3'||wp==VK_NUMPAD3)) set_leg_target(g_gui->ctrl,.350);
  }
  if(g_gui && msg==WM_KILLFOCUS) {
    g_gui->forward=g_gui->backward=g_gui->turn_left=g_gui->turn_right=0;
    g_gui->ctrl->rear_pose_active=0;
    return 0;
  }
  if(g_gui && msg==WM_LBUTTONDOWN) { g_gui->drag_left=1;g_gui->last_x=(short)LOWORD(lp);g_gui->last_y=(short)HIWORD(lp);SetCapture(hwnd);return 0; }
  if(g_gui && msg==WM_RBUTTONDOWN) { g_gui->drag_right=1;g_gui->last_x=(short)LOWORD(lp);g_gui->last_y=(short)HIWORD(lp);SetCapture(hwnd);return 0; }
  if(g_gui && (msg==WM_LBUTTONUP||msg==WM_RBUTTONUP)) {g_gui->drag_left=g_gui->drag_right=0;ReleaseCapture();return 0;}
  if(g_gui && msg==WM_MOUSEMOVE && (g_gui->drag_left||g_gui->drag_right)) {
    int x=(short)LOWORD(lp),y=(short)HIWORD(lp),dx=x-g_gui->last_x,dy=y-g_gui->last_y;g_gui->last_x=x;g_gui->last_y=y;
    if(g_gui->drag_left){g_gui->camera_azimuth-=.35*dx;g_gui->camera_elevation=clip(g_gui->camera_elevation-.25*dy,-89,89);}
    if(g_gui->drag_right) g_gui->camera_distance=clip(g_gui->camera_distance*(1+.01*dy),.25,10);
    return 0;
  }
  if(g_gui && msg==WM_MOUSEWHEEL){int delta=(short)HIWORD(wp);g_gui->camera_distance=clip(g_gui->camera_distance*(delta>0?.90:1.10),.25,10);return 0;}
  if(msg==WM_DESTROY && g_gui) { g_gui->running=0; return 0; }
  return DefWindowProc(hwnd,msg,wp,lp);
}
static int open_window(Gui*g) {
  WNDCLASSA wc; memset(&wc,0,sizeof(wc)); wc.style=CS_OWNDC; wc.lpfnWndProc=window_proc;
  wc.hInstance=GetModuleHandleA(NULL); wc.lpszClassName="WheelLegMuJoCoC"; wc.hCursor=LoadCursor(NULL,IDC_ARROW);
  if(!RegisterClassA(&wc) && GetLastError()!=ERROR_CLASS_ALREADY_EXISTS) return 0;
  g->hwnd=CreateWindowA(wc.lpszClassName,"WheelLeg C -- MuJoCo",WS_OVERLAPPEDWINDOW|WS_VISIBLE,
                        CW_USEDEFAULT,CW_USEDEFAULT,1500,900,NULL,NULL,wc.hInstance,NULL);
  if(!g->hwnd) return 0; g->dc=GetDC(g->hwnd);
  PIXELFORMATDESCRIPTOR pfd={0}; pfd.nSize=sizeof(pfd);pfd.nVersion=1;pfd.dwFlags=PFD_DRAW_TO_WINDOW|PFD_SUPPORT_OPENGL|PFD_DOUBLEBUFFER;
  pfd.iPixelType=PFD_TYPE_RGBA;pfd.cColorBits=24;pfd.cDepthBits=24;pfd.iLayerType=PFD_MAIN_PLANE;
  int pf=ChoosePixelFormat(g->dc,&pfd); if(!pf || !SetPixelFormat(g->dc,pf,&pfd)) return 0;
  g->gl=wglCreateContext(g->dc); if(!g->gl || !wglMakeCurrent(g->dc,g->gl)) return 0;
  return 1;
}
static void close_window(Gui*g) {
  if(g->gl){wglMakeCurrent(NULL,NULL);wglDeleteContext(g->gl);} if(g->dc&&g->hwnd)ReleaseDC(g->hwnd,g->dc);if(g->hwnd)DestroyWindow(g->hwnd);
}
static void init_figure(mjvFigure*f,const char*title,const char*const*names,int lines) {
  mjv_defaultFigure(f); f->flg_legend=1;f->flg_ticklabel[0]=f->flg_ticklabel[1]=1;f->flg_extend=0;f->linewidth=1.2f;
  strncpy(f->title,title,sizeof(f->title)-1);strncpy(f->xlabel,"simulation time [s]",sizeof(f->xlabel)-1);
  for(int i=0;i<lines;i++){strncpy(f->linename[i],names[i],sizeof(f->linename[i])-1);f->linergb[i][0]=(i%3)==0?0.95f:0.15f;f->linergb[i][1]=(i%3)==1?0.85f:0.35f;f->linergb[i][2]=(i%3)==2?0.95f:0.45f;}
}
static void update_figure(mjvFigure*f,const Scope*s,int offset,int lines,int inputs) {
  if(!s->count) return; float ymin=0,ymax=0; int first=1;
  for(int l=0;l<lines;l++) { f->linepnt[l]=s->count; for(int p=0;p<s->count;p++) {float y=inputs?s->u[offset+l][p]:s->x[offset+l][p];f->linedata[l][2*p]=s->time[p];f->linedata[l][2*p+1]=y;if(first){ymin=ymax=y;first=0;}else{if(y<ymin)ymin=y;if(y>ymax)ymax=y;}} }
  float end=s->time[s->count-1], span=end>5.0f?5.0f:end;f->range[0][0]=end-span;f->range[0][1]=end+0.001f;
  float pad=(ymax-ymin)*0.12f+0.01f;f->range[1][0]=ymin-pad;f->range[1][1]=ymax+pad;
}
static void update_state_figure(mjvFigure*f,const Scope*s,const int*map,int lines) {
  if(!s->count) return; float ymin=0,ymax=0;int first=1;
  for(int l=0;l<lines;l++){f->linepnt[l]=s->count;for(int p=0;p<s->count;p++){float y=s->x[map[l]][p];f->linedata[l][2*p]=s->time[p];f->linedata[l][2*p+1]=y;if(first){ymin=ymax=y;first=0;}else{if(y<ymin)ymin=y;if(y>ymax)ymax=y;}}}
  float end=s->time[s->count-1],span=end>5?5:end,pad=(ymax-ymin)*.12f+.01f;f->range[0][0]=end-span;f->range[0][1]=end+.001f;f->range[1][0]=ymin-pad;f->range[1][1]=ymax+pad;
}
static void update_support_figure(mjvFigure*f,const Scope*s) {
  if(!s->count)return;float ymin=0,ymax=0;int first=1;
  for(int l=0;l<SUPPORT_LINES;l++){f->linepnt[l]=s->count;for(int p=0;p<s->count;p++){float y=s->support[l][p];f->linedata[l][2*p]=s->time[p];f->linedata[l][2*p+1]=y;if(first){ymin=ymax=y;first=0;}else{if(y<ymin)ymin=y;if(y>ymax)ymax=y;}}}
  float end=s->time[s->count-1],span=end>5?5:end;f->range[0][0]=end-span;f->range[0][1]=end+.001f;
  ymin=fminf(ymin,-5.0f);ymax=fmaxf(ymax,30.0f);float pad=(ymax-ymin)*.08f;f->range[1][0]=ymin-pad;f->range[1][1]=ymax+pad;
}
static double wall_time(void) { LARGE_INTEGER f,n;QueryPerformanceFrequency(&f);QueryPerformanceCounter(&n);return (double)n.QuadPart/(double)f.QuadPart; }
static int run_gui(Ctrl*c) {
  Gui gui={0};gui.running=1;gui.follow=1;gui.camera_azimuth=135;gui.camera_elevation=-18;gui.camera_distance=2.1;gui.ctrl=c;g_gui=&gui;if(!open_window(&gui)){fprintf(stderr,"Could not create the native OpenGL window\n");return 0;}
  mjvCamera cam;mjv_defaultCamera(&cam);cam.azimuth=gui.camera_azimuth;cam.elevation=gui.camera_elevation;cam.distance=gui.camera_distance;cam.lookat[0]=c->d->qpos[0];cam.lookat[1]=c->d->qpos[1];cam.lookat[2]=.20;
  mjvOption opt;mjv_defaultOption(&opt);mjvScene scene;mjv_defaultScene(&scene);mjv_makeScene(c->m,&scene,3000);mjrContext con;mjr_defaultContext(&con);mjr_makeContext(c->m,&con,mjFONTSCALE_150);
  const char*body[]={"s","ds","yaw","dyaw","pitch","dpitch"};const char*leg[]={"thL","dthL","thR","dthR"};const char*out[]={"TwL","TwR","TbL","TbR"};
  const char*support[]={"Fn est L","Fn est R","Fn contact L","Fn contact R","off 15N","on 22.5N"};
  /* mjvFigure holds mjMAXLINE x mjMAXLINEPNT samples; keep it off the small
     Windows thread stack or opening the GUI immediately raises 0xC00000FD. */
  static mjvFigure fig_body,fig_leg,fig_out,fig_support;
  init_figure(&fig_body,"state x: body",body,6);init_figure(&fig_leg,"state x: legs",leg,4);init_figure(&fig_out,"MPC output u [N m]",out,4);init_figure(&fig_support,"ground support Fn [N]",support,SUPPORT_LINES);
  double last=wall_time(),acc=0;
  while(gui.running) { MSG msg;while(PeekMessage(&msg,NULL,0,0,PM_REMOVE)){TranslateMessage(&msg);DispatchMessage(&msg);}double now=wall_time(),frame_dt=clip(now-last,0,.05);acc+=frame_dt;last=now;
    { int direction=gui.forward-gui.backward; if(direction) c->speed=clip(c->speed+direction*frame_dt,-1,1); else c->speed=0; c->turn_rate=.5*(gui.turn_left-gui.turn_right); }
    if(!gui.paused){int safety=0;while(acc>=c->dt&&safety++<50){tick(c);acc-=c->dt;}} else acc=0;
    RECT r;GetClientRect(gui.hwnd,&r);mjrRect full={0,0,r.right-r.left,r.bottom-r.top};cam.azimuth=gui.camera_azimuth;cam.elevation=gui.camera_elevation;cam.distance=gui.camera_distance;if(gui.follow){cam.lookat[0]=c->d->qpos[0];cam.lookat[1]=c->d->qpos[1];}opt.flags[mjVIS_CONTACTFORCE]=gui.show_contacts;mjv_updateScene(c->m,c->d,&opt,NULL,&cam,mjCAT_ALL,&scene);mjr_render(full,&scene,&con);
    char left[512],right[640];snprintf(left,sizeof(left),"C VMC + explicit MPC\n%s8/5 speed  4/6 turn  1/2/3 leg\nhold 0 compliant hips: %s\nJ airborne test  V contact arrows:%s\nF follow:%s  mouse left rotate  right/wheel zoom\nSpace pause | R reset | C camera | Esc close",gui.paused?"PAUSED\n":"",c->rear_pose_active?"ON (2 Nm)":"off (10 Nm)",gui.show_contacts?"on":"off",gui.follow?"on":"off");snprintf(right,sizeof(right),"t %.2f s  %s\nleg target %.1f mm  F0 %.1f N\nhip limit %.1f Nm\nmotor F L/R %6.1f / %6.1f N\nspring L/R  %6.1f / %6.1f N\nFn est L/R  %6.1f / %6.1f N\nFn raw L/R  %6.1f / %6.1f N\nFn contact  %6.1f / %6.1f N\nleg state   %s / %s\nconfirm     %u / %u ticks\nMPC region %u\nlink L deg F1 %+5.1f F2 %+5.1f R1 %+5.1f R2 %+5.1f",c->d->time,ground_detection_airborne(&c->ground_detector)?"AIRBORNE":"GROUNDED",1000*c->target_len,c->f0,c->rear_pose_active?COMPLIANT_HIP_LIMIT:HIP_LIMIT,c->leg_force_cmd[0],c->leg_force_cmd[1],c->spring_force[0],c->spring_force[1],c->ground[0].fn_filtered_n,c->ground[1].fn_filtered_n,c->ground[0].fn_raw_n,c->ground[1].fn_raw_n,c->contact_fn[0],c->contact_fn[1],c->ground[0].off_ground?"AIR":"ON",c->ground[1].off_ground?"AIR":"ON",c->ground[0].confirm_count,c->ground[1].confirm_count,c->mpc.region,57.2957795*c->visual.angles[0][0],57.2957795*c->visual.angles[0][1],57.2957795*c->visual.angles[0][4],57.2957795*c->visual.angles[0][5]);mjr_overlay(mjFONT_NORMAL,mjGRID_TOPLEFT,full,left,right,&con);
    { const int body_map[]={0,1,2,3,8,9},leg_map[]={4,5,6,7}; update_state_figure(&fig_body,&c->scope,body_map,6);update_state_figure(&fig_leg,&c->scope,leg_map,4); } update_figure(&fig_out,&c->scope,0,4,1);update_support_figure(&fig_support,&c->scope);
    int w=full.width/3,h=full.height/4;mjr_figure((mjrRect){full.width-w,3*h,w,h},&fig_body,&con);mjr_figure((mjrRect){full.width-w,2*h,w,h},&fig_leg,&con);mjr_figure((mjrRect){full.width-w,h,w,h},&fig_out,&con);mjr_figure((mjrRect){full.width-w,0,w,h},&fig_support,&con);SwapBuffers(gui.dc);Sleep(1);
  }
  mjr_freeContext(&con);mjv_freeScene(&scene);close_window(&gui);g_gui=NULL;return 1;
}
int main(int argc,char**argv) {
  int gui=(argc>1 && !strcmp(argv[1],"--gui")),base=gui?2:1;
  const char*xml=argc>base?argv[base]:"wheelbipe_lqr.xml";
  double seconds=gui?0:(argc>base+1?atof(argv[base+1]):5);
  double leg=gui?(argc>base+1?atof(argv[base+1]):LEG_NOMINAL):(argc>base+2?atof(argv[base+2]):LEG_NOMINAL);
  double speed=gui?(argc>base+2?atof(argv[base+2]):0):(argc>base+3?atof(argv[base+3]):0);
  int jump_test=!gui && argc>base+4 && atoi(argv[base+4])!=0;char err[1024]={0};
  if(!gui && seconds<=0){fprintf(stderr,"Usage: %s [model.xml] [seconds] [leg_m] [speed_mps] [jump_test_0_or_1]\n",argv[0]);return 1;}
  mjModel*m=mj_loadXML(xml,0,err,sizeof(err));if(!m){fprintf(stderr,"Could not load %s: %s\n",xml,err);return 1;}m->opt.disableflags|=mjDSBL_MIDPHASE;
  mjData*d=mj_makeData(m);Ctrl c;if(!d||!init(&c,m,d,leg,speed)){fprintf(stderr,"C controller init failed\n");mj_deleteData(d);mj_deleteModel(m);return 1;}
  if(gui){int ok=run_gui(&c);mj_deleteData(d);mj_deleteModel(m);return ok?0:1;}
  int steps=(int)ceil(seconds/m->opt.timestep),each=(int)fmax(1,floor(.1/m->opt.timestep));
  printf("C VMC + explicit MPC: dt=%.3f ms, leg=%.1f mm, static F0=%.2f N\n",1000*c.dt,1000*c.target_len,c.f0);
  printf("    t       z    pitch   FnEstL   FnEstR  FnTrueL  FnTrueR  offL offR\n");
  int jumped=0;
  for(int i=0;i<steps;i++){
    if(jump_test&&!jumped&&d->time>=1.0){d->qvel[2]+=1.8;jumped=1;printf("# upward test impulse at t=%.3f s\n",d->time);}
    Meas q;measure(&c,&q);tick(&c);
    for(int side=0;side<2;side++)if(c.ground[side].changed)printf("# %s leg -> %s at t=%.3f s, Fn_est=%.2f N, Fn_contact=%.2f N\n",side?"right":"left",c.ground[side].off_ground?"AIR":"GROUND",d->time,c.ground[side].fn_filtered_n,c.contact_fn[side]);
    if(i%each==0)printf("%6.3f %7.3f %+8.4f %8.2f %8.2f %8.2f %8.2f %5d %4d\n",d->time,d->qpos[2],q.pitch,c.ground[0].fn_filtered_n,c.ground[1].fn_filtered_n,c.contact_fn[0],c.contact_fn[1],c.ground[0].off_ground,c.ground[1].off_ground);
  }
  printf("final: t=%.3f z=%.3f x=%.3f MPC region=%u checks=%u\n",d->time,d->qpos[2],d->qpos[0],c.mpc.region,c.mpc.checks);mj_deleteData(d);mj_deleteModel(m);return 0;
}
