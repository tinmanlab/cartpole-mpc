// Task interface pattern (Copyright 2022 DeepMind Technologies Limited): DeepMind MJPC humanoid/stand (Apache-2.0).
#include <mujoco/mujoco.h>
#include "mjpc/task.h"
#include "mjpc/planners/sampling/planner.h"
#include "mjpc/states/state.h"
#include "mjpc/threadpool.h"
#include "mjpc/utilities.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

class Wheelbot : public mjpc::Task {
 public:
  double anchor=0, start=0, side=1;
  bool staged=false, safe=false;
  class ResidualFn : public mjpc::BaseResidualFn {
   public:
    explicit ResidualFn(const Wheelbot* t): BaseResidualFn(t), t(t) {}
    void Residual(const mjModel* m, const mjData* d, double* r) const override {
      const double* q = mjpc::GetCustomNumericData(m,"trim_q");
      // One explicit support phase, then the same wheel-balance objective.
      bool support=t->staged && d->time < t->start+1.2;
      double pitch=support ? t->side*.8 : q[2];
      double hip=support ? t->side*1.1 : q[3];
      double knee=support ? -t->side*2.3 : q[4];
      int n=0;
      r[n++]=sin(d->qpos[2]-pitch); r[n++]=1-cos(d->qpos[2]-pitch);
      r[n++]=(d->qpos[1]-(support ? .20 : q[1]))/q[1];
      r[n++]=d->qpos[3]-hip; r[n++]=d->qpos[4]-knee;
      r[n++]=d->qpos[0]-t->anchor;
      for(int j=0;j<m->nv;j++) r[n++]=d->qvel[j];
      for(int j=0;j<m->nu;j++) r[n++]=d->ctrl[j]/m->actuator_ctrlrange[2*j+1];
      for(int j=3;j<=4;j++) r[n++]=std::max({0.,m->jnt_range[2*j]-d->qpos[j],d->qpos[j]-m->jnt_range[2*j+1]})/(t->safe ? .01 : 1.);
      if(t->safe) {
        double penetration=0;
        for(int j=0;j<d->ncon;j++) penetration=std::max(penetration,-d->contact[j].dist);
        r[n++]=std::max(0.,penetration-.002)/.01;
      }
    }
   private: const Wheelbot* t;
  };
  Wheelbot(): residual(this) {}
  std::string Name() const override { return "Full-contact wheelbot"; }
  std::string XmlPath() const override { return "generated task.xml"; }
 protected:
  std::unique_ptr<mjpc::ResidualFn> ResidualLocked() const override { return std::make_unique<ResidualFn>(this); }
  ResidualFn* InternalResidual() override { return &residual; }
 private: ResidualFn residual;
};
static const double* Numeric(const mjModel* m,const char* name,int size) {
  int id=mj_name2id(m,mjOBJ_NUMERIC,name);
  if(id<0 || m->numeric_size[id]!=size) throw std::runtime_error(std::string("invalid numeric: ")+name);
  const double* value=m->numeric_data+m->numeric_adr[id];
  for(int i=0;i<size;i++) if(!std::isfinite(value[i])) throw std::runtime_error("nonfinite task numeric");
  return value;
}
// Declared admission domain, not a region-of-attraction certificate.
static bool Local(const mjModel* m,const mjData* d) {
  auto q=Numeric(m,"trim_q",6);
  const double bounds[]={.1,.1,.1,.15,.15};
  for(int j=0;j<5;j++) if(!std::isfinite(d->qpos[j]) || std::abs(d->qpos[j]-q[j])>bounds[j]) return false;
  for(int j=0;j<6;j++) if(!std::isfinite(d->qvel[j]) || std::abs(d->qvel[j])>(j==5 ? 2:1)) return false;
  return true;
}
static void LQR(const mjModel* m,double* u,const double* x) {
  auto K=Numeric(m,"standing_K",33), q=Numeric(m,"trim_q",6), ref=Numeric(m,"trim_u",3);
  for(int a=0;a<3;a++) {
    u[a]=ref[a];
    for(int j=0;j<11;j++) u[a]-=K[a*11+j]*(j<5 ? x[j]-q[j] : x[j+1]);
    u[a]=std::clamp(u[a],m->actuator_ctrlrange[2*a],m->actuator_ctrlrange[2*a+1]);
  }
}
static bool Physical(const mjModel* m,const mjData* d,double& maximum) {
  for(int j=0;j<6;j++) if(!std::isfinite(d->qpos[j]) || !std::isfinite(d->qvel[j])) return false;
  for(int j=0;j<3;j++) if(!std::isfinite(d->ctrl[j]) || !std::isfinite(d->actuator_force[j]) || std::abs(d->ctrl[j])>m->actuator_ctrlrange[2*j+1]+1e-12 || std::abs(d->actuator_force[j])>m->actuator_ctrlrange[2*j+1]+1e-10) return false;
  for(int j=3;j<=4;j++) maximum=std::max({maximum,(m->jnt_range[2*j]-d->qpos[j])/.05,(d->qpos[j]-m->jnt_range[2*j+1])/.05});
  for(int j=0;j<d->ncon;j++) maximum=std::max(maximum,-d->contact[j].dist/.02);
  return maximum<=1;
}
static Wheelbot* task_ptr;
static void Sensor(const mjModel* m,mjData* d,int stage) {
  if(stage==mjSTAGE_ACC && task_ptr) task_ptr->Residual(m,d,d->sensordata);
}
int main(int argc,char** argv) {
 try {
  if(mj_version()!=mjVERSION_HEADER || mj_version()!=3007000) throw std::runtime_error("MuJoCo header/runtime mismatch or not 3.7.0");
  if(argc==2 && std::string(argv[1])=="--version") {std::cout<<mjVERSION_HEADER<<" "<<mj_version()<<"\n";return 0;}
  if(argc!=6 && argc!=7) throw std::runtime_error("usage: wheelbot task.xml trace.csv standing|perturbed|left|right|check-task seconds staged(0|1)");
  std::string mode=argv[3];
  if(mode!="standing" && mode!="perturbed" && mode!="left" && mode!="right" && mode!="check-task" && mode!="check-admission" && mode!="perturbed-left") throw std::runtime_error("bad scenario");
  size_t consumed=0; double seconds=std::stod(argv[4],&consumed);
  if(consumed!=std::string(argv[4]).size() || !std::isfinite(seconds) || seconds<=0 || seconds>6 || std::abs(seconds/.002-std::round(seconds/.002))>1e-8) throw std::runtime_error("duration must be finite 2ms multiple in (0,6]");
  if(std::string(argv[5])!="0" && std::string(argv[5])!="1") throw std::runtime_error("staged must be 0 or 1");
  char error[2048]; mjModel* m=mj_loadXML(argv[1],nullptr,error,sizeof(error));
  if(!m) throw std::runtime_error(error);
  if(m->nq!=6 || m->nv!=6 || m->nu!=3 || m->opt.timestep!=.002) throw std::runtime_error("plant dimensions/timestep");
  const char* joints[]={"x","z","pitch","hip","knee","wheel"};
  const char* actuators[]={"hip_motor","knee_motor","wheel_motor"};
  const double limits[]={16,16,1.7};
  for(int i=0;i<6;i++) if(mj_name2id(m,mjOBJ_JOINT,joints[i])!=i || m->jnt_qposadr[i]!=i || m->jnt_dofadr[i]!=i) throw std::runtime_error("joint order mismatch");
  for(int i=0;i<3;i++) if(mj_name2id(m,mjOBJ_ACTUATOR,actuators[i])!=i || m->actuator_trnid[2*i]!=i+3 || m->actuator_ctrlrange[2*i]!=-limits[i] || m->actuator_ctrlrange[2*i+1]!=limits[i]) throw std::runtime_error("actuator identity/limits mismatch");
  std::string numerics=argc==7 ? argv[6] : "reproducible";
  if(numerics!="legacy" && numerics!="reproducible") throw std::runtime_error("unknown numerics");
  bool reproducible=numerics=="reproducible";
  if(Numeric(m,"numerics_reproducible",1)[0]!=(reproducible ? 1:0)) throw std::runtime_error("numerics metadata mismatch");
  double base_disable=Numeric(m,"numerics_base_disableflags",1)[0];
  double base_enable=Numeric(m,"numerics_base_enableflags",1)[0];
  if(base_disable!=0 || base_enable!=0) throw std::runtime_error("unsupported base numerical flags");
  if(m->opt.disableflags!=(reproducible ? mjDSBL_WARMSTART:0) || m->opt.enableflags!=0) throw std::runtime_error("numerical flag allowlist mismatch");
  if(m->opt.gravity[0]!=0 || m->opt.gravity[1]!=0 || m->opt.gravity[2]!=-9.81) throw std::runtime_error("gravity mismatch");
  bool safe=Numeric(m,"safe_task",1)[0]==1;
  if(Numeric(m,"safe_task",1)[0]!=0 && !safe) throw std::runtime_error("invalid safe_task flag");
  Numeric(m,"trim_q",6); Numeric(m,"trim_u",3);
  if(Numeric(m,"trim_q",6)[1]<=0) throw std::runtime_error("invalid trim height");
  int knots=Numeric(m,"sampling_spline_points",1)[0], rollouts=Numeric(m,"sampling_trajectories",1)[0];
  if(!((knots==24 && rollouts==64)||(knots==8 && rollouts==128))) throw std::runtime_error("unsupported sampler variant");
  const char* terms[]={"orientation","height","pose","movement","velocity","effort","joint_limit","penetration"};
  const int dims[]={2,1,2,1,6,3,2,1}; const double weights[]={15,30,2,.2,.015,.01,100,100};
  if(m->nsensor!=(safe ? 8:7) || m->nsensordata!=(safe ? 18:17) || m->nuser_sensor!=4) throw std::runtime_error("task residual dimension mismatch");
  for(int i=0;i<m->nsensor;i++) if(mj_name2id(m,mjOBJ_SENSOR,terms[i])!=i || m->sensor_dim[i]!=dims[i] || m->sensor_user[4*i]!=0 || m->sensor_user[4*i+1]!=weights[i]) throw std::runtime_error("task sensor name/dimension/weight mismatch");
  mjData* d=mj_makeData(m); Wheelbot task; task.safe=safe; task.Reset(m); task_ptr=&task; mjcb_sensor=Sensor;
  const double* trim=mjpc::GetCustomNumericData(m,"trim_q");
  mju_copy(d->qpos,trim,6);
  task.side=mode=="left" ? -1:1;
  if(mode=="perturbed" || mode=="perturbed-left") d->qpos[2]+=(mode=="perturbed" ? .02:-.02);
  if(mode=="left" || mode=="right") {
    d->qpos[1]=.7; d->qpos[2]=task.side*1.1;
    for(int k=0;k<2000;k++) mj_step(m,d); // Four seconds of uninterrupted gravity.
  }
  task.anchor=d->qpos[0]; task.start=d->time; task.staged=std::stoi(argv[5])!=0;
  mj_forward(m,d);
  if(mode=="check-task") {
    d->qpos[3]=m->jnt_range[7]+.02; d->qpos[4]=m->jnt_range[8]-.01;
    d->ncon=0; mjContact contact{};contact.dist=-.012;
    if(mj_addContact(m,d,&contact)) throw std::runtime_error("test contact allocation failed");
    double r[18]; task.Residual(m,d,r);
    std::cout<<"{\"dimensions\":"<<task.num_residual<<",\"joint_residuals\":["<<r[15]<<","<<r[16]<<"],\"penetration_residual\":"<<(safe ? r[17]:0)<<"}\n";
    mjcb_sensor=nullptr;task_ptr=nullptr;mj_deleteData(d);mj_deleteModel(m);return 0;
  }
  if(mode=="check-admission") {
    double maximum=0; bool first=Physical(m,d,maximum);
    // Same predicate used at every nonlinear horizon step, independent of cost.
    d->qpos[3]=m->jnt_range[7]+.051;
    bool late=Physical(m,d,maximum);
    if(!first || late) throw std::runtime_error("late unsafe admission escaped");
    std::cout<<"{\"late_unsafe_rejected\":true,\"applied_unsafe_inputs\":0}\n";
    return 0;
  }
  auto planner=std::make_unique<mjpc::SamplingPlanner>(); 
  // Check the complete serialized model before/after upstream Initialize.
  std::vector<char> before(mj_sizeModel(m)),after(before.size());
  mj_saveModel(m,nullptr,before.data(),before.size());
  planner->Initialize(m,task);
  mj_saveModel(m,nullptr,after.data(),after.size());
  if(before!=after) throw std::runtime_error("planner Initialize mutated model");
  planner->Allocate();
  constexpr int horizon=501; planner->Reset(horizon,mjpc::GetCustomNumericData(m,"trim_u"));
  mjpc::State state; state.Allocate(m); mjpc::ThreadPool pool(4);
  std::ofstream out(argv[2]); if(!out) throw std::runtime_error("trace open failed"); out<<std::setprecision(17);
  out<<"time,q0,q1,q2,q3,q4,q5,v0,v1,v2,v3,v4,v5,u0,u1,u2,force0,force1,force2,contacts,penetration,limit,cost,iteration,warm0,warm1,warm2,warm3,warm4,warm5,selectedpolicy,rejectedcandidatecount,feasiblecandidates,nonlinear_horizon_feasibility_max,terminalerror,lqr_cost,selected_q_error,selected_v_error\n";
  int iterations=0; double cost=0;
  std::string selectedpolicy="none";
  int rejected_candidates=0,feasible_candidates=0,sample_selected=0,backup_selected=0;
  double horizon_max=0,terminal_error=0,lqr_cost=-1,selected_pe=0,selected_ve=0;
  bool backup=false;
  mjpc::Trajectory backup_trajectory;
  backup_trajectory.Initialize(12,3,task.num_residual,task.num_trace,horizon);
  backup_trajectory.Allocate(horizon);
  const mjpc::Trajectory* selected=nullptr;
  mjData* diagnostic=mj_makeData(m);
  auto array=[](std::ostream& stream,const double* values,int n) {
    stream<<"[";for(int i=0;i<n;i++) {if(i) stream<<",";stream<<values[i];}stream<<"]";
  };
  std::vector<double> integration(mj_stateSize(m,mjSTATE_INTEGRATION));
  mj_getState(m,d,integration.data(),mjSTATE_INTEGRATION);
  std::ofstream initial(std::string(argv[2])+".initial.json");
  initial<<std::setprecision(17)<<"{\"state_spec\":"<<mjSTATE_INTEGRATION<<",\"numerics\":\""<<numerics<<"\",\"state\":";
  array(initial,integration.data(),integration.size()); initial<<"}\n"; initial.close();
  auto record=[&]() {
    mj_copyData(diagnostic,m,d); mj_forward(m,diagnostic);
    double penetration=0,limit=0;
    for(int i=0;i<diagnostic->ncon;i++) penetration=std::max(penetration,-diagnostic->contact[i].dist);
    for(int j=3;j<=4;j++) limit=std::max({limit,m->jnt_range[2*j]-d->qpos[j],d->qpos[j]-m->jnt_range[2*j+1]});
    for(int j=0;j<6;j++) if(!std::isfinite(d->qpos[j]) || !std::isfinite(d->qvel[j])) throw std::runtime_error("nonfinite state");
    for(int j=0;j<3;j++) if(!std::isfinite(diagnostic->actuator_force[j]) || std::abs(diagnostic->actuator_force[j])>limits[j]+1e-10) throw std::runtime_error("actual torque limit");
    out<<d->time;
    for(int i=0;i<6;i++) out<<","<<d->qpos[i];
    for(int i=0;i<6;i++) out<<","<<d->qvel[i];
    for(int i=0;i<3;i++) out<<","<<d->ctrl[i];
    for(int i=0;i<3;i++) out<<","<<diagnostic->actuator_force[i];
    out<<","<<diagnostic->ncon<<","<<penetration<<","<<limit<<","<<cost<<","<<iterations;
    for(int j=0;j<6;j++) out<<","<<d->qacc_warmstart[j];
    out<<","<<selectedpolicy<<","<<rejected_candidates<<","<<feasible_candidates<<","<<horizon_max<<","<<terminal_error<<","<<lqr_cost<<","<<selected_pe<<","<<selected_ve<<"\n";
    for(int j=0;j<m->nv;j++) if(d->qfrc_applied[j]!=0) throw std::runtime_error("external generalized force");
    for(int j=0;j<6*m->nbody;j++) if(d->xfrc_applied[j]!=0) throw std::runtime_error("external body force");
  };
  // Fresh verification before each plan can apply torque; legacy remains diagnostic.
  double position_error=0,velocity_error=0,action_error=0,execution_position_error=0,execution_velocity_error=0;
  int verified=0,executed=0,rejected=0; std::string rejection;
  int steps=std::lround(seconds/.002);
  for(int k=0;k<steps;k++) {
    if(k%5==0) {
      state.Set(m,d); planner->SetState(state); iterations++;
      int count=planner->OptimizePolicyCandidates(rollouts,horizon,pool);
      rejected_candidates=0;feasible_candidates=0; backup=false;selected=nullptr;lqr_cost=-1;
      // Independently roll feedback with the same upstream trajectory/cost path.
      auto feedback=[&](double* u,const double* x,double) { LQR(m,u,x); };
      auto feasible=[&](const mjpc::Trajectory* trajectory,auto action,double& maximum) {
        if(trajectory->failure || !std::isfinite(trajectory->total_return)) return false;
        mjData* probe=mj_makeData(m);state.CopyTo(m,probe);maximum=0;
        bool ok=true;
        for(int t=0;t<horizon;t++) {
          double x[12];mju_copy(x,probe->qpos,6);mju_copy(x+6,probe->qvel,6);
          action(probe->ctrl,x,probe->time);
          mj_copyData(diagnostic,m,probe);mj_forward(m,diagnostic);
          if(!Physical(m,diagnostic,maximum)) {ok=false;break;}
          if(t<horizon-1) mj_step(m,probe);
        }
        mj_deleteData(probe);return ok;
      };
      double backup_max=0;
      if(Local(m,d)) {
        mjData* probe=mj_makeData(m);
        backup_trajectory.Rollout(feedback,&task,m,probe,planner->state.data(),d->time,planner->mocap.data(),planner->userdata.data(),horizon);
        mj_deleteData(probe);
        if(feasible(&backup_trajectory,feedback,backup_max)) {
          selected=&backup_trajectory;backup=true;lqr_cost=selected->total_return;horizon_max=backup_max;
        }
      }
      int chosen=-1;
      for(int c=0;c<count;c++) {
        auto candidate=&planner->trajectory[planner->trajectory_order[c]];
        double maximum=0;
        auto action=[&](double* u,const double* x,double time) {planner->ActionFromCandidatePolicy(u,c,x,time);};
        if(!feasible(candidate,action,maximum)) {rejected_candidates++;continue;}
        feasible_candidates++;
        if(!selected || candidate->total_return<selected->total_return) {
          selected=candidate;backup=false;chosen=c;horizon_max=maximum;
        }
      }
      if(!selected) {rejected++;rejection="NO_ADMISSIBLE_PLAN";selectedpolicy="none";break;}
      if(backup) {selectedpolicy="LQRbackup";backup_selected++;}
      else {planner->CopyCandidateToPolicy(chosen);selectedpolicy="officialsample";sample_selected++;}
      auto best=selected;cost=best->total_return;
      terminal_error=0;
      for(int j=0;j<5;j++) terminal_error=std::max(terminal_error,std::abs(best->states[(horizon-1)*12+j]-trim[j]));
      mjData* replay=mj_makeData(m); state.CopyTo(m,replay);
      double pe=0,ve=0,ae=0;
      std::vector<double> first_actions;
      for(int t=0;t<horizon-1;t++) {
        if(backup) {double x[12];mju_copy(x,replay->qpos,6);mju_copy(x+6,replay->qvel,6);LQR(m,replay->ctrl,x);}
        else planner->ActionFromPolicy(replay->ctrl,nullptr,replay->time);
        for(int j=0;j<3;j++) {
          if(!std::isfinite(replay->ctrl[j])) throw std::runtime_error("nonfinite replay action");
          ae=std::max(ae,std::abs(replay->ctrl[j]-best->actions[t*3+j]));
          if(t<5) first_actions.push_back(replay->ctrl[j]);
        }
        mj_step(m,replay);
        for(int j=0;j<6;j++) {
          if(!std::isfinite(replay->qpos[j]) || !std::isfinite(replay->qvel[j])) throw std::runtime_error("nonfinite replay state");
          pe=std::max(pe,std::abs(replay->qpos[j]-best->states[(t+1)*12+j]));
          ve=std::max(ve,std::abs(replay->qvel[j]-best->states[(t+1)*12+6+j]));
        }
      }
      mj_deleteData(replay);selected_pe=pe;selected_ve=ve;
      position_error=std::max(position_error,pe);velocity_error=std::max(velocity_error,ve);action_error=std::max(action_error,ae);
      if(k==0) {
        std::ofstream prediction(std::string(argv[2])+".predictions.json");prediction<<std::setprecision(17);
        prediction<<"{\"time\":";array(prediction,best->times.data(),6);
        for(auto name:{"qpos","qvel","actions","recorded_actions"}) {
          prediction<<",\""<<name<<"\":[";
          bool q=std::string(name)=="qpos",v=std::string(name)=="qvel";
          for(int t=0;t<(q||v ? 6:5);t++) {
            if(t) prediction<<",";
            array(prediction,q||v ? best->states.data()+t*12+(v?6:0) : (std::string(name)=="actions" ? first_actions.data():best->actions.data())+t*3,q||v?6:3);
          }
          prediction<<"]";
        }
        prediction<<"}\n";
      }
      if(reproducible && (pe>1e-9 || ve>1e-8 || ae>1e-12)) {rejected++;rejection="candidate_replay";break;}
      verified++;
    }
    if(backup) {double x[12];mju_copy(x,d->qpos,6);mju_copy(x+6,d->qvel,6);LQR(m,d->ctrl,x);}
    else planner->ActionFromPolicy(d->ctrl,nullptr,d->time);
    for(int j=0;j<3;j++) if(!std::isfinite(d->ctrl[j]) || std::abs(d->ctrl[j])>m->actuator_ctrlrange[2*j+1]+1e-12) throw std::runtime_error("invalid control");
    // Logging uses a copy; actual integration history is never forwarded or reset here.
    record(); mj_step(m,d);executed++;
    auto best=selected;int t=k%5+1;
    for(int j=0;j<6;j++) {
      execution_position_error=std::max(execution_position_error,std::abs(d->qpos[j]-best->states[t*12+j]));
      execution_velocity_error=std::max(execution_velocity_error,std::abs(d->qvel[j]-best->states[t*12+6+j]));
    }
    if(reproducible && (execution_position_error>1e-9 || execution_velocity_error>1e-8)) {rejected++;rejection="execution_prediction";break;}
  }
  record();
  std::cout<<std::setprecision(17)<<"{\"iterations\":"<<iterations<<",\"horizon_steps\":501,\"rollouts_per_iteration\":"<<rollouts<<",\"threads\":4,\"spline_knots\":"<<knots
    <<",\"numerics\":\""<<numerics<<"\",\"model_unchanged_after_initialize\":true,\"nominal_native_max_error\":"<<std::max(position_error,velocity_error)
    <<",\"position_max_error_m_or_rad\":"<<position_error<<",\"velocity_max_error_m_s_or_rad_s\":"<<velocity_error<<",\"policy_action_max_error_nm\":"<<action_error
    <<",\"execution_position_max_error_m_or_rad\":"<<execution_position_error<<",\"execution_velocity_max_error_m_s_or_rad_s\":"<<execution_velocity_error
    <<",\"verified_plans\":"<<verified<<",\"executed_steps\":"<<executed<<",\"rejected_plans\":"<<rejected<<",\"consistency_pass\":"<<(reproducible && !rejected ? "true":"false")
    <<",\"sample_selected\":"<<sample_selected<<",\"backup_selected\":"<<backup_selected
    <<",\"rejection\":\""<<rejection<<"\",\"position_tolerance\":1e-9,\"velocity_tolerance\":1e-8,\"action_tolerance_nm\":1e-12,\"final_cost\":"<<cost<<",\"header_version\":"<<mjVERSION_HEADER<<",\"runtime_version\":"<<mj_version()<<"}\n";
  mj_deleteData(diagnostic);
  mjcb_sensor=nullptr;task_ptr=nullptr; mj_deleteData(d); // Planner owns data and must die before model.
  planner.reset(); mj_deleteModel(m); return rejected ? 2:0;
 } catch(const std::exception& e) {std::cerr<<e.what()<<"\n";return 2;}
}
