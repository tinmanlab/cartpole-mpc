'use strict';
const CONTROL_LAB_TOPICS = {
 overview:{group:"개요",title:"Controller와 Observer를 한 시스템으로 읽기",lead:"Controller는 무슨 힘을 낼지, Observer는 지금 상태가 무엇인지 푼다. 둘은 직렬 연결되므로 좋은 controller도 나쁜 state estimate 위에서는 무너질 수 있다.",chips:["same plant","same sensor contract","truth = evaluator only"],runtime:null,body:[
  "<div class='section'><h3>먼저 분리한다</h3><div class='math'>Plant:      x(k+1) = f(x(k),u(k))\\nSensor:     y(k) = h(x(k)) + noise\\nObserver:   (y,u) -> x_hat, P\\nController: x_hat -> u</div><div class='take'>plant를 고정하고 controller 또는 observer 하나만 바꾸는 것이 이 lab의 비교 규칙이다.</div></div>",
  "<div class='section'><h3>두 개의 계보</h3><p>Controller는 feedback과 prediction을 확장하고, Observer는 model/geometry에 learning을 어디에 삽입할지 확장한다.</p></div>"
 ].join("")},
 pid:{group:"Controller",title:"PID — 모델 없이 오차에 직접 반응",lead:"pole angle과 angle-rate를 빠르게 잡고 cart position/velocity를 느리게 되돌리는 결합 PID/PD.",chips:["feedback","no plant model","gain tuning"],runtime:{controller:"pid"},body:[
  "<div class='section'><h3>왜 필요한가</h3><p>목표와 실제값의 오차만 있으면 모델을 몰라도 feedback을 만들 수 있다. 대신 여러 state의 trade-off는 사람이 gain에 넣는다.</p></div>",
  "<div class='section'><h3>핵심 식</h3><div class='math'>e_p = p - p*\\nu = k_theta theta + k_omega omega + k_p e_p + k_v v + k_i integral(e_p)dt</div></div>",
  "<div class='section'><h3>Humanoid와 연결</h3><p>MPC/WBC 위에도 actuator 근처 PD feedback이 남을 수 있다. PID는 상위 planner가 아니라 가장 아래 feedback의 직관이다.</p></div>"
 ].join("")},
 lqr:{group:"Controller",title:"LQR — 모델과 비용으로 feedback gain을 계산",lead:"선형 dynamics와 state/control 비용 Q,R에서 Riccati equation으로 K를 계산한다.",chips:["linear model","Riccati","state feedback"],runtime:{controller:"lqr"},body:[
  "<div class='section'><h3>PID에서 무엇이 달라지나</h3><p>gain을 개별 오차마다 손으로 조절하는 대신 A,B,Q,R이 전체 state feedback K를 결정한다.</p></div>",
  "<div class='section'><h3>핵심 식</h3><div class='math'>x(k+1) = A x(k) + B u(k)\\nJ = sum[x^T Q x + u^T R u]\\nu = -Kx</div></div>",
  "<div class='section'><h3>MPC로 가는 다리</h3><p>LQR의 quadratic cost와 Riccati recursion은 MPC/NMPC의 local quadratic subproblem에서도 다시 나온다.</p></div>"
 ].join("")},
 linear_mpc:{group:"Controller",title:"Linear MPC — LQR에 미래 horizon을 명시",lead:"같은 upright A,B를 쓰지만 finite horizon을 펼치고 첫 입력만 적용한 뒤 다음 tick에 다시 계산한다.",chips:["finite horizon","receding horizon","force bound"],runtime:{controller:"linear_mpc"},body:[
  "<div class='section'><h3>현재 실제 구현</h3><p>N=30 finite-horizon Riccati recursion으로 future feedback sequence를 계산하고 force limit을 적용한다. v1의 단순 gradient MPC를 대체한다.</p></div>",
  "<div class='section'><h3>문제</h3><div class='math'>min sum(k=0..N-1) [x_k^T Q x_k + R u_k^2] + x_N^T Q_f x_N\\ns.t. x_(k+1)=A x_k+B u_k\\n     |u_k| <= u_max\\napply u_0 only</div></div>",
  "<div class='section'><h3>한계</h3><p>horizon 전체가 하나의 upright 선형모델이다. 실제 nonlinear plant가 멀어지면 model error가 커진다.</p></div>"
 ].join("")},
 centroidal_mpc:{group:"Controller",title:"Centroidal MPC — reduced planner와 lower-level realization",lead:"Figure의 centroidal OCS2 lane은 momentum/configuration을 state로, contact wrench와 joint velocity를 input으로 최적화하고 inverse dynamics로 joint command를 만든다.",chips:["reduced-order","momentum","planner -> lower level"],runtime:{controller:"centroidal_mpc"},body:[
  "<div class='section'><h3>Figure pinned comparator의 실제 구조</h3><div class='math'>x = [h, q_b, q_j]\\nh = [v_com, L/m]\\nu = [W_left, W_right, qdot_j]\\n\\ncentroidal dynamics + state/input cost + ICP cost\\n+ gait/contact schedule + friction/contact constraints\\n-> SQP policy\\n-> q_des, qdot_des, contact wrench\\n-> inverse dynamics -> tau_ff</div></div>",
  "<div class='section'><h3>CartPole에 적용한 실제 대응</h3><p>발/contact wrench가 없고 pole은 underactuated라 humanoid 식을 그대로 복사하지 않는다. 시스템 전체 수평 CoM을 reduced MPC로 계획하고 look-ahead CoM reference를 full-state inner LQR가 실현한다.</p><div class='math'>M = m_c + m_p\\nc = x + (m_p l/M) sin(theta)\\nc_dot = v + (m_p l/M) cos(theta) omega\\nc_ddot ~= F/M\\n\\nouter MPC: [c,c_dot] -> future CoM reference\\ninner feedback: [x-x_ref,v-v_ref,theta,omega] -> force</div><div class='take warn'>Figure centroidal OCS2를 포팅했다는 뜻이 아니라, 실제 reduced-planner -> lower-level 구조를 CartPole 물리에 맞게 구현한 analogue다.</div></div>",
  "<div class='section'><h3>핵심 trade-off</h3><p>작은 predictive problem을 빠르게 풀 수 있지만 reduced model과 실제 full dynamics 사이 realization gap이 생긴다.</p></div>"
 ].join("")},
 full_nmpc:{group:"Controller",title:"Full nonlinear NMPC — nonlinear plant 자체를 horizon에 넣기",lead:"Figure full-order lane은 q와 qdot 전체를 state로 두고 contact wrench와 joint acceleration을 input으로 최적화한다. CartPole판도 nonlinear f(x,u)를 직접 rollout한다.",chips:["full state","nonlinear rollout","iterative solve"],runtime:{controller:"full_nmpc"},body:[
  "<div class='section'><h3>Figure pinned comparator의 실제 구조</h3><div class='math'>x = [q_b, q_j, qdot_b, qdot_j]\\nu = [W_left, W_right, qddot_j]\\n\\nfull rigid-body acceleration dynamics\\n+ state/input + joint torque costs\\n+ contact/swing/friction constraints\\n-> OCS2 multiple-shooting SQP\\n-> policy -> q_des, qdot_des, torque</div></div>",
  "<div class='section'><h3>CartPole의 실제 solver</h3><p>N=30. 동일 nonlinear Plant.integrate로 nominal trajectory를 rollout하고, 매 stage에서 A_k=d f/dx, B_k=d f/du를 수치 선형화한다. local quadratic backward solve 후 bounded forward line-search를 2회 수행하고 warm-start한다.</p><div class='math'>x_(k+1)=f(x_k,u_k)\\nA_k = df/dx, B_k = df/du\\nquadratic local subproblem\\n-> backward solve\\n-> line search + |u|<=u_max\\n-> apply u_0 only</div></div>",
  "<div class='section'><h3>왜 항상 더 좋은가?</h3><p>아니다. 더 정확한 model을 horizon에 넣는 대신 계산시간, local optimum, model mismatch, estimator error의 영향을 더 직접 받는다.</p></div>"
 ].join("")},
 ppo:{group:"Controller",title:"PPO — runtime solver를 training으로 옮기기",lead:"실행 때 horizon optimization 대신 학습된 actor가 observation에서 action을 낸다.",chips:["learned policy","offline training","cheap inference"],runtime:{controller:"ppo"},body:[
  "<div class='section'><h3>현재 구현</h3><p>기존 CartPole PPO Studio robust final-budget actor를 그대로 사용한다.</p></div>",
  "<div class='section'><h3>구조</h3><div class='math'>observation -> Actor -> pi(left), pi(right)\\ntraining: PPO clipped objective + GAE\\nruntime: network forward -> action</div></div>",
  "<div class='section'><h3>MPC와 차이</h3><p>MPC는 현재 model로 미래를 매번 푼다. PPO는 대응을 training optimization을 통해 weight에 압축한다.</p></div>"
 ].join("")},
 raw:{group:"Observer",title:"Raw sensor — 센서를 그냥 쓰면 왜 안 되나",lead:"p와 theta만 직접 측정하고 v와 omega를 finite difference로 만든다.",chips:["baseline","finite difference","noise amplification"],runtime:{observer:"raw"},body:[
  "<div class='section'><h3>핵심 식</h3><div class='math'>v_hat ~= (p_k-p_(k-1))/dt\\nomega_hat ~= wrap(theta_k-theta_(k-1))/dt</div></div>",
  "<div class='section'><h3>문제</h3><p>작은 measurement noise가 dt로 나뉘면서 velocity noise로 크게 증폭된다.</p></div>"
 ].join("")},
 kf:{group:"Observer",title:"Kalman Filter — prediction과 measurement 신뢰도를 결합",lead:"선형 dynamics로 hidden velocity까지 예측하고 P,Q,R로 sensor correction의 크기를 매 tick 계산한다.",chips:["P Q R","Kalman gain","linear/Gaussian"],runtime:{observer:"kf"},body:[
  "<div class='section'><h3>핵심</h3><div class='math'>x_hat_minus = A x_hat_plus + B u\\nP_minus = A P_plus A^T + Q\\nr = y - H x_hat_minus\\nK = P_minus H^T (H P_minus H^T + R)^-1\\nx_hat_plus = x_hat_minus + K r</div></div>",
  "<div class='section'><h3>hidden state가 왜 고쳐지나</h3><p>A의 dynamics coupling이 P에 cross-covariance를 만들고, measurement residual이 K의 velocity row를 통해 hidden state correction으로 들어간다.</p></div>"
 ].join("")},
 ekf:{group:"Observer",title:"EKF — nonlinear mean, local linear covariance",lead:"state mean은 nonlinear f로 움직이고 covariance 전파에 필요한 Jacobian만 현재점에서 계산한다.",chips:["nonlinear f","Jacobian","local linearization"],runtime:{observer:"ekf"},body:[
  "<div class='section'><h3>KF에서 바뀌는 부분</h3><div class='math'>x_hat_minus = f(x_hat_plus,u)\\nF = df/dx evaluated at x_hat\\nP_minus = F P_plus F^T + Q\\n\\nmeasurement correction은 Kalman 구조 유지</div></div>",
  "<div class='section'><h3>다음 문제</h3><p>rotation/pose는 평범한 Euclidean vector가 아니므로 error 자체를 어떤 geometry에서 정의할지가 중요해진다.</p></div>"
 ].join("")},
 inekf:{group:"Observer",title:"InEKF — 상태공간 geometry에 맞는 invariant error",lead:"CartPole은 SO(2) angle wrap까지만 직접 보여주고, Hartley의 핵심은 humanoid floating-base SO(3)/SE_(N+2)(3)와 contact-aided observation이다.",chips:["Lie group","invariant error","contact aided"],runtime:{observer:"so2"},body:[
  "<div class='section'><h3>CartPole runtime bridge</h3><div class='math'>r_theta = wrap(theta_meas-theta_hat_minus)\\ntheta_hat_plus = wrap(theta_hat_minus + delta_theta)</div><p>이것은 Hartley InEKF 그 자체가 아니다.</p></div>",
  "<div class='section'><h3>Hartley에서 실제로 하는 일</h3><div class='math'>state: R, v, p, contact positions d_i\\nIMU propagation + leg FK contact observations\\ninvariant error dynamics\\n\\nunobservable: global translation, global yaw</div></div>",
  "<div class='section'><h3>다음 병목</h3><p>filter가 좋아도 어느 contact measurement를 믿을지 틀리면 update가 오히려 state를 망친다.</p></div>"
 ].join("")},
 lin:{group:"Hybrid estimator",title:"Lin — contact를 learned binary event로",lead:"contact sensor가 없을 때 proprioceptive history에서 contact on/off를 학습해 InEKF contact update를 켜고 끈다.",chips:["learned contact","binary gate","InEKF"],runtime:null,body:[
  "<div class='section'><h3>문제 -> 해결</h3><div class='math'>[q,qdot,a,omega,foot kinematics] history\\n-> temporal CNN\\n-> c in {0,1}\\n-> InEKF contact update</div></div>",
  "<div class='section'><h3>한계</h3><p>contact=true가 FK stationary constraint의 품질을 보장하지 않는다. slip/partial support를 한 비트로 표현하기 어렵다.</p></div>"
 ].join("")},
 youm:{group:"Hybrid estimator",title:"Youm NMN — NN이 body velocity를 measurement로",lead:"contact probability뿐 아니라 body linear velocity 자체를 learned pseudo-measurement로 InEKF에 추가한다.",chips:["GRU","neural measurement","velocity"],runtime:null,body:[
  "<div class='section'><h3>구조</h3><div class='math'>[a,omega,q,qdot,q_des(prev)]\\n-> GRU + MLP\\n-> contact probability\\n-> body velocity v_NN\\n-> InEKF measurement update</div></div>",
  "<div class='section'><h3>권한이 커진다</h3><p>Lin은 update gate를 학습하지만 Youm은 metric velocity measurement를 학습한다. sim-to-real bias가 더 직접적으로 들어올 수 있다.</p></div>"
 ].join("")},
 innkf:{group:"Hybrid estimator",title:"InNKF — physics filter posterior의 residual을 학습",lead:"InEKF를 버리지 않고 posterior가 남기는 systematic state error를 TCN이 예측해 Lie-group-consistent correction을 추가한다.",chips:["TCN","posterior residual","hybrid"],runtime:{observer:"residual"},body:[
  "<div class='section'><h3>논문 구조</h3><div class='math'>InEKF -> X_bar_plus\\nhistory + X_bar_plus -> TCN -> eta_hat in se_2(3)\\nE_hat = Exp(eta_hat)\\nX_bar_plusplus = E_hat^-1 X_bar_plus</div></div>",
  "<div class='section'><h3>CartPole 구현</h3><p>SO(2)-aware EKF posterior 뒤에 실제로 학습한 10->16->4 residual MLP를 붙였다. held-out residual RMSE는 약 0.114에서 0.066으로 감소했지만 closed-loop 악화도 숨기지 않는다.</p></div>"
 ].join("")},
 coco:{group:"Hybrid estimator",title:"CoCo-InEKF — binary contact 대신 directional covariance",lead:"partial contact와 slip을 binary gate 대신 각 contact candidate의 velocity covariance로 표현한다.",chips:["learned covariance","directional slip","differentiable filter"],runtime:{observer:"adaptive"},body:[
  "<div class='section'><h3>핵심</h3><div class='math'>L = NN(sensor history)\\nSigma_contact = L L^T >= 0\\n\\nsmall variance -> trust\\nlarge variance -> down-weight</div></div>",
  "<div class='section'><h3>CartPole runtime bridge</h3><p>현재 Adaptive-R은 learned CoCo network가 아니라 innovation heuristic이다. R을 방향별로 키우면 K가 줄어드는 수학적 자리를 실제 실행으로 보여준다.</p></div>"
 ].join("")},
 focus:{group:"Hybrid estimator",title:"FOCUS — contact가 아니라 FK measurement reliability",lead:"발이 닿았는지보다 그 발로 계산한 FK velocity를 지금 믿어도 되는지를 continuous weight로 학습한다.",chips:["continuous reliability","FK velocity","sensor history"],runtime:{observer:"adaptive"},body:[
  "<div class='section'><h3>핵심 식</h3><div class='math'>w_i in [0,1]\\nR_vel,i = R0 [1 + (1-w_i) S]\\ntau_i = clip(w_i/w_sat,0,1)\\nz_v,i = (1-tau_i) v_IMU + tau_i v_FK,i</div></div>",
  "<div class='section'><h3>Figure와 직접 연결</h3><p>Figure에서 contact source 품질이 velocity estimate와 closed-loop를 크게 바꿨기 때문에 measurement reliability가 현재 핵심 연구축이다.</p></div>",
  "<div class='section'><h3>CartPole 한계</h3><p>foot FK가 없으므로 FOCUS를 복제할 수 없다. 현재 mode는 reliability -> R -> K -> correction 구조만 대응한다.</p></div>"
 ].join("")}
};
const CONTROL_LAB_TOPIC_ORDER=["overview","pid","lqr","linear_mpc","centroidal_mpc","full_nmpc","ppo","raw","kf","ekf","inekf","lin","youm","innkf","coco","focus"];
if(typeof module!=="undefined")module.exports={CONTROL_LAB_TOPICS,CONTROL_LAB_TOPIC_ORDER};
