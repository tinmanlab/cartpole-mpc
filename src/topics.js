'use strict';
// Theory sources and local implementation scope are per-topic; reading does not select a runtime.
const CONTROL_LAB_TOPICS = {
  "overview": {
    "group": "개요",
    "title": "제어·추정 — 기능의 연결이지 성능 순위가 아니다",
    "lead": "목표 추종, 상태 추정, 계산 성공은 서로 다른 평가다. 읽는 주제를 바꾸는 것과 실제 실행 구성을 바꾸는 것도 구분한다.",
    "chips": [
      "signal contract",
      "units",
      "oracle exception"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>신호와 시각</h3><div class=\"math\">x_hat(k) → u_cmd(k) → actuator → MuJoCo x(k+1)\ny(k+1) → observer predict/update → x_hat(k+1)</div><p>x=[p,v,θ,ω]의 단위는 [m,m/s,rad,rad/s]. controller는 추정값으로 입력을 만들고, 다음 측정을 받은 observer가 같은 시각의 상태를 갱신한다. 센서가 누락되면 새 관측처럼 재사용하지 않는다.</p></div><div class=\"section\"><h3>정답의 사용 범위</h3><p>일반 observer 모드에서 truth는 표시·평가용이다. <b>Truth / oracle을 선택하면 controller도 정답을 받는 예외</b>이며 배포 가능한 센서가 아니다. 식별·학습 데이터에도 시뮬레이션 truth가 사용될 수 있으므로 runtime과 offline 정보 출처를 나눈다.</p></div><div class=\"section\"><h3>비교 읽는 법</h3><p>Q_c,R_c는 제어 비용, Q_e,R_e는 추정 잡음 공분산이다. 위치 추종 오차와 추정 오차를 따로 보고, 서로 다른 단위를 무가중 합한 legacy rmseState로 controller 순위를 매기지 않는다. 같은 seed라도 controller가 바뀌면 실제 궤적도 바뀐다.</p></div>",
    "scope": "concept",
    "sources": [
      {
        "label": "실제 구현과 범위",
        "url": "docs/IMPLEMENTATION_BOUNDARIES.md"
      },
      {
        "label": "평가 설계",
        "url": "docs/EXPERIMENT_DESIGN.md"
      }
    ]
  },
  "pid": {
    "group": "Controller",
    "title": "PID/PD 결합 — 직접 오차 피드백",
    "lead": "이 모드는 cart position의 적분항과 angle/rate/position/velocity 피드백을 조합한다. 모든 로봇에 이 gain이나 부호가 맞는 것은 아니다.",
    "chips": [
      "feedback",
      "gain/sign convention",
      "saturation"
    ],
    "runtime": {
      "controller": "pid"
    },
    "body": "<div class=\"section\"><h3>현재 코드</h3><div class=\"math\">e_p = p_hat - p_goal\nu = angle/rate feedback + position/velocity feedback + integral term</div><p>별도의 예측 dynamics 없이 추정 state의 오차에 gain을 곱해 force를 만든다. 이 관측값을 만드는 observer는 모델을 사용할 수 있으므로 시스템 전체가 model-free라는 뜻은 아니다.</p></div><div class=\"section\"><h3>적용 전제</h3><p>부호는 좌표·입력 방향에 의존한다. 포화, 적분 windup, 센서 미분 잡음, actuator 대역폭을 확인해야 한다. 이 실험은 전역 swing-up이나 다관절 안정성을 증명하지 않는다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      },
      {
        "label": "실제 구현과 범위",
        "url": "docs/IMPLEMENTATION_BOUNDARIES.md"
      }
    ]
  },
  "lqr": {
    "group": "Controller",
    "title": "LQR — 명시한 선형·이차 문제의 최적 피드백",
    "lead": "무한시간 선형계·이차 비용의 해와, 그 gain을 비선형 plant에 적용한 결과를 구분한다. 여기서는 실제 force가 포화되므로 무제약 LQR 정리가 그대로 적용되지 않는다.",
    "chips": [
      "linear model",
      "Riccati",
      "state feedback"
    ],
    "runtime": {
      "controller": "lqr"
    },
    "body": "<div class=\"section\"><h3>문제와 조건</h3><div class=\"math\">e(k+1)=A e(k)+B u(k)\nJ=Σ[eᵀ Q_c e + uᵀ R_c u]\nu=-K e</div><p>일반 이론은 Q_c가 양의 준정부호, R_c가 양의 정부호이며 stabilizability와 적절한 detectability 조건 등을 요구한다. 이 lab의 Riccati 옵션은 더 좁게 양의 정부호 Q_c를 요구한다.</p></div><div class=\"section\"><h3>남는 한계</h3><p>upright nominal 선형화에서 계산한 K를 비선형계에 적용하므로 유효 영역과 포화를 따로 검증한다. finite-horizon LQR도 존재하며 horizon 자체가 MPC만의 특징은 아니다. Kalman filter와 조합한 분리원리 역시 그 선형·잡음·비용 가정 안에서 해석한다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "MIT LQR 강의",
        "url": "https://underactuated.mit.edu/lqr.html"
      },
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      }
    ]
  },
  "linear_mpc": {
    "group": "Controller",
    "title": "Linear MPC — 유한 구간을 풀고 첫 입력만 적용",
    "lead": "MPC는 하나의 모델 종류가 아니라 반복 최적화 제어 방식이다. 여기서는 fixed-LTI dynamics, convex quadratic cost, input bounds를 사용한다.",
    "chips": [
      "finite horizon",
      "receding horizon",
      "box-constrained input"
    ],
    "runtime": {
      "controller": "linear_mpc"
    },
    "body": "<div class=\"section\"><h3>실제 solver</h3><div class=\"math\">min Σ[e_kᵀ Q_c e_k + R_c u_k²] + e_Nᵀ P_f e_N\ns.t. e_(k+1)=A e_k+B u_k, |u_k|≤u_max</div><p>N=30, 20 ms 주기. upright MuJoCo 전이의 Jacobian으로 A,B를 만들고 pinned quadprog로 QP를 푼다. 입력·잔차 검사 후 첫 force만 적용한다. 이 quadprog 경로는 warm start를 사용하지 않는다.</p></div><div class=\"section\"><h3>Terminal cost</h3><p>Original은 지정된 대각 P_f, Riccati는 같은 A,B,Q_c,R_c의 전체 tail-cost 행렬이다. 비대각 결합항을 유지한다. 무제약 선형 무한시간 문제에서의 의미와 비선형·제약 시스템의 local 근사를 구분하며, terminal invariant set이나 recursive feasibility 보장은 추가로 필요하다.</p></div><div class=\"section\"><h3>모델·제약·결과</h3><p>hard_mpc는 별도로 world-position rail 제약을 넣는다. input-only MPC도 올바른 MPC다. 선형 dynamics라는 이유만으로 임의 비용·제약의 문제가 convex가 되는 것은 아니며, 예측 제약 만족이 true plant의 제약 만족은 아니다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "OSQP 공식 MPC 예제",
        "url": "https://osqp.org/docs/examples/mpc.html"
      },
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      },
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      }
    ]
  },
  "state_mpc": {
    "group": "Controller",
    "title": "상태 비용과 제약 — soft penalty와 hard rail을 구분",
    "lead": "state_mpc는 위치 오차에 비용을 더하는 근사 구현이다. hard_mpc의 물리적 레일 제약과 동일하지 않다.",
    "chips": [
      "state constraint",
      "soft penalty",
      "feasibility"
    ],
    "runtime": {
      "controller": "state_mpc"
    },
    "body": "<div class=\"section\"><h3>현재 좌표 기준</h3><div class=\"math\">soft cost: λ max(0, |p−p_goal|−b)²\nhard rail: −2.4 m ≤ p_k ≤ 2.4 m</div><p>state_mpc의 softPosition은 <b>|p−p_goal|</b>에 대한 목표 주변 band다. 목표가 바뀌면 band도 이동한다. hard_mpc는 world coordinate의 |p|≤2.4 m를 QP에 직접 넣는다.</p></div><div class=\"section\"><h3>수치 판정</h3><p>soft 모드는 projected gradient와 line search를 제한 횟수 실행한다. 비용 감소·루프 종료는 최적해 수렴 증명이 아니므로 converged는 미검증으로 표시한다. hard/soft 선택은 요구사항에 따른 설계이며 soft MPC를 가짜 MPC라고 부르지 않는다.</p></div><div class=\"section\"><h3>확장 조건</h3><p>발 접촉·마찰·충돌·토크 제한은 모델과 운용 조건에 맞춰 명시한다. no-slip은 가정이며 미끄러짐·rolling·compliant contact에는 다른 상태와 제약이 필요하다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      },
      {
        "label": "OSQP 공식 MPC 예제",
        "url": "https://osqp.org/docs/examples/mpc.html"
      }
    ]
  },
  "ltv_mpc": {
    "group": "Controller",
    "title": "LTV 근사 — 한 번의 iLQR형 갱신",
    "lead": "LTV는 시간에 따라 변하는 선형 모델, SQP는 최적화 방법, RTI는 제한된 갱신·실행 전략이다. 서로 동의어가 아니다.",
    "chips": [
      "LTV model",
      "one iLQR update",
      "not SQP-RTI reproduction"
    ],
    "runtime": {
      "controller": "ltv_mpc"
    },
    "body": "<div class=\"section\"><h3>현재 실제 동작</h3><div class=\"math\">A_k=∂f/∂x, B_k=∂f/∂u at (xbar_k,ubar_k)\nδx_(k+1)≈A_k δx_k+B_k δu_k+d_k\nd_k=f(xbar_k,ubar_k)−xbar_(k+1)</div><p>이 버튼은 nonlinear rollout의 stage별 Jacobian을 만들고 iLQR형 backward pass와 bounded line search를 한 번 수행한다. constrained QP와 preparation/feedback phase를 갖춘 acados SQP-RTI를 재현하지 않는다.</p></div><div class=\"section\"><h3>일반화 경계</h3><p>실행 가능한 nominal rollout이면 d_k=0이지만 임의 trajectory에서는 생략하면 안 된다. LTV MPC가 꼭 online relinearization에서 나오는 것도, 한 번의 갱신이 항상 deadline을 충족하는 것도 아니다. 수렴 여부 대신 실제 갱신 수락 여부를 따로 표시한다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "acados 공식 기능별 예제",
        "url": "https://docs.acados.org/features/"
      },
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      }
    ]
  },
  "centroidal_mpc": {
    "group": "Controller",
    "title": "축약 CoM 계획 — 전신 제어기의 구조적 비유",
    "lead": "두 상태의 수평 CoM planner와 full-state LQR를 연결한다. 접촉력·floating base를 포함한 humanoid centroidal controller의 재현은 아니다.",
    "chips": [
      "reduced-order",
      "momentum",
      "planner -> lower level"
    ],
    "runtime": {
      "controller": "centroidal_mpc"
    },
    "body": "<div class=\"section\"><h3>정확한 물리 관계와 근사</h3><div class=\"math\">M=m_c+m_p, α=m_p l/M\nc=p+α sinθ, c_dot=v+α cosθ·ω\nM c_ddot=F_net</div><p>이 이상화된 CartPole의 연속시간 수평 운동량 관계에서 F_net은 actuator와 외부 수평 힘의 합이다. 외력과 actuator 오차는 현재 nominal planner가 모른다.</p></div><div class=\"section\"><h3>실제 연결</h3><p>N=32의 2-state QP는 future [c,c_dot]를 계획한다. look-ahead reference를 받은 inner LQR가 다른 실제 force를 출력한다. planner force와 applied force는 같다고 가정할 수 없다.</p></div><div class=\"section\"><h3>시각화 경계</h3><p>이 planner에는 θ,ω 예측이 없다. 이를 임의로 0까지 줄인 4-state ghost를 실제 예측처럼 그리지 않는다. context graph에는 실제 [c,c_dot]만 표시하며, 축약 reference의 실현 가능성은 별도 문제다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      },
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      }
    ]
  },
  "full_nmpc": {
    "group": "Controller",
    "title": "Nonlinear MPC — 4-state nominal dynamics를 유지",
    "lead": "여기서 Full은 CartPole의 네 상태 전체를 뜻한다. humanoid full rigid-body model이나 전역적으로 정확한 solver라는 뜻은 아니다.",
    "chips": [
      "full state",
      "nonlinear rollout",
      "iterative solve"
    ],
    "runtime": {
      "controller": "full_nmpc"
    },
    "body": "<div class=\"section\"><h3>실제 구현</h3><p>N=30, official MuJoCo WASM의 nonlinear nominal transition으로 single shooting한다. 수치 Jacobian과 iLQR형 backward/forward 갱신을 최대 두 번 수행하고 force를 clip한다. 일반 constrained SQP나 box-DDP 해법과 같다고 표현하지 않는다.</p></div><div class=\"section\"><h3>모델과 알고리즘</h3><p>NMPC는 nonlinear f를 OCP에 남긴다는 뜻이다. SQP/iLQR 계열은 내부에서 국소 미분 근사를 사용하지만, sampling·derivative-free MPC까지 모두 선형화한다고 일반화할 수 없다. single/multiple shooting은 전사 방식이고 iLQR/SQP/IPM은 해법이다.</p></div><div class=\"section\"><h3>검증할 것</h3><p>제한 횟수 종료는 수렴·전역 최적성 보장이 아니다. clip한 rollout의 입력 범위, nonlinear defect, feasibility, 계산시간과 실패를 따로 기록한다. 모델 fidelity와 controller 성능은 동의어가 아니다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "acados 공식 기능별 예제",
        "url": "https://docs.acados.org/features/"
      },
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      },
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      }
    ]
  },
  "safety": {
    "group": "Controller",
    "title": "Backup 감독기 — 위험 예측과 실제 안전을 구분",
    "lead": "현재 NMPC의 nominal 예측이 margin을 넘으면 LQR로 바꾸는 휴리스틱이다. 실패 상황을 모두 처리하는 안전 필터는 아니다.",
    "chips": [
      "heuristic supervisor",
      "primary-plan scope",
      "not certified safety"
    ],
    "runtime": {
      "controller": "supervised_nmpc"
    },
    "body": "<div class=\"section\"><h3>현재 조건</h3><p>predicted world |p|와 |θ|를 검사한다. 화면의 primary plan은 backup 입력을 적용한 뒤의 경로가 아니므로 별도로 표시한다. 모든 solver exception·센서 오류·지연을 이 감독기가 복구하는 것은 아니다.</p></div><div class=\"section\"><h3>실물 적용 경계</h3><p>포화 LQR가 안전한 backup이라는 보장은 없다. 검증한 복구 영역, watchdog, 힘·에너지 제한, platform별 보호정지와 interlock이 필요하다. 시뮬레이터를 일시정지하는 처리는 실물 비상정지를 대신하지 않는다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "실제 구현과 범위",
        "url": "docs/IMPLEMENTATION_BOUNDARIES.md"
      },
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      }
    ]
  },
  "robust_mpc": {
    "group": "Controller",
    "title": "불확실성 MPC — model set, tube, 확률 제약은 서로 다르다",
    "lead": "현재 실행은 finite-model scenario-risk 목적함수다. 이 결과를 tube MPC나 확률적 안전 보장으로 승격하지 않는다.",
    "chips": [
      "5 nominal variants",
      "finite-model cost",
      "no probability guarantee"
    ],
    "runtime": {
      "controller": "scenario_mpc"
    },
    "body": "<div class=\"section\"><h3>현재 모델과 solver</h3><div class=\"math\">J=(1−ρ) mean_i J_i + ρ max_i J_i\n0≤ρ≤1인 해석: 평균과 표본 중 최대 비용의 절충</div><p>기본 5개 mass/length/friction 모델에 같은 control sequence를 적용한다. mean과 worst sampled cost를 혼합하고 목표 주변 soft band를 더한다. 제한된 projected-gradient 갱신이며 scenario별 hard rail을 강제하지 않는다.</p></div><div class=\"section\"><h3>별도 방법</h3><p>Tube는 허용 오차 집합과 feedback, min-max는 명시한 adversarial set, chance constraint는 분포·확률 가정이 필요하다. 몇 개 모델을 시험한 것만으로 이 보장이 생기지 않는다. model sampling과 domain randomization도 같은 알고리즘은 아니다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      },
      {
        "label": "제어기 구현",
        "url": "docs/CONTROLLERS.md"
      }
    ]
  },
  "ppo": {
    "group": "Controller",
    "title": "PPO — 학습된 정책의 추론",
    "lead": "PPO는 policy-gradient 학습 알고리즘이지 MPC 해를 그대로 저장하는 방법이 아니다. 현재는 고정된 actor의 forward pass만 수행한다.",
    "chips": [
      "learned policy",
      "offline training",
      "cheap inference"
    ],
    "runtime": {
      "controller": "ppo"
    },
    "body": "<div class=\"section\"><h3>현재 실행</h3><p>정규화된 observation을 actor에 넣고 left/right 중 높은 확률의 방향을 골라 ±force를 출력한다. action을 확률적으로 샘플링하는 평가와도 구분한다. 새 actor를 여기서 학습한 것은 아니다.</p></div><div class=\"section\"><h3>공정한 비교</h3><p>PPO의 reward, training observations, 종료조건, 데이터가 다른데 MPC와 같은 표에 놓았다는 사실만으로 우열을 정할 수 없다. 특정 MPC를 모방 학습했다는 근거도 없으며, policy가 constraint나 안전을 보장하지 않는다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "PPO 원 논문",
        "url": "https://arxiv.org/abs/1707.06347"
      },
      {
        "label": "실제 구현과 범위",
        "url": "docs/IMPLEMENTATION_BOUNDARIES.md"
      }
    ]
  },
  "raw": {
    "group": "Observer",
    "title": "Raw + 차분 — 관측 가능한 값의 기준선",
    "lead": "센서에서 p,θ만 받고 속도는 연속된 유효 측정의 차분으로 계산한다.",
    "chips": [
      "baseline",
      "finite difference",
      "noise amplification"
    ],
    "runtime": {
      "observer": "raw"
    },
    "body": "<div class=\"section\"><h3>시간 간격</h3><div class=\"math\">v_hat=(p_new−p_previous_valid)/Δt\nω_hat=wrap(θ_new−θ_previous_valid)/Δt</div><p>누락 중에는 값을 유지하되 측정 갱신으로 계산하지 않는다. 다음 유효 측정의 속도는 실제 경과한 샘플 간격 Δt로 나눈다. calibrated P는 제공하지 않는다.</p></div><div class=\"section\"><h3>잡음 해석</h3><p>독립이고 동일 분산인 두 측정 잡음에서 차분 분산은 2σ²/Δt²다. 유색 잡음이면 상관항을 포함해야 한다. 이 기준선이 모든 센서에 부적절하다는 뜻은 아니다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "kf": {
    "group": "Observer",
    "title": "Kalman filter — 선형 모형 아래의 재귀 추정",
    "lead": "잡음·초기 prior·관측 가능성 가정과 코드의 heuristic 설정을 구분한다. Gaussian 가정에서는 조건부 평균/공분산을 계산하며, 비Gaussian에서는 같은 최적성 주장을 그대로 적용하지 않는다.",
    "chips": [
      "P Q R",
      "Kalman gain",
      "linear/Gaussian"
    ],
    "runtime": {
      "observer": "kf"
    },
    "body": "<div class=\"section\"><h3>핵심 식</h3><div class=\"math\">x_minus=A x_plus+B u\nP_minus=A P_plus Aᵀ+Q_e\nν=y−H x_minus\nS=H P_minus Hᵀ+R_e\nK=P_minus Hᵀ S⁻¹\nx_plus=x_minus+Kν</div><p>코드에서 controller와 observer는 각각 Q/R 이름을 쓰지만 역할은 다르다. 여기의 Q_e,R_e는 discrete-time 잡음 공분산이다. correlated noise에는 이 기본식 외의 항이 필요하다.</p></div><div class=\"section\"><h3>실제 센서와 정보</h3><p>현재 H는 [p,θ] 선택 행렬이다. dynamics coupling과 cross covariance로 속도를 교정하지만 관측 불가능한 상태를 자동 복구하지는 않는다. R_e는 시뮬레이터 주입 분산을 사용하므로 실측 calibration과 구분한다. 누락에는 predict-only, covariance 갱신에는 Joseph form을 사용한다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      },
      {
        "label": "FilterPy 공식 KF 문서",
        "url": "https://filterpy.readthedocs.io/en/latest/kalman/KalmanFilter.html"
      }
    ]
  },
  "ekf": {
    "group": "Observer",
    "title": "EKF — nonlinear mean과 국소 covariance 근사",
    "lead": "mean은 nominal nonlinear transition으로, covariance는 그 Jacobian으로 전파한다. EKF도 선형화하며, 언제나 KF보다 정확한 것은 아니다.",
    "chips": [
      "nonlinear f",
      "Jacobian",
      "local linearization"
    ],
    "runtime": {
      "observer": "ekf"
    },
    "body": "<div class=\"section\"><h3>실제 구현</h3><div class=\"math\">x_minus=f(x_plus,u)\nF=∂f/∂x at x_plus\nP_minus=F P_plus Fᵀ+Q_e</div><p>20 ms MuJoCo nominal 전이의 central-difference Jacobian을 사용한다. 측정 함수는 현재 linear selection H라서 update는 기본 Kalman 형태다. 일반 nonlinear h에는 측정 Jacobian도 필요하다.</p></div><div class=\"section\"><h3>비교 전제</h3><p>Nonlinear bench는 controller·noise·초기 상태를 고정한 예제다. 과거의 mixed-unit RMSE 수치를 현재 보편 성능처럼 제시하지 않는다. 관측 가능성, 초기 오차, 편향, 공분산 일관성을 함께 평가한다. 기본 EKF의 unwrapped measurement residual과 SO(2) 비교 모드의 차이도 확인한다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      },
      {
        "label": "모델·전사·솔버 구분",
        "url": "docs/MODEL_HIERARCHY.md"
      }
    ]
  },
  "ukf": {
    "group": "Observer",
    "title": "UKF — sigma points를 통한 모멘트 근사",
    "lead": "sigma points는 무작위 표본이 아니다. 비선형 전이를 통한 평균·공분산 근사이며, Jacobian을 쓰지 않는다는 이유로 항상 정확하거나 빠른 것은 아니다.",
    "chips": [
      "sigma points",
      "nonlinear propagation",
      "Gaussian belief"
    ],
    "runtime": {
      "observer": "ukf"
    },
    "body": "<div class=\"section\"><h3>현재 prediction</h3><p>4-state covariance에서 sigma points를 만들고 nonlinear nominal 전이에 통과시킨다. 각도는 circular mean과 wrapped difference를 사용한다. 단일 local belief 표현이므로 넓은 각도·multimodal contact에는 한계가 있다.</p></div><div class=\"section\"><h3>현재 measurement update</h3><div class=\"math\">P_xz=P_minus Hᵀ\nS=H P_minus Hᵀ+R_e\nK=P_xz S⁻¹</div><p>이 lab의 h(x)=[p,θ]는 local angle chart에서 선형이므로 측정 sigma points를 다시 만드는 구현이 아니다. process Q_e까지 포함한 P_minus를 H로 투영한다. 일반 nonlinear h에서는 재-sigma-point나 augmented-noise 구성을 별도로 설계해야 한다.</p></div><div class=\"section\"><h3>분포 해석</h3><p>Gaussian posterior를 정확히 계산한다고 부르지 않는다. nonlinear mapping은 Gaussian을 보존하지 않으며, 이 구현은 모멘트를 단일 local covariance로 요약한다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "inekf": {
    "group": "Observer",
    "title": "InEKF와 SO(2) — 같은 구현이 아니다",
    "lead": "적절한 Lie-group 대칭성을 쓰는 invariant filter와 각도 wrap을 구분한다.",
    "chips": [
      "Lie group",
      "invariant error",
      "contact aided"
    ],
    "runtime": {
      "observer": "so2"
    },
    "body": "<div class=\"section\"><h3>실행되는 것</h3><div class=\"math\">θ_meas=−179°, θ_hat=+179°\nraw difference=−358°; wrapped difference=+2°</div><p>SO(2) 모드는 EKF의 각도 innovation과 보정 각도를 wrap한다. 별도의 Hartley contact-aided InEKF 상태·접촉모델은 없다. 일반 EKF에도 angle wrap을 넣을 수 있으며 이것만으로 InEKF가 되지 않는다.</p></div><div class=\"section\"><h3>논문의 범위</h3><p>Hartley의 contact-inertial model은 IMU와 leg kinematics를 결합한다. 적절한 group-affine 구조에서 log-linear error 특성이 유용하다. bias 확장과 접촉 오류가 있을 때 모든 성질이 그대로 유지된다고 일반화하지 않는다. absolute sensing이 없으면 전역 translation/yaw gauge가 남는다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "Hartley contact-aided InEKF",
        "url": "https://arxiv.org/abs/1904.09251"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "mhe": {
    "group": "Observer",
    "title": "MHE — window 최적화라는 별도 추정 선택",
    "lead": "MHE는 EKF 뒤에 반드시 추가하는 상위 단계가 아니다. window, 제약, 지연 처리의 이점과 계산·arrival approximation 부담을 비교해 선택한다.",
    "chips": [
      "windowed optimization",
      "arrival cost",
      "constraints"
    ],
    "runtime": {
      "observer": "mhe"
    },
    "body": "<div class=\"section\"><h3>일반 형태</h3><div class=\"math\">min Γ(x_(k−N))+Σ||w_i||²_(Q_e⁻¹)+Σ||v_i||²_(R_e⁻¹)\ns.t. x_(i+1)=f(x_i,u_i)+w_i, y_i=h(x_i)+v_i\noptional state/input constraints</div><p>arrival cost는 window 이전 정보의 요약이다. 선형 MHE도 있고 nonlinear MHE도 있으며, hard constraints는 사용 목적에 따라 선택한다.</p></div><div class=\"section\"><h3>현재 제한</h3><p>window=8 transition, 0.16 s. EKF posterior를 approximate arrival로 두고 첫 state의 4개 변수만 Gauss-Newton으로 최적화한다. arrival의 측정은 다시 세지 않는다. process-noise trajectory와 hard state constraints, calibrated output covariance는 없다. 표시된 residual은 window fit 뒤의 값이지 prefit innovation NIS가 아니다.</p></div><div class=\"section\"><h3>논문 예제와 구분</h3><p>EKF+MHE legged 연구는 orientation과 velocity 문제를 분리하고 OSQP를 사용한 200 Hz/0.1 s 예제를 보고했다. 그 환경·센서·구성의 결과이며 이 browser의 실행 주기나 필수 표준이 아니다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "EKF+MHE 원문",
        "url": "https://arxiv.org/html/2405.20567v1"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "consistency": {
    "group": "Observer",
    "title": "공분산 일관성 — 통계의 가정을 먼저 확인",
    "lead": "낮은 추정 오차와 정직한 불확실성은 다르다. 수치상 P가 존재하는 것과 calibrated P라는 것은 같은 말이 아니다.",
    "chips": [
      "NIS",
      "NEES",
      "chi-square",
      "calibration"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>두 통계</h3><div class=\"math\">NIS=νᵀ S⁻¹ν  (measurement dimension)\nNEES=eᵀ P⁻¹e (tested nonsingular error coordinates)</div><p>NIS는 prefit innovation과 S를, NEES는 동일 시각의 truth와 estimate error/P를 사용한다. Gaussian·정확한 covariance 모형 아래에서 각각 유효 차원의 χ² 분포를 따른다. nonlinear/adaptive filter의 통계는 진단이지 자동 증명이 아니다.</p></div><div class=\"section\"><h3>해석 주의</h3><p>평균이 기대 차원에 가깝다는 것만으로 충분하지 않다. 시간상관, 편향, whiteness, coverage, 반복 trial과 유효 자유도를 확인한다. adaptive R을 같은 innovation으로 정하면 고정-noise χ² 가정도 달라진다. raw/residual/MHE의 미제공 output P를 0이나 base P로 대체하지 않는다.</p></div>",
    "scope": "concept",
    "sources": [
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      },
      {
        "label": "평가 설계",
        "url": "docs/EXPERIMENT_DESIGN.md"
      }
    ]
  },
  "lin": {
    "group": "Hybrid estimator",
    "title": "Lin — learned contact event",
    "lead": "논문은 proprioceptive history로 contact event를 추정해 contact-aided invariant filter에 전달한다. CartPole에서 이 contact classifier를 실행하지는 않는다.",
    "chips": [
      "learned contact configuration",
      "temporal CNN",
      "InEKF"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>학습이 들어가는 위치</h3><p>센서 이력 → contact event → kinematic update의 포함 여부. 이는 state 전체를 직접 추정하는 NN이나 covariance learning과 다른 interface다.</p></div><div class=\"section\"><h3>일반화 경계</h3><p>contact=true여도 발의 stationary/no-slip 가정이 유효한지는 별도다. 특정 논문의 window·layer·threshold를 모든 로봇의 필수 설계로 가져오지 않는다. 정확한 재현 시에는 원문과 code revision을 고정해 확인한다.</p></div>",
    "scope": "paper",
    "sources": [
      {
        "label": "Lin et al. CoRL 논문",
        "url": "https://proceedings.mlr.press/v164/lin22b.html"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "youm": {
    "group": "Hybrid estimator",
    "title": "Youm NMN — learned measurement",
    "lead": "신경망의 contact·velocity 정보를 invariant filter의 측정으로 결합하는 논문 계열이다. 이 lab에는 해당 로봇과 network의 재현이 없다.",
    "chips": [
      "GRU",
      "neural measurement",
      "velocity"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>학습이 들어가는 위치</h3><p>시뮬레이션에서 학습한 상태 관련 예측을 measurement로 fuse한다. contact gating만 학습하는 방법보다 metric velocity의 편향이 더 직접적으로 추정에 들어갈 수 있다.</p></div><div class=\"section\"><h3>신뢰도와 중복 정보</h3><p>같은 IMU·encoder로 만든 learned measurement와 filter prior는 독립이지 않을 수 있다. source frame, 지연, normalization, bias, cross-correlation과 OOD를 검증해야 한다. 특정 layer 크기와 threshold는 이 페이지의 일반 규칙으로 제시하지 않는다.</p></div>",
    "scope": "paper",
    "sources": [
      {
        "label": "Youm NMN 원문 초록",
        "url": "https://arxiv.org/abs/2402.00366"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "innkf": {
    "group": "Hybrid estimator",
    "title": "InNKF 비유 — posterior 출력 보정",
    "lead": "원 논문은 invariant state와 temporal neural compensator를 결합한다. 여기서 실행되는 것은 SO(2)-aware EKF 뒤의 작은 residual MLP다.",
    "chips": [
      "TCN",
      "posterior residual",
      "hybrid"
    ],
    "runtime": {
      "observer": "residual"
    },
    "body": "<div class=\"section\"><h3>현재 정보 흐름</h3><p>기본 EKF posterior → residual MLP → corrected output. corrected output을 base EKF에는 되먹이지 않지만 controller는 그 출력을 받으므로 plant/sensor를 통한 폐루프 영향은 존재한다.</p></div><div class=\"section\"><h3>공분산과 재현 범위</h3><p>base P는 corrected-output P가 아니다. 이 구현에는 보정 출력의 calibrated covariance가 없다. 논문의 TCN, SE₂(3) 구성, 학습·하드웨어 성능을 재현한 것으로 부르지 않는다. offline residual fit 향상은 폐루프 개선의 충분조건이 아니다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "InNKF 원문",
        "url": "https://arxiv.org/html/2503.00344v1"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "coco": {
    "group": "Hybrid estimator",
    "title": "CoCo-InEKF — contact process covariance",
    "lead": "원문은 contact candidate의 velocity/process covariance를 학습하고 candidate state를 유지한다. observation R을 단순히 키우는 방법과 다르다.",
    "chips": [
      "process covariance",
      "directional slip",
      "differentiable InEKF"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>논문에서 학습하는 것</h3><p>각 후보의 lower-triangular L로 Σ=L Lᵀ를 구성해 directional uncertainty를 표현한다. 이는 positive-semidefinite를 보장할 뿐 모든 고유값이 양수이거나 물리 contact label이 정확하다는 보장은 아니다.</p></div><div class=\"section\"><h3>학습 데이터의 범위</h3><p>직접 contact label이 필요 없다는 말은 state ground truth도 필요 없다는 뜻이 아니다. 원문의 state-error training은 시뮬레이션 정답을 사용한다. CartPole에는 해당 contact-state가 없으므로 이 페이지는 논문 설명이며 Adaptive-R을 CoCo 구현으로 연결하지 않는다.</p></div>",
    "scope": "paper",
    "sources": [
      {
        "label": "CoCo-InEKF 원문 §III",
        "url": "https://arxiv.org/html/2605.15122v1"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "focus": {
    "group": "Hybrid estimator",
    "title": "FOCUS 비유 — contact와 FK 신뢰도를 구분",
    "lead": "공식 초록에서 확인되는 아이디어는 per-foot FK reliability로 velocity observation과 covariance를 조정하는 것이다.",
    "chips": [
      "abstract verified",
      "local heuristic only",
      "no reproduction"
    ],
    "runtime": {
      "observer": "adaptive"
    },
    "body": "<div class=\"section\"><h3>논문과 로컬의 차이</h3><p>FOCUS는 FK velocity와 IMU-propagated velocity를 reliability에 따라 결합한다. 현재 Adaptive-R은 position/angle innovation 기반 휴리스틱이며 foot, FK blending, learned reliability network가 없다.</p></div><div class=\"section\"><h3>가정과 근거 수준</h3><p>동일 센서에서 얻은 prior·pseudo-measurement의 상관을 무시하면 과신할 수 있다. 현재 R_e 조절은 같은 innovation을 사용하므로 classical fixed-noise optimality는 주장하지 않는다. 이번 검토에서 원문 상세식을 직접 재확인하지 못해 network 크기·세부 scale 공식을 일반 법칙처럼 싣지 않는다.</p></div>",
    "scope": "analogue",
    "sources": [
      {
        "label": "FOCUS 공식 초록",
        "url": "https://arxiv.org/abs/2609.02222"
      },
      {
        "label": "관측기·단위·통계 조건",
        "url": "docs/OBSERVERS.md"
      }
    ]
  },
  "sysid": {
    "group": "Commissioning",
    "title": "System ID — 최적화 숫자와 식별 가능성을 구분",
    "lead": "모델 파라미터의 유일성·물리 타당성·별도 궤적 예측을 확인한 다음 제어 튜닝을 해석한다.",
    "chips": [
      "persistent excitation",
      "physical parameters",
      "identifiability"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>현재 offline 예제</h3><p>src/commissioning.js는 simulation truth state로 one-step prediction error를 맞춘다. actuator 실험도 realized force를 사용한다. 이는 privileged simulation ID이며 encoder/IMU만으로 같은 파라미터를 식별했다는 증거는 아니다.</p></div><div class=\"section\"><h3>확인할 것</h3><p>excitation, sensitivity rank/conditioning, parameter correlations와 admissible inertia를 확인한다. 낮은 one-step fit이 유일한 파라미터나 장기 rollout 정확도를 보장하지 않는다. contact force·inertia·friction·gain이 서로 오차를 설명할 수 있으므로 실험을 분리한다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "Commissioning 구현",
        "url": "docs/COMMISSIONING.md"
      },
      {
        "label": "식별 한계",
        "url": "docs/THEORY_FAILURE_MAP.md"
      }
    ]
  },
  "tuning": {
    "group": "Commissioning",
    "title": "Auto-tuning — 방법보다 평가·가정이 먼저",
    "lead": "단일 black-box optimizer도 타당한 baseline일 수 있다. 파라미터 구조·미분 가능성·비용·위험에 따라 방법을 선택하며 SOTA라는 명칭으로 결정하지 않는다.",
    "chips": [
      "DiffTune",
      "Bayesian optimization",
      "Safe BO",
      "bilevel calibration"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>방법 선택</h3><p>연속 weight는 유효한 민감도가 있을 때 gradient, 이산 horizon·solver는 structured/mixed-variable search가 후보이다. 일반 CMA-ES를 정수·범주 변수에 그대로 적용하는 것은 별도 설계가 필요하다. 양수 parameterization은 한 방법이지 모든 값에 지수함수가 필수인 것은 아니다.</p></div><div class=\"section\"><h3>Safe BO 경계</h3><p>Safe BO는 안전 seed, confidence/model 가정, 제한된 탐색 영역과 독립 보호장치가 있을 때 고려한다. 모든 hardware tuning에 필수이거나 failure-free 보장을 자동 제공하지 않는다. DiffTune도 smooth/regular solution과 solver 민감도 가정을 확인해야 한다.</p></div><div class=\"section\"><h3>현재 구현과 시험 분리</h3><p>현재 CEM과 작은 structured search는 연구 방법의 재현이 아니다. train/validation으로 선택하고 잠근 test로 평가한다. 현재 legacy tuner는 validation과 test를 admission에 함께 사용하므로 반복 개발 후 그 test를 untouched라고 부를 수 없다. 새 terminal 검증은 고정 manifest의 별도 실험이다.</p></div>",
    "scope": "concept",
    "sources": [
      {
        "label": "DiffTune-MPC",
        "url": "https://arxiv.org/abs/2312.11384"
      },
      {
        "label": "Safe BO 원 논문",
        "url": "https://proceedings.mlr.press/v229/widmer23a.html"
      },
      {
        "label": "튜닝 방법과 한계",
        "url": "docs/TUNING_METHODS.md"
      }
    ]
  },
  "commissioning": {
    "group": "Commissioning",
    "title": "Commissioning — 모델·정보·평가를 검증하는 반복 과정",
    "lead": "식별→보정→설계→검증은 실무상 기본 순서이지 유일한 최적 경로나 필수 알고리즘 목록이 아니다. 검증 결과에 따라 앞 단계로 돌아간다.",
    "chips": [
      "system ID",
      "calibration",
      "held-out validation",
      "co-design"
    ],
    "runtime": null,
    "body": "<div class=\"section\"><h3>먼저 고정할 계약</h3><p>좌표·단위·시간·sensor validity·command/realized input·정보 출처를 먼저 정의한다. 그다음 모델/actuator 식별, observer calibration, controller 설계, 폐루프 검증을 수행한다. model ID를 튜닝 마지막에 넣는 고정 순서는 권장하지 않는다.</p></div><div class=\"section\"><h3>같이 기록할 항목</h3><p>tracking, componentwise estimation error, force와 slew, 실제/예측 제약, 실패/거부, residuals와 sensor-to-application age를 구분한다. state envelope 안에서 끝났다는 DONE은 목표 추종이나 safety certificate가 아니다.</p></div><div class=\"section\"><h3>일반화 한계</h3><p>finite scenario의 zero failure는 임의 OOD의 보장이 아니다. Q_e의 continuous spectral density와 discrete covariance도 다르다. robot-scale에서는 sensor observability와 contact/hardware 보호조건을 다시 검증해야 한다.</p></div>",
    "scope": "concept",
    "sources": [
      {
        "label": "Commissioning",
        "url": "docs/COMMISSIONING.md"
      },
      {
        "label": "시험 범위",
        "url": "docs/VALIDATION.md"
      }
    ]
  },
  "sim2real": {
    "group": "Commissioning",
    "title": "Sim2real stress — 동일 오차에도 원인은 다르다",
    "lead": "동역학·센서·actuator·전송의 단독 변화와 결합 변화를 나눠 보되, 실제 구현한 것만 설명한다.",
    "chips": [
      "single-factor ablation",
      "model mismatch",
      "latency",
      "actuator lag",
      "sensor bias",
      "held-out seeds"
    ],
    "runtime": {
      "scenario": "sim2real"
    },
    "body": "<div class=\"section\"><h3>현재 plant</h3><p>정해진 mass/length/friction/gain/lag/delay와 sensor-noise/bias 조건이다. actuator는 horizontal force port의 이상화 또는 heuristic 제한이다. 모터 전기회로·전원·접촉 robot의 완전한 모델이 아니다.</p></div><div class=\"section\"><h3>시간과 관측</h3><p>요청 u_cmd와 applied force는 다를 수 있고 observer가 모르는 입력 오차가 생긴다. 현재 timing은 synchronous simulated loop 계산 시간이다. acquisition/arrival timestamp·out-of-sequence fusion·하드웨어 통신 deadline을 실증한 것이 아니다.</p></div><div class=\"section\"><h3>재현성과 확장</h3><p>randomization은 identification을 대체하지 않는다. 조건·seed·초기 상태·goal·주기·asset hash를 함께 기록한다. 경계 사례를 발견했다고 전체 uncertainty set을 탐색한 것은 아니며 simulation-to-simulation 수치 일치도 real-world 검증과 다르다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "MuJoCo 모델 경계",
        "url": "docs/MUJOCO_WASM_RUNTIME.md"
      },
      {
        "label": "진단 항목",
        "url": "docs/THEORY_FAILURE_MAP.md"
      }
    ]
  },
  "faults": {
    "group": "Commissioning",
    "title": "고장·성능저하 — 누락, 고착, 잡음은 서로 다르다",
    "lead": "센서 누락을 이전 값의 새 관측으로 처리하지 않는다. 같은 값이 도착한 고착과 알려진 통신 누락은 구분한다.",
    "chips": [
      "colored noise",
      "dropout",
      "stuck sensor",
      "jitter",
      "torque-speed",
      "thermal"
    ],
    "runtime": {
      "scenario": "colored"
    },
    "body": "<div class=\"section\"><h3>센서</h3><p>colored는 시간상관 잡음, dropout은 stale metadata와 predict-only, stuck은 repeated but delivered 값을 갖는 별도 fault다. 일정 횟수 outlier 검출만으로 fault isolation·관측 가능성 회복을 보장하지 않는다.</p></div><div class=\"section\"><h3>구동·전송</h3><p>jitter는 command hold/변동 지연의 단순화, torque_speed와 thermal은 force 제한의 heuristic이다. 실제 motor의 전압·전류·역기전력·냉각·배터리 모델을 검증한 것으로 해석하지 않는다.</p></div><div class=\"section\"><h3>진단 원칙</h3><p>무엇이 주입됐는지 아는 test harness와 실제 sensor-only fault detector를 혼동하지 않는다. 이 페이지의 성공은 주어진 fault script의 결과이며 실물 fault-tolerant control의 인증이 아니다.</p></div>",
    "scope": "implemented",
    "sources": [
      {
        "label": "구현 범위",
        "url": "docs/IMPLEMENTATION_BOUNDARIES.md"
      },
      {
        "label": "MuJoCo runtime",
        "url": "docs/MUJOCO_WASM_RUNTIME.md"
      }
    ]
  }
};
const CONTROL_LAB_TOPIC_ORDER=["overview", "pid", "lqr", "linear_mpc", "state_mpc", "ltv_mpc", "centroidal_mpc", "full_nmpc", "safety", "robust_mpc", "ppo", "raw", "kf", "ekf", "ukf", "inekf", "mhe", "consistency", "lin", "youm", "innkf", "coco", "focus", "sysid", "tuning", "commissioning", "sim2real", "faults"];
if(typeof module!=="undefined")module.exports={CONTROL_LAB_TOPICS,CONTROL_LAB_TOPIC_ORDER};
