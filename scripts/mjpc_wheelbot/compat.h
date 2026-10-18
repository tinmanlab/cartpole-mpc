// MuJoCo 3.7 mujoco.h:670 declares the added normal output nullable.
// Preserve the old eight-argument convenience call; discard only that new output.
#pragma once
#include <mujoco/mujoco.h>
inline mjtNum mjpc_ray_compat(const mjModel* m, const mjData* d,
    const mjtNum* p, const mjtNum* v, const mjtByte* group, mjtByte fixed,
    int exclude, int* geom) {
  return mj_ray(m,d,p,v,group,fixed,exclude,geom,nullptr);
}
#define mj_ray(m,d,p,v,g,f,e,id) mjpc_ray_compat(m,d,p,v,g,f,e,id)
