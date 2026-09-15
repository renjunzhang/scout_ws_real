"""A moving, braking, delayed-queue and liquid-tail fixture from production F."""
import numpy as np
from planning.task import load_task
from planning.optimizer import dynamics
from planning.validation import validate_plan


def make_plan():
    task=load_task(dict(task_id="moving-fixture",frame_id="map",route=[[0.,0.],[.2,0.]],
        start_state=[0.]*28,goal_pose=[.2,0.,0.],deadline=4.,stop_window=1.,
        actuator_parameters=[.112,.119,1.018,1.096],
        liquid_parameters=[3.2358258380412885,1047.0568854135608,1.,1.],height_coeff=1.8179308248624537,
        motion_limits=dict(actual_v_min=-.002,v_max=.8,omega_max=1.2,a_max=.6,alpha_max=1.2,jerk_max=1.),
        region=dict(id="room",frame_id="map",footprint_radius=.45,margin=.02,cells=[
            dict(id="room",s_begin=0.,s_end=.2,vertices=[[-1.,-1.],[2.,-1.],[2.,1.],[-1.,1.]])])) )
    F=dynamics(task); x=np.zeros(28); rows=[]; dt=task["dt"]
    for k in range(151):
        u=np.zeros(3)
        if k<150:
            target=.12*np.sin(np.pi*(k+1)/90)**2 if k+1<90 else 0.
            u[0]=(target-x[6])/dt
            next_x=np.asarray(F(x,u)).ravel()
            u[2]=(next_x[0]-x[0])/dt if k<120 else 0.
            next_x[4]=x[4]+dt*u[2]
        rows.append(dict(t=k*dt,state=x.tolist(),control=u.tolist(),
            phase="TAIL" if k>=120 else ("BRAKE" if k>=45 else "MOVE"),region_cell_id="room"))
        if k<150: x=next_x
    length=rows[-1]["state"][4]
    task["route"][-1][0]=length;task["goal_pose"][0]=rows[120]["state"][0]
    task["region"]["cells"][0]["s_end"]=length
    plan={key:task[key] for key in ("dt","transport_duration","deadline","stop_window","height_coeff",
        "frame_id","actuator_parameters","liquid_parameters","motion_limits","goal_pose",
        "goal_position_tolerance","goal_yaw_tolerance","stop_speed_tolerance","stop_omega_tolerance","route","region")}
    plan.update(schema_version=1,liquid_model_version=1,cost_model_version=3,plan_id="moving-fixture",
                region_id="room",task=task,samples=rows,optimization={"status":"FIXTURE_ONLY"})
    plan["validation"]=validate_plan(plan)
    return plan
