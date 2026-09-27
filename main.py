from arm_kinematics import kin_2dof_solver
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
from matplotlib.animation import FuncAnimation

# list of adjustable values
param_names = [r"$L_1$", r"$L_2$", r"$\omega_1$", r"$\omega_2$", r"$\dot{\omega_1}$", r"$\dot{\omega_2}$"]

# in order: L1, L2, w1, w2, wdot1, wdot2 (see param_names)
param_vals = [0.35, 0.30, 0.5, -1, 0.05, 0]
param_ranges = [(0.1,1.0),(0.1,1.0),(-2.0, 2.0),(-2.0, 2.0),(-0.5, 0.5),(-0.5, 0.5)]

# animation runs from 0 seconds to 5 seconds
T_stop = 5.0


fig = plt.figure(figsize=(11, 7))
# gridspec used to divide plot into velocity, acceleration, and arm motion plots
gs = fig.add_gridspec(2,2, left= 0.03, right=0.97, top= 0.95, bottom= 0.35, wspace=0.20, hspace=0.45)

# velocity vs. time (top right), acceleration vs. time (bottom right), arm motion (left) 
ax_arm = fig.add_subplot(gs[:,0])
ax_vel = fig.add_subplot(gs[0,1])
ax_accel = fig.add_subplot(gs[1,1])

# create sliders to allow adjustable input parameters 
sliders = {}
for i in range(len(param_vals)):
    ax_s = fig.add_axes([0.12, 0.26 - i * 0.04, 0.60, 0.025])
    lo, hi = param_ranges[i]
    sliders[i] = Slider(ax_s, param_names[i], lo, hi, valinit=param_vals[i])

# create reset button
ax_reset = fig.add_axes([0.85, 0.05, 0.1, 0.04])
reset_button = Button(ax_reset, "Reset")

# run solver from arm_kinematics to get wrist position, velocity, acceleration and elbow position
results = kin_2dof_solver(L1=param_vals[0], L2=param_vals[1], w1=param_vals[2], w2=param_vals[3], wdot1=param_vals[4], wdot2=param_vals[5], t_stop=T_stop)
time = results["t"]

v_mag = np.linalg.norm(results["v_wrist"], axis=1)
a_mag = np.linalg.norm(results["a_wrist"], axis=1)

# plot velocity & acceleration magnitude against time in dedicated gridspec locations
v_line, = ax_vel.plot(time, v_mag, lw=2, color="tab:blue")
a_line, = ax_accel.plot(time, a_mag, lw=2, color="tab:red")
v_marker = ax_vel.axvline(0, color="k", ls="--", lw=1)
a_marker = ax_accel.axvline(0, color="k", ls="--", lw=1)

# format & label vel/accel graphs
ax_vel.set_ylabel(r"|v| $(\frac{m}{s})$")
ax_accel.set_xlabel(r"$t$ (s)")
ax_accel.set_ylabel(r"|a| $(\frac{m}{s^2})$")
ax_accel.set_xlabel(r"$t$ (s)")
ax_vel.grid(True, ls=":")
ax_accel.grid(True, ls=":")
for ax in (ax_vel, ax_accel):
        ax.relim()
        ax.autoscale_view()
ax_vel.set_ylim(bottom=0.0, top=1.1*np.max(v_mag))
ax_accel.set_ylim(bottom=0.0, top=1.1*np.max(a_mag))
        

# arm motion plot: set axis limits to ensure arm is always in view
reach = param_vals[0] + param_vals[1]
ax_arm.set_xlim(-1.2 * reach, 1.2 * reach)
ax_arm.set_ylim(-1.2 * reach, 1.2 * reach)

# format plot
ax_arm.set_aspect("equal")
ax_arm.grid(True, ls=":", alpha=0.6)
ax_arm.set_title("Arm Motion")
ax_arm.text(0.02, 0.05, "Velocity/Acceleration arrows capped at 0.3 * reach",
            transform=ax_arm.transAxes, fontsize=8, color="gray")
ax_arm.text(0.02, 0.02, "* see magnitude plots (right) for accurate info.",
            transform=ax_arm.transAxes, fontsize=8, color="gray")

# initialize wrist velocity & acceleration vectors on plot at wrist location 
arm_line, = ax_arm.plot([],[], "o-", lw=3, color="k")
wx,wy= results["p_wrist"][0]
vel_arrow = ax_arm.quiver(wx,wy, *results["v_wrist"][0], color="tab:blue", 
                          angles="xy", scale_units="xy", scale=1, label=r"$v_{wrist}$")
accel_arrow = ax_arm.quiver(wx, wy, *results["a_wrist"][0], color="tab:red",
                        angles="xy",scale_units="xy", scale=1, label=r"$a_{wrist}$")
ax_arm.legend(loc="upper right")

state = {"k":0, "res": results}
frame_step = 4
def update(_frame):
    '''Updates the current arm visualizer to the next frame; updates velocity/acceleration vectors magnitude/position to next frame'''
    r = state["res"]
    k = state["k"]

    # gets current elbow/wrist position for new frame
    ex,ey = r["p_elbow"][k]
    wx,wy = r["p_wrist"][k]
    arm_line.set_data([0,ex,wx],[0,ey,wy])
    reach = sliders[0].val + sliders[1].val

    # updates arm motion velocity/acceleration vector direction in new time step
    v = r["v_wrist"][k]
    n = np.linalg.norm(v)
    if n <= 0.5 * reach:
        vel_arrow.set_UVC(*v)
    else:
        vel_arrow.set_UVC(*(v / n * 0.5 * reach))

    a = r["a_wrist"][k]
    m = np.linalg.norm(a)
    if m <= 0.5 * reach:
        accel_arrow.set_UVC(*a)
    else:
        accel_arrow.set_UVC(*(a / m * 0.5 * reach))    

    vel_arrow.set_offsets([wx,wy])
    accel_arrow.set_offsets([wx,wy])

    t_k = r["t"][k]
    # shows on velocity/acceleration plots where the arm motion currently is 
    v_marker.set_xdata([t_k,t_k])
    a_marker.set_xdata([t_k,t_k])

    # updates next frame to be two time array indices away to reduce total animation frames
    state["k"] = (k + frame_step) % len(r["t"])   # loop back to start
    return arm_line, vel_arrow, accel_arrow, v_marker, a_marker

anim = FuncAnimation(fig, update, interval=1000 * 0.02 * frame_step, blit=False, cache_frame_data=False)

def on_change(_val):
    '''Resets current animation cycle upon changed slider values & solves for new kinematics arrays with new inputs'''
    vals = [sliders[i].val for i in range(len(sliders))]
    r = kin_2dof_solver(*vals, t_stop=T_stop);
    state["res"] = r
    state["k"] = 0

    v_mag = np.linalg.norm(r["v_wrist"], axis=1)
    a_mag = np.linalg.norm(r["a_wrist"], axis=1)

    v_line.set_data(r["t"], v_mag)
    a_line.set_data(r["t"], a_mag)
    ax_vel.set_ylim(bottom=0.0, top=1.1 * np.max(v_mag))
    ax_accel.set_ylim(bottom=0.0, top=1.1 * np.max(a_mag))
    reach = vals[0] + vals[1]
    ax_arm.set_xlim(-1.2 * reach, 1.2 * reach)
    ax_arm.set_ylim(-1.2 * reach, 1.2 * reach)

for s in sliders.values():
    s.on_changed(on_change)

def on_reset(_event):
    '''Resets every slider value to standard params (param_vals) upon reset button click'''
    for s in sliders.values():
        s.reset()

reset_button.on_clicked(on_reset)

plt.show()
