import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from arm_kinematics import kin_2dof_solver

# sample parameters used to generate gif
L1 = 0.35 
L2= 0.30
w1 = 1.0
w2 =-2.0         
wdot1 = 0.05
wdot2 = 0.0
t_stop = 6.0                

frame_step = 3
out_path = "figures/demo.gif"

# In essence, replicates the figure in main.py without sliders
results = kin_2dof_solver(L1, L2, w1, w2, wdot1, wdot2, t_stop=t_stop)
t = results["t"]
v_mag = np.linalg.norm(results["v_wrist"], axis=1)
a_mag = np.linalg.norm(results["a_wrist"], axis=1)
reach = L1 + L2
arrow_len = 0.3 * reach 

fig = plt.figure(figsize=(8, 4.2))
gs = fig.add_gridspec(2, 2, width_ratios=[1, 1.15], left=0.06, right=0.97,
                      top=0.90, bottom=0.13, wspace=0.28, hspace=0.45)
ax_arm = fig.add_subplot(gs[:, 0])
ax_v = fig.add_subplot(gs[0, 1])
ax_a = fig.add_subplot(gs[1, 1], sharex=ax_v)

ax_arm.set_xlim(-1.3 * reach, 1.3 * reach)
ax_arm.set_ylim(-1.3 * reach, 1.3 * reach)
ax_arm.set_aspect("equal")
ax_arm.grid(True, ls=":", alpha=0.6)
ax_arm.set_title("Arm motion (arrows show direction)", fontsize=10)

ax_v.plot(t, v_mag, lw=2)
ax_a.plot(t, a_mag, lw=2, color="tab:red")
ax_v.set_ylim(0, 1.1 * v_mag.max())
ax_a.set_ylim(0, 1.1 * a_mag.max())
ax_v.set_ylabel("|v| (m/s)")
ax_a.set_ylabel("|a| (m/s²)")
ax_a.set_xlabel("t (s)")
for ax in (ax_v, ax_a):
    ax.set_xlim(0, t_stop)
    ax.grid(True, ls=":")
v_marker = ax_v.axvline(0, color="k", ls="--", lw=1)
a_marker = ax_a.axvline(0, color="k", ls="--", lw=1)

arm_line, = ax_arm.plot([], [], "o-", lw=3, color="k")
vel_arrow = ax_arm.quiver(0, 0, 0, 0, color="tab:blue", angles="xy",
                        scale_units="xy", scale=1, label="velocity")
accel_arrow = ax_arm.quiver(0, 0, 0, 0, color="tab:red", angles="xy",
                        scale_units="xy", scale=1, label="acceleration")
ax_arm.legend(loc="upper right", fontsize=8)

fig.suptitle(rf"$L_1$={L1}, $L_2$={L2} m, $\omega_1$={w1}, $\omega_2$={w2} rad/s, "
             rf"$\dot\omega_1$={wdot1}, $\dot\omega_2$={wdot2} rad/s$^2$", fontsize=9)


def unit_scaled(vec):
    n = np.linalg.norm(vec)
    return vec / n * arrow_len if n > 0 else np.zeros(2)


def update(frame):
    k = frame * frame_step
    ex, ey = results["p_elbow"][k]
    wx, wy = results["p_wrist"][k]
    arm_line.set_data([0, ex, wx], [0, ey, wy])
    vel_arrow.set_offsets([[wx, wy]])
    vel_arrow.set_UVC(*unit_scaled(results["v_wrist"][k]))
    accel_arrow.set_offsets([[wx, wy]])
    accel_arrow.set_UVC(*unit_scaled(results["a_wrist"][k]))
    v_marker.set_xdata([t[k], t[k]])
    a_marker.set_xdata([t[k], t[k]])
    return arm_line, vel_arrow, accel_arrow, v_marker, a_marker


if __name__ == "__main__":
    n_frames = len(t) // frame_step
    fps = round(1 / (0.02 * frame_step))          # real-time playback
    anim = FuncAnimation(fig, update, frames=n_frames, blit=False)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    anim.save(out_path, writer=PillowWriter(fps=fps), dpi=90)
    size_mb = os.path.getsize(out_path) / 1e6
    print(f"saved {out_path}: {n_frames} frames at {fps} fps, {size_mb:.1f} MB")