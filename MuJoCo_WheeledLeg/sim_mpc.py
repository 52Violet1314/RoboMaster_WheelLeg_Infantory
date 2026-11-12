"""MuJoCo VMC + offline explicit MPC; shares the existing robot control loop."""
from sim_lqr import WheelLegLQR, main


class WheelLegMPC(WheelLegLQR):
    def __init__(self, *args, **kwargs):
        kwargs["controller"] = "mpc"
        super().__init__(*args, **kwargs)


if __name__ == "__main__":
    main(default_controller="mpc")
