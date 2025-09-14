# mc-safe-rl-qp

This repository contains the source code used for the experiments in the paper *Safe Execution of RL Policies via Acceleration-based QP Constraint Enforcement*.  
It is designed to be built as part of an [`mc_rtc`](https://github.com/mc-rtc/mc-rtc-superbuild) superbuild environment.

Experiment logs used in the paper are also provided in the `ExperimentLogs` folder.

## Installation

1. **Install `mc_rtc` superbuild**  
   Follow the official instructions here:  
   👉 https://github.com/mc-rtc/mc-rtc-superbuild

2. **Add project sources**  
   Once you have a working installation, copy all folders from `all_mc_rtc_src` into your `mc_rtc` workspace.  
   Then, add these folders to the CMake configuration of the superbuild to compiled them.  
   Each sub-project also contains a `README` with additional details.

3. **Configure `mc_rtc`**  
   Copy the content of `config_mc_rtc` into your `~/.config/mc_rtc` directory.

## Running on the Unitree H1

- Navigate to your `mc_rtc` workspace at `src/rl_controller/policy`.
- To run in simulation:  

  ```bash
  mc_mujoco --sync
  ```


- To run on the real robot (or with the [Unitree Mujoco simulator](https://github.com/unitreerobotics/unitree_mujoco) if installed):

  ```bash
  MCControlH1
  ```

* Make sure to update `~/.config/mc_rtc/mc_rtc.yaml` to set the correct network interface.

**Switching modes (PureRL, TorqueTask, FDTask):**
Edit the `init` variable in:

* `rl_controller/etcRLController.yaml` (in `src`, requires rebuild), or the `install` folder (no rebuild needed).

## Running on the Kinova Gen3

1. **Install the Kinova policy environment**
   Follow the instructions in
   `open_source_kinova_policy/kinova_ussaclab_sim2real/README.md`
   to install the required Python environment.
   ⚠️ Note: This policy was not developed by the authors of the paper. Only the running scripts were modified to reproduce the experiments.

2. **Update `mc_rtc` configuration**

   * Uncomment the Kinova-related sections in:

     * `~/.config/mc_rtc/mc_rtc.yaml`
     * `~/.config/mc_rtc/plugins/ExternalForcesEstimator.yaml`
   * Comment out the H1-related sections.

3. **Run the controller**

   * For simulation:

     ```bash
     mc_mujoco --sync
     ```

   * For real hardware:

     ```bash
     mc_kortex
     ```

4. **Run the policy**
   Open a second terminal and run one of the following:

   * **Nominal behavior test**
     (switch the mode in the `mc_rtc` GUI FSM):

     ```bash
     cd open_source_kinova_policy/kinova_ussaclab_sim2real
     python3 scripts/sim2real/run_task_reach_two_points.py
     ```

   * **Collision behavior test**

     ```bash
     cd open_source_kinova_policy/kinova_ussaclab_sim2real
     python3 scripts/sim2real/run_task_reach_collision.py
     ```

