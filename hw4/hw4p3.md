---
geometry: "margin=3cm"
---

## Problem 3

<!-- For each paper, write a brief summary (max half a page per paper) highlighting the main contributions,methodologies, and findings. Your summaries should demonstrate a clear understanding of the material -->

### i [Entry Guidance: A Unified Method, Ping Lu](https://arc.aiaa.org/doi/10.2514/1.62605)

The Main contrbutions of this paper are the development of a unified entry guidance algorithm that can be used for a wide range of entry vehicles. This algorithm is designed to be robust and adaptable, and minimizes the need to tune parameters for different vehicles reducing complexity for designing new missions based on existing vehicle designs as well.

The methodology used is a numerical predictor-corrector algorithm that adjusts bank angles dynamically by iteratively perdicting and correcting trajectory deviations. This algorithm is similar to a model predictive control scheme, but is more robust to changes in external conditions or model mismatch. In case of high Lift-to-Drag ratio vehicles, the algorithm compensates for attitude rate deviations between different phases of entry, ennabling efficient control over heating and structural load factors.

The key findings in this paper are that one, this algorith effectively demonstrates reliable performance across all tested vehicle types and mission scenarios. It works especially well in vehicles with a high lift-to-drag ratio vehicles, achieving a stable, controlled descent. Two, the algorithm is able to manage structural and thermal constraints effectively, ensuring the safety and performance of the vehicle in a high-stress atmospheric reentry scenario.

### ii [Mid-Lift-to-Drag Ratio Rigid Vehicle Control System Design and Simulation for Human Mars Entry, Breanna Johnson et al.](https://arc.aiaa.org/doi/epdf/10.2514/6.2018-0615)

The main contribution is the The Mid-Lift-to-Drag (Mid-L/D) Rigid Vehicle, MRV's, control system, which combines aerodynamic surfaces and a reaction control system (RCS) to enable stable and precise control during Mars entry. This hybrid approach provides an effective balance between aerodynamic stability and maneuverability in Mars’ thin atmosphere, addressing the complex requirements of human Mars landing missions. In addition, instead of traditional parachutes being used for descent, this paper adapts a Supersonic Retro-Propulsion (SRP) system to slow the vehicle down which is useful for landing heavier payloads on Mars given its thin atmosphere.

The methodology used is a 6DOF simulation with Monte Carlo analysis which helps achieve robust control design during the simulation phase under a variety of conditions. This simulation assesses MRV’s control performance, with specific attention to variations in RCS jet thrust, vehicle aerodynamic stability, and atmospheric conditions. This allowed for the identification of optimal bank rate limits and effective jet placements to maximize torque while minimizing propellant usage under a variety of conditions.

The key findings are that the MRV’s control system demonstrated sufficient stability and accuracy for a precise Mars landing, even under various atmospheric dispersions. The combined aerosurface and RCS control structure proved effective for managing vehicle orientation with minimal fuel consumption while carrying a heavy load. This paper also found that Supersonic Retro-Propulsion (SRP)s are feasible and effective for controlled descent in Mars’s low-density atmosphere, supporting the MRV’s ability to deliver large payloads while maintaining landing precision.

### iii [Pterodactyl: Development and Performance of Guidance Algorithms for a Mechanically Deployed Entry Vehicle, Breanna Johnson et al](https://arc.aiaa.org/doi/10.2514/6.2020-1011)

The main contributions here are specific for the Pterodactyl vehicle, which is a mechanically deployed entry vehicle (DEV) designed for Mars missions. These DEVs lack the rigid structure of traditional capsules, and offer more flexibility and potential efficiency in packing and deploying payloads for Mars or lunar missions. This paper also delves into the integration of a fully numerical predictor-corrector guidance algorithm for the Pterodactyl vehicle, which provides precise control over the vehicle’s entry trajectory, including bank angle adjustments for configurations requiring significant lateral maneuvering. For other configurations, an uncoupled range control (URC) was added to provide guidance through adjustments in angle of attack and sideslip angle. This paper also has extensive 6-DOF simulations to validate the guidance algorithms.

The methodology used in this paper is a combination of numerical predictor-corrector guidance algorithms and 6-DOF simulations with the use of Monte carlo analysis. This provides robust optimization of the controller which is essential for the Pterodactyl vehicle given its unique structure and mission profile. The guidance algorithms also takes into account the structural, aerodynamic, and thermal performance of the vehicle to ensure in-flight stablity. They use a custom FNPEG for bank angle control, and developed URC for vehicles requiring angle of attack and sideslip guidance.

They key findings are that the GNC algorithm was able to achieve precise control over the Pterodactyl vehicle’s landings under the contraints of miss distance, heat rate, and g-loads. It was also able to minimize fuel use by mainly using aerodynamic adjustments instead of RCS thrusters, which is especially valuable for future interplanetary missions with limited fuel on-board. Finally, this paper demonstrated the scalability of the GNC algorithm for future missions, showing that it can suppert a wide range of planetary missions with different vehicle configurations and mission profiles.

### iv Question for Breanna Johnson

How modular are the models used for atmosphere/gravity for different missions? Do you develop a new model for each mission based on the vehicle and mission profile or reuse parts of existing models used in previous missions? What I mean is, when doing simulations for a new mission with a new vehicle, how much of the vehicle's dynamics are made from scrach and how much is reused from previous missions?
