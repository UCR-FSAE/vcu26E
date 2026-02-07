# VCU26E  

UCR FSAE Highlander Racing's Vehicle Control Unit. Developed and tested on 24E to be applied to 26E.  
Utilizes a simple while loop for reliability. The loop is as follows:
- Initialize
- Collect ADC
- Check Ready to Drive
  - Brakes Active
  - Driver Action
  - Tractive System Active
- Check rules compliance
  - APPS (Accelerator Pedal Position Sensor) Plausibility
  - BSE (Brake System Encoder) Plausibility
  - APPS + BSE Plausility
- Send Computed Torque Request

The Torque Request is calculated using a linear equation based on the maximum and minimum voltages of the APPS. 
BSE and APPS plausibility is checked in consideration to the measured maximum and minimum voltages of the APPS and the BSE.

To-Do:
- RTD with Tractive System Active
