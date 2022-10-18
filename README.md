# Hack-the-North-2022
Physiotherapy tool to track patients' progress and improve their form at home!

## Link to Prototypes
- UI for Patients: https://tinyurl.com/bdeddw7c
- UI for Physiotherapists: https://tinyurl.com/fda3wett

## Details

What it does

Our project gives patients audio tips on where they are going wrong in real time and responds to voice commands to control the flow of their workout. 
Data from these workouts is then uploaded to the cloud for physiotherapists to track the progress of their patients!


How we built it

Using the Kinect C# SDK we extracted a human wireframe from the sensors and performed calculations on the different limbs to detect improper form. 
We also used the .NET speech libraries to create an interactive "trainer" experience.


Challenges we ran into

Analyzing movements over time is pretty hard due to temporal stretching and temporal misalignment. 
We solved this by option to code particular rules for difference exercises which were far more robust. 
This also allowed us to be more inclusive of all body sizes and shapes (everyone has different limb sizes relative to their entire body).
