# <div align="center">Hazard Identification Bot</div>


<h3>Team 3</h3>
<h3>MAE 148 Final Project Fall 2025</h3>
<p>
</p>
<img src="images\Car.jpg" width="605" height="501">
</div>

## Table of Contents
  <ol>
    <li><a href="#team-members">Team Members</a></li>
    <li><a href="#abstract">Abstract</a></li>
    <li><a href="#what-we-promised">What We Promised</a></li>
    <li><a href="#accomplishments">Accomplishments</a></li>
    <li><a href="#challenges">Challenges</a></li>
    <li><a href="#demonstration">Demonstration</a></li>
    
    
  </ol>
  
## Team Members

<ul>
  <li>Rohan Bhakta - Aerospace Engineering</li>
  <li>Ghislain Demeester - Electrical Engineering</li>
  <li>Darsh Pawani - Mechanical Engineering</li>
  <li>Andrew Gooboian - Electrical Engineering</li>
</ul>

## Abstract
The goal of our project, the "Hazard Identification Bot", is to more effectively and safely plan disposal of hazardous materials. By identifying various common hazard icons and saving their GPS coordinates for later reference, this bot will allow cleanup teams to target resources at particular locations depending on hazard conditions. This project involves developing a model to recognize these hazard icons, navigating to the hazard icon, and saving GPS coordinates.

## What We Promised
### Must Have:
* Identify at least 10 hazard icons
* Navigate to a hazard location after detection
* Save the GPS coordinates of the detection 

### Nice to Have:
* Idly traverse map in predefined route to scan larger area
* Display GPS coordinates in a UI

## Accomplishments
* OAKD Hazard Detection
  * We developed a YOLO model to detect 10+ hazard icons, and interfaced the model with the OAKD camera
* Hazard Navigation
  * We successfully used the Hazard Detections to make a PID controller to allow the bot to move towards the hazard icon
* GPS Logging
  * Integrated the GPS into ROS2
  * Developed a minimal UI to display the GPS coordiates and hazard classification of detections (displayed on localhost:5000)



## Challenges
* One of main issues we had was having the bot not crash into the detection, as we found the detection model wasn't fast enough to stop using traditional stopping critiera based on the image
  * We worked to implement the LIDAR with ROS2, allowing for the bot to stop itself when it detects the hazard is within the stopping distance
* Additionally, there were isses interfacing the depth detection with the ROS2 environment. As this project builds on the UCSD ECEMAE 148 docker image (djnighti/ucsd_robocar), it has and outdated depthai library that we updated
 
## Demonstration
* link the vid

