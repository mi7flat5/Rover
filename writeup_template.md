## Project: Search and Sample Return
### 

---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

The output video is located in the output folder in this repository. I utilized several cells in this notebook to test settings on individua photos, and to test different navigation heuristic functions, such as np.median(), np.mean() and np.max()

 

[//]: # (Image References)

[image1]: ./misc/rover_image.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points

### Notebook Analysis
#### 1. Running the notebook functions
I added transalation and rotation code to their respective functions, so that pix_to world() would function correctly. For the perception function first I transformed the image coming from the rover in the perspective transform function using the source and desttination variable defined earlier in the notebook. I then defined 3 single channel images one for each feature in the environment I wanted to select out. I then applied color threshholding for each. For navigable pixels I chose to mark any pixel navigable that had a value of 155 or above in all three channels. to single out the rocks I used a combination of of filtering for upper and lower bounds int eh color range, focusing on yellows.  For obstacles I chose to say that anything above a certain threshold would not be an obstacle. I used the same values as I did for navigable terrain. Anything above RGB(155,155,155) will be navigable, anything below will not be. After defining the binary truth images, I create rovercentric coordinates, which can then be converted to pixel coordinates and then added to the worldmap. 


### Autonomous Navigation and Mapping

I filled in the perception fucntion for the autonomous run the same as I did in the notebook. the only difference being that I limited update of worldmap to only times when the rover is a a low degree of pitch, and roll to help maintain higher fidelity. I also extracted rock and obstacle distances and angles. After completing and tuning the perception step, the rover was satisfying requirements to pass without changing the decision function. After several unsuccesful experiments with path planning, I chose to have the rover home in on nearby rocks and chose those angles for steering when they are within a certain distance. I added the ability to get unstuck, because there are a couple rocks in the scene that hang over what the rover sees as navigable. 

As it is now the rover will on most runs complete the requirements to pass, if it should fail it is usually due to picking up a rock in a strange place and getting stuck, though somewhat rare, it can usually unstick itself. This would not happen if I turn off rock collection. 

I tried to implement an a A-star pathing system, and was able to get it to navigate to one edge of the map, it was extremly jerky, and not really usabale. I think the problem was that I was trying to use the pixels in the worldmap, which can be both obstacle and navigable. To improve this I think I would need to create my own map, using the angles and distances from the perception step to identify what objects are where. I haven't yet came up with a solid scheme for doing that though. I came up with a way to triangulate object positions with this data, but it isn't very numerically robust. I loved this project, and I plan to keep thinking of ways to improve it. 






