# tom-jerry   
#Project Description   
Our project is a game of cat and mouse between two robots we named “Tom” and “Jerry.” Jerry wins when he obtains the cheese, Tom wins when he catches Jerry. To avoid stalemates, Tom receives a penalty the longer he takes to catch Jerry (and accordingly Jerry gets a reward for staying alive longer). Our game is represented as a grid of squares, with Tom and Jerry traveling strictly along this grid. Neither Tom or Jerry are explicitly programmed to play this game, they were trained by a reinforcement learning algorithm known as the Minimax Q learning algorithm.This algorithm finds the optimal solution for both agents in the game in the sense both agents maximize their rewards assuming that their opponent is also maximizing their reward perfectly.

#System Architecture    
## *reset_world.py*   

## *Minimax Q Learning*: Minimax_Qlearning.py, Q_matrix.py, tom_jerry_game.py   
`q_matrix.py` is meant to be an abstract representation of a q-matrix, so that incase we had to change its underlying representation due to time complexity concerns, we wouldn’t have to change much. `minimax_qlearning` is a representation of the training procedure on a particular game with a particular set of parameters. This is the main driver of our learning algorithm, and includes code from the other modules above. The main training loop occurs in the `train_q_matrix. It publishes actions for live games in `publish_according_to_state_updates`. The module `tom_jerry_game.py` is meant to be an abstract representation of a Tom & Jerry game, i.e. two robots chasing each other around. It contains code to generate state and action matrices, as well as the reward function for the training algorithm, and other parameters.   


## *State.py*     
The state component involved generating the state list, action list, and action matrix. We chose to represent states as Tom’s position and Jerry’s position. Our position object had 3 parts: an x, y, and z coordinate. The x and y coordinate correspond to real coordinates in the Gazebo world. The z coordinate was an integer in range [0,3] where 0 = North, 1 = East, 2 = South, 3 = West. We determined the x and y coordinates by dividing the occupancy grid into squares and then taking the midpoint of those squares. We then used the map data information (origin, resolution, etc.) to convert these values to real-world coordinates. This portion of the code happens in get_grid, get_midpoint, and get_valid. For each x,y pair, we permuted the 4 directions. We then permuted all the positions in pairs of 2, one for Tom, one for Jerry. This constituted our final state list. There were only 4 options for actions: go forward, turn 90 degrees clockwise, turn 90 degrees counterclockwise, or do nothing. These actions were represented by integers [0,3]. The action list is pairs of these actions, one for Tom, one for Jerry. Finally, the action matrix is a 2D array with dimensions number_of_states X number_of_actions. An entry action_matrix[state1][action] is the state that would result if the action pair were taken from starting state1.  

## *Movement*   
There are 2 main movements in our setup: go forward or turn 90 degrees. We use a timing based approach to execute a 90 degree turn. Essentially, we use the time function to compute how much the robot has turned so far and continue turning until this number matches our desired angle (see function turn_90 in movement.py). We take the same approach for moving forward, moving until the distance calculated based on time matches the desired distance (see function move_forward in movement.py). 


# Challenges, Future work, Takeaways   

## Challenges    
We faced many challenges. The first was selecting an algorithm. We considered several options, including variations of q learning and hereditary algorithms. Eventually, we settled on minimax, since our game has two diametrically opposed players. It was difficult to understand the literature and use it to implement an algorithm that worked for our particular design. We also had trouble setting up the boiler plate for the project, including launch files, messages, topics, worlds, maps, etc. Earlier in the quarter, this stuff was taken care of for us, so doing it on our own proved challenging. Implementing the state code was difficult because we wanted it to be somewhat map agnostic, so we had to write code that adapted the state space to the occupancy grid. This led to lots of bugs as we tried to map between the abstraction of the occupancy grid, the real coordinate system, and our imposed grid system. Finally, time management was difficult, because it was hard to estimate correctly how long something was going to take. Issues with NoMachine, subpar prioritization, and the general craziness of finals contributed to a time crunch. 

## Future work   
We did not get to implement everything that we dreamed about due to lack of time and lack of skill. For instance, we wanted the mouse to use its environment to help it avoid the cat, either by hiding or by taking paths that the cat could not fit through. We also simplified the states and movements quite a bit to reduce complexity of implementation and for space efficiency, as the state matrix can get unwieldy. With more time, we would experiment with the trade-offs between complexity and efficiency. If the states were more granular, for example, the movements would be smoother and more closely resemble actual beings. Finally, we might include ways of fine tuning the game length. The user could customize how long they wanted the chase to take, and we would adjust the algorithm and rewards accordingly. 

## Takeaways    
*It’s harder than it seems* At the onset of this project, we had big ideas about what we wanted to accomplish. The reality of implementation meant we had to give up several of our initial ideas. Our takeaway is that estimating the difficulty of a task is, in itself, a difficult task, and that care should be taken to set appropriate goals, especially given a tight turnaround.   
*Map out dependencies* With larger group projects, it’s important to map out dependencies in code so you know what to prioritize. Doing so will help prevent people from getting stuck due to dependency issues.   
*Start early* Related to point 1, start early. It always takes longer than you think, and NoMachine can smell fear.   

 
Sources:   
Littman, Michael. “Markov games as a framework for multi-agent reinforcement learning”  
https://courses.cs.duke.edu//spring07/cps296.3/littman94markov.pdf   
