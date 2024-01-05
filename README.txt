Assignment 1 - Keyframed Animation

Name: Yuanlong Zhou
UIN: 634005710
Email: ryanbowz@tamu.edu

----------
-Summary:
All of the tasks are accomplished.
All of the bonus are done and finished. 
Fulfills all points requirements of the assignment.

-Tmax value from Task 3: 
tmax = 5.0f

-Time control functions used in Task 6:
(8 modes in total, determined by how many times you pressed 's') 
1. No arc-length
2. With arc-length
3. Ease in/out
4. Stop in the middle
The helicopter preforms s=sqrt(t) in the first half (s<=0.5), until it reaches the middle point of the whole arc-length. The helicopter will stop in the middle point until the next loop.
5. Go backwards 
Normally, the helicopter performs s=-3(t-0.5)^2+0.75. In this function, when t=0.5 the helicopter will come to the 3/4 arc-length point and then go backwards to the origin.
6. Disappear & Transport
When t<=0.4, s=1.8t, which is linear relevant to t. When t is between 0.4 and 0.6, the helicopter simply disappears [in code it is rendered in origin, overlapped with the 1st keyframe]. After t=0.6, s=1.3t-0.3, making it transport to another point.
7. Solving a 4x4 linear system
The time control function is subject to:
f(0)=0; f(0.4)=0.68, f(0.65)=0.45, f(1)=1.
Using glm matrix to solve them, we can come up with a curve that satisfy the above conditions.
8. Solving a 8x8 linear system <Bonus 1: Multiple Cubics>
The time control function is composed of two individual functions f(x) and g(x).
The subjecting conditions are:
f(0)=0, f'(0)=0, f(0.4)=g(0.4), f'(0.4)=g'(0.4),
g(0.5)=0.2, g'(0.5)=0, g(1)=1, g'(1)=0
We can use the Eigen library to come up with 2 cubics that satisfy the above conditions.

-Misc.
1. Used a class to represent a helicopter.
2. Used a class to represent a keyframe (including the helicopter frames and spline curves that connect them)
3. Put all quantities that do not change over time in init(), quantities that relevant to time are put in render().
4. The last bonus: Spline Surface is finished by drawing a grid of lines at fixed u and v values, and the normal vectors are illustrated by additional lines coming out of each point. (To save algorithm complexity and optimize performance, the normal computation is integrated in the surface spline drawing. Maybe a little confusing when reading codes, but it makes the program run more smoothly.) The control points of the surface are drawn in red.
5. The equal-distance points on curve are not seen as a part of keyframe, rather, they are treated as a part of the animation, so they are always shown on screen.
6. The default mode to build the arc-length table has been set to Gaussian Quadrature. If you want to use the linear approximation way, you can uncomment the buildTable() function (because the table is computed in the init(), you cannot change it dynamically). Searching the index of s in arc-length table is implemented by binary search function to reduce time complexity.
7. You can change the mode of curves between B splines and Catmull-Rom (default) splines by pressing 'b'. (Just for testing purpose, not useful in this assignment because the keyframe will not be on curves when using B splines)
8. Reorganized the code as much as possible, by creating several classes and extracting complicated processes to separate functions, unless there is not necessity or need to do so (when you separate them, you would face more arguments references, larger complexity or longer codes).
9. This project is for course CSCE-689-606 only.



