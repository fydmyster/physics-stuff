This is code for some stuff I was tinkering with for some cheap, fake-ass physics I could use
in my python games without much hassle. For collision detection I use the Separating Axis Theorem(code for which is in sat.py aong with some convenience polygonal objects you can create instances of eg. SATDeg45 
etc.)

When two bodies collide I get collision data from SAT. But for resolution I don't use the min
penetration distance and vector to separate the shapes as this results in a loss of torque/
angular momentum. What I do is I decompose the shape into line segments(what I refer to as line colliders in the code) and use the point those intersect to get the contact points. I then plug those points into the equations I got from Jakobsens paper on Verlet Integration and voila!

#Circular bodies are still WIP so don't bother using them as they buggy on the collision resolution aspect. 

# All in all I think Jakobsen is a genius and he deserves a ton of credit for his particle based verlet work. I'm horrid when it comes to math and all that physics stuff but Jakobsens stuff was relatively simple to implement(I say that now but banged my head a couple times when I got stuck), and works, for lack of a better word, like magic.

Run test example.py and use the Interface I demonstrated in there. The other scripts,like verlet_stick.py, verlet_box.py were several iterations before the final product so they are buggy but you can see where I started from whilst writing the implementation.

# The thing I learnt is that you can get by with a lot of faking in physics. I basically pulled the algorithm I use for calculating contact points out of my bum, but it worked!

# Everything is fake. Friction is handled fakely as well. What matters is what the end user sees.

Enough rambling.. Sorry I didnt have time to document stuff clearly.

Feel free to contact me on twitter:
	@fydmyster
gmail:
	fienixgdev@gmail.com

fienix out!  