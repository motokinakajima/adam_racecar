# racecar for adam team
## Folders
### real
The scripts in this folder means that it worked on the real racecar.

### sim
The scripts in this folder means that it worked in sim, and haven't testes on the real racecar.

## scripts
### real
main_real.py => the main program that has been tested on the real racecar.

### sim
cone_slalom.py => have tested on sim and works. HSV is not adjusted for real racecar.

curve02.py => wallfollow + turn left, only right side.

wall_follow.py => wallfollow + turn left, both sides.

main_3.py => main program for sim.

wall_follow4.py => right angle curve

#memos
## cone HSV
((74,98,0),(118,255,255))

## cone regression
y = 2174.2984490852577 / x**0.5 + 0.07838245131948973

## line hsv
### 6th floor
((68,49,131),(100,184,255))
