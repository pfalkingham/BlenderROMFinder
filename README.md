Finds all possible poses within min/max/step for rotation and translation, where two objects don't collide.

NOT HEAVILY TESTED

Have two meshes, proximal and distal.  Distal needs to be the one that moves.  If it's on a joint/armature, set the armature as the rotational object.  If you've just moved the pivot of the distal object, set the distal object as the rotation object.

Select if you want a csv file, or the data stored as a json attribute on the rotational object.  You can also select 'visualize' which will keyframe every non-colliding position (there will probably be lots)

Video: https://youtu.be/sQL41YbC_TY
