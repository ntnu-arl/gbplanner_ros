\page transform_fbx_to_dae Transform FBX to Collada in Blender

This tutorial shows how to transform a mesh in fbx format to collada with Blender and change its origin.

You can download Blender +2.81 from [this web site](https://www.blender.org/download/).

# Importing the FBX file

Remove all the elements in the scene. You can select the elements pushing `Ctrl + Shift + left mouse button`
from the list of elements that you can see in the right top corner of the Blender window.

@image html img/transform_fbx_to_dae/remove_elemets.gif

Then import your FBX model. `File -> Import -> Fbx (.fbx).`

@image html img/transform_fbx_to_dae/import_model.gif

# Modify properties of the model

Now we can edit some of the properties of the object.

## Change the origin of the model (Optional)

Sometimes a mesh may have an origin at an offset from the desired location. For example,
you may want to move the origin of a robot arm to the base or move the origin of a wheel to its center.

Select the object by pushing the letter `a` and enter to `Edit mode` by pushing `Tab`.
Select the part of the object at which you’d like to place the origin point (vertex, edge or face).
When we are ready push `Shift + s` to bring up the *snap menu* and select `Cursor to active` to locate
the Blender cursor (red/white circle thing) in the selected new place.
Switch back to `Object Mode` and see the Blender cursor where your future origin is going to be.
In the `Options` menu select `Origins` and then select in the object menu `Object -> Set Origin -> Origin to 3D Cursor`.
Now your origin is at the desired place. We can move this origin to another place by editing the
transform dialog.

@image html img/transform_fbx_to_dae/edit_origin.gif

# Exporting to Collada

Finally export your model to collada `File -> Export -> Collada (default)(.dae).`
