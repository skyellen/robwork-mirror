------------------------------------------------------------------------------------------
-- This script file includes utilities for generating video clips from RobWorkStudio.
-- 
--
------------------------------------------------------------------------------------------

------------------------------------------------------------------------------------------
-- Creates a video by rotation around the axis rot_vec which goes through the
-- point rot_pos. 
------------------------------------------------------------------------------------------
function makeVideoAxis(video_name_arg, rot_pos_arg, rot_vec_arg, rot_vec2_arg, stepAngle)

video_name = "DefaultRWVideo"
if video_name_arg~=nil then  video_name = video_name_arg end

-- a point on the rotation axis
rot_pos = rw.Vector3D(0,0,0)
if rot_pos_arg~=nil then  rot_pos = rot_pos_arg end

-- a vector pointing parallel with rotation axis
rot_vec = rw.Vector3D(0,0,1)
if rot_vec_arg~=nil then  rot_vec = rot_vec_arg end

-- a vector pointing parallel with rotation axis
rot_vec2 = rw.Vector3D(1,0,0)
if rot_vec2_arg~=nil then  rot_vec2 = rot_vec2_arg end

-- the delay in seconds between setting the state to grabbing the view
local delay = 0.2

-- the angle increments in degree
angle = 5
if stepAngle~=nil then  angle = stepAngle end

local tmpdir = "tmp" .. os.clock()
os.execute("mkdir " .. tmpdir)

cview = rwstudio:getViewTransform()
count = 0
rot = rw.EAA(rot_vec, angle/180*3.14):toRotation3D()
tmp2 = 0

for tmp=0,359,angle do
 vt3d = rwstudio:getViewTransform()
 arm = vt3d:P()-rot_pos
 narm = rot*arm
 nrot = rot * vt3d:R()
 nvt3d = rw.Transform3D(narm, nrot)
 rwstudio:setViewTransform(nvt3d)
 rw.sleep( delay )
 rwstudio:saveViewGL(tmpdir .. "/" .. video_name .. count .. ".png")
 tmp2=tmp
 count = count + 1
end

rot = rw.EAA(rot_vec2, angle/180*3.14):toRotation3D()
for tmp=0,359,angle do
 tmp3 = tmp2+tmp
 vt3d = rwstudio:getViewTransform()
 arm = vt3d:P()-rot_pos
 narm = rot*arm
 nrot = rot * vt3d:R()
 nvt3d = rw.Transform3D(narm, nrot)
 rwstudio:setViewTransform(nvt3d)
 rw.sleep( delay )

 rwstudio:saveViewGL(tmpdir .. "/" .. video_name .. count .. ".png")
 count = count + 1
end

rwstudio:setViewTransform(cview)
print("\nExecuting: ffmpeg -qscale 5 -r 10 -b 9600 -i " .. tmpdir .. "/" .. video_name .. "%d.png " .. video_name .. ".avi")
os.execute("ffmpeg -qscale 5 -r 10 -b 9600 -i " .. tmpdir .. "/" .. video_name .. "%d.png " .. video_name .. ".avi")
print("\nDeleting temporary files..")
os.execute("rm -rf " .. tmpdir)

end


