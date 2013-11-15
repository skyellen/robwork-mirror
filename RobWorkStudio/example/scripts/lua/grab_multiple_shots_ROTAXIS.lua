-- a vector pointing parallel with rotation axis
rot_vec = rw.Vector3D(0,0,1)
-- a point on the rotation axis
rot_pos = rw.Vector3D(0,0,0)
-- the angle increments in degree
angle = 5
video_name = "MyRWVideo"

tmpdir = "tmp" .. os.clock()
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
 rw.sleep(0.2)
 rwstudio:saveViewGL(tmpdir .. "/" .. video_name .. count .. ".png")
 tmp2=tmp
 count = count + 1
end

rot_vec = rw.Vector3D(1,0,0)
rot = rw.EAA(rot_vec, angle/180*3.14):toRotation3D()
for tmp=0,359,angle do
 tmp3 = tmp2+tmp
 vt3d = rwstudio:getViewTransform()
 arm = vt3d:P()-rot_pos
 narm = rot*arm
 nrot = rot * vt3d:R()
 nvt3d = rw.Transform3D(narm, nrot)
 rwstudio:setViewTransform(nvt3d)
 rw.sleep(0.2)

 rwstudio:saveViewGL(tmpdir .. "/" .. video_name .. count .. ".png")
 count = count + 1
end

rwstudio:setViewTransform(cview)
print("\nExecuting: ffmpeg -qscale 5 -r 10 -b 9600 -i " .. tmpdir .. "/" .. video_name .. "%d.png " .. video_name .. ".avi")
os.execute("ffmpeg -qscale 5 -r 10 -b 9600 -i " .. tmpdir .. "/" .. video_name .. "%d.png " .. video_name .. ".avi")
print("\nDeleting temporary files..")
os.execute("rm -rf " .. tmpdir)

