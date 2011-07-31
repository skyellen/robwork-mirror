-- a vector pointing parallel with rotation axis
rot_vec = rw.Vector3D(1,0,0)
-- a point on the rotation axis
rot_pos = rw.Vector3D(0,0,0)
-- the angle increments in degree
angle = 10

rot = rw.EAA(rot_vec, angle/180*3.14):toRotation3D()

for tmp=0,359,angle do
 vt3d = rwstudio:getViewTransform()
 arm = vt3d:P()-rot_pos
 narm = rot*arm
 nrot = rot * vt3d:R()
 nvt3d = rw.Transform3D(narm, nrot)
 rwstudio:setViewTransform(nvt3d)
 rw.sleep(0.5)
 rwstudio:saveViewGL("view_" .. tmp .. ".png")
end
form(nvt3d)sform(nvt3d)